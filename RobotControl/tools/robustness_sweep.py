"""Robustness explorer (idea #19): how does the deployed Level 0 policy cope when the twin is wrong?

Runs the deployed ``policy.onnx`` (onnxruntime, batched, exactly the file the robot runs) on many perturbed copies
of the headless MuJoCo stand twin at once. Each copy changes ONE factor from nominal (sensitivity rows), and a
gain x latency grid shows how the two worst sim-to-real unknowns interact. Every copy runs the validation sweep
command (head yaw +-30 deg @0.25 Hz, shoulder pitch sweep @0.15 Hz) and is scored like the verification gate:

  fail  mean tracking error > 3 deg, or max joint acceleration > 80 rad/s^2, or any safety trip / non-finite state
  warn  mean tracking error > 2 deg, or max joint acceleration > 60 rad/s^2
  pass  otherwise

Pure simulation: nothing here talks to the robot or changes the policy.

    .venv/Scripts/python.exe -m robotcontrol.tools.robustness_sweep            # full: 7 levels, 7x7 grid
    .venv/Scripts/python.exe -m robotcontrol.tools.robustness_sweep --quick    # 5 levels, 5x5 grid

Writes logs/robustness/latest.json (schema: docs/service_api.md section 2), report.md and heatmap.png.
Progress lines ``@progress <done>/<total>`` are read by the behaviour service.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import pathlib
import sys
import time
from dataclasses import dataclass

import mujoco
import numpy as np
from mujoco import rollout

from .. import PROJECT_ROOT
from ..training.level0_env import CONTROLLED_JOINTS, POLICY_DT, SUBSTEPS, Level0Config, Level0StandEnv

MODEL_DIR = PROJECT_ROOT / "RobotControl/models/r1_level0_upperbody"
OUT_DIR = PROJECT_ROOT / "logs/robustness"
PHYSICS_DT = POLICY_DT / SUBSTEPS  # 5 ms
PRESETTLE_S = 2.0
WARMUP_S = 1.0  # tracking error is judged after the first second, like validate_level0
THRESH = {"mean_err_deg": 3.0, "max_acc": 80.0, "warn_mean_err_deg": 2.0, "warn_acc": 60.0}
LIMIT_TRIP_RAD = np.deg2rad(1.0)  # beyond the 0.9 soft range by more than this = safety trip
TRAIN_NOISE_POS, TRAIN_NOISE_VEL = 0.01, 0.5  # Level0Config obs noise (uniform half-widths)


@dataclass(frozen=True)
class Factor:
    id: str
    label: str
    unit: str
    nominal: float
    full: tuple
    quick: tuple


FACTORS = (
    Factor("mass", "Body mass", "×", 1.0, (0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3), (0.7, 0.85, 1.0, 1.15, 1.3)),
    Factor("friction", "Joint friction + damping", "×", 1.0, (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0), (0.25, 0.5, 1.0, 2.0, 3.0)),
    Factor("gain", "Motor gains kp/kd", "×", 1.0, (0.6, 0.7, 0.85, 1.0, 1.15, 1.3, 1.5), (0.6, 0.8, 1.0, 1.25, 1.5)),
    Factor("latency_ms", "Action latency", "ms", 0.0, (0, 10, 20, 30, 40, 60, 80), (0, 20, 40, 60, 80)),
    Factor("noise", "Sensor noise", "× training", 0.0, (0, 0.5, 1, 2, 3, 4, 6), (0, 1, 2, 4, 6)),
    Factor("torque", "Torque limit", "×", 1.0, (0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0), (0.4, 0.55, 0.7, 0.85, 1.0)),
)  # fmt: skip
NOMINAL = {f.id: f.nominal for f in FACTORS}
GRID_X, GRID_Y = "gain", "latency_ms"


def sweep_command(t: np.ndarray, default_shoulder_pitch: float) -> np.ndarray:
    """Same deterministic validation command as training/export_level0.sweep_command (pytest checks they agree)."""
    yaw = np.deg2rad(30.0) * np.sin(2 * np.pi * 0.25 * t)
    pitch = default_shoulder_pitch - 0.6 * (1 - np.cos(2 * np.pi * 0.15 * t))
    return np.stack([yaw, pitch], axis=-1)


def build_cases(quick: bool) -> tuple[list[dict], list[tuple], list[tuple]]:
    """Perturbation dicts: index 0 is nominal, then the sensitivity rows, then the interaction grid.
    Returns (cases, sens_index[(factor_idx, level_idx) -> case], grid_index[(y_idx, x_idx) -> case])."""
    cases = [dict(NOMINAL)]
    sens, grid = [], []
    for fi, f in enumerate(FACTORS):
        for li, v in enumerate(f.quick if quick else f.full):
            c = dict(NOMINAL)
            c[f.id] = float(v)
            cases.append(c)
            sens.append((fi, li))
    fx = next(f for f in FACTORS if f.id == GRID_X)
    fy = next(f for f in FACTORS if f.id == GRID_Y)
    for yi, vy in enumerate(fy.quick if quick else fy.full):
        for xi, vx in enumerate(fx.quick if quick else fx.full):
            c = dict(NOMINAL)
            c[GRID_X], c[GRID_Y] = float(vx), float(vy)
            cases.append(c)
            grid.append((yi, xi))
    return cases, sens, grid


def apply_case(env: Level0StandEnv, i: int, case: dict) -> None:
    """Write one perturbation into env i's own MjModel (the env was reset with nominal physics)."""
    mi, base = env.models[i], env.base_model
    mi.body_mass[:] = base.body_mass * case["mass"]
    mi.body_inertia[:] = base.body_inertia * case["mass"]
    h = env.hinge_dofs
    mi.dof_frictionloss[h] = base.dof_frictionloss[h] * case["friction"]
    mi.dof_damping[h] = base.dof_damping[h] * case["friction"]
    kp = env.kp_all * case["gain"]
    kd = env.kd_all * case["gain"]
    kp[env.passive_idx] = 0.0
    kd[env.passive_idx] = env.passive_kd
    mi.actuator_gainprm[:, 0] = kp
    mi.actuator_biasprm[:, 1] = -kp
    mi.actuator_biasprm[:, 2] = -kd
    mi.actuator_forcerange[:] = base.actuator_forcerange * case["torque"]


def verdict(cell: dict) -> str:
    if (
        not cell["finite"]
        or cell["trips"] > 0
        or cell["mean_err_deg"] > THRESH["mean_err_deg"]
        or cell["max_acc"] > THRESH["max_acc"]
    ):
        return "fail"
    if cell["mean_err_deg"] > THRESH["warn_mean_err_deg"] or cell["max_acc"] > THRESH["warn_acc"]:
        return "warn"
    return "pass"


def run_sweep(
    model_dir: pathlib.Path = MODEL_DIR,
    quick: bool = False,
    rollout_s: float | None = None,
    nthread: int | None = None,
    progress=None,
) -> dict:
    import onnxruntime as ort

    cases, sens_idx, grid_idx = build_cases(quick)
    n = len(cases)
    rollout_s = rollout_s if rollout_s is not None else (5.0 if quick else 8.0)
    cfg = Level0Config(num_envs=n, seed=0)
    if nthread:
        cfg.nthread = nthread
    t0 = time.time()
    env = Level0StandEnv(cfg)
    try:
        env.set_eval(True)
        env.reset(np.arange(n))
        for i, c in enumerate(cases):
            apply_case(env, i, c)
        env.presettle(PRESETTLE_S)  # nominal-delay zero action while the hanging legs settle (validation procedure)

        sess = ort.InferenceSession(str(model_dir / "policy.onnx"), providers=["CPUExecutionProvider"])
        rng = np.random.default_rng(1)
        delay_sub = np.array([int(round(c["latency_ms"] / (PHYSICS_DT * 1000))) for c in cases])
        noise = np.array([c["noise"] for c in cases])[:, None]
        hist = int(delay_sub.max() // SUBSTEPS) + 2  # policy steps of target history needed
        targets = np.tile(env.default, (hist, n, 1))  # targets[k] = decided k policy steps ago
        last_action = np.zeros((n, env.num_actions))
        ip = CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")
        iy = CONTROLLED_JOINTS.index("head_yaw_joint")
        steps = int(round(rollout_s / POLICY_DT))
        err_sum = np.zeros(n)
        err_cnt = 0
        err_max = np.zeros(n)
        acc_max = np.zeros(n)
        acc_joint = np.zeros((n, env.num_actions))
        jitter_sum = np.zeros(n)
        trips = np.zeros(n, dtype=np.int64)
        finite = np.ones(n, dtype=bool)
        ctrl = np.tile(env.default_all, (n, SUBSTEPS, 1))
        q, dq = env.q.copy(), env.dq.copy()
        state0 = env.state.copy()  # settled nominal state, used to keep a diverged copy numerically alive
        for k in range(steps):
            t = k * POLICY_DT
            cmd = np.tile(sweep_command(np.array(t), env.default[ip]), (n, 1))
            qn = q + rng.uniform(-1, 1, q.shape) * TRAIN_NOISE_POS * noise
            dqn = dq + rng.uniform(-1, 1, dq.shape) * TRAIN_NOISE_VEL * noise
            obs = np.concatenate([qn - env.default, dqn, last_action, cmd], axis=1).astype(np.float32)
            obs = np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
            act = sess.run(None, {"obs": obs})[0].astype(np.float64)
            jitter_sum += np.abs(act - last_action).mean(axis=1)
            last_action = act
            targets = np.roll(targets, 1, axis=0)
            targets[0] = env.targets_from_action(act)
            # substep s of this policy step (time t + s*5 ms) applies the newest target decided at or before
            # (t + s*5 ms - latency): that one was decided ceil((latency - s*5 ms) / 20 ms) policy steps ago.
            for s in range(SUBSTEPS):
                back = np.clip(np.ceil((delay_sub - s) / SUBSTEPS).astype(int), 0, hist - 1)
                ctrl[:, s, env.ctrl_idx] = targets[back, np.arange(n)]
            state, _ = rollout.rollout(
                env.models,
                env.datas,
                env.state,
                ctrl,
                control_spec=mujoco.mjtState.mjSTATE_CTRL,
                nstep=SUBSTEPS,
                persistent_pool=True,
            )
            env.state[:] = state[:, -1, :]
            bad = ~np.isfinite(env.state).all(axis=1)
            finite &= ~bad
            env.state[bad] = state0[bad]  # keep the batch numerically alive; the cell is already a fail
            q_new = env.state[:, env.q_cols]
            dq_new = env.state[:, env.dq_cols]
            acc = np.abs(dq_new - dq) / POLICY_DT
            q, dq = q_new, dq_new
            if t + POLICY_DT > WARMUP_S:
                e = np.rad2deg(np.abs(np.stack([q[:, iy] - cmd[:, 0], q[:, ip] - cmd[:, 1]], axis=1)))
                err_sum += e.mean(axis=1)
                err_cnt += 1
                err_max = np.maximum(err_max, e.max(axis=1))
                acc_max = np.maximum(acc_max, acc.max(axis=1))
                acc_joint = np.maximum(acc_joint, acc)
            over = (q > env.soft_hi + LIMIT_TRIP_RAD) | (q < env.soft_lo - LIMIT_TRIP_RAD)
            trips += over.any(axis=1)
            if progress and (k % 25 == 0 or k == steps - 1):
                progress(k + 1, steps)
    finally:
        env.close()

    def cell(i: int) -> dict:
        c = {
            "mean_err_deg": round(float(err_sum[i] / max(err_cnt, 1)), 3),
            "max_err_deg": round(float(err_max[i]), 3),
            "max_acc": round(float(acc_max[i]), 2),
            "acc_joint": CONTROLLED_JOINTS[int(np.argmax(acc_joint[i]))],
            "jitter": round(float(jitter_sum[i] / steps), 4),
            "trips": int(trips[i]),
            "finite": bool(finite[i]),
        }
        if not c["finite"]:
            c.update(mean_err_deg=None, max_err_deg=None, max_acc=None, jitter=None)
            c["verdict"] = "fail"
        else:
            c["verdict"] = verdict(c)
        return c

    cells = [cell(i) for i in range(n)]
    sensitivity = []
    for fi, f in enumerate(FACTORS):
        levels = list(f.quick if quick else f.full)
        row = [cells[1 + j] for j, (ffi, _) in enumerate(sens_idx) if ffi == fi]
        sensitivity.append(
            {"id": f.id, "label": f.label, "unit": f.unit, "nominal": f.nominal, "levels": levels, "cells": row}
        )
    fx = next(f for f in FACTORS if f.id == GRID_X)
    fy = next(f for f in FACTORS if f.id == GRID_Y)
    xs, ys = list(fx.quick if quick else fx.full), list(fy.quick if quick else fy.full)
    g0 = 1 + len(sens_idx)
    grid_cells = [[cells[g0 + yi * len(xs) + xi] for xi in range(len(xs))] for yi in range(len(ys))]

    labelled = [(c, f"{f['id']}={v:g} {f['unit']}") for f in sensitivity for c, v in zip(f["cells"], f["levels"])]
    labelled += [
        (grid_cells[yi][xi], f"{GRID_Y}={ys[yi]:g} ms, {GRID_X}={xs[xi]:g}×")
        for yi in range(len(ys))
        for xi in range(len(xs))
    ]
    counts = {v: sum(1 for c, _ in labelled if c["verdict"] == v) for v in ("pass", "warn", "fail")}

    def badness(c):
        return float("inf") if not c["finite"] else c["mean_err_deg"] + c["trips"] * 100 + max(0, c["max_acc"] - 80)

    worst_c, worst_l = max(labelled, key=lambda x: badness(x[0]))
    worst = (
        f"{worst_l}: non-finite state"
        if not worst_c["finite"]
        else f"{worst_l}: mean error {worst_c['mean_err_deg']:.1f}°, max accel {worst_c['max_acc']:.0f} rad/s² "
        f"({worst_c['acc_joint'].replace('_joint', '')})"
        + (f", {worst_c['trips']} limit trips" if worst_c["trips"] else "")
    )
    policy = model_dir / "policy.onnx"
    try:
        policy_rel = str(policy.resolve().relative_to(PROJECT_ROOT)).replace("\\", "/")
    except ValueError:
        policy_rel = policy.name
    return {
        "available": True,
        "created_at": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "policy": policy_rel,
        "policy_sha256": hashlib.sha256(policy.read_bytes()).hexdigest(),
        "quick": quick,
        "duration_s": round(time.time() - t0, 1),
        "rollout_s": rollout_s,
        "num_cases": n,
        "thresholds": dict(THRESH),
        "nominal": cells[0],
        "sensitivity": sensitivity,
        "interaction": {
            "x": {"id": fx.id, "label": fx.label, "unit": fx.unit, "levels": xs},
            "y": {"id": fy.id, "label": fy.label, "unit": fy.unit, "levels": ys},
            "cells": grid_cells,
        },
        "summary": {**counts, "worst": worst},
    }


GLYPH = {"pass": "✓ pass", "warn": "! warn", "fail": "✗ fail"}


def write_report(rep: dict, out: pathlib.Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "latest.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")

    def fmt(c):
        if not c["finite"]:
            return "✗ fail (non-finite)"
        return f"{GLYPH[c['verdict']]} {c['mean_err_deg']:.2f}° / {c['max_acc']:.0f}"

    s = rep["summary"]
    lines = [
        "# Level 0 robustness sweep",
        "",
        f"Policy `{rep['policy']}` (sha256 {rep['policy_sha256'][:12]}), {rep['created_at']}, "
        f"{'quick' if rep['quick'] else 'full'} sweep: {rep['num_cases']} perturbed twins, "
        f"{rep['rollout_s']:g} s validation sweep each, {rep['duration_s']:g} s wall time.",
        "",
        f"**{s['pass']} pass, {s['warn']} warn, {s['fail']} fail.** Worst: {s['worst']}.",
        "",
        "Cells: verdict, mean tracking error (deg) / max joint acceleration (rad/s²). Fail: mean error > 3°, "
        "accel > 80 rad/s², a joint past its soft limit, or non-finite state. Warn: error > 2° or accel > 60.",
        "",
        f"Nominal: {fmt(rep['nominal'])}; largest acceleration on {rep['nominal']['acc_joint']}, "
        f"action jitter {rep['nominal']['jitter']} per step (the verification gate allows 0.05).",
        "",
        "## One factor at a time",
        "",
    ]
    for f in rep["sensitivity"]:
        lines += [f"### {f['label']} ({f['unit']})", "", "| level | result |", "|---|---|"]
        for v, c in zip(f["levels"], f["cells"]):
            mark = " (nominal)" if abs(v - f["nominal"]) < 1e-9 else ""
            lines.append(f"| {v:g}{mark} | {fmt(c)} |")
        lines.append("")
    it = rep["interaction"]
    lines += [
        f"## {it['y']['label']} ({it['y']['unit']}) × {it['x']['label']} ({it['x']['unit']})",
        "",
        "| "
        + it["y"]["unit"]
        + " \\ "
        + it["x"]["unit"]
        + " | "
        + " | ".join(f"{x:g}" for x in it["x"]["levels"])
        + " |",
        "|---" * (len(it["x"]["levels"]) + 1) + "|",
    ]
    for y, row in zip(it["y"]["levels"], it["cells"]):
        lines.append(f"| {y:g} | " + " | ".join(fmt(c) for c in row) + " |")
    lines += ["", "![heatmap](heatmap.png)", ""]
    (out / "report.md").write_text("\n".join(lines), encoding="utf-8")
    _plot(rep, out / "heatmap.png")


def _plot(rep: dict, path: pathlib.Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    status = ListedColormap(["#0ca30c", "#fab219", "#d03b3b"])  # good / warning / critical
    code = {"pass": 0, "warn": 1, "fail": 2}
    sens = rep["sensitivity"]
    it = rep["interaction"]
    ncol = max(len(f["levels"]) for f in sens)
    fig, axs = plt.subplots(1, 2, figsize=(15, 5.5), gridspec_kw={"width_ratios": [1.25, 1]})
    for ax, (title, rows, row_labels, col_labels) in zip(
        axs,
        [
            (
                "One factor at a time",
                [f["cells"] + [None] * (ncol - len(f["cells"])) for f in sens],
                [f"{f['label']} ({f['unit']})" for f in sens],
                None,
            ),
            (
                f"{it['y']['label']} × {it['x']['label']}",
                it["cells"],
                [f"{y:g} {it['y']['unit']}" for y in it["y"]["levels"]],
                [f"{x:g}{it['x']['unit']}" for x in it["x"]["levels"]],
            ),
        ],
    ):
        z = np.array([[code[c["verdict"]] if c else np.nan for c in r] for r in rows], dtype=float)
        ax.imshow(np.ma.masked_invalid(z), cmap=status, vmin=0, vmax=2, aspect="auto")
        for yi, r in enumerate(rows):
            for xi, c in enumerate(r):
                if c is None:
                    continue
                txt = "NaN" if not c["finite"] else f"{c['mean_err_deg']:.1f}° {c['max_acc']:.0f}/s²"
                ax.text(xi, yi, f"{c['verdict']}\n{txt}", ha="center", va="center", fontsize=7.5,
                        color="#0b0b0b" if c["verdict"] == "warn" else "white")  # fmt: skip
        ax.set_yticks(range(len(row_labels)), row_labels, fontsize=8)
        if col_labels:
            ax.set_xticks(range(len(col_labels)), col_labels, fontsize=8)
        else:
            ax.set_xticks([])
            for yi, f in enumerate(sens):
                for xi, v in enumerate(f["levels"]):
                    ax.text(xi, yi + 0.42, f"{v:g}", ha="center", va="bottom", fontsize=6.5, color="#333")
        ax.set_title(title)
    fig.suptitle(f"Level 0 robustness: {rep['summary']['pass']} pass, {rep['summary']['warn']} warn, "
                 f"{rep['summary']['fail']} fail (mean tracking error, max joint acceleration rad/s²)")  # fmt: skip
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true", help="5 levels per factor and a 5x5 grid, 5 s rollouts")
    ap.add_argument("--model-dir", type=pathlib.Path, default=MODEL_DIR)
    ap.add_argument("--out", type=pathlib.Path, default=OUT_DIR)
    ap.add_argument("--rollout-s", type=float, default=None)
    args = ap.parse_args(argv)

    def progress(done, total):
        print(f"@progress {done}/{total}", flush=True)

    rep = run_sweep(args.model_dir, quick=args.quick, rollout_s=args.rollout_s, progress=progress)
    write_report(rep, args.out)
    s = rep["summary"]
    print(f"robustness: {s['pass']} pass, {s['warn']} warn, {s['fail']} fail. Worst: {s['worst']}", flush=True)
    print(f"wrote {args.out / 'latest.json'}, report.md, heatmap.png", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
