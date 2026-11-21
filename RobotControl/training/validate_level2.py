"""Level 2 verification gate: may this walking policy drive the walk sandbox?

A candidate is a folder with policy.onnx, deploy.yaml (our Level 2 format, docs/level2_walk.md) and golden.npz
(obs/action pairs + raw signals recorded in the trainer). Checks:

  Contract   ONNX tensor names and shapes (obs 83 -> actions 24), joint order = r1_joint_map.yaml policy_index
             0..23, default pose and gains vs the joint map
  Parity     our observation builder reproduces the trainer's observations from raw signals (< 1e-4), and
             onnxruntime reproduces the trainer's actions for them (< 1e-4)
  Sim2sim    in OUR calibrated MuJoCo ground scene (5 ms, 50 Hz): stand still, walk forward / backward / sideways,
             turn, and a random command sequence, all without a fall and with velocity tracking inside the limits

With --promote and every check passed, the policy is copied to RobotControl/models/r1_level2_velocity/ and its
deploy.yaml gets ``validation: {passed: true, policy_sha256: ...}``; only then does the sandbox load it.
The walk sandbox is simulation only: passing this gate never sends Level 2 to the real robot.

    .venv/Scripts/python.exe -m robotcontrol.training.validate_level2 --candidate logs/level2_training/candidate [--promote]
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import pathlib
import shutil
import sys

import numpy as np
import yaml

from .. import PROJECT_ROOT
from ..configs.joint_map import JointMap
from ..walk.policy import (
    MODEL_DIR,
    NUM_POLICY_JOINTS,
    Level2Deploy,
    Level2Policy,
    ObsSignals,
    build_obs,
    rel_path,
    sha256,
)
from ..walk.sim import POLICY_DT, WalkSim

OUT = PROJECT_ROOT / "logs/level2_validation"
PASS, FAIL, INFO = "PASS", "FAIL", "INFO"

# (name, command vx vy wz, seconds, max mean |error| per axis after 2 s: vx, vy (m/s), wz (rad/s))
SCENARIOS = [
    ("stand still", (0.0, 0.0, 0.0), 10.0, (0.10, 0.10, 0.15)),
    ("forward 0.5 m/s", (0.5, 0.0, 0.0), 10.0, (0.15, 0.15, 0.25)),
    ("forward 1.0 m/s", (1.0, 0.0, 0.0), 8.0, (0.25, 0.20, 0.30)),
    ("backward 0.3 m/s", (-0.3, 0.0, 0.0), 8.0, (0.15, 0.15, 0.25)),
    ("sideways 0.3 m/s", (0.0, 0.3, 0.0), 8.0, (0.15, 0.15, 0.25)),
    ("turn 0.5 rad/s", (0.0, 0.0, 0.5), 8.0, (0.15, 0.15, 0.25)),
]


def body_rates(sim: WalkSim) -> np.ndarray:
    """(vx, vy) in the heading frame, and world yaw rate."""
    v = sim.base_lin_vel_world()
    yaw = sim.yaw()
    c, s = math.cos(yaw), math.sin(yaw)
    w, x, y, z = sim.base_quat
    wl = sim.base_ang_vel_body()
    # world z component of the body angular velocity: third row of R(q) @ w_local
    wz = 2 * (x * z - w * y) * wl[0] + 2 * (y * z + w * x) * wl[1] + (1 - 2 * (x * x + y * y)) * wl[2]
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], wz])


def run_scenario(policy: Level2Policy, cmd, seconds: float, seed: int = 0, random_cmds: bool = False) -> dict:
    sim = WalkSim("flat", mode="policy", policy=policy)
    rng = np.random.default_rng(seed)
    lim = policy.deploy.cmd_limits
    sim.set_command(*cmd)
    errs, max_dq = [], 0.0
    start = sim.base_pos.copy()
    steps = int(round(seconds / POLICY_DT))
    for k in range(steps):
        if random_cmds and k % int(2.0 / POLICY_DT) == 0:
            sim.set_command(rng.uniform(*lim["vx"]), rng.uniform(*lim["vy"]) * 0.6, rng.uniform(*lim["wz"]) * 0.6)
        sim.step()
        max_dq = max(max_dq, float(np.abs(sim.dq[:NUM_POLICY_JOINTS]).max()))
        if sim.falls:
            break
        if sim.t > 2.0:
            errs.append(np.abs(body_rates(sim) - sim.cmd))
    e = np.mean(errs, axis=0) if errs else np.full(3, np.inf)
    return {
        "falls": sim.falls,
        "t_end": round(sim.t, 2),
        "err": e,
        "drift": float(np.linalg.norm(sim.base_pos[:2] - start[:2])),
        "max_dq": max_dq,
    }


def course_lap(policy: Level2Policy, limit_s: float = 120.0) -> dict:
    """Informational: drive the 'loop' course with a simple heading controller toward the next checkpoint."""
    sim = WalkSim("loop", mode="policy", policy=policy)
    for _ in range(int(limit_s / POLICY_DT)):
        nxt = np.asarray(sim.geometry["checkpoints"][sim.cp_index][:2])
        d = nxt - sim.base_pos[:2]
        err = math.atan2(d[1], d[0]) - sim.yaw()
        err = math.atan2(math.sin(err), math.cos(err))
        sim.set_command(0.6 if abs(err) < 0.5 else 0.15, 0.0, float(np.clip(1.5 * err, -1.0, 1.0)))
        sim.step()
        if sim.laps >= 1:
            break
    return {"laps": sim.laps, "lap_s": sim.last_lap_s, "falls": sim.falls, "t": round(sim.t, 1)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--candidate", type=pathlib.Path, default=PROJECT_ROOT / "logs/level2_training/candidate")
    ap.add_argument("--out", type=pathlib.Path, default=OUT)
    ap.add_argument("--promote", action="store_true", help="copy into RobotControl/models/r1_level2_velocity on PASS")
    args = ap.parse_args(argv)
    cand = args.candidate
    args.out.mkdir(parents=True, exist_ok=True)
    results = []

    def check(name, ok, value, detail=""):
        results.append((PASS if ok else FAIL, name, value, detail))
        return ok

    jm = JointMap.load()
    for f in ("policy.onnx", "deploy.yaml", "golden.npz"):
        if not (cand / f).exists():
            print(f"FAIL: {cand / f} missing")
            return 1
    dep = Level2Deploy.load(cand / "deploy.yaml")
    policy = Level2Policy(cand, require_validated=False)

    # ---- contract
    check("joint order = joint map policy_index 0..23", dep.joint_names == jm.names[:NUM_POLICY_JOINTS], "")
    check("num_obs = 83", dep.num_obs == 83, dep.num_obs)
    d_def = float(np.abs(dep.default - np.asarray(jm.defaults[:NUM_POLICY_JOINTS])).max())
    check("default pose vs joint map (max |diff| rad)", d_def < 1e-3, round(d_def, 5))
    n = NUM_POLICY_JOINTS
    gains_ok = bool(np.allclose(dep.kp, jm.kp[:n]) and np.allclose(dep.kd, jm.kd[:n]))
    check("kp/kd = joint map (the gains the real motors use)", gains_ok, "")

    # ---- parity with the trainer
    g = np.load(cand / "golden.npz")

    def gk(*names):  # golden key: ours, or the name export_level2.py uses
        return next(n for n in names if n in g.files)

    obs_err = 0.0
    for i in range(len(g["obs"])):
        s = ObsSignals(
            base_ang_vel=g[gk("base_ang_vel", "ang_vel_b")][i],
            base_quat=g[gk("base_quat", "root_quat")][i],
            command=g["command"][i],
            phase_time=float(g["phase_time"][i]),
            q=g["q"][i],
            dq=g["dq"][i],
            last_action=g["last_action"][i],
        )
        obs_err = max(obs_err, float(np.abs(build_obs(dep, s) - g["obs"][i]).max()))
    check("observation builder vs trainer (max |diff|)", obs_err < 1e-4, f"{obs_err:.2e}")
    act = np.stack([policy.act(o) for o in g["obs"]])
    act_err = float(np.abs(act - g["actions"]).max())
    check("onnxruntime actions vs trainer (max |diff|)", act_err < 1e-4, f"{act_err:.2e}")

    # ---- sim2sim in our calibrated twin
    for name, cmd, secs, tol in SCENARIOS:
        r = run_scenario(policy, cmd, secs)
        ok = r["falls"] == 0 and bool(np.all(r["err"] <= np.asarray(tol)))
        if name == "stand still":
            ok = ok and r["drift"] < 0.5
        e = r["err"]
        check(
            f"{name}: no fall, tracking within {tol}",
            ok,
            f"falls {r['falls']} at {r['t_end']} s; mean |err| vx {e[0]:.3f} vy {e[1]:.3f} wz {e[2]:.3f}"
            + (f"; drift {r['drift']:.2f} m" if name == "stand still" else ""),
        )
    r = run_scenario(policy, (0, 0, 0), 30.0, seed=7, random_cmds=True)
    check("random commands every 2 s for 30 s: no fall", r["falls"] == 0, f"falls {r['falls']} at {r['t_end']} s")
    check("joint speed sanity (max |dq| < 30 rad/s)", r["max_dq"] < 30.0, f"{r['max_dq']:.1f} rad/s")
    lap = course_lap(policy)
    results.append((INFO, "loop course lap with a heading controller", f"{lap}", "informational"))

    n_fail = sum(1 for r in results if r[0] == FAIL)
    meta = dep.raw.get("metadata", {})
    lines = [
        "# Level 2 verification gate (walk sandbox, simulation only)",
        "",
        f"Candidate: `{rel_path(cand)}` "
        f"(policy sha256 {sha256(cand / 'policy.onnx')[:12]}), source: {meta.get('source', '?')} "
        f"{meta.get('commit', '')} task {meta.get('task', '?')} iteration {meta.get('iteration', '?')}",
        f"Twin: scene_ground.xml + calibrated joint dynamics, 5 ms physics, 50 Hz policy. Run {dt.datetime.now():%Y-%m-%d %H:%M}.",
        "",
        "| Result | Check | Value |",
        "|---|---|---|",
    ]
    lines += [f"| {r} | {n} | {v} |" for r, n, v, _ in results]
    lines += ["", f"**{'ALL CHECKS PASSED' if n_fail == 0 else f'{n_fail} CHECK(S) FAILED'}**", ""]
    (args.out / "report.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))

    if args.promote:
        if n_fail:
            print("not promoted: the gate failed")
            return 1
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        shutil.copy2(cand / "policy.onnx", MODEL_DIR / "policy.onnx")
        shutil.copy2(cand / "golden.npz", MODEL_DIR / "golden.npz")
        d = yaml.safe_load((cand / "deploy.yaml").read_text(encoding="utf-8"))
        d["validation"] = {
            "passed": True,
            "policy_sha256": sha256(MODEL_DIR / "policy.onnx"),
            "report": "logs/level2_validation/report.md",
            "at": dt.datetime.now().isoformat(timespec="seconds"),
        }
        (MODEL_DIR / "deploy.yaml").write_text(yaml.safe_dump(d, sort_keys=False), encoding="utf-8")
        print(f"promoted to {rel_path(MODEL_DIR)}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
