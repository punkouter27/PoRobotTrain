"""Validation gate for a head skill. A policy runs (twin or robot) only after this passed for its exact ONNX file.

    .venv/Scripts/python.exe -m robotcontrol.training.validate_head_skill look_at

Checks, all with the exported ONNX file (not the PyTorch weights):
  1. export parity   ONNX vs PyTorch actions on random observations (max |diff| < 1e-4)
  2. nominal         training env, no randomisation: tracking error against the skill's reference motion
  3. randomised      the same rollout on 8 domain-randomised envs (gains, mass, friction, damping, delay)
  4. runtime path    the runtime behaviour class (the code that runs on the Jetson) through the runner on the
                     separate realistic MuJoCo twin (SimBackend: control delay, low-pass, BEMF, sensor noise,
                     compliant stand, head-only power), including the runner's other-joint check
Writes validation.json and the ``validation`` block of deploy.yaml (passed, policy_sha256, metrics, failures).
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import io
import json
import sys
from pathlib import Path

import numpy as np
import yaml

from .. import LOGS_DIR, PROJECT_ROOT
from ..behaviours import head_skills as hs

# mean tracking error bars (deg): nominal, randomised, runtime path
BARS = {
    "hold_centre": (1.0, 1.5, 1.5),
    "look_at": (2.0, 3.0, 3.0),
    "nod_yes": (3.0, 4.0, 4.0),
    "shake_no": (3.0, 4.0, 4.0),
    "glance_around": (4.0, 5.0, 5.0),
}
POSE_BARS = (1.5, 2.0, 2.5)  # taught poses (pose_<id>): a held angle, like Hold Centre but off centre
MAX_OTHER_DEG = 5.0
EVAL_AMP_DEG = {"hold_centre": 0.0, "look_at": 30.0}


def bars(skill_id: str) -> tuple:
    return BARS.get(skill_id, POSE_BARS)


def _duration(sk: hs.HeadSkill) -> float:
    if sk.id == "hold_centre" or sk.pose is not None:
        return 4.0
    if sk.id == "look_at":
        return 15.0
    return max(2 * sk.period_s, sk.period_s)


def _score_mask(sk: hs.HeadSkill, t: np.ndarray) -> np.ndarray:
    """Which samples count: hold after 1.5 s; look_at the last 0.5 s of each 2.5 s target; periodic after 0.5 s."""
    if sk.id == "hold_centre" or sk.pose is not None:
        return t >= 1.5
    if sk.id == "look_at":
        return np.mod(t, 2.5) >= 2.0
    return t >= 0.5


def _env_rollout(skill_id: str, policy, num_envs: int, randomised: bool, seed: int) -> dict:
    from .head_skill_env import HeadSkillConfig, HeadSkillEnv

    sk = hs.SKILLS[skill_id]
    env = HeadSkillEnv(HeadSkillConfig(skill=skill_id, num_envs=num_envs, seed=seed))
    try:
        env.set_eval(not randomised)  # randomised: physics, sensor noise, delay and start pose all randomised
        rng = np.random.default_rng(seed)
        off = np.deg2rad(rng.uniform(-10, 10, env.k)) if sk.id == "hold_centre" and not randomised else None
        env.reset(np.arange(num_envs), start_offset=off)
        env.t_skill[:] = 0.0  # deterministic command schedule from t = 0
        if sk.period_s > 0:
            env.set_size(1.0)
            env._update_periodic_cmd()
        q_start = env.state[:, env.q_all_cols].copy()
        T = int(_duration(sk) / hs.POLICY_DT)
        errs, ts, other, frc = [], [], 0.0, 0.0
        for k in range(T):
            t = k * hs.POLICY_DT
            if sk.id == "look_at":
                y, p = hs.look_at_script(t, EVAL_AMP_DEG["look_at"])
                env.set_command(np.tile([y, p, 0.0], (num_envs, 1)))
            obs = env.get_obs()
            act = policy(obs)
            env.cmd_timer[:] = 1e9
            env.ep_step[:] = 0  # never time out inside the validation rollout
            _, _, _, _ = env.step(act)
            ref = env.reference() if sk.period_s > 0 else hs.reference(sk, env.cmd)
            errs.append(np.rad2deg(np.abs(env.q - ref)).mean(axis=1))
            ts.append(t + hs.POLICY_DT)
            other = max(other, float(np.rad2deg(env.other_joint_drift(q_start)).max()))
            frc = max(frc, float(np.abs(env.frc).max()))
        errs, ts = np.array(errs), np.array(ts)
        m = _score_mask(sk, ts)
        return {
            "mean_err_deg": round(float(errs[m].mean()), 3),
            "max_err_deg": round(float(errs[m].max()), 3),
            "other_joint_max_deg": round(other, 3),
            "max_torque_nm": round(frc, 3),
        }
    finally:
        env.close()


def _runtime_rollout(skill_id: str, onnx_path: Path, seed: int = 0) -> dict:
    """The Jetson code path (TrainedHeadBehaviour + runner.run) on the realistic SimBackend twin."""
    from ..behaviours.backends import SimBackend
    from ..behaviours.neck_turn import SweepParams
    from ..behaviours.runner import CsvLog, run
    from ..configs.joint_map import JointMap

    sk = hs.SKILLS[skill_id]
    jm = JointMap.load()
    pol = hs.OnnxPolicy(onnx_path, hs.obs_size(sk))
    amp = 0.0 if sk.pose is not None else EVAL_AMP_DEG.get(skill_id, sk.max_deg)
    beh = hs.SKILL_CLASSES[skill_id](
        jm, SweepParams(amplitude_deg=max(amp, 1.0), duration_s=_duration(sk)), check_tracking=True, policy=pol
    )
    rec = []
    inner = beh.targets

    def targets(t, q, dq, age):
        out = inner(t, q, dq, age)
        if t < beh.run_s:
            ref = beh.reference_at(t)
            rec.append((t, float(np.rad2deg(np.abs(np.array([q[j] for j in sk.powered]) - ref)).mean())))
        return out

    beh.targets = targets
    buf = io.StringIO()
    log_path = LOGS_DIR / "head_skills" / f"validate_{skill_id}.csv"
    with contextlib.redirect_stdout(buf):  # keep runner lines out of the service log (it parses @head lines)
        backend = SimBackend(jm, beh.powered, realistic=True, seed=seed)
        try:
            code = run(backend, beh, jm, CsvLog(log_path, beh.powered, jm.names), {"flag": False})
        finally:
            backend.close()
    lines = buf.getvalue().splitlines()
    abort = next((ln for ln in lines if ln.startswith(("ABORT", "NOT STARTED"))), "")
    t = np.array([r[0] for r in rec])
    e = np.array([r[1] for r in rec])
    m = _score_mask(sk, t) if len(t) else np.zeros(0, bool)
    return {
        "exit_code": int(code),
        "abort": abort,
        "mean_err_deg": round(float(e[m].mean()), 3) if m.any() else None,
        "max_err_deg": round(float(e[m].max()), 3) if m.any() else None,
        "log": log_path.relative_to(
            PROJECT_ROOT
        ).as_posix(),  # repo-relative: no local absolute paths in committed files
    }


def _parity(folder: Path, sk: hs.HeadSkill) -> float:
    import torch

    from .ppo import ActorCritic

    n_obs = hs.obs_size(sk)
    ac = ActorCritic(n_obs, len(sk.powered), hidden=(128, 64))
    ac.load_state_dict(torch.load(folder / "model.pt", map_location="cpu"))
    ac.eval()
    pol = hs.OnnxPolicy(folder / "policy.onnx", n_obs)
    x = np.random.default_rng(0).normal(size=(256, n_obs)).astype(np.float32)
    with torch.no_grad():
        ref = ac.act_inference(torch.from_numpy(x)).numpy()
    got = pol.sess.run(None, {pol.inp: x})[0]
    return float(np.abs(ref - got).max())


def validate(skill_id: str, folder: Path) -> dict:
    sk = hs.SKILLS[skill_id]
    onnx_path = folder / "policy.onnx"
    pol = hs.OnnxPolicy(onnx_path, hs.obs_size(sk))

    def batch_policy(obs):
        return pol.sess.run(None, {pol.inp: obs.astype(np.float32)})[0]

    bar_nom, bar_dr, bar_rt = bars(skill_id)
    failures = []
    parity = _parity(folder, sk)
    if not parity < 1e-4:
        failures.append(f"ONNX/PyTorch mismatch {parity:.2e}")
    nominal = _env_rollout(skill_id, batch_policy, 1, randomised=False, seed=123)
    randomised = _env_rollout(skill_id, batch_policy, 8, randomised=True, seed=7)
    runtime = _runtime_rollout(skill_id, onnx_path)
    for name, res, bar in (("nominal", nominal, bar_nom), ("randomised", randomised, bar_dr)):
        if not res["mean_err_deg"] < bar:
            failures.append(f"{name} mean error {res['mean_err_deg']:.2f} deg >= {bar} deg")
        if not res["other_joint_max_deg"] < MAX_OTHER_DEG:
            failures.append(f"{name}: another joint moved {res['other_joint_max_deg']:.1f} deg")
    if runtime["exit_code"] != 0:
        failures.append(f"runtime twin run stopped: {runtime['abort'] or 'exit ' + str(runtime['exit_code'])}")
    elif runtime["mean_err_deg"] is None or not runtime["mean_err_deg"] < bar_rt:
        failures.append(f"runtime twin mean error {runtime['mean_err_deg']} deg >= {bar_rt} deg")
    report = {
        "skill": skill_id,
        "passed": not failures,
        "failures": failures,
        "policy_sha256": hs.sha256_file(onnx_path),
        "validated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "bars_deg": {"nominal": bar_nom, "randomised": bar_dr, "runtime": bar_rt, "other_joints": MAX_OTHER_DEG},
        "metrics": {
            "onnx_parity": parity,
            "nominal": nominal,
            "randomised": randomised,
            "runtime": runtime,
            # headline number the clients show
            "track_err_deg": runtime["mean_err_deg"],
        },
    }
    (folder / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    dep_path = folder / "deploy.yaml"
    dep = yaml.safe_load(dep_path.read_text(encoding="utf-8")) or {}
    dep["validation"] = {k: report[k] for k in ("passed", "failures", "policy_sha256", "validated_at", "metrics")}
    dep_path.write_text(yaml.safe_dump(dep, sort_keys=False), encoding="utf-8")
    write_engine_json(skill_id, folder)
    return report


def write_engine_json(skill_id: str, folder: Path) -> Path:
    """deploy.json for the Unreal scenario lab (C++ reads JSON, not YAML): deploy.yaml incl. its validation block, plus
    what the engine needs to run the policy exactly like the runtime does (effort and soft limits for the torque cap,
    the joints to hold at their defaults). Unreal loads the skill only when validation.passed and the sha256 match."""
    from ..configs.joint_map import JointMap

    sk = hs.SKILLS[skill_id]
    jm = JointMap.load()
    dep = yaml.safe_load((folder / "deploy.yaml").read_text(encoding="utf-8")) or {}
    idx = [jm.index_of(j) for j in sk.powered]
    mid = 0.5 * (jm.limits_lo + jm.limits_hi)
    half = 0.5 * (jm.limits_hi - jm.limits_lo) * hs.SOFT_LIMIT_FACTOR
    legs = set(jm.groups.get("left_leg", []) + jm.groups.get("right_leg", []))
    hold = [i for i in range(jm.num_joints) if i not in legs and i not in idx]
    dep.update(
        {
            "desc": sk.desc,
            "effort": [float(jm.effort[i]) for i in idx],
            "soft_limit_lo": [float((mid - half)[i]) for i in idx],
            "soft_limit_hi": [float((mid + half)[i]) for i in idx],
            "hold_joints": [jm.names[i] for i in hold],
            "hold_default": [float(jm.defaults[i]) for i in hold],
            "hold_kp": [float(jm.kp[i]) for i in hold],
            "hold_kd": [float(jm.kd[i]) for i in hold],
            "glance_keys": [list(k) for k in hs.GLANCE_KEYS],
            "glance_move": hs.GLANCE_MOVE,
            "yaw_range": sk.yaw_range,
            "pitch_range": sk.pitch_range,
            # taught pose: the fixed head reference (rad), null for the other skills
            "pose": None if sk.pose is None else {"yaw": sk.pose[0], "pitch": sk.pose[1]},
        }
    )
    out = folder / "deploy.json"
    out.write_text(json.dumps(dep, indent=1), encoding="utf-8")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Validate a trained head skill (writes deploy.yaml validation)")
    ap.add_argument("skill", choices=sorted(hs.SKILLS))
    ap.add_argument("--dir", type=Path, default=None, help="policy folder (default: models/head_skills/<skill>)")
    ap.add_argument("--engine-json-only", action="store_true", help="only (re)write deploy.json for Unreal")
    args = ap.parse_args(argv)
    from .train_head_skill import out_dir

    folder = args.dir or out_dir(args.skill)
    if not (folder / "policy.onnx").exists():
        print(f"no policy in {folder}; train it first")
        return 1
    if args.engine_json_only:
        print(write_engine_json(args.skill, folder))
        return 0
    rep = validate(args.skill, folder)
    m = rep["metrics"]
    print(
        f"{args.skill}: nominal {m['nominal']['mean_err_deg']} deg, randomised {m['randomised']['mean_err_deg']} deg, "
        f"runtime twin {m['runtime']['mean_err_deg']} deg -> {'PASSED' if rep['passed'] else 'FAILED'}"
    )
    for f in rep["failures"]:
        print(f"  - {f}")
    return 0 if rep["passed"] else 4


if __name__ == "__main__":
    sys.exit(main())
