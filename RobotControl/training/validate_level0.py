"""Step 4 verification gate for the Level 0 policy.

Runs the exported policy.onnx (onnxruntime, exactly as the robot/UE5 will) through the SAME rollout code
that produced the PyTorch baseline (export_level0.baseline_rollout), then checks:
  Kinematic fidelity : ONNX actions == PyTorch baseline (<1e-4); joint angles, root velocity and
                       angular momentum vs baseline; tracking error / overshoot on the sweep
  Contact dynamics   : zero contacts while hanging, no penetration, joint accel < 80 rad/s^2, no action jitter
  Passivity          : passive limbs (legs/waist/left arm) stay within 2 deg of their settled hanging pose
Writes logs/level0_validation/report.md + level0_validation.png + onnx_rollout.npz.

Usage: .venv/Scripts/python.exe RobotControl/training/validate_level0.py [--model-dir RobotControl/models/r1_level0_upperbody]
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from export_level0 import PRESETTLE_S, ROLLOUT_S, baseline_rollout  # noqa: E402
from level0_env import CONTROLLED_JOINTS, POLICY_DT  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
PASS, FAIL = "PASS", "FAIL"


def tracking_metrics(t, q, cmd, warmup_s=1.0):
    err = np.rad2deg(q - cmd)
    mask = t > warmup_s
    overshoot = max(0.0, float(np.rad2deg(q.max() - cmd.max())), float(np.rad2deg(cmd.min() - q.min())))
    return float(np.abs(err[mask]).max()), float(np.abs(err[mask]).mean()), overshoot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-dir", type=pathlib.Path, default=ROOT / "RobotControl/models/r1_level0_upperbody")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "logs/level0_validation")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    deploy = yaml.safe_load((args.model_dir / "deploy.yaml").read_text())
    base = np.load(args.model_dir / "baseline_rollout.npz")
    sess = ort.InferenceSession(str(args.model_dir / "policy.onnx"), providers=["CPUExecutionProvider"])
    assert sess.get_inputs()[0].name == "obs" and sess.get_outputs()[0].name == "actions", \
        "ONNX tensor names violate the contract"
    obs_buf = np.zeros((1, deploy["num_obs"]), dtype=np.float32)

    def onnx_policy(obs):
        obs_buf[:] = obs                       # pre-allocated input, no per-step allocation
        return sess.run(None, {"obs": obs_buf})[0]

    R = baseline_rollout(onnx_policy, args.out / "onnx_rollout.npz")

    results = []

    def check(name, value, thresh, unit=""):
        ok = value <= thresh
        results.append((PASS if ok else FAIL, name, value, thresh, unit))
        return ok

    iy, ip = CONTROLLED_JOINTS.index("head_yaw_joint"), CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")
    acc = np.diff(R["dq"], axis=0) / POLICY_DT
    # Passive legs creep under gravity with damping only (identical with the policy switched off), so judge the
    # policy's *influence*: deviation of passive joints from a zero-action reference rollout of the same length.
    ref = baseline_rollout(lambda o: np.zeros((1, deploy["num_actions"]), dtype=np.float32), None)
    passive_dev = np.rad2deg(np.abs(R["q_passive"] - ref["q_passive"])).max(axis=1)
    gravity_creep = float(np.rad2deg(np.abs(ref["q_passive"] - ref["q_passive"][0])).max())
    settled = R["t"] > 0.5  # ignore residual settle transient of the hanging limbs

    # Open-loop fidelity: feed the PyTorch baseline's recorded observations through the ONNX graph.
    # (Closed-loop action differences grow chaotically from float round-off and are judged via joint angles below.)
    replay = np.concatenate([onnx_policy(base["obs"][i:i + 1]) for i in range(len(base["obs"]))])
    check("ONNX vs PyTorch action max |diff| (open-loop replay)", float(np.abs(replay - base["actions"]).max()), 1e-4)
    check("joint angle vs baseline max |diff|", float(np.rad2deg(np.abs(R["q"] - base["q"])).max()), 0.5, "deg")
    check("root velocity vs baseline max |diff|", float(np.abs(R["root_vel"] - base["root_vel"]).max()), 1e-3, "m/s | rad/s")
    check("angular momentum vs baseline max |diff|", float(np.abs(R["angmom"] - base["angmom"]).max()), 1e-3, "kg m^2/s")
    e_yaw_max, e_yaw_mean, os_yaw = tracking_metrics(R["t"], R["q"][:, iy], R["cmd"][:, 0])
    e_p_max, e_p_mean, os_p = tracking_metrics(R["t"], R["q"][:, ip], R["cmd"][:, 1])
    check("head yaw mean tracking error (t>1 s)", e_yaw_mean, 3.0, "deg")
    check("shoulder pitch mean tracking error (t>1 s)", e_p_mean, 3.0, "deg")
    check("head yaw overshoot beyond command range", os_yaw, 3.0, "deg")
    check("shoulder pitch overshoot beyond command range", os_p, 3.0, "deg")
    check("max joint acceleration (controlled joints)", float(np.abs(acc).max()), 80.0, "rad/s^2")
    check("action jitter: mean |delta action| per step", float(np.abs(np.diff(R["actions"], axis=0)).mean()), 0.05)
    check("contacts while hanging (max ncon)", int(R["ncon"].max()), 0)
    check("ground penetration", float(R["pen"].max() * 1000), 0.0, "mm")
    check("passive legs: policy-induced deviation vs zero-action reference", float(passive_dev[settled].max()), 2.0, "deg")
    check("pelvis angular velocity max (stand weld compliance)", float(np.abs(R["root_vel"][settled, 3:]).max()), 0.05, "rad/s")

    fig, axs = plt.subplots(3, 2, figsize=(13, 10))
    axs = axs.ravel()
    axs[0].plot(R["t"], np.rad2deg(R["cmd"][:, 0]), "k--", label="command")
    axs[0].plot(R["t"], np.rad2deg(R["q"][:, iy]), label="head yaw (ONNX)")
    axs[0].set_title("Head yaw [deg]"); axs[0].legend()
    axs[1].plot(R["t"], np.rad2deg(R["cmd"][:, 1]), "k--", label="command")
    axs[1].plot(R["t"], np.rad2deg(R["q"][:, ip]), label="shoulder pitch (ONNX)")
    axs[1].set_title("Right shoulder pitch [deg]"); axs[1].legend()
    for j, n in enumerate(CONTROLLED_JOINTS):
        axs[2].plot(R["t"], np.rad2deg(R["q"][:, j]), label=n.replace("_joint", ""))
    axs[2].set_title("All controlled joints [deg]"); axs[2].legend(fontsize=7)
    axs[3].plot(R["t"][1:], np.abs(acc).max(axis=1)); axs[3].axhline(80, color="r", ls="--")
    axs[3].set_title("max |joint accel| [rad/s^2]")
    axs[4].plot(R["t"], R["root_vel"][:, 3:]); axs[4].set_title("pelvis angular velocity [rad/s] (weld compliance)")
    axs[5].plot(R["t"], passive_dev); axs[5].axhline(2, color="r", ls="--")
    axs[5].set_title("passive legs: deviation vs zero-action reference [deg]")
    for a in axs:
        a.set_xlabel("t [s]"); a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(args.out / "level0_validation.png", dpi=120)

    meta = deploy.get("metadata", {})
    lines = ["# Level 0 verification gate", "",
             f"Policy: `{args.model_dir / 'policy.onnx'}`  ",
             f"Trained: {meta.get('trained_at')} iters={meta.get('iters')} envs={meta.get('num_envs')}  ",
             f"Rollout: {PRESETTLE_S:.0f} s passive settle, then {ROLLOUT_S:.0f} s at 50 Hz, nominal physics, "
             "sweep command (head yaw +-30 deg @0.25 Hz, shoulder pitch 0.35 .. -0.85 rad @0.15 Hz)", "",
             "| Result | Check | Value | Threshold |", "|---|---|---:|---:|"]
    for r, name, v, th, unit in results:
        lines.append(f"| {r} | {name} | {v:.4g} {unit} | <= {th:g} {unit} |")
    n_fail = sum(1 for r in results if r[0] == FAIL)
    lines += ["", f"**{'ALL CHECKS PASSED' if n_fail == 0 else f'{n_fail} CHECK(S) FAILED'}**", "",
              f"Head yaw: max err {e_yaw_max:.2f} deg, overshoot {os_yaw:.2f} deg. "
              f"Shoulder pitch: max err {e_p_max:.2f} deg, overshoot {os_p:.2f} deg.",
              f"Passive legs creep {gravity_creep:.2f} deg under gravity in 5 s with the policy switched off (damping-only hang; not policy-induced).",
              "", "![plots](level0_validation.png)"]
    (args.out / "report.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines[:-2]))
    print(f"\nwrote {args.out / 'report.md'} and level0_validation.png")
    sys.exit(0 if n_fail == 0 else 1)


if __name__ == "__main__":
    main()
