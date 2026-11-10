"""Export a trained Level 0 checkpoint to policy.onnx + deploy.yaml + baseline_rollout.npz.

Called by train_level0.py at the end of training, or standalone:
  .venv/Scripts/python.exe RobotControl/training/export_level0.py --checkpoint RobotControl/models/r1_level0_upperbody/model.pt
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import pathlib
import subprocess
import sys

import mujoco
import numpy as np
import onnx
import torch
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from level0_env import CONTROLLED_JOINTS, POLICY_DT, Level0Config, Level0StandEnv  # noqa: E402
from ppo import ActorCritic  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "RobotControl/models/r1_level0_upperbody"
PRESETTLE_S = 2.0
ROLLOUT_S = 5.0


class ExportWrapper(torch.nn.Module):
    """Normaliser + actor mean in one graph: input 'obs' (B,N) -> output 'actions' (B,A)."""

    def __init__(self, ac: ActorCritic):
        super().__init__()
        self.ac = ac

    def forward(self, obs):
        return self.ac.act_inference(obs)


def sweep_command(t: np.ndarray, default_shoulder_pitch: float) -> np.ndarray:
    """Deterministic validation command: head yaw +-30 deg @0.25 Hz, shoulder pitch default..default-1.2 @0.15 Hz."""
    yaw = np.deg2rad(30.0) * np.sin(2 * np.pi * 0.25 * t)
    pitch = default_shoulder_pitch - 0.6 * (1 - np.cos(2 * np.pi * 0.15 * t))
    return np.stack([yaw, pitch], axis=-1)


def run_meta(iters, num_envs, horizon, seed, device):
    try:
        git = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, text=True,
                                      stderr=subprocess.DEVNULL).strip()
    except Exception:
        git = "nogit"
    return {"trained_at": dt.datetime.now().isoformat(timespec="seconds"), "iters": iters, "num_envs": num_envs,
            "horizon": horizon, "seed": seed, "git": git, "device": str(device), "env": "Level0StandEnv",
            "joint_map_sha256": hashlib.sha256((ROOT / "RobotControl/configs/r1_joint_map.yaml").read_bytes()).hexdigest()[:16],
            "presettle_s": PRESETTLE_S}


def make_eval_env() -> Level0StandEnv:
    ev = Level0StandEnv(Level0Config(num_envs=1, seed=123))
    ev.set_eval(True)
    ev.reset(np.array([0]))
    ev.presettle(PRESETTLE_S)
    return ev


def baseline_rollout(policy_fn, out_path: pathlib.Path):
    """5 s deterministic sweep in nominal physics after a 2 s passive settle. policy_fn: obs(1,N)->actions(1,A)."""
    ev = make_eval_env()
    T = int(ROLLOUT_S / POLICY_DT)
    sp_default = ev.default[CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")]
    m0, d0 = ev.models[0], ev.datas[0]
    angmom_adr = m0.sensor_adr[m0.sensor("root_angmom").id]
    passive_cols = ev.qpos_off + 7 + np.array(ev.passive_idx)
    log = {k: [] for k in ("t", "obs", "actions", "q", "dq", "target", "cmd", "frc", "root_vel", "angmom", "ncon", "pen", "q_passive")}
    for k in range(T):
        t = k * POLICY_DT
        ev.set_command(sweep_command(np.array([t]), sp_default))
        obs = ev.get_obs()
        a = np.asarray(policy_fn(obs), dtype=np.float32)
        log["t"].append(t); log["obs"].append(obs[0]); log["actions"].append(a[0])
        log["q"].append(ev.q[0].copy()); log["dq"].append(ev.dq[0].copy())
        log["target"].append(ev.targets_from_action(a[0])); log["cmd"].append(ev.cmd[0].copy())
        ev.step(a)
        log["frc"].append(ev.frc[0].copy())
        log["root_vel"].append(ev.state[0, ev.qvel_off:ev.qvel_off + 6].copy())
        mujoco.mj_setState(m0, d0, ev.state[0], ev.spec)
        mujoco.mj_forward(m0, d0)
        log["angmom"].append(d0.sensordata[angmom_adr:angmom_adr + 3].copy())
        log["ncon"].append(d0.ncon)
        log["pen"].append(float(-d0.contact.dist[:d0.ncon].min()) if d0.ncon else 0.0)
        log["q_passive"].append(ev.state[0, passive_cols].copy())
    ev.close()
    arrays = {k: np.array(v) for k, v in log.items()}
    if out_path is not None:
        np.savez(out_path, **arrays, joint_names=np.array(CONTROLLED_JOINTS),
                 passive_joint_names=np.array([ev.jm["joints"][i]["name"] for i in ev.passive_idx]))
    return arrays


def export(ac: ActorCritic, env: Level0StandEnv, out: pathlib.Path, meta: dict):
    out.mkdir(parents=True, exist_ok=True)
    ac = ac.cpu().eval()
    torch.save(ac.state_dict(), out / "model.pt")
    wrapper = ExportWrapper(ac)
    dummy = torch.zeros(1, env.num_obs)
    onnx_path = out / "policy.onnx"
    torch.onnx.export(wrapper, dummy, str(onnx_path), input_names=["obs"], output_names=["actions"],
                      dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}}, opset_version=17,
                      dynamo=False)
    model = onnx.load(str(onnx_path))
    for k, v in meta.items():
        p = model.metadata_props.add()
        p.key, p.value = k, str(v)
    onnx.save(model, str(onnx_path))

    jm = env.jm
    joints = [jm["joints"][i] for i in env.ctrl_idx]
    passive = [jm["joints"][i]["name"] for i in env.passive_idx]
    deploy = {
        "policy_name": "r1_level0_upperbody",
        "step_dt": POLICY_DT,
        "num_obs": env.num_obs, "num_actions": env.num_actions,
        "joint_names": [j["name"] for j in joints],
        "motor_slots": [j["motor_slot"] for j in joints],
        "default_joint_pos": [float(j["default"]) for j in joints],
        "kp": [float(j["kp"]) for j in joints], "kd": [float(j["kd"]) for j in joints],
        "action_scale": [float(s) for s in env.action_scale],
        "action_clip": [5.0] * env.num_actions,
        "soft_limit_factor": 0.9,
        "observations": [{"name": "joint_pos_rel", "size": env.num_actions},
                         {"name": "joint_vel", "size": env.num_actions},
                         {"name": "last_action", "size": env.num_actions},
                         {"name": "command", "size": 2}],
        "commands": {"head_yaw_target": 1, "right_shoulder_pitch_target": 1},
        "command_ranges": {"head_yaw_target": [-1.0472, 1.0472],
                           "right_shoulder_pitch_target": [float(joints[0]["default"]) - 1.6, float(joints[0]["default"])]},
        "passive_joints": passive,
        "passive_kd": 3.0,
        "held_joints": [jm["joints"][i]["name"] for i in env.held_idx],
        "presettle_s": PRESETTLE_S,
        "metadata": meta,
    }
    (out / "deploy.yaml").write_text(yaml.safe_dump(deploy, sort_keys=False))

    with torch.no_grad():
        baseline_rollout(lambda o: wrapper(torch.from_numpy(o)).numpy(), out / "baseline_rollout.npz")
    print(f"exported {onnx_path} ({onnx_path.stat().st_size/1e3:.1f} kB), deploy.yaml, baseline_rollout.npz")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path, default=OUT)
    ap.add_argument("--iters", type=int, default=-1)
    args = ap.parse_args()
    env = Level0StandEnv(Level0Config(num_envs=1, seed=0))
    ac = ActorCritic(env.num_obs, env.num_actions)
    ac.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    export(ac, env, args.out, run_meta(args.iters, "n/a", "n/a", "n/a", "cpu-export"))
    env.close()


if __name__ == "__main__":
    main()
