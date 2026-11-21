"""Export a Unitree mjlab R1 velocity checkpoint as a Level 2 CANDIDATE for PoRobotTrain.

Runs inside the trainer image (mjlab + unitree_rl_mjlab installed):

    docker run --rm --gpus all -v <repo>/logs/level2_training:/opt/unitree_rl_mjlab/logs porobot-l2:dev \
        export --checkpoint logs/rsl_rl/r1_velocity/<run>/model_<it>.pt

Writes <out> (default logs/candidate inside the container = <repo>/logs/level2_training/candidate):

  policy.onnx   input "obs" [batch, 83] -> output "actions" [batch, 24]; the actor's empirical observation
                normaliser is part of the graph (rsl_rl exports it with the MLP)
  deploy.yaml   PoRobotTrain-style deployment contract (joint order = joint-map policy_index 0..23, gains,
                default pose, action scale, observation terms in order with exact definitions, timing)
  golden.npz    obs/action pairs from an mjlab play rollout + the raw signals each obs term is built from, so
                RobotControl's own observation builder can be parity-tested against mjlab

This is only a candidate. RobotControl/training/validate_level2.py decides whether it is promoted.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import hashlib
import os
import re
from dataclasses import asdict
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
import yaml

TASK = "Unitree-R1-Flat"
SOURCE_REPO = "https://github.com/unitreerobotics/unitree_rl_mjlab"
# joint-map policy_index 0..23 (RobotControl/configs/r1_joint_map.yaml); the head (24, 25) is not in Level 2
EXPECTED_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_roll_joint", "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint",
]  # fmt: skip
GAIT_PERIOD = 0.6
# observation terms of the actor group, in order (src/tasks/velocity/velocity_env_cfg.py actor_terms, flat task
# drops height_scan); definitions verified against mjlab 1.2.0 / unitree_rl_mjlab source
OBS_TERMS = [
    {"name": "base_ang_vel", "dim": 3, "scale": 1.0,
     "definition": "gyro sensor 'imu_ang_vel' at site 'imu' (pelvis origin, identity orientation): pelvis angular "
                   "velocity in the PELVIS (body) frame, rad/s"},
    {"name": "projected_gravity", "dim": 3, "scale": 1.0,
     "definition": "unit gravity (0,0,-1) rotated into the pelvis frame: quat_apply_inverse(root_quat_w, [0,0,-1])"},
    {"name": "command", "dim": 3, "scale": 1.0,
     "definition": "velocity command [vx, vy, wz] in the base (yaw) frame, m/s, m/s, rad/s"},
    {"name": "phase", "dim": 2, "scale": 1.0,
     "definition": "p = (k * step_dt mod 0.6) / 0.6 with k = policy steps since episode reset (0 at reset, +1 per "
                   "step before the obs is computed); [sin(2 pi p), cos(2 pi p)]; BOTH set to 0 when "
                   "|command| (L2 norm of all 3 components) < 0.1"},
    {"name": "joint_pos", "dim": 24, "scale": 1.0,
     "definition": "q - default_joint_pos, joint_names order (unbiased: encoder bias does not enter the obs)"},
    {"name": "joint_vel", "dim": 24, "scale": 1.0, "definition": "dq - 0 (default joint vel is zero), rad/s"},
    {"name": "actions", "dim": 24, "scale": 1.0,
     "definition": "last raw policy output (before scale/offset), zero after reset"},
]  # fmt: skip


def _sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _latest_checkpoint(root: Path) -> Path:
    cands = glob.glob(str(root / "rsl_rl" / "r1_velocity" / "*" / "model_*.pt"))
    if not cands:
        raise SystemExit(f"no checkpoints under {root / 'rsl_rl' / 'r1_velocity'}")

    def it(p):
        return (os.path.getmtime(os.path.dirname(p)), int(re.search(r"model_(\d+)\.pt$", p).group(1)))

    return Path(max(cands, key=it))


def _fix_onnx_io(path: Path, num_obs: int, num_act: int) -> None:
    """Rename IO to the PoRobotTrain contract (obs -> actions) and make the batch dimension symbolic."""
    m = onnx.load(str(path))
    g = m.graph
    assert len(g.input) == 1 and len(g.output) == 1, (g.input, g.output)
    old_in, old_out = g.input[0].name, g.output[0].name
    for node in g.node:
        node.input[:] = ["obs" if x == old_in else x for x in node.input]
        node.output[:] = ["actions" if x == old_out else x for x in node.output]
    g.input[0].name, g.output[0].name = "obs", "actions"
    for vi, n in ((g.input[0], num_obs), (g.output[0], num_act)):
        dims = vi.type.tensor_type.shape.dim
        assert dims[-1].dim_value in (0, n), (vi.name, dims)
        dims[0].ClearField("dim_value")
        dims[0].dim_param = "batch"
    onnx.checker.check_model(m)
    onnx.save(m, str(path))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=Path, default=None, help="model_<it>.pt (default: newest)")
    ap.add_argument("--logs", type=Path, default=Path("logs"))
    ap.add_argument("--out", type=Path, default=Path("logs/candidate"))
    ap.add_argument("--task", default=TASK)
    ap.add_argument("--golden-steps", type=int, default=60)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    import mjlab.tasks  # noqa: F401  (populate the registry)
    import src.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

    ckpt = args.checkpoint or _latest_checkpoint(args.logs)
    iteration = int(re.search(r"model_(\d+)\.pt$", ckpt.name).group(1))
    print(f"[export] checkpoint {ckpt} (iteration {iteration})")

    env_cfg = load_env_cfg(args.task, play=True)
    env_cfg.scene.num_envs = 1
    # deterministic command for the golden rollout: no heading control, no resampling, no standing envs
    twist = env_cfg.commands["twist"]
    twist.heading_command = False
    twist.ranges.heading = None
    twist.resampling_time_range = (1.0e9, 1.0e9)
    twist.rel_standing_envs = 0.0
    play_ranges = {k: list(getattr(twist.ranges, k)) for k in ("lin_vel_x", "lin_vel_y", "ang_vel_z")}
    agent_cfg = load_rl_cfg(args.task)
    agent_cfg.logger = "tensorboard"
    agent_cfg.upload_model = False
    env =ManagerBasedRlEnv(cfg=env_cfg, device=args.device)
    venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner_cls = load_runner_cls(args.task)
    runner = runner_cls(venv, asdict(agent_cfg), None, args.device)
    runner.load(str(ckpt))
    policy = runner.get_inference_policy(device=args.device)

    robot = env.scene["robot"]
    names = list(robot.joint_names)
    if names != EXPECTED_JOINTS:
        raise SystemExit(f"mjlab joint order differs from joint-map policy_index 0..23:\n{names}")
    obs_names = list(env.observation_manager.active_terms["actor"])
    if obs_names != [t["name"] for t in OBS_TERMS]:
        raise SystemExit(f"actor observation terms changed: {obs_names}")
    num_obs = int(env.observation_manager.group_obs_dim["actor"][0])
    num_act = len(names)
    assert num_obs == sum(t["dim"] for t in OBS_TERMS) == 83, num_obs

    args.out.mkdir(parents=True, exist_ok=True)
    onnx_path = args.out / "policy.onnx"
    runner.export_policy_to_onnx(str(args.out), "policy.onnx")
    _fix_onnx_io(onnx_path, num_obs, num_act)
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])

    # ---- golden rollout: stand, walk forward, turn, sidestep, back, stop
    schedule = [(10, (0.0, 0.0, 0.0)), (15, (0.5, 0.0, 0.0)), (10, (0.5, 0.0, 0.5)), (10, (0.0, 0.3, 0.0)),
                (10, (-0.3, 0.0, 0.0)), (5, (0.0, 0.0, 0.0))]  # fmt: skip
    cmd_seq = np.concatenate([np.tile(c, (n, 1)) for n, c in schedule])[: args.golden_steps]
    term = env.command_manager.get_term("twist")
    gyro = env.scene["robot/imu_ang_vel"]
    action_term = env.action_manager.get_term("joint_pos")
    rec = {k: [] for k in ("obs", "actions", "actions_onnx", "ang_vel_b", "projected_gravity", "command",
                           "phase_time", "q", "dq", "last_action", "root_pos", "root_quat")}  # fmt: skip
    obs_td = venv.get_observations()
    for k in range(len(cmd_seq)):
        term.vel_command_b[:] = torch.as_tensor(cmd_seq[k], device=env.device, dtype=torch.float32)
        # recompute the obs with the new command so the recorded pair is self-consistent (compute() without
        # update_history returns the cached buffer; history_length is 1, so a fresh compute has no side effects)
        from tensordict import TensorDict

        obs_td = TensorDict(env.observation_manager.compute(update_history=True), batch_size=[env.num_envs])
        obs = obs_td["actor"] if hasattr(obs_td, "keys") else obs_td
        with torch.inference_mode():
            act = policy(obs_td)
        o = obs.detach().cpu().numpy().astype(np.float32)
        a = act.detach().cpu().numpy().astype(np.float32)
        rec["obs"].append(o[0])
        rec["actions"].append(a[0])
        rec["actions_onnx"].append(sess.run(None, {"obs": o})[0][0])
        rec["ang_vel_b"].append(gyro.data[0].cpu().numpy())
        rec["projected_gravity"].append(robot.data.projected_gravity_b[0].cpu().numpy())
        rec["command"].append(term.command[0].cpu().numpy())
        rec["phase_time"].append(float(env.episode_length_buf[0].item() * env.step_dt))
        rec["q"].append(robot.data.joint_pos[0].cpu().numpy())
        rec["dq"].append(robot.data.joint_vel[0].cpu().numpy())
        rec["last_action"].append(action_term.raw_action[0].cpu().numpy())
        rec["root_pos"].append(robot.data.root_link_pos_w[0].cpu().numpy())
        rec["root_quat"].append(robot.data.root_link_quat_w[0].cpu().numpy())
        with torch.inference_mode():
            obs_td, _, _, _ = venv.step(act)
    gold = {k: np.asarray(v, dtype=np.float64 if k == "phase_time" else np.float32) for k, v in rec.items()}
    onnx_err = float(np.abs(gold["actions_onnx"] - gold["actions"]).max())
    batch = np.repeat(gold["obs"][:4], 2, axis=0)
    batch_err = float(np.abs(sess.run(None, {"obs": batch})[0] - np.repeat(gold["actions"][:4], 2, axis=0)).max())
    print(f"[export] ONNX vs torch max |diff| {onnx_err:.2e} (batch of 8: {batch_err:.2e})")
    if max(onnx_err, batch_err) > 1e-4:
        raise SystemExit("ONNX export does not match the torch policy")
    np.savez_compressed(args.out / "golden.npz", **gold, default_joint_pos=robot.data.default_joint_pos[0].cpu().numpy(),
                        step_dt=env.step_dt, gait_period=GAIT_PERIOD)  # fmt: skip

    # ---- deploy.yaml (PoRobotTrain style)
    meta_src = {p.key: p.value for p in onnx.load(str(onnx_path)).metadata_props}
    ctrl = {}
    for act in robot.spec.actuators:
        ctrl[act.target.split("/")[-1]] = act.id
    ids = [ctrl[n] for n in names]
    kp = env.sim.mj_model.actuator_gainprm[ids, 0]
    kd = -env.sim.mj_model.actuator_biasprm[ids, 2]
    scale = action_term._scale[0].cpu().numpy() if isinstance(action_term._scale, torch.Tensor) else np.full(num_act, action_term._scale)  # fmt: skip
    default = robot.data.default_joint_pos[0].cpu().numpy()
    effort = env.sim.mj_model.actuator_forcerange[ids, 1]
    commit = os.environ.get("L2_SOURCE_COMMIT", "unknown")
    # compiled training dynamics, so the PoRobotTrain walk scene can match them (sim-to-sim)
    import mujoco

    mm = env.sim.mj_model
    jnames = [mujoco.mj_id2name(mm, mujoco.mjtObj.mjOBJ_JOINT, j) or "" for j in range(mm.njnt)]
    dofs = [int(mm.jnt_dofadr[next(j for j, s in enumerate(jnames) if s.split("/")[-1] == n)]) for n in names]
    gnames = [mujoco.mj_id2name(mm, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(mm.ngeom)]
    foot = next(g for g, s in enumerate(gnames) if s.endswith("left_foot4_collision"))
    training_dynamics = {
        "armature": [round(float(mm.dof_armature[d]), 6) for d in dofs],
        "damping": [round(float(mm.dof_damping[d]), 6) for d in dofs],
        "frictionloss": [round(float(mm.dof_frictionloss[d]), 6) for d in dofs],
        "total_mass_kg": round(float(mm.body_mass.sum()), 4),
        "foot_geom": {"condim": int(mm.geom_condim[foot]), "friction": [float(x) for x in mm.geom_friction[foot]],
                      "priority": int(mm.geom_priority[foot]), "randomised_friction": [0.3, 1.6]},  # fmt: skip
        "opt": {"timestep": float(mm.opt.timestep), "integrator": int(mm.opt.integrator), "cone": int(mm.opt.cone),
                "iterations": int(mm.opt.iterations), "ls_iterations": int(mm.opt.ls_iterations),
                "impratio": float(mm.opt.impratio)},  # fmt: skip
    }
    deploy = {
        "level": 2,
        "behaviour": "velocity walking (flat ground)",
        "num_obs": num_obs,
        "num_actions": num_act,
        "joint_names": names,
        "policy_indices": list(range(num_act)),
        "default_joint_pos": [round(float(x), 6) for x in default],
        "kp": [round(float(x), 6) for x in kp],
        "kd": [round(float(x), 6) for x in kd],
        "effort_limit": [round(float(x), 6) for x in effort],
        "action_scale": [round(float(x), 6) for x in scale],
        "action": "target_q = default_joint_pos + action_scale * actions (no clip); PD: tau = kp*(target_q - q) - kd*dq, "
        "clamped to effort_limit",
        "held_joints": {"head_pitch_joint": 0.0, "head_yaw_joint": 0.0},
        "step_dt": float(env.step_dt),
        "physics_dt": float(env.physics_dt),
        "decimation": int(env.cfg.decimation),
        "gait_period": GAIT_PERIOD,
        "obs_terms": OBS_TERMS,
        "commands": {"lin_vel_x": play_ranges["lin_vel_x"], "lin_vel_y": play_ranges["lin_vel_y"],
                     "ang_vel_z": play_ranges["ang_vel_z"], "standing_threshold": 0.1},  # fmt: skip
        "termination": {"bad_orientation_deg": 70.0},
        "training_dynamics": training_dynamics,
        "metadata": {
            "source_repo": SOURCE_REPO,
            "source_commit": commit,
            "task": args.task,
            "training_mjcf": "src/assets/robots/unitree_r1/xmls/r1.xml (unitree_rl_mjlab)",
            "checkpoint": str(ckpt),
            "iteration": iteration,
            "trained_at": dt.datetime.fromtimestamp(ckpt.stat().st_mtime, dt.timezone.utc).isoformat(),
            "exported_at": dt.datetime.now(dt.timezone.utc).isoformat(),
            "policy_sha256": _sha256(onnx_path),
            "onnx_vs_torch_max_abs": onnx_err,
            "mjlab_onnx_metadata": meta_src,
        },
    }
    (args.out / "deploy.yaml").write_text(yaml.safe_dump(deploy, sort_keys=False, width=120), encoding="utf-8")
    print(f"[export] wrote {args.out}/policy.onnx, deploy.yaml, golden.npz ({len(cmd_seq)} steps)")
    env.close()


if __name__ == "__main__":
    main()
