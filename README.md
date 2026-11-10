# PoRobotTrain: Unitree R1 digital twin and sim-to-real pipeline

Train behaviours for a Unitree R1 EDU (A5 arms, 26 motors) in MuJoCo, rehearse them in an Unreal Engine 5 twin
(URLab plugin, embedded MuJoCo), then run the **same ONNX file and the same controller code** on the real robot
through Unitree SDK2. Start trivially safe (robot on its stand, head and one arm only) and expand level by level.

```
RobotControl/configs/r1_joint_map.yaml    <- single source of truth: joint order, motor slots, limits, gains
          |                                   (read by every tool below)
          v
scripts/build_mjcf.py  -> UnrealProject/MJCF/{r1_twin, r1_twin_stand, scene_*}.xml
          |
          v
RobotControl/training/train_level0.py  -> RobotControl/models/r1_level0_upperbody/{policy.onnx, deploy.yaml, baseline_rollout.npz}
          |
          v
RobotControl/training/validate_level0.py  (verification gate: fidelity + contact + passivity)   -> logs/level0_validation/report.md
          |
          v
robotcontrol.main_runner --target sim | urlab | real     (identical code; only the transport changes)
```

## Levels

| Level | Behaviour | Policy scope | Robot state |
|---|---|---|---|
| 0 | Head yaw sweep, right arm raise/lower | 7 joints, 23-dim obs | hanging on the stand, legs passive, waist + left arm held at default |
| 1 | Voice-triggered poses with spoken confirmation | arms, waist, head | standing (stand controller) |
| 2 | Omnidirectional walking (Unitree mjlab velocity policy) | 24 joints, 83-dim obs | ground |
| 3 | Camera-driven following | Level 2 + perception commands | ground |
| 4 | Ball search, approach, align, kick, recover | walk + kick policies with alpha-blend | ground |

## Quick start (Windows, this machine)

```powershell
# 1. Python env (already created): .venv with Python 3.11, mujoco 4.0, torch 2.11+cu128, onnxruntime
# 2. Build the MJCF twin from Unitree's official model + joint map, and validate it
.venv\Scripts\python.exe scripts\build_mjcf.py
.venv\Scripts\python.exe scripts\validate_mjcf.py

# 3. Train Level 0 (about 20 min on the RTX 5070 Ti; 512 envs, 1500 PPO iterations)
.venv\Scripts\python.exe -u RobotControl\training\train_level0.py --iters 1500 --num-envs 512

# 4. Verification gate (must print ALL CHECKS PASSED before anything goes to the robot)
.venv\Scripts\python.exe RobotControl\training\validate_level0.py

# 5. Inspect / validate / pack the ONNX for deployment
.venv\Scripts\python.exe -m robotcontrol.tools.onnx_inspect  RobotControl\models\r1_level0_upperbody\policy.onnx
.venv\Scripts\python.exe -m robotcontrol.tools.onnx_validate RobotControl\models\r1_level0_upperbody\policy.onnx RobotControl\models\r1_level0_upperbody\deploy.yaml
.venv\Scripts\python.exe -m robotcontrol.tools.onnx_replay   RobotControl\models\r1_level0_upperbody\policy.onnx RobotControl\models\r1_level0_upperbody\baseline_rollout.npz
.venv\Scripts\python.exe -m robotcontrol.tools.deploy_pack   RobotControl\models\r1_level0_upperbody

# 6. Run the policy against the headless MuJoCo twin, then UE5 (URLab), then the robot
.venv\Scripts\python.exe -m robotcontrol.main_runner --target sim   --policy RobotControl\models\r1_level0_upperbody\policy.onnx --scene stand --command-mode sweep
.venv\Scripts\python.exe -m robotcontrol.main_runner --target urlab --policy ... --command-mode keyboard
.venv\Scripts\python.exe -m robotcontrol.main_runner --target real  --policy ... --network <ethernet-interface> --command-mode keyboard
```

See `docs/ue5_setup.md` for the Unreal 5.7 + URLab install and `docs/sim_to_real_level0.md` for the robot procedure.

## ONNX policy contract

* Input `obs` float32 `(1, N)`, output `actions` float32 `(1, A)`; observation normalisation is baked into the graph.
* `deploy.yaml` beside the model lists joint names (action order), motor slots, default joint positions, kp, kd,
  per-joint action scale, observation term order, commands and passive joints.
* Target joint position `q* = default + action_scale * action`, clipped to 0.9 of the joint range.
* Level 0 observation: `joint_pos_rel(7), joint_vel(7), last_action(7), command(2)`; command = `[head_yaw_target, right_shoulder_pitch_target]` in rad.

## Coordinate convention (MuJoCo -> UE5)

`pos_ue = (x, -y, z) * 100` (metres to centimetres, flip Y). Quaternion MuJoCo `(w, x, y, z)` -> UE `FQuat(X=x, Y=-y, Z=z, W=-w)`.
Bones are driven from MuJoCo body world poses every render frame, never from joint angles.

## Repository layout

```
RobotControl/           Python package `robotcontrol` (interfaces, execution, tools, training, configs)
UnrealProject/          UE 5.7 C++ project (R1Twin), MJCF models + meshes under MJCF
scripts/                MJCF generator and validator
docs/                   setup and procedures
external/               reference clones (unitree_mujoco, unitree_sdk2_python, unitree_rl_mjlab, urlab_bridge) - not committed
logs/                   training logs, validation reports - not committed
```

## Sources

Unitree R1 model: `unitreerobotics/unitree_mujoco` and `unitree_ros` (head joints, camera). Gains and training
conventions: `unitreerobotics/unitree_rl_mjlab`. Real-robot interface: `unitreerobotics/unitree_sdk2_python`
(`example/r1/low_level`). UE5 physics: `URLab-Sim/UnrealRoboticsLab` and `urlab_bridge`.

## Running the Unreal twin (verified 2026-11-09 on UE 5.8.4 + URLab with MuJoCo 4.0.0)

One-time level build (editor open, Play stopped):

```powershell
& "C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe" UnrealProject\R1Twin.uproject
.venv\Scripts\python.exe scripts\urlab_setup_scene.py --force               # level R1_Stand + MJCF import + URLab manager
.venv\Scripts\python.exe scripts\ue_remote.py scripts\ue_finalize_level.py   # GameMode, sky, exposure, floor, PlayerStart, Save All
.venv\Scripts\python.exe scripts\ue_remote.py scripts\ue_import_policy.py    # policy.onnx -> /Game/Humanoid/Policies (NNE asset)
```

Two ways to drive the robot, sharing the same ONNX, gains and joint map:

| Mode | How | Who steps physics |
|---|---|---|
| In-engine | Press **Play**. `R1TwinGameMode` loads the NNE model from `Config/DefaultGame.ini`, resets to `home`, unpauses physics and runs the Level 0 sweep at 50 Hz. | Unreal (URLab live mode) |
| Python | `python -m robotcontrol.main_runner --target urlab --policy RobotControl\models\r1_level0_upperbody\policy.onnx --command-mode sweep` | Python (URLab direct mode) |

Handoff is automatic: when a Python client switches URLab to direct mode the in-engine policy releases the actuators,
and it reclaims them when the client finishes. The follow camera frames the stand; no eject needed.

* MJCF sources live in `UnrealProject/MJCF/`, outside `Content/`, so Unreal never auto-imports URLab's generated `*_ue.xml`.
* The runner pushes the training physics options (5 ms step, implicitfast), resets to `home`, and auto-resets on a
  safety trip or workspace exit in simulation (never on the real robot).
* Parity: a 20 s sweep in Unreal vs. the headless MuJoCo twin differs by at most 0.37 deg on any of the 26 joints.
* URLab is pinned locally to MuJoCo 4.0.0 (branch `mujoco-4.0` in `UnrealProject/Plugins/UnrealRoboticsLab`) with small
  compile fixes for MuJoCo 4.0 and UE 5.8. Runtime numbers and known issues: `docs/step5_runtime.md`.

## Real robot (Level 0 on the stand)

Step-by-step: `docs/robot_connection_guide.md`. In short: `scripts\robot_network_setup.ps1` (laptop on 192.168.123.x),
`scripts\push_to_robot.ps1` (builds the offline robot kit and installs it on the R1's Jetson), then on the robot
`./run.sh robotcontrol.tools.robot_preflight` (read-only), `--observe` shadow run, and the first powered run with
`--amplitude 0.3`. The policy runs on the Jetson; the laptop only builds, copies and logs in.
