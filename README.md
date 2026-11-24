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
| 2 | Omnidirectional walking (Unitree mjlab velocity policy) | 24 joints, 83-dim obs | ground; for now only in the simulated **walk sandbox** (`docs/level2_walk.md`) |
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
UnrealProject/          UE 5.8 C++ project (R1Twin), MJCF models + meshes under MJCF, URLab recreated in Plugins/ by scripts/setup_urlab.ps1
scripts/                MJCF generator and validator, Unreal editor helpers, robot setup
web/                    static showcase site + /diag (web/build.py), deployed to swa-porobottrain
src/                    Blazor WASM control UI + .NET API; browser 3D twin assets in Client/wwwroot/twin (docs/web_twin.md)
docs/                   setup and procedures
external/               reference clones (unitree_mujoco, unitree_sdk2_python, unitree_rl_mjlab, urlab_bridge) - not committed
logs/                   training logs, validation reports - not committed
SCREENSHOTS/            before/after reports for UI changes (<feature>_report.html), made by scripts/screenshots.ps1
```

UI changes get a before/after report in [`SCREENSHOTS/`](SCREENSHOTS/):
`powershell -ExecutionPolicy Bypass -File scripts/screenshots.ps1 -Feature <name> [-Before <git ref> | -BeforeDir <PNGs>]`
shoots the local Release publish at 1366x768 and 390x844 (never the live site) and writes
`SCREENSHOTS/<feature>_report.html` with one side-by-side `<feature>_sidebyside.png` (AGENTS.md part 3).

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
.venv\Scripts\python.exe scripts\ue_remote.py scripts\ue_import_policy.py    # policy.onnx -> /Game/Project/RL_Policies (NNE asset)
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
* URLab lives in `UnrealProject/Plugins/UnrealRoboticsLab/` but is not tracked (its own clone, ~2 GB built):
  `powershell -ExecutionPolicy Bypass -File scripts\setup_urlab.ps1` clones the pinned upstream commit, moves MuJoCo to
  4.0.0, applies this repo's compile fixes (`scripts/patches/urlab_mujoco4_ue58.patch`) and builds the native
  dependencies; then build `R1TwinEditor`. Runtime numbers and known issues: `docs/step5_runtime.md`.
  The fixes cover `mjtLimited` enum
  assignments, float to bool for `active` / `actgravcomp`, and `FString(*Key)` for UE 5.8's `FSharedString` JSON keys
  in `RpcDispatcher.cpp` / `MsgpackHelpers.cpp`. Build with `-MaxParallelActions=6` if MSVC reports PCH heap errors.

## Browser twin (Blazor control UI)

The control UI shows a live three.js / WebGPU twin with limit, error and temperature tints, a Rapier check of
slider-preview poses, reach clouds and browser-side policy parity on `/diag`. Build its assets with
`scriptsuild_web_twin.py --compress`; details and the library-by-library map are in `docs/web_twin.md`.

## Voice, robustness explorer, walk sandbox

Three features available in both the Blazor control UI and the Unreal twin panel, served by the behaviour service
(`python -m robotcontrol.behaviours.service`; API contract `docs/service_api.md`):

- **Voice commands** (`docs/voice.md`): speak a request and the panel answers out loud. Voice only *suggests* a
  selection; the gates are unchanged.
- **Robustness explorer** (`docs/robustness.md`): `python -m robotcontrol.tools.robustness_sweep [--quick]` runs the
  deployed Level 0 policy on 91 perturbed twins (mass, friction, gains, latency, noise, torque limit).
- **Walk sandbox** (`docs/level2_walk.md`): drive the R1 around a course with WASD, a gamepad or your voice. It runs
  the Level 2 velocity policy in headless MuJoCo, and Unreal shows it in puppet mode. Simulation only: the robot
  kit excludes it, and a policy loads only after `python -m robotcontrol.training.validate_level2 --promote` passed.

## Operator console

The Blazor UI opens on a short "What do you want to do?" chooser (try a motion on the twin, check the real robot, run on
the real robot, walk). Running on the robot shows its four gates as a progress bar; expert tools stay behind the
**Advanced** switch, and a header badge always says TWIN ONLY or REAL ROBOT CONNECTED.

The behaviour service also provides 20 features modelled on the Unitree Explore app. The Blazor UI shows them on
the **Motions**, **Health** and **Setup** pages, behind a **? Help** drawer and an **E-STOP** button (`Esc`) on
every page. The Unreal app does not duplicate them: it is the simulation-only scenario lab for testing trained
brains (`docs/scenario_lab.md`).
The contract is in `docs/service_api.md` sections 4-7 and the code is in `RobotControl/behaviours/console.py`.

| Area | Features |
|---|---|
| Monitoring (read-only) | Health summary, alarm and fault log (CSV export), battery estimate, motor temperature trends, head camera (twin view), versions and policy gates, IMU check |
| Control (simulation only) | Modes (damp / hold / zero torque), named poses, record and replay (teach mode), timeline editor, local motion store (checksummed import/export), session limits that can only tighten, touch thumbsticks for the walk sandbox |
| Setup | Pre-flight checklist (an extra gate before any real-robot run), joint zero-offset wizard (stored, never applied), network and connection profiles |
| Help | `docs/*.md` per page, and a local chat assistant with memory that answers status questions and stays suggest-only |

Poses, motions and modes power several joints at once, so they run only on the simulated stand twin (practice,
headless or mirrored onto Unreal); the service refuses target `real`. The operator's motions, profiles and offsets
live in the git-ignored `RobotControl/user_data/`.

## Real robot (Level 0 on the stand)

Step-by-step: `docs/robot_connection_guide.md`. In short: `scripts\robot_network_setup.ps1` (laptop on 192.168.123.x),
`scripts\push_to_robot.ps1` (builds the offline robot kit and installs it on the R1's Jetson), then on the robot
`./run.sh robotcontrol.tools.robot_preflight` (read-only), `--observe` shadow run, and the first powered run with
`--amplitude 0.3`. The policy runs on the Jetson; the laptop only builds, copies and logs in.
