# Level 2 walk sandbox (simulation only)

Take the twin off the stand and drive it around a course with WASD, a gamepad, the on-screen pad or your voice.
The walk runs the Level 2 velocity policy in headless MuJoCo inside the behaviour service. The Blazor **Walk
sandbox** page and the Unreal scenario lab's **Walk (sim)** tab both drive and show the same run.

**Never the real robot.** The sandbox has no robot backend, `robot_kit.py` excludes `RobotControl/walk/`, and a
Level 2 policy is loaded only after the Level 2 gate passed.

## Pipeline

```
scripts/level2/train_level2.ps1          Docker + GPU: Unitree unitree_rl_mjlab (mjlab + MuJoCo Warp), task Unitree-R1-Flat
        |                                 checkpoints and TensorBoard logs -> logs/level2_training/
        v
scripts/level2/train_level2.ps1 -Export  newest checkpoint -> logs/level2_training/candidate/{policy.onnx, deploy.yaml, golden.npz}
        |
        v
python -m robotcontrol.training.validate_level2 --candidate logs/level2_training/candidate --promote
        |   contract, observation/action parity with the trainer, sim2sim walking in OUR calibrated twin
        v
RobotControl/models/r1_level2_velocity/   (deploy.yaml validation.policy_sha256 = the ONNX's sha256)
        |
        v
behaviour service /walk/*  ->  Blazor Walk page  +  Unreal Walk tab (URLab puppet mode)
```

```powershell
pwsh scripts\level2\train_level2.ps1                   # build the image, start training detached (container porobot-l2-train)
docker logs -f porobot-l2-train                        # follow progress
pwsh scripts\level2\train_level2.ps1 -Export           # export the newest checkpoint as a candidate
.venv\Scripts\python.exe -m robotcontrol.training.validate_level2 --promote
```

## Training

- Trainer: Unitree `unitree_rl_mjlab` pinned at `1425b15f73bd4095f0df53709d7c389c3eb9e790` (reference clone in
  `external/unitree_rl_mjlab`, gitignored), mjlab 1.2.0 on MuJoCo Warp, task `Unitree-R1-Flat`, 4096 envs,
  3000 iterations (below the velocity-curriculum switch at 120k env steps). Upstream ships no trained R1 policy.
- Needs Docker Desktop with GPU passthrough (`docker run --gpus all ... nvidia-smi` must work). The image pins
  `warp-lang==1.12.*` (mjlab 1.2.0 still calls `wp.context`), adds `scipy` (an undeclared mjlab import) and lets
  mjlab pull torch cu132, which supports the RTX 50-series (sm_120). The Warp kernel cache lives in a named volume.
- Speed on the RTX 5070 Ti Laptop: about 4 s per iteration cool, 7–12 s when the GPU thermally throttles at 88 °C,
  so a 3000-iteration run takes about 6–9 hours. Checkpoints every 100 iterations in
  `logs/level2_training/rsl_rl/r1_velocity/<run>/model_<it>.pt`.
- The export (`-Export`, newest checkpoint unless `-Checkpoint` is given) writes the candidate: `policy.onnx` with the
  empirical observation normaliser in the graph and a symbolic batch dimension, `deploy.yaml` (gains, defaults,
  action scale, observation terms with their definitions, command ranges, held head joints, the compiled training
  dynamics, source metadata) and `golden.npz` (60 obs/action pairs with the raw signals behind them).

First gate run, iteration 100 (2026-11-21): contract and parity pass (observations 8e-7, actions 6e-7), and in our
calibrated twin it already stands 10 s and survives 30 s of random commands without a fall, but it does not walk yet
(forward error 0.5 m/s at a 0.5 m/s command). The gate correctly refuses it.

## Policy contract

Observation (83, in this order): `base_ang_vel(3)` (pelvis frame), `projected_gravity(3)`, `velocity_commands(3)`
(vx, vy m/s, wz rad/s), `gait_phase(2)`, `joint_pos_rel(24)`, `joint_vel_rel(24)`, `last_action(24)`.
Exact definitions (mjlab source, recorded in deploy.yaml): `base_ang_vel` is the pelvis IMU gyro (site at the pelvis
origin, identity orientation); `projected_gravity` = (0, 0, −1) rotated into the pelvis frame; the command is in the
yaw frame; the gait phase is p = (k · 0.02 mod 0.6) / 0.6 with k = policy steps since the episode reset (0 for the
first observation), giving [sin 2πp, cos 2πp], and **both are 0 while |command| < 0.1** (standing); joint position is
q − default without encoder bias; last action is the raw policy output (0 after a reset). Play-time command ranges:
vx −0.5…1.0 m/s, vy −0.5…0.5 m/s, wz −0.5…0.5 rad/s. Training ends an episode at 70° tilt.

Actions (24) are in joint-map `policy_index` 0–23 order (legs, waist, arms). Target =
`default + action_scale * action`, clipped to the joint limits. The head is held at its default pose. The kp/kd gains
are the joint map's (the gate checks this). Term scales, clips, the gait period and the command ranges come from the
candidate's `deploy.yaml`; `robotcontrol/walk/policy.py` builds the observation from them, and the gate proves it
matches the trainer's own observations to 1e-4 using `golden.npz`.

## Known sim-to-sim differences (training model vs our twin)

The gate's sim2sim scenarios exist because these differ:

| | training (mjlab `r1.xml`) | our twin |
|---|---|---|
| head | no head links | head links + joints: 30.15 kg vs 28.93 kg, 1.2 kg more at the top |
| joint friction | armature 0.01, damping 0.05, frictionloss 0 | `sim_calibration.yaml` (legs: armature 0.035, damping 0.25, frictionloss 0.35) |
| feet | condim 3, friction 0.6 (randomised 0.3–1.6) | condim 6, friction 1.0, floor 0.9 |
| randomisation | encoder bias ±0.015 rad, torso CoM ±5 cm, pushes every 5–6 s | none |

## The Level 2 gate (`validate_level2.py`)

| group | checks |
|---|---|
| contract | joint order, 83 → 24 shapes, default pose and kp/kd equal the joint map |
| parity | our observation builder vs the trainer's observations, onnxruntime vs the trainer's actions (both < 1e-4) |
| sim2sim | in our calibrated ground twin: stand still 10 s, forward 0.5 and 1.0 m/s, backward, sideways, turn 0.5 rad/s, random commands for 30 s. No fall, velocity tracking within tolerance |
| info | one lap of the loop course with a simple heading controller |

`--promote` copies the policy into `RobotControl/models/r1_level2_velocity/` only when every check passes.

## Sandbox

- Courses (`robotcontrol/walk/course.py`): **loop** (six checkpoints, three pillars, a wall to walk around and a
  2 cm plank) and **flat** (same checkpoints, no obstacles). A lap ends back at the start; laps, best lap and falls
  are counted. A fall (pelvis below 0.45 m or tilted more than 60°) resets after 1 s.
- Commands are clamped to the policy's ranges. A watchdog ramps the command to zero 0.5 s after the last one, so a
  client that disappears stops the robot.
- **Hold mode** works without a policy: every joint holds its default pose and a soft virtual harness keeps the
  pelvis up (the R1 cannot balance passively and would tip over in about 1 s). It tests the scene, the web view
  and the Unreal puppet path; driving is disabled.
- Unreal never simulates the robot: the service pushes the full pose (floating base + 26 joints) every 20 ms in
  URLab puppet mode, the stand is hidden while walking, and the follow camera tracks the pelvis.

API: `docs/service_api.md` section 3. Tests: `RobotControl/tests/test_walk_sandbox.py`.

## Simulation-only extras (docs/sim_features.md)

The sandbox also reports a physics overlay (CoM, capture point, foot support polygon and signed capture-point
margin, up to 24 robot contacts with world forces), motor speed / torque with the display-only heat + battery model
(`RobotControl/configs/motor_model.json`), and the action filter view (raw vs EMA, jitter score; applied only when the
operator switches it on). Without a walking policy, mode `free_hold` (PD hold, no harness) topples in about a second:
it exercises the fall analysis (`t_unstable` = start of the final run of capture point outside the feet; cause
`cp_outside | tilt | height | prop_hit`) and the 10 s replay buffer (the simulation pauses while replaying).
`POST /walk/throw` adds balls / boxes (MuJoCo free bodies pre-added at load, parked far away until thrown).

Course `procedural` (`walk/course.py`) is generated from a seed and a difficulty; the same terrain can go into a
training scene: `python -m robotcontrol.walk.course --export procedural --seed S --difficulty D --out terrain.xml`
writes an MJCF fragment to `<include>`. Scenario scripts (`RobotControl/scenarios/*.json`, target `walk`) replay a
fixed sequence of commands, throws, resets, replays and filter changes on sim time (`POST /walk/scenario`).

