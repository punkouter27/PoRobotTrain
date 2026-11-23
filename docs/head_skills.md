# Trained head skills: train in simulation, play on the twin, run on the R1

Five reinforcement-learned head motions for the **stand phase**. Each one powers only the head motors it needs. Every
other motor stays off (real robot) or is held still (twins), and the runner aborts if any other joint moves more than 5°.

| Skill | Motors | What the policy does | Size (panel 10 / 20 / 30°) |
|---|---|---|---|
| **Hold Centre** | head pitch + yaw | Holds the head still facing forward, from any start within ±10° | ignored |
| **Look At** | head pitch + yaw | Turns to a target (yaw, pitch) and settles. The demo cycles centre, left, right, down, up, half-left. Later the camera supplies the target. | reach, max 30° |
| **Nod Yes** | head pitch only | Smooth "yes" nod, 2 s per cycle | max 15° |
| **Shake No** | head yaw only | Smooth "no" shake, 3 s per cycle | max 20° |
| **Glance Around** | head pitch + yaw | Centre, left, right, slightly down, centre, pausing at each (12 s cycle) | max 25° |

## One path, three places

1. **Train** in the web app (Control → *Train a skill*, or `/skills`). Training runs on the PC that runs the
   behaviour service, never on the robot:
   `python -m robotcontrol.training.train_head_skill <skill|all>`. It uses PPO on 512 domain-randomised copies of the
   stand twin (`r1_twin_stand_train.xml` + `sim_calibration.yaml`; gains ±20 %, mass ±10 %, friction ±40 %, damping and
   armature, 0–20 ms action delay, sensor noise, random start pose). It takes about 2–5 min per skill on an RTX 5070 Ti
   Laptop. Then it exports `policy.onnx` and runs the validation gate.
2. **Play on the twin**: in the web app, once the skill is validated (★ chips on Control, *Play on twin* on
   `/skills`). It runs through the same runner as the scripted behaviours. **Test it in scenarios** in the Unreal
   scenario lab (docs/scenario_lab.md): a moving look-at ball, a head camera, pushes, delay, noise, weak motors and
   skill timelines. The brain runs inside Unreal (NNE) with the same contract and validation gate.
3. **Run on the robot**: the usual four steps (twin preview with these exact settings → read-only check → pre-flight
   checklist → confirm). The read-only check first copies the validated `policy.onnx` + `deploy.yaml` to
   `~/porobot/head_skills/<skill>/` on the Jetson (`behaviours/skill_push.py`). The robot kit also ships every
   validated skill.

**Twin | Robot** (`/mission`, the Control task *Twin | Robot*, or *Run on robot* on the Skills page) shows both
side by side. The virtual robot is on the left. On the right, the real R1 is drawn only from its **measured** joint
angles: all 26, streamed read-only during *Read-only check*, *Live view* and the powered run. The page shows what the
physical robot actually did, and the gates between the two sides are the same ones the service enforces. When the
robot is not connected, the right side shows the practice twin.

The robot must run the current `robotcontrol` package. After updating the PC, push it once with
`scripts\push_to_robot.ps1`, which builds the kit, copies it and installs it, sending nothing to the motors.

## The web app: five steps and the Library

The web app guides you through **1 Check** (both robots), **2 Coded moves**, **3 Train**, **4 Virtual robot** (watch a
trained skill) and **5 Real robot** (the four safety steps). A step opens when the one before it is done; steps complete
from what really happened (connection known, a clean virtual run, a trained skill). The **Library** lists every trained
skill with all its training sessions: when, how much data (steps and hours of simulated practice), training time,
ONNX size, accuracy, the check result, **▶ Play** (that exact session on the virtual robot), **Results** (training
curve + benchmarks against the bars), **Make active** (rollback), **Copy to robot** (no motion) and **View in Unreal**
(the scenario lab plays it, docs/scenario_lab.md).

## Validation gate (nothing runs without it)

`python -m robotcontrol.training.validate_head_skill <skill>` writes `validation.json`, `deploy.json` (for Unreal) and the `validation` block of
`deploy.yaml`. A skill loads only when `validation.passed` is true **and** `validation.policy_sha256` matches the ONNX
file on disk. The PC, the Unreal path and the Jetson all re-check this before loading.

| Check | Bar (mean tracking error vs. the skill's reference motion) |
|---|---|
| ONNX vs PyTorch actions | max difference < 1e-4 |
| Training env, nominal physics | hold 1.0°, look-at 2.0° (settled), nod/shake 3.0°, glance 4.0° |
| 8 randomised robots | hold 1.5°, look-at 3.0°, nod/shake 4.0°, glance 5.0° |
| **Runtime path**: the Jetson code (`TrainedHeadBehaviour` + `runner.run`) on the separate realistic twin (control delay, low-pass, back-EMF, sensor noise, compliant stand, head-only power) | same as randomised, no abort, other joints < 5° |

Retraining keeps the previous policy until the new one has been exported. A failed gate is shown with its reason
("FAILED GATE"), and the skill stays locked.

## Contract (training ≡ runtime)

`RobotControl/behaviours/head_skills.py` is the single source for both training and runtime. It is numpy-only and
compatible with Python 3.8 for the Jetson.

- **Joints:** the skill's powered joints, in the order (head_pitch, head_yaw).
- **Observation:** `[q − default (k), dq (k), last_action (k), command (3)]`.
- **Action:** `target = default + action_scale · clip(a, ±5)`, clipped to 90 % of the joint range.
- **Policy rate:** 50 Hz (`POLICY_DT` 0.02 s). The sim backend ticks at 200 Hz and the real one at 500 Hz, and both
  hold the last target between policy steps.
- **Command:** Hold `[0,0,0]`; Look At `[yaw, pitch, 0]` rad; periodic skills `[sin 2πt/T, cos 2πt/T, size]`, so
  the motion lives in the network and the panel's size scales it.

At runtime every target also passes the per-joint safety envelope (`JointSafetyChannel`): stale state (> 0.1 s), the
torque cap `kp·|target − q| ≤ effort` (0.86 Nm on head yaw), joint limits, and a tracking timeout against the
reference motion. The timeout is 8° for 0.4 s on the smooth skills and 15° for 1 s on Look At's target jumps. A
skill refuses to start unless both head joints are within ±10° of straight ahead. It ends with 1 s of Hold Centre,
and the head is released facing forward.

## Files

| Path | What |
|---|---|
| `RobotControl/behaviours/head_skills.py` | skill definitions, reference motions, ONNX runtime behaviour, validation-gate loader |
| `RobotControl/training/head_skill_env.py` | vectorised MuJoCo training env (head joints only) |
| `RobotControl/training/train_head_skill.py` | PPO training + ONNX export + gate |
| `RobotControl/training/validate_head_skill.py` | validation gate |
| `RobotControl/behaviours/skill_push.py` | copies a validated skill to the robot before the read-only check |
| `RobotControl/models/head_skills/<skill>/` | `policy.onnx`, `deploy.yaml`, `deploy.json`, `validation.json`, `model.pt`, `train_log.csv` |
| `src/PoRobotTrain.Client/Features/Skills/SkillsPage.razor` | Blazor *Trained skills* page |
| `UnrealProject/Source/R1Twin/Private/R1SkillBrain.cpp`, `R1SkillLibrary.cpp`, `R1PanelLab.cpp` | Unreal scenario lab: in-engine brain (NNE), `deploy.json` + SHA-256 gate, Lab tab |

## Next steps (not built yet)

- Camera targets for **Look At**: the head camera (`head_rgbd`, 58° vertical FOV) plus a hand detector supplying
  `[yaw, pitch]` in place of the demo sequence. The policy and its contract stay the same.
- Off-stand skills need their own safety design before any leg or waist motor is powered on the real robot.
