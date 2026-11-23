# Scenario lab (Unreal): hatch the trained brains and test them

The two apps have different jobs:

| Web app (`http://127.0.0.1:5080`) | Unreal app (`UnrealProject\R1Twin.uproject`) |
|---|---|
| Debugging and training with the R1 on its stand: train and validate skills, the skill library, every real-robot step (twin preview, read-only check, pre-flight, confirm, live mirror), health, setup and logs | **Scenario lab, simulation only**: load the brains the web app trained, put them in a scene, and see whether they hold up |

Unreal never trains and never connects to the physical robot.

## Start it

Open `UnrealProject\R1Twin.uproject` and press **Play** (`R1_Stand`), or run it standalone:

```powershell
& "C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe" UnrealProject\R1Twin.uproject /Game/Levels/R1_Stand -game -windowed -ResX=1600 -ResY=900
```

The lab needs no Python. The brains run inside Unreal through NNE (ONNX Runtime CPU, about 0.1 ms per step). Only
the **Walk (sim)** tab needs the behaviour service (`python -m robotcontrol.behaviours.service`).

## The Lab tab

| Section | What it does |
|---|---|
| **Brain** | One button per trained skill (Hold Centre, Look At, Nod Yes, Shake No, Glance Around). A skill that has not passed its validation gate is locked; click it to see why. Pick the size (10 / 20 / 30°) and run length (6 / 12 / 30 s), then **Play**. **Reload** picks up a skill you just trained in the web app. |
| **Target** | The orange ball that **Look At** follows. *Script* repeats the web app's demo sequence. *Manual*: the arrow keys or the Left / Right / Up / Down buttons move it. *Orbit* traces a figure-8 in front of the robot. A ball outside the trained reach (±40° yaw, ±20° pitch, capped by the size) is clamped, and the result card says so. |
| **Disturb** | **Push** the head (a real MuJoCo force on `head_yaw_link`, stronger than the 0.86 Nm yaw motor). Add 20–80 ms of **delay** and 0.5–2° of joint-angle **noise**, or weaken the head **motors** to 75 / 50 %. These test whether the brain copes. |
| **Timeline** | Chain skills into a scene: **+ Add current** appends the selected skill, size and length. **Play timeline** runs the steps in order, and **Loop** repeats them. |
| **Result** | Each joint's *is / should be / brain cmd*. The tracking error (now / mean / max) is shown next to the error the skill's validation gate measured, in amber when it is much worse. It also tells you whether the **real robot's safety envelope** would let the run continue, using the same rules as the runner: refuse to start unless the head faces forward within 10°, stop after a tracking error beyond 8° for 0.4 s (15° for 1 s on Look At), stop if any other joint moves more than 5°. |

The right column shows the **head camera** (`head_rgbd`: 58° vertical FOV, tilted 20° down, the robot itself hidden),
with a crosshair at the image centre. When Look At is working, the ball sits on the vertical line. **Joints (26)**
swaps the camera for the joint matrix. **STOP** stops the brain and the walk sandbox. **Details** shows URLab's
debug panels.

## Same brain as the robot

`UR1SkillBrainComponent` (`R1SkillBrain.cpp`) mirrors `RobotControl/behaviours/head_skills.py`:

- **Inputs:** the same observation, `[q − default, dq, last_action, command]`, at the trained 50 Hz.
- **Outputs:** the same action, `default + action_scale · clip(a, ±5)`, then the soft limits.
- **Torque cap:** the runtime's `kp · |target − q| ≤ effort`.
- **Commands:** hold `[0,0,0]`; Look At `[yaw, pitch, 0]`; periodic skills `[sin, cos, size]`. Every skill ends with
  1 s of Hold Centre.

Only the head motors the skill owns are driven. The waist and arms are held at the stand pose.

A skill loads only when its `deploy.json` says `validation.passed` and `validation.policy_sha256` equals the
SHA-256 of its `policy.onnx`. This is the same gate the PC runner and the Jetson apply. The files come from
`RobotControl/models/head_skills/<skill>/` (override with `-R1SkillsDir=<path>`).
`python -m robotcontrol.training.validate_head_skill <skill>` writes `deploy.json`, and `--engine-json-only`
rewrites it from an existing `deploy.yaml`.

**View in Unreal** (web app Library): the lab polls the service every second (`GET /lab/poll`); a new request
reloads the skills (picking up a just-trained or rolled-back version), switches to the Lab tab and plays that skill.

While a Python client drives URLab, the lab lets go of the actuators and waits. That covers the web app's twin
preview with *Twin engine: Unreal*, the live mirror of a real run, and the walk sandbox.

## Scripted check

`-PanelCapture=<dir>` runs an unattended scenario and saves a screenshot of each stage, then quits:

1. Look At following the orbiting ball.
2. A push.
3. A four-skill timeline.
4. Shake No with 80 ms delay, 2° noise and half-strength motors.
5. The joint matrix.
6. The Walk tab.

The latest run is in `SCREENSHOTS/unreal_lab/`.
