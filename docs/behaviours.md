# Behaviours: preview in Unreal, run on the robot

A *behaviour* is a small Python controller that declares which joints it powers. The same code runs on
three targets; only the transport changes:

| Target | Where | What happens to joints the behaviour does not power |
|---|---|---|
| `sim` | headless MuJoCo on this PC | actuators off; limp waist and head pitch locked (MuJoCo has no gearbox friction, the real ones hold); a head-off copy of the scene is stepped alongside, so the other-joint check only sees motion the behaviour caused |
| `urlab` | Unreal twin (URLab, MuJoCo inside Unreal), Python steps physics | held at their start pose (visual twin of the motion) |
| `real` | R1 Jetson, Unitree SDK2 low-level | motor mode 0 (disabled), identical to zero-torque |

Every target aborts when any non-powered joint moves more than `--max-other` (5 deg), plus the behaviour's
own checks. The run log (all 26 joint angles) is flushed every 0.2 s.

First behaviour: `neck_turn` (powers `head_yaw_joint` only; +-10 deg at 0.2 Hz by default, max 30 deg,
torque capped at the motor's 0.86 Nm; refuses to start unless the head is within 10 deg of straight ahead;
aborts if the head does not follow).

## Command line

```powershell
.venv\Scripts\python.exe -m robotcontrol.behaviours.runner neck_turn --target sim
.venv\Scripts\python.exe -m robotcontrol.behaviours.runner neck_turn --target urlab      # Unreal in Play
ssh -t unitree@192.168.123.164 "cd ~/porobot && ./run.sh robotcontrol.behaviours.runner neck_turn --target real"        # read-only
ssh -t unitree@192.168.123.164 "cd ~/porobot && ./run.sh robotcontrol.behaviours.runner neck_turn --target real --go"   # powered
```

## Unreal panel + live mirror

1. Start the service on the PC: `.venv\Scripts\python.exe -m robotcontrol.behaviours.service`
2. Start the twin: open `UnrealProject\R1Twin.uproject` and press **Play** (`R1_Stand` is the startup map), or
   run it as a standalone window from PowerShell (Git Bash rewrites `/Game/...` into a file path):
   `& "C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe" UnrealProject\R1Twin.uproject /Game/Levels/R1_Stand -game -windowed -ResX=1600 -ResY=900`
   With `bRunLevel0PolicyOnPlay=False` (Config/DefaultGame.ini) the twin holds its pose. The panel is top left;
   URLab's debug panels are hidden (**Details** or Tab shows them, plus the raw service output).
3. The panel has two colour-coded sections. Pick the turn size (10 / 20 / 30 deg) first.
   * **VIRTUAL ROBOT** (blue): **Move VIRTUAL robot** runs the behaviour on the Unreal twin. Nothing real moves.
   * **REAL ROBOT** (orange): **Check connection** runs it read-only on the robot (nothing published: robot
     reachable, head centred). **Move REAL robot** unlocks only when both checklist steps are green for the
     selected turn size (a clean virtual move, and a check less than 5 minutes old). It then asks for confirmation
     (robot on stand, head forward, remote in hand). During the run the twin switches to **live mirror**: it shows
     the robot's measured joint angles (read-only `stream_state` on the Jetson, URLab puppet mode).
   Each section shows a **HEAD** readout (yaw, pitch, target, other-joint drift, and how old the reading is) with a
   -30..+30 deg yaw gauge. Sources: the runner's 5 Hz `@head` line, the 1 Hz progress line older deployed runners
   print (no pitch), and during a robot run the live mirror stream. The robot kit picks up the `@head` line on the
   next `scripts\push_to_robot.ps1`.
4. **STOP** sends Ctrl-C to whatever is running (robot: the head is released to damping, then disabled). It goes
   over the network; the **remote (L2+B for 5 s) is the emergency stop**.

On the twin the legs have no stiffness (the URLab MJCF is built for the upper-body Level 0 policy), so they hang
and creep. The urlab backend lets them settle for 2 s before taking the start pose and leaves joints with model
kp 0 out of the other-joint check on the twin only (it prints which). The real run checks all 26 joints.

## Adding a behaviour

Write a class like `behaviours/neck_turn.py::NeckTurn` (`name`, `powered`, `duration_s`, `start`, `targets`,
`phase`, `describe`, `abort_reason`, `check_tracking`), register it in `behaviours/runner.py::BEHAVIOURS`
and `behaviours/service.py::BEHAVIOURS`, run it on `sim`, then `urlab`, then `real` read-only, then
`real --go`.
