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
2. Open `UnrealProject\R1Twin.uproject`, level `R1_Stand`, press **Play**. With
   `bRunLevel0PolicyOnPlay=False` (Config/DefaultGame.ini) the twin holds its pose; the panel is bottom left.
3. **Preview in twin** runs the behaviour on the Unreal twin.
4. **Check robot** runs it read-only on the robot (nothing published): robot reachable, head centred.
5. **Run on robot** is enabled only after a clean preview *with the same settings* and a read-only check
   less than 5 minutes old. It opens a confirmation (robot on stand, head forward, remote in hand).
   During the run the twin switches to **live mirror**: it shows the robot's measured joint angles
   (read-only `stream_state` on the Jetson, URLab puppet mode).
6. **STOP** sends Ctrl-C to the robot run (the head is released to damping, then disabled). It goes over
   the network; the **remote (L2+B for 5 s) is the emergency stop**.

## Adding a behaviour

Write a class like `behaviours/neck_turn.py::NeckTurn` (`name`, `powered`, `duration_s`, `start`, `targets`,
`phase`, `describe`, `abort_reason`, `check_tracking`), register it in `behaviours/runner.py::BEHAVIOURS`
and `behaviours/service.py::BEHAVIOURS`, run it on `sim`, then `urlab`, then `real` read-only, then
`real --go`.
