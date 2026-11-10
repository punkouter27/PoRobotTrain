# Level 0 on the real R1: head sweep and arm raise on the stand

Written for the robot operator. Every step below is first rehearsed in the twin with the identical command,
then repeated on the robot. Nothing in this procedure lets the robot leave the stand.

## What the policy does

* Drives **7 motors only**: right shoulder pitch/roll/yaw, right elbow, right wrist roll (slots 22 to 26),
  head pitch (slot 29) and head yaw (slot 30).
* **Legs** (slots 0 to 11) are put in **passive damping**: kp = 0, kd = 3. They hang under gravity and resist motion only.
* **Waist and left arm** (slots 12, 13, 15 to 19) are **held at the default pose** with the joint-map gains. A passive waist
  lets the torso twist under the arm's reaction torque and drags the hanging hand into the thigh, so it is held.
* Commands: head yaw target (±30° in the default sweep, up to ±60°) and right shoulder pitch target
  (from the 0.35 rad default up to 1.6 rad forward/up).

## Before the first robot run

1. `validate_level0.py` printed **ALL CHECKS PASSED** on the current `policy.onnx`.
2. `onnx_validate` passed against `deploy.yaml` and `r1_joint_map.yaml`.
3. The same ONNX ran for at least 60 s in the UE5 twin on the stand (`--target urlab`) with keyboard commands and no safety trips.
4. Robot on its stand, feet clear of the floor, nothing within arm's reach of the right arm or head.
5. Robot kit installed on the Jetson and the read-only preflight passed (`docs/robot_connection_guide.md`, Stages 1 to 4).

## Robot session

| Step | Robot | Twin equivalent |
|---|---|---|
| 1 | Power on, robot in zero-torque on the stand | `scene_stand.xml` home keyframe |
| 2 | Runner connects, calls `MotionSwitcherClient.ReleaseMode()` until no built-in mode is active | n/a |
| 3 | Runner sets legs to passive damping, holds waist/left arm at default, waits 2 s (legs settle) | `presettle(2.0)` |
| 4 | Runner ramps the 7 controlled motors from their current position to the policy default over 2 s with Level 0 gains | reset to home |
| 5 | Policy starts at 50 Hz with command = default (no motion) | same |
| 6 | Operator drives commands: arrow keys (left/right = head yaw, up/down = arm) or `--command-mode sweep` | same |
| 7 | Stop: any key or Ctrl-C ramps commands back to default, then drops to damping, then zero-torque | same |

The safety guard trips to damping on: joint acceleration > 80 rad/s², command staleness > 100 ms, soft-limit
violation, or torque estimate above the motor limit. On the stand the base-tilt check is informational only.

## After the run

`sim_real_diff` compares the robot trajectory log with the twin log for the same command sequence. The gate is
max per-joint error < 5° at every 20 ms tick. If it fails, the usual causes are, in order: motor slot mapping,
sign convention of a joint, kp/kd not applied, or a stale `deploy.yaml`.

## Known open items

* The stand model in the twin is a generic gantry with the pelvis welded at 1.0 m. Send a photo or measurements of
  your stand to match the mesh and hook point.
* Unitree's documentation site could not be read automatically. The low-level procedure above follows Unitree's own
  `r1_low_level_example.py`; confirm against the R1 developer quick start that debug mode does not need an extra
  remote-controller step on your firmware.
