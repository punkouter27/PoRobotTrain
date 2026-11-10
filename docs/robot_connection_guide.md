# Connecting the laptop to the R1 and running Level 0 on the stand

Written for the person operating the robot. Follow the stages in order and do not skip the read-only stages.
The behaviour is the Level 0 policy: head turns left and right, right arm raises and lowers, legs hang passive.

## How the pieces fit

```
Windows laptop (this project)                 Unitree R1 EDU
 - builds the robot kit (.tar.gz)    RJ45     - Jetson Orin NX dev computer  192.168.123.164  <- our code runs here
 - copies it with scp, logs in with ssh  ---> - motion controller             192.168.123.161  (not user-accessible)
   192.168.123.222/24                          Jetson <-> controller: DDS topics rt/lowstate, rt/lowcmd
```

The policy runs **on the robot's Jetson**, not on the laptop: Unitree's SDK and DDS stack target Ubuntu, and a
wired link inside the robot avoids Wi-Fi dropouts in the 50 Hz control loop. The laptop only builds, copies,
and logs in. The kit installs everything offline, so the robot does not need internet.

## Before you start

* Robot hanging on its stand, feet clear of the floor, nothing within reach of the right arm or head.
* Remote controller in hand. **Emergency damping: hold L2 + B for 5 s.** Zero torque: L2 + Y.
* Ethernet cable. The laptop has a wired port ("Ethernet", Realtek 2.5GbE).
* After power-on the R1 boots into **zero-torque** (about 1 minute). Leave it there.
  **Do not press L2 + UP or R2 + A**: those start Unitree's stand and walk controllers. Our program takes over the
  arm and head motors from zero-torque.

## Stage 1: wire the laptop to the robot

1. Plug the cable into the RJ45 port on the robot's upper-body interface panel and into the laptop.
2. Power the robot on (short press, then hold the battery button for more than 2 s) and wait about 1 minute.
3. In PowerShell **as Administrator**, from the project folder:
   ```powershell
   powershell -ExecutionPolicy Bypass -File scripts\robot_network_setup.ps1
   powershell -ExecutionPolicy Bypass -File scripts\robot_network_setup.ps1 -Adapter "Ethernet" -Apply
   ```
   This sets the laptop to `192.168.123.222/24` with no gateway (Wi-Fi keeps your internet), pings the Jetson and
   checks SSH. Undo later with `-Restore`.
4. Log in once and change the factory password (`123`):
   ```powershell
   ssh unitree@192.168.123.164
   passwd
   exit
   ```

## Stage 2: build and copy the robot kit

```powershell
.venv\Scripts\python.exe scripts\fetch_robot_vendor.py          # once, needs internet on the laptop
powershell -ExecutionPolicy Bypass -File scripts\push_to_robot.ps1
```

`push_to_robot.ps1` builds `dist\r1_robot_kit_<policy>_<time>.tar.gz` (about 82 MB), copies it to the Jetson,
unpacks it into `~/porobot`, and runs `setup_robot.sh`. What lands on the robot:

| Path on robot | What it is |
|---|---|
| `~/porobot/bundle/` | `policy.onnx`, `deploy.yaml`, `r1_joint_map.yaml`, `MANIFEST.json` (SHA-256 checksums) |
| `~/porobot/robotcontrol/` | the same runner, safety guard and tools used in the twin (no training code) |
| `~/porobot/.venv/` | Python environment built offline from `wheels/` |
| `~/porobot/run.sh` | runs any tool with the right environment |

Setup ends with a software-only preflight. It must print `PREFLIGHT PASSED`. If CycloneDDS is not preinstalled on
the Jetson, setup compiles it from the bundled source (needs `cmake` and `gcc`, about 5 minutes).

## Stage 3: read-only preflight on the live robot

On the robot (via `ssh unitree@192.168.123.164`):

```bash
cd ~/porobot
./run.sh robotcontrol.tools.robot_preflight --bundle bundle
```

This **only subscribes** to `rt/lowstate`. It checks the bundle checksums, the robot network interface, the state
message rate and CRC, that all 26 motors report angles inside their limits, the head motors, IMU attitude and motor
temperatures, and prints a table of every joint angle. It must print `PREFLIGHT PASSED`.

Sanity check while it runs: move the right arm gently by hand (robot in zero-torque) and run it again; the right
shoulder and elbow angles should change, with the sign the twin uses (arm forward/up = shoulder pitch decreasing).

## Stage 4: shadow run (policy on live data, nothing sent)

```bash
./run.sh robotcontrol.main_runner --target real --observe --policy bundle/policy.onnx \
    --command-mode sweep --amplitude 0.3 --duration 20
```

The policy reads the real joint angles at 50 Hz and computes the commands it would send; the log shows
`[observe] would send ...` and nothing is published. The run log is written to `~/porobot/logs/`. Watch for a
safety trip or warnings; there should be none.

## Stage 5: first powered run (small motions)

Only after Stages 3 and 4 passed, with the robot on the stand and the remote in hand:

```bash
./run.sh robotcontrol.main_runner --target real --policy bundle/policy.onnx \
    --command-mode sweep --amplitude 0.3 --duration 20
```

What happens (identical to the twin): the program releases any active Unitree motion mode, puts the legs in
passive damping (kp 0, kd 3), holds waist and left arm at the default pose, moves the right arm and head to the
default pose over 3 s, then runs the policy at 50 Hz. With `--amplitude 0.3` the head turns about +-9 degrees
and the arm lifts about 20 degrees. Ctrl-C (or the end of `--duration`) ramps to damping and exits.
The safety guard drops to damping on joint acceleration above 80 rad/s^2, stale commands, limit violations or
torque above the motor limit.

Then repeat with `--amplitude 0.6` and `1.0`, and try `--command-mode keyboard` (arrow keys: left/right = head yaw,
up/down = arm; `r` reset, `q` quit).

## Stage 6: compare with the twin

Copy the robot log back and compare it with the same sweep in the twin:

```powershell
scp unitree@192.168.123.164:porobot/logs/*.npz logs\robot\
.venv\Scripts\python.exe -m robotcontrol.main_runner --target sim --policy RobotControl\models\r1_level0_upperbody\policy.onnx --command-mode sweep --amplitude 0.3 --duration 20 --no-realtime
.venv\Scripts\python.exe -m robotcontrol.tools.sim_real_diff logs\<sim log>.npz logs\robot\<robot log>.npz
```

Pass: every joint within 5 degrees of the twin at every 20 ms tick.

## Updating the policy later

Train or retrain, run the verification gate, `deploy_pack`, then `push_to_robot.ps1` again. The robot keeps its
`.venv`; only the bundle and code are replaced. `MANIFEST.json` checksums are re-verified by the preflight.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| ping fails | robot still booting, cable in the wrong port, wrong `-Adapter`, laptop IP not applied (run as Administrator) |
| `no rt/lowstate message within 5s` | wrong network interface on the Jetson (pass `--iface`), or the robot is not an EDU unit |
| preflight shows a joint far outside its limits | joint map or robot variant mismatch (this kit assumes the A5 arm variant, 26 motors). Stop and report it. |
| `ReleaseMode failed` | a Unitree mode is active and cannot be released: power-cycle and stay in zero-torque |
| setup cannot build CycloneDDS | `sudo apt install cmake build-essential` (needs internet once) |

## Still unverified for your specific robot

These come from Unitree's SDK examples and third-party R1 guides (Weston Robot, quadruped.de), not from your unit:

* Jetson address `192.168.123.164`, login `unitree` / `123`, motion controller `192.168.123.161`.
* The Jetson's network interface name for the internal LAN (auto-detected from its 192.168.123.x address).
* Head motors on low-level slots 29 (pitch) and 30 (yaw), and the A5 arm variant (no wrist pitch/yaw).
* Whether CycloneDDS and `unitree_sdk2py` come preinstalled (setup handles both cases).

Stage 3 checks all of these read-only before anything moves.
