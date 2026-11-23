# Simulation features (contract)

Fifteen simulation-only features added on 2026-11-22. This file is the shared contract between the Python behaviour
service (`RobotControl/`), the Unreal scenario lab (`UnrealProject/`) and the web app (`src/`). Nothing here reaches
the physical robot: the walk sandbox has no robot backend, the scenario lab never connects to the robot, and the
skill tools (duel, gap check, montage, synthetic camera data) run headless MuJoCo or read log files. Taught poses
(#14) become ordinary head skills and go through the existing train + validation gate like every other skill.

| # | Feature | Computed in | Shown in |
|---|---|---|---|
| 1 | Motor sound engine | Unreal (procedural synth) from joint speed + torque | Unreal (Lab + Walk) |
| 2 | Physics debug overlay (contacts, forces, CoM, capture point, support polygon) | Python walk sim; Unreal lab (head torque) | Unreal Walk, web Walk |
| 3 | Procedural terrain shared with training | `walk/course.py` (seeded generator, MJCF export) | Unreal Walk, web Walk |
| 4 | Sim-to-real gap check + ghost | `tools/gap_check.py` | web Library (ghost in the 3D view) |
| 5 | Motor heat + battery model | `sim_features/motor_model.py` (Python), C++ port for the lab | Unreal (Lab + Walk), web Walk |
| 7 | Synthetic head-camera data | `tools/synth_camera.py` (MuJoCo `head_rgbd`); Unreal "Record dataset" | files under `datasets/` |
| 9 | Throwable objects | Python walk sim (MuJoCo free bodies) | Unreal Walk, web Walk |
| 10 | Policy-vs-policy duel | `tools/skill_duel.py` (headless MuJoCo, same seeded lab scenario) | web Library |
| 11 | Training montage | `train_head_skill.py --checkpoint-every`, `tools/training_montage.py` | web Library |
| 12 | Slow-motion replay + free camera | Python walk sandbox ring buffer | Unreal Walk (free camera), web Walk |
| 14 | Teach a pose by demonstration (head only) | `behaviours/taught_poses.py` -> skill `pose_<id>` | web app (pose sliders -> save -> train) |
| 15 | Action filter view (raw vs low-pass, jitter score) | Python walk sim; Unreal lab brain | Unreal (Lab + Walk), web Walk |
| 17 | Torque alert tone | Unreal (same synth) when any torque fraction > 0.8 | Unreal |
| 18 | Scenario scripts | `RobotControl/scenarios/*.json` (walk: Python; lab: Unreal + duel) | Unreal, web Walk |
| 19 | Fall analysis | Python walk sim | Unreal Walk timeline, web Walk |

## Shared config: `RobotControl/configs/motor_model.json`

Read by Python (`robotcontrol.sim_features.motor_model`) and by the Unreal lab (C++ port). A pytest checks the C++
defaults match the file.

```json
{
  "ambient_c": 25.0,
  "thermal_time_constant_s": 120.0,
  "heat_rate_c_per_s_at_full_torque": 0.8333,
  "warn_c": 70.0,
  "limit_c": 90.0,
  "battery_wh": 500.0,
  "idle_w": 60.0,
  "drive_efficiency": 0.8,
  "copper_loss_w_at_full_torque": 40.0,
  "torque_alert_fraction": 0.8,
  "action_filter_alpha": 0.5,
  "jitter_window_s": 1.0
}
```

Model (per joint i, every policy step dt): `f_i = |tau_i| / effort_i` (effort from `r1_joint_map.yaml`);
`T_i += dt * (heat * f_i^2 - (T_i - ambient) / time_constant)`; electrical power
`P = idle_w + sum(max(tau_i*dq_i, 0)) / drive_efficiency + sum(copper_loss * f_i^2)`; `soc -= P*dt / (battery_wh*3600)`.
Display only: it never derates a motor. Battery size is a placeholder assumption, not the R1 datasheet value.
Torque source: walk sim = MuJoCo `actuator_force`; lab = PD estimate `kp*(target-q) - kd*dq`, clipped to effort.
Jitter score = RMS of the action second difference `a_t - 2a_{t-1} + a_{t-2}` over `jitter_window_s`.
Filtered action = EMA `y += alpha*(a - y)`; shown beside the raw action; applied to the walk sim only when the
operator switches `apply` on (the shipped policies were trained without it).

## Walk sandbox (service, docs/service_api.md section 3 extended)

### `POST /walk/start`
Body adds optional fields: `{"course": "flat|loop|procedural", "mode": "policy|hold|free_hold", "seed": 0,
"difficulty": 0.5}`. `free_hold` = every joint PD-held at default WITHOUT the hold harness: the robot tips over in
about a second (useful for the physics overlay, fall analysis and replay without a walking policy). `seed` and
`difficulty` (0..1) only matter for `procedural`.

### `POST /walk/throw`
`{"kind": "ball|box", "speed": 4.0, "from_deg": 0.0}` -> `{"ok": true}`. speed 1..8 m/s; from_deg = direction the
object comes FROM, relative to the robot heading (0 = in front, 90 = robot's left). Spawns 2.0 m from the pelvis at
pelvis height, velocity aimed at the pelvis. ball: radius 0.10 m, 0.45 kg. box: 0.25 m cube, 2.0 kg. At most 4
props (oldest is recycled). Errors: not running -> ok false.

### `POST /walk/replay`
`{"seconds": 3.0, "speed": 0.25}` starts a replay of the last `seconds` (1..10) at `speed` (0.1..1). The simulation is
paused while replaying and resumes after. `{"stop": true}` ends it. Option: `{"auto_on_fall": true|false}` (default
false) arms an automatic 3 s / 0.25x replay 1 s after every fall.

### `POST /walk/filter`
`{"alpha": 0.5, "apply": false}`.

### `POST /walk/scenario`
`{"id": "push_test"}` loads `RobotControl/scenarios/push_test.json` (target must be `walk`), (re)starts the sandbox
with its course/mode/seed and plays its steps on sim time. `{"stop": true}` stops the script (sandbox keeps running).

### `GET /scenarios`
`{"ok": true, "scenarios": [{"id", "name", "target": "walk|lab", "description", "duration_s"}]}`.

### `GET /walk/state` additions (all present when a sim exists; `null` otherwise)
```jsonc
{
  "seed": 0, "difficulty": 0.5,                  // procedural only, else null
  "physics": {
    "com": [x, y, z],                            // m, MuJoCo world
    "com_vel": [vx, vy, vz],
    "capture_point": [x, y],                     // com_xy + com_vel_xy / sqrt(9.81 / com_z)
    "support": [[x, y], ...],                    // convex hull of foot/ground contact points (may be empty)
    "cp_margin_m": 0.03,                         // signed distance capture point -> support hull, + inside; null if no support
    "contacts": [{"pos": [x,y,z], "force": [fx,fy,fz], "a": "left_foot", "b": "floor"}]  // N in world, <= 24, robot-involved
  },
  "motors": {"names": [26], "speed": [26 rad/s], "torque": [26 Nm], "torque_frac": [26 0..1+]},
  "thermal": {"temp_c": [26], "hottest": "left_knee_joint", "hottest_c": 41.2, "warn": [names], "limit": [names]},
  "battery": {"soc_pct": 99.7, "power_w": 312.0, "minutes_left": 92.0},
  "action_filter": {"alpha": 0.5, "apply": false, "raw": [24], "filtered": [24], "jitter": 0.012},
  "props": [{"id": 0, "kind": "ball", "pos": [x,y,z], "quat": [w,x,y,z], "size": [r] | [hx,hy,hz]}],
  "fall_analysis": {                             // last fall, null until one happened
    "t_fall": 12.34, "t_unstable": 11.92, "lead_s": 0.42,
    "reason": "capture point left the feet (6 cm outside)" ,
    "cause": "cp_outside|tilt|height|prop_hit",
    "history": [{"t", "cp_margin_m", "tilt_deg", "height"}]   // last 3 s before the fall, 10 Hz
  },
  "falls_log": [{"t_fall", "t_unstable", "lead_s", "cause"}],   // last 10
  "replay": {"active": false, "speed": 0.25, "t": 0.0, "duration_s": 3.0, "auto_on_fall": false},
  "scenario": {"id": "push_test", "t": 4.2, "step": 2, "steps": 5, "done": false} // or null
}
```
While a replay is active, the pose fields (`base`, `q`, `props`, `physics`) are the replayed frame, and `t` is the
replayed frame's sim time.

### Procedural course (`walk/course.py`)
`course("procedural", seed=s, difficulty=d)` returns the usual `{"checkpoints", "checkpoint_radius", "boxes"}`;
the same seed always gives the same boxes. Pieces: stairs (stacked boxes, rise 0.02 + 0.10*d m), ramps (pitched
boxes up to 4 + 8*d deg), stepping stones, clutter, low planks. The first 1.5 m around the origin stays clear.
`python -m robotcontrol.walk.course --export procedural --seed S --difficulty D --out terrain.xml` writes an MJCF
fragment (`<mujoco><worldbody><geom .../></worldbody></mujoco>`) for a training scene to `<include>`. Clients keep
drawing `geometry.boxes`, unchanged.

## Scenario scripts (`RobotControl/scenarios/*.json`)
```jsonc
{ "id": "push_test", "name": "Ball from the left", "target": "walk", "description": "...",
  "course": "flat", "mode": "free_hold", "seed": 0, "difficulty": 0.5, "duration_s": 8,
  "steps": [ {"t": 0.5, "do": "command", "vx": 0.3, "vy": 0, "wz": 0},
             {"t": 2.0, "do": "throw", "kind": "ball", "speed": 5, "from_deg": 90},
             {"t": 6.0, "do": "replay", "seconds": 3, "speed": 0.25} ] }
```
Walk step kinds: `command`, `throw`, `reset`, `replay`, `filter`.
Lab scenarios (`"target": "lab"`, played by the Unreal lab brain and by the duel tool):
```jsonc
{ "id": "lab_stress", "name": "Look-at under stress", "target": "lab", "skill": "look_at", "size_deg": 15,
  "duration_s": 12, "seed": 1,
  "steps": [ {"t": 0, "do": "target", "yaw_deg": 20, "pitch_deg": 5},
             {"t": 3, "do": "push", "yaw": 1, "pitch": 0},
             {"t": 5, "do": "noise", "deg": 1.0}, {"t": 6, "do": "delay", "steps": 2},
             {"t": 8, "do": "strength", "pct": 70}, {"t": 9, "do": "target", "yaw_deg": -15, "pitch_deg": -5} ] }
```
Lab step kinds: `target` (yaw_deg left +, pitch_deg down +), `push` (yaw/pitch direction, same shove as the panel's
Push button: 0.25 s), `noise` (deg), `delay` (steps 0..4), `strength` (pct 30..100). Shipped scripts:
`push_test`, `topple` (free_hold), `procedural_walk`, `lab_stress`, `lab_calm`.

## Skill tools (service)

- `POST /skills/duel` `{"skill": "look_at", "a": null, "b": 2, "scenario": "lab_stress"}` (a/b = version number,
  null = current) -> `{"ok": true, "result": {"skill", "scenario", "a": Score, "b": Score, "winner": "a|b|tie",
  "why": "..."}}`, Score = `{"version", "err_mean_deg", "err_max_deg", "torque_rms", "jitter", "energy_j",
  "would_stop": "" , "score"}` (lower score wins; weights in the tool). Headless MuJoCo stand scene, same seed for
  both, same disturbances.
- `GET /runs` -> `{"ok": true, "runs": [{"file": "look_at_urlab_20261122_191054.csv", "skill", "target":
  "sim|urlab|real", "stamp"}]}` (logs/*.csv, newest first, max 100).
- `POST /gap` `{"a": file, "b": file, "tol_deg": 5}` -> `{"ok", "joints": [names], "t": [..<=400], "a_deg":
  [[per t per joint]], "b_deg": [...], "err_deg": [...], "max_deg": [per joint], "rms_deg": [per joint], "pass"}`.
  Files are names inside `logs/` only (no paths).
- `GET /skills/montage?skill=look_at` -> `{"ok", "frames", "checkpoints": [iters], "gif_base64"}` or ok false with
  a reason (no checkpoints: train with `--checkpoint-every`).
- `GET /poses` -> `{"ok", "poses": [{"id", "name", "yaw_deg", "pitch_deg", "skill": "pose_<id>"}]}`;
  `POST /poses` `{"name", "yaw_deg", "pitch_deg"}` (name 1..32 chars `[A-Za-z0-9 _-]`, yaw -30..30, pitch -15..20)
  -> `{"ok", "pose"}`; `POST /poses/delete` `{"id"}`. Stored in `RobotControl/configs/taught_poses.json`. Each pose
  is a head skill `pose_<id>` (hold that head angle; head pitch + yaw only), listed by `/skills`, trained by
  `/skills/train`, validated like the others, and loadable by the Unreal lab once its `deploy.json` passed.
- Synthetic camera data (CLI only): `python -m robotcontrol.tools.synth_camera --frames 200 --out datasets/head_cam`
  writes `rgb/00000.png`, `depth/00000.npy` (m), `seg/00000.png` (object id per pixel) and `labels.json`
  (per frame: head yaw/pitch, light, objects with class, colour, bbox [x0,y0,x1,y1]). `datasets/` is git-ignored.

## Python implementation notes (clarifications, 2026-11-22)

- `replay.t` = position inside the replay window (0..`duration_s`, sim seconds); the top-level `t` is the replayed
  frame's sim time. `duration_s` is shorter than requested when less has been recorded. `/walk/state` also lists
  `"modes": ["policy", "hold", "free_hold"]`.
- Walk scenarios may carry `"fallback_mode"` (e.g. `procedural_walk`: `policy`, fallback `hold`): used when the mode is
  `policy` and no validated Level 2 policy exists; the `/walk/scenario` reply then has a `note`.
- `GET /poses` was already the operator console's preset-pose endpoint (`docs/service_api.md` section 5). It now
  returns the presets unchanged followed by the taught poses; a taught entry has `id, name, yaw_deg, pitch_deg, skill`
  plus the preset fields (`label, say, words, q_deg`) so both readers parse it. `POST /poses/preview` also accepts a
  taught pose. A taught pose skill settles at its pose (not at centre) and uses the wide 15 deg / 1 s tracking window
  (it starts with a move); validation bars 1.5 / 2.0 / 2.5 deg, 150 default training iterations.
- `POST /gap` also returns `a, b, tol_deg, overlap_s, worst`; `GET /skills/montage` also `fps`, and on failure both
  `error` and `reason`. `POST /skills/train` takes `"checkpoint_every": n`.
- `train_head_skill.py --scratch DIR` trains + validates into DIR only (no version stored or activated).
- Synthetic camera depth: pixels that hit nothing hold the far-plane distance.
- Procedural course: only low planks (<= 2 cm) lie on the checkpoint path; every other piece keeps 0.35 m from it.
- `sim_features/` and `scenarios/` are left out of the robot kit (simulation only).

