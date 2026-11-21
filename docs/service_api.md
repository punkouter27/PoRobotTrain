# Behaviour service API: voice, robustness, walk sandbox, operator console

`python -m robotcontrol.behaviours.service` (127.0.0.1:8765) is the one backend for both clients:

- **Blazor control UI** (`src/`), through the .NET API proxy at `/api/robot/...`
- **Unreal twin** (`AR1BehaviourPanel` and friends), which calls the service directly

Both clients implement the same features against the same endpoints. The existing endpoints (`/status`,
`/preview`, `/check`, `/run`, `/stop`, `/monitor`, `/snapshot`, `/move_joint`, `/robot_view.png`) are unchanged.
All bodies are JSON. Errors come back as `{"ok": false, "error": "..."}` (HTTP 400/404/409).

| Feature | Service | .NET API (Blazor) |
|---|---|---|
| Voice intent | `POST /assistant/interpret` | `POST /api/robot/assistant/interpret` |
| Robustness report | `GET /robustness` | `GET /api/robot/robustness` |
| Robustness run | `POST /robustness/run` | `POST /api/robot/robustness/run` |
| Walk state | `GET /walk/state` | `GET /api/robot/walk/state` |
| Walk start / stop / reset | `POST /walk/start`, `/walk/stop`, `/walk/reset` | `POST /api/robot/walk/start` ... |
| Walk command | `POST /walk/command` | `POST /api/robot/walk/command` |

---

## 1. Voice commands + spoken replies (idea #11)

Speech to text and text to speech run in the client:

| | Speech to text | Text to speech |
|---|---|---|
| Blazor | Web Speech API `SpeechRecognition` (Chrome/Edge), fallback Whisper `Xenova/whisper-tiny.en` via Transformers.js (local) | `speechSynthesis` |
| Unreal | Windows SAPI in-process recogniser, dictation grammar, default microphone | Windows SAPI `ISpVoice` (async) |

The recognised text is interpreted into a **suggestion**. Voice never starts, checks or runs a robot behaviour;
the client fills in the selection exactly as a click would, and Steps 1 + 2 + the confirm dialog still gate any run.
The one exception is the walk sandbox (simulation only): a walk intent sets the sandbox velocity command directly.

### `POST /assistant/interpret`

Request: `{"text": "nod the head twenty degrees", "arm_unlocked": false, "walk_active": false}`

Response (always HTTP 200):

```json
{
  "ok": true,
  "kind": "behaviour",            // "behaviour" | "joint" | "walk" | "unknown"
  "id": "neck_nod",               // behaviour id (kind=behaviour)
  "label": "Neck Nod",
  "amplitude": 20.0,              // snapped to 10/20/30 and the behaviour's max, or null
  "joint": null,                  // joint name (kind=joint)
  "deg": null,                    // clamped slider target (kind=joint), or null
  "walk": null,                   // kind=walk: {"vx": 0.5, "vy": 0.0, "wz": 0.0} or {"stop": true}
  "blocked": false,               // arm behaviour while arm motions are locked
  "score": 0.82,
  "note": "Neck Nod at 20°. Still needs Step 1 (virtual) and Step 2 (check) before any run.",
  "say": "Neck nod, twenty degrees, selected. Run it on the virtual robot first."
}
```

`note` is what the panel shows. `say` is what the client speaks: short and spoken-friendly, no symbols.
The service matcher (`robotcontrol/behaviours/voice_intent.py`) is lexical and deterministic. The Blazor UI keeps
its MiniLM matcher (`assistant.js`) and fills in the same fields, including `say` and `walk`, with the same templates.

Spoken reply templates:

| kind | `say` |
|---|---|
| behaviour | `"{label}, {amplitude in words} degrees, selected. Run it on the virtual robot first."` (no amplitude: drop that part) |
| behaviour, blocked | `"{label} needs arm motions unlocked first."` |
| joint | `"Use the {label} slider."` / stand-locked: `"{label} is locked on the stand."` |
| walk | `"Walking forward."`, `"Turning left."`, `"Stopping."` and so on |
| unknown | `"Sorry, I did not catch a safe motion. Try: turn the head twenty degrees."` |

Walk phrases (only when `walk_active`): forward / back(ward) / left / right (sidestep) / turn left / turn right /
faster / slower / stop. Velocity steps: vx ±0.5 m/s (faster: 1.0), vy ±0.3 m/s, wz ±0.6 rad/s.

---

## 2. Robustness explorer (idea #19)

`python -m robotcontrol.tools.robustness_sweep [--quick]` runs the deployed Level 0 `policy.onnx` (onnxruntime,
batched) on many perturbed copies of the headless MuJoCo stand twin at once. Every copy runs the validation sweep
command (head yaw ±30° @ 0.25 Hz, shoulder pitch sweep @ 0.15 Hz), and the sweep records tracking error,
acceleration and safety trips. Output goes to `logs/robustness/`: `latest.json` (schema below), `report.md` and
`heatmap.png`.

### `GET /robustness`

`{"available": false}` when no sweep has run yet, else the content of `logs/robustness/latest.json`:

```json
{
  "available": true,
  "created_at": "2026-11-21T10:00:00Z",
  "policy": "RobotControl/models/r1_level0_upperbody/policy.onnx",
  "policy_sha256": "…",
  "quick": false,
  "duration_s": 54.2,
  "rollout_s": 8.0,
  "thresholds": {"mean_err_deg": 3.0, "max_acc": 80.0, "warn_mean_err_deg": 2.0, "warn_acc": 60.0},
  "num_cases": 91,
  "nominal": {"mean_err_deg": 0.9, "max_err_deg": 3.1, "max_acc": 22.0, "acc_joint": "head_yaw_joint",
              "jitter": 0.01, "trips": 0, "finite": true, "verdict": "pass"},
  "sensitivity": [
    {"id": "mass", "label": "Body mass", "unit": "×", "nominal": 1.0,
     "levels": [0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3],
     "cells": [{"mean_err_deg": 1.1, "max_err_deg": 3.0, "max_acc": 25.0, "trips": 0, "verdict": "pass"}, "…"]}
  ],
  "interaction": {
    "x": {"id": "gain", "label": "Motor gains kp/kd", "unit": "×", "levels": [0.6, 0.7, 0.85, 1.0, 1.15, 1.3, 1.5]},
    "y": {"id": "latency_ms", "label": "Action latency", "unit": "ms", "levels": [0, 10, 20, 30, 40, 60, 80]},
    "cells": [["…one cell per x level…"], "…one row per y level…"]
  },
  "summary": {"pass": 80, "warn": 7, "fail": 4, "worst": "latency_ms=80 ms, gain=0.6×: mean error 6.2°"}
}
```

Sensitivity factors (one at a time, everything else nominal), in this order: `mass` (×), `friction` (joint
frictionloss + damping ×), `gain` (kp/kd ×), `latency_ms`, `noise` (sensor noise × the training noise),
`torque` (actuator torque limit ×). Quick mode runs 5 levels per factor and a 5×5 interaction grid.

Every cell has the fields shown for `nominal`: `acc_joint` is the joint with the largest acceleration and `jitter` is the mean |Δaction| per step (informational; the Level 0 gate allows 0.05). A non-finite cell has `finite: false` and null metrics.

Cell verdict: `fail` if mean error > 3° or max acceleration > 80 rad/s² or trips > 0 or non-finite state;
`warn` if mean error > 2° or max acceleration > 60 rad/s²; else `pass`. Status colours (good / warning / critical)
always come with the verdict word or icon.

### `POST /robustness/run`

Body `{"quick": true}`. Starts the sweep as service job `robustness`; refused (409) while another job runs.
While it runs, `/status` shows `job: "robustness"` and `"progress": {"job": "robustness", "done": k, "total": n}`
(null when idle; parsed from the tool's `@progress k/n` lines, which are kept out of `lines`). When it finishes,
`GET /robustness` returns the new report. A full sweep (91 twins) takes about 15 s, a quick one (56) about 8 s.

---

## 3. Level 2 walk sandbox (idea #20), simulation only

The service runs a headless MuJoCo ground scene (`UnrealProject/MJCF/scene_ground.xml` plus the course boxes from
`robotcontrol/walk/course.py`, added at load; calibrated joint dynamics; 5 ms physics, 50 Hz policy) with the Level 2 velocity policy from `RobotControl/models/r1_level2_velocity/`. That folder holds a
policy only after `RobotControl/training/validate_level2.py` passed. Walking never targets the real robot: the
sandbox has no real backend, and the robot kit does not ship Level 2.

Policy contract (Unitree mjlab `r1_velocity`): obs 83 = `base_ang_vel(3)`, `projected_gravity(3)`,
`velocity_commands(3)`, `gait_phase(2)`, `joint_pos_rel(24)`, `joint_vel_rel(24)`, `last_action(24)`.
Actions are 24, in joint-map `policy_index` 0–23 order (legs, waist, arms; the head is held at default).
Exact term definitions and scales come from the policy's `deploy.yaml`.

Unreal shows the sandbox in URLab **puppet mode** (the service pushes the full qpos, root pose included, every
policy step, the same way as the live mirror, through `UnrealMirror.push_full(root7, q26)`; it retries every 3 s
while Unreal is not reachable, and `puppet.connected` reports it). Unreal never simulates the robot. The Blazor twin polls `/walk/state`.

### `GET /walk/state`

```json
{
  "ok": true,
  "policy_available": true,
  "policy_reason": null,          // e.g. "no validated Level 2 policy in RobotControl/models/r1_level2_velocity"
  "running": true,
  "mode": "policy",               // "policy" | "hold"
  "course": "loop",               // "flat" | "loop"
  "t": 12.34,                     // sim time, s
  "cmd": [0.5, 0.0, 0.0],         // applied velocity command vx, vy (m/s), wz (rad/s), after clamping
  "cmd_limits": {"vx": [-0.5, 1.0], "vy": [-0.5, 0.5], "wz": [-0.5, 0.5]},   // from the policy's deploy.yaml
  "base": {"pos": [1.2, 0.1, 0.78], "quat": [1, 0, 0, 0], "lin_vel": [0.48, 0.0, 0.0], "yaw": 0.02},
  "q": [/* 26 joint angles, rad, joint-map order (head included) */],
  "joint_names": ["left_hip_pitch_joint", "…26 names…"],
  "fallen": false,
  "falls": 0,
  "checkpoint": {"index": 2, "total": 6, "next": [4.0, 2.0, 0.0]},
  "lap": {"current_s": 8.1, "last_s": 31.2, "best_s": 29.7, "laps": 2},
  "geometry": {                   // static per course; clients build the course from this
    "checkpoints": [[3.0, 0.0, 0.0], "…"],
    "checkpoint_radius": 0.5,
    "boxes": [{"name": "ramp", "pos": [6, 0, 0.05], "size": [1.0, 0.6, 0.05], "quat": [0.999, 0, 0.035, 0], "rgba": [0.4, 0.45, 0.55, 1]}]
  },
  "puppet": {"connected": true}   // whether the Unreal twin is receiving the poses
}
```

Before Start (`running: false`) the state still carries `policy_available`, `policy_reason`, `courses`, `course`,
`geometry` and `mode: null`, so clients can draw the course; the live fields appear once it runs.

MuJoCo world frame, metres, z up, quaternions `(w, x, y, z)`, box `size` = half extents (MuJoCo convention).
Unreal converts with `MjUeConvert.h` (`pos_ue = (x, -y, z) * 100`).

### Commands

| endpoint | body | effect |
|---|---|---|
| `POST /walk/start` | `{"course": "loop", "mode": "policy"}` | load the policy and scene, reset at the start, start stepping in real time; 409 if another job runs, 400 + `policy_reason` if no validated policy. `"mode": "hold"` always works: no policy, every joint is PD-held at its default pose and the pelvis hangs in a soft *virtual harness* (a weld, because the R1 cannot balance passively and would tip over in about 1 s). It tests the scene, rendering and puppet path; velocity commands are ignored and `cmd_limits` are all `[0, 0]`. `/walk/state` then reports `"mode": "hold"`, and clients must label it "Hold pose: no walking policy (held by a virtual harness)" |
| `POST /walk/command` | `{"vx": 0.5, "vy": 0, "wz": 0}` | set the velocity command (clamped to `cmd_limits`). **Watchdog:** with no command for 0.5 s the command decays to zero, so a lost client stops the robot |
| `POST /walk/reset` | `{}` | back to the start pose, lap timer reset |
| `POST /walk/stop` | `{}` | stop stepping and release URLab puppet mode |

Clients send `/walk/command` at about 20 Hz while a key or stick is held, and once with zeros on release. Falls
(pelvis below 0.45 m or tilted more than 60°) count up `falls` and auto-reset after 1 s.

---

## Operator console (sections 4-7)

Twenty Explore-style operator features, served by `robotcontrol/behaviours/console.py` (with `telemetry_log.py`,
`motions.py`, `ops.py`, `chat.py`). The .NET API forwards every path below unchanged under `/api/robot/...`
(for example `GET /api/robot/health`, `POST /api/robot/motions/play`), query string and body included.

Safety contract, which both clients must keep:

- Read-only features (health, events, history, power, IMU, camera, versions, network, docs, chat) never command a motor.
- Motions, poses and modes power several joints at once, so they run **only** on the simulated stand twin: target
  `practice` (practice twin), `sim` (headless, shown as the virtual robot) or `urlab` (the same, mirrored onto
  Unreal in puppet mode). Target `real` is refused with HTTP 400 and a reason. Never offer it as a choice.
- The pre-flight checklist and the session limits only add restrictions to the robot gates; they never remove one.
- `POST /estop` is a software stop. The remote (L2+B for 5 s) stays the real emergency stop; clients say so next to the button.

`GET /status` gains a `console` object, so clients can show these without extra polling:

```json
"console": {
  "limits": {"amplitude_cap_deg": 30.0, "freq_cap_hz": 0.5, "joint_range_frac": 0.9, "walk_speed_scale": 1.0, "motion_speed_scale": 1.0},
  "limits_tightened": [],
  "event_counts": {"info": 3, "warn": 1, "critical": 0},       // last 10 min
  "recording": {"active": false, "source": null, "samples": 0, "elapsed_s": 0.0},
  "player": {"running": false, "kind": null, "name": null, "t": null},
  "preflight": {"complete": false, "open": 3, "reason": "3 item(s) to tick"}  // manual ticks, then cached checks
}
```

`/run` and `/move_joint` (target `real`) now also need a complete pre-flight checklist; the refusal reason is
`"complete the pre-flight checklist (N item(s) open)"`. Practice runs do not need it. `/preview`, `/check` and
`/run` refuse params above the session caps (HTTP 400, reason names the cap). `/move_joint` clamps its target to
the session's joint range fraction. `/walk/command` is scaled by `walk_speed_scale`.

| # | Feature | Endpoints |
|---|---|---|
| 1 | Health summary | `GET /health` |
| 2 | Alarm and fault log | `GET /events?level=&kind=&limit=&since=` |
| 3 | Battery and power | `GET /power?source=` |
| 4 | Temperature trends | `GET /history/temps?source=&window=` |
| 5 | Camera | `GET /camera.png?source=twin&mode=virtual` |
| 6 | Versions | `GET /versions` |
| 7 | Modes | `POST /mode` |
| 8 | E-stop | `POST /estop` |
| 9 | Joystick | (walk sandbox, section 3; touch thumbsticks in the clients) |
| 10 | Session limits | `GET /limits`, `POST /limits`, `POST /limits/reset` |
| 11 | Poses | `GET /poses`, `POST /poses/preview` |
| 12 | Record and replay | `POST /motions/record/start`, `POST /motions/record/stop`, `GET /motions/record`, `POST /motions/play`, `GET /player`, `POST /player/stop` |
| 13 | Timeline editor | `GET /motions/{name}`, `POST /motions/check`, `POST /motions/save`, `POST /motions/delete` |
| 14 | Local action store | `GET /motions`, `GET /motions/{name}/export`, `POST /motions/import` |
| 15 | Calibration wizard | `POST /calibration/capture`, `POST /calibration/save`, `GET /calibration/offsets` |
| 16 | IMU check | `GET /imu?source=&window=` |
| 17 | Network | `GET /network`, `POST /network/test`, `POST /network/profiles`, `POST /network/profiles/delete`, `POST /network/active` |
| 18 | Pre-flight checklist | `GET /preflight`, `POST /preflight`, `POST /preflight/reset` |
| 19 | Docs | `GET /docs`, `GET /docs/{name}` |
| 20 | Chat assistant | `POST /assistant/chat`, `GET /assistant/chat/history?session=`, `POST /assistant/chat/reset` |

`source` is `real`, `practice` or `virtual`. When it is left out, the first one with data is used, in that order.

---

## 4. Monitoring (read-only)

### `GET /health`

Probes run in parallel, each with a short timeout (the whole call takes well under 1 s).

```json
{"ok": true, "overall": "ok", "failing": 0, "checked_at": 1795274707.2,
 "rows": [{"section": "Services", "name": "Unreal twin (URLab)", "ok": false, "optional": true, "ms": 2.1,
           "detail": "127.0.0.1:5555 ConnectionRefusedError"}]}
```

Sections: `Services` (service, URLab, robot SSH, robot state stream), `Data` (joint map, poses, motions, robustness
report, Level 2 policy, docs, one row per policy with its validation gate), `Robot` (critical events in the last
10 min, motor temperatures, battery), `Jobs`. `overall` is `degraded` if a non-optional row fails. Show optional
failures as OFF, not FAIL.

### `GET /events`

`{"ok": true, "events": [{"id": 7, "at": 1795274707.1, "level": "critical", "kind": "estop", "message": "E-STOP pressed: stopped player", "source": "practice", "joint": "head_yaw"}], "counts": {"info": 3, "warn": 1, "critical": 1}}`

Newest first. `level` filters at that level and above (`info` < `warn` < `critical`). Kinds: `temperature`
(55 C warn, 65 C critical), `limit` (within 2 deg of a joint limit), `power` (cell voltage under 3.5 V), `job`,
`fault` (a job failed), `trip` (a safety abort), `estop`, `mode`, `limits`, `preflight`, `calibration`. Alarms
are de-duplicated per joint and source (60 s). The service also appends them to `logs/events/events.jsonl`.
Clients export the filtered list as CSV.

### `GET /history/temps`

1 Hz samples over the last `window` seconds (at most 600):

```json
{"ok": true, "source": "real", "warn_c": 55, "crit_c": 65, "joint_names": ["left_hip_pitch", "…26"],
 "t": [-59.0, -58.0, "…"],                // seconds relative to now
 "series": [[38, 38, "…"], "…26 rows"],   // per joint, aligned with t
 "latest": [38, "…"], "peak": [40, "…"], "rate_c_per_min": [0.2, "…"],
 "hottest": {"joint": "head_yaw", "temp_c": 41}}
```

`latest`, `peak`, `rate_c_per_min` and `hottest` are present only when there are samples.

### `GET /power`

```json
{"ok": true, "source": "real", "available": true, "latest_v": 51.6, "cell_v": 3.97, "soc_pct": 77.0,
 "trend_v_per_min": -0.02, "minutes_left": 140.0, "low": false, "series_cells": 13,
 "assumption": "state of charge estimated from bus voltage, generic Li-ion curve, 13S pack",
 "current_a": null, "current_note": "the low-level state stream does not report pack current",
 "t": [-120.0, "…"], "bus_v": [51.9, "…"]}
```

`soc_pct` is an **estimate**: label it so and show `assumption`. The cell count comes from
`RobotControl/configs/battery.json`. `minutes_left` needs at least 60 s of falling voltage, otherwise it is `null`.

### `GET /imu`

```json
{"ok": true, "source": "real", "available": true, "verdict": "pass", "samples": 240, "window_s": 59.8,
 "rpy_deg": [0.1, -0.2, 12.0], "gravity_body": [0.0035, 0.0017, 1.0], "tilt_deg": 0.22,
 "roll_std_deg": 0.04, "pitch_std_deg": 0.05, "yaw_drift_deg_per_min": 0.3, "still": true, "notes": []}
```

`verdict`: `pass`, `warn` (moved during the window, yaw drift over 1 deg/min while still, or tilt over 10 deg) or
`no_data` (`available: false`). `gravity_body` is the unit "up" vector in the body frame.

### `GET /camera.png`

`source=twin` (default): PNG from the twin's `head_rgbd` camera (the R1 head camera position) at the current pose,
`mode=virtual|real` picking the virtual or measured pose. Clients refresh it about 4 times a second.
`source=real`: HTTP 503 with a JSON reason. The repo has no video streamer on the robot yet (planned with Level 3).

### `GET /versions`

```json
{"ok": true, "service": "2026.11.21", "repo": {"commit": "c719161", "dirty_files": 12}, "python": "3.11.9",
 "packages": {"mujoco": "4.0.0", "onnxruntime": "1.22.0", "numpy": "2.2.6", "unitree_sdk2py": "not installed"},
 "robot": {"firmware": null, "note": "the low-level state stream does not report firmware; read it on the robot"},
 "policies": [{"name": "r1_level0_upperbody", "policy_sha256": "…64 hex", "expected_sha256": null,
               "gate": "validated (Level 0 gate report)", "ok": true, "trained_at": "2026-11-09T15:18:27",
               "num_obs": 23, "num_actions": 7, "joint_map_sha256": "df0a086ae8017262", "joint_map_matches": true}],
 "warnings": []}
```

`gate` compares `deploy.yaml` `validation.policy_sha256` with the file (`MISMATCH: ...` when the ONNX changed after
validation). Level 0 has no hash, so it reports its gate report instead. Show `warnings` prominently.

---

## 5. Control and motions (simulation only)

### `POST /estop`

Body `{}`. Stops the running job, the simulation player, the walk sandbox and any recording, and logs a
critical `estop` event. Response: `{"ok": true, "stopped": ["player", "walk"], "note": "software stop sent; ..."}`.
Clients show a STOP button on every page/tab, also bound to `Esc` (Blazor) or `End` (Unreal; Esc quits PIE).

### Session limits

`GET /limits` and every `POST` reply:

```json
{"ok": true, "values": {"amplitude_cap_deg": 30.0, "freq_cap_hz": 0.5, "joint_range_frac": 0.9, "walk_speed_scale": 1.0, "motion_speed_scale": 1.0},
 "specs": {"amplitude_cap_deg": {"max": 30.0, "min": 1.0, "unit": "deg", "label": "Behaviour sweep size cap"}, "…": {}},
 "tightened": ["walk_speed_scale"], "changed_at": 1795274707.2}
```

`POST /limits {"values": {"walk_speed_scale": 0.5}}` sets any subset. A value outside `[min, max]` gets HTTP 400,
and `max` is the built-in limit, so a cap can only tighten. `POST /limits/reset {}` restores the defaults.
Caps last for this service session only.

### `POST /mode`

`{"mode": "damp" | "zero_torque" | "hold" | "off", "target": "practice" | "sim" | "urlab", "joints": ["right_elbow_joint"], "max_s": 120}`

- `damp`: the named joints are velocity-damped only (kp 0), so they are back-drivable.
- `hold`: the named joints are PD-held at their current angle.
- `zero_torque`: nothing is powered (the stand twin locks the waist and head pitch the way the gearboxes do).
- `off`: ends the running mode.

`damp` and `hold` need `joints`, because only the named joints get stiffness. The mode runs as job `mode` until
`off`, STOP or `max_s`. `target: "real"` gets HTTP 400.

### Poses

`GET /poses` returns `{"ok": true, "poses": [{"name": "look_left", "label": "Look left", "say": "", "words": [], "q_deg": {"head_yaw_joint": 30.0, "head_pitch_joint": 0.0}}]}`
(from `RobotControl/configs/poses.json`; `home` = the joint map defaults for the waist, arms and head).
`POST /poses/preview {"name": "look_left", "target": "practice"}` moves there smoothly as job `pose`
(HTTP 404 for an unknown pose).

### Motions

Clip format (`RobotControl/motions/*.json` built-in, read-only; the operator's clips in the git-ignored
`RobotControl/user_data/motions/`):

```json
{"name": "wave_hello", "title": "Wave hello", "description": "…", "author": "…", "tags": ["arm"], "loop": false,
 "keyframes": [{"t": 0.0, "q_deg": {"right_elbow_joint": 90.0}, "ease": "smooth"}, {"t": 0.6, "q_deg": {"right_elbow_joint": 60.0}}]}
```

A joint that a keyframe leaves out keeps its previous value. Joints that no keyframe names are not powered.
`ease` is `smooth` (default) or `linear`. Playback starts with a lead-in from the current pose (at least 0.8 s,
and slow enough for half the speed limit).

Validation (`validate_clip`, also run by save, play and import) checks the following. On failure the request gets
HTTP 400 with `{"error": "motion failed validation", "validation": {...}}`.

- name `^[a-z0-9][a-z0-9_]{0,39}$`
- 1-200 keyframes, times increasing, 120 s maximum
- known joints
- every angle inside the soft range (the session's `joint_range_frac` of the joint range, 0.9 by default)
- joint speed between keyframes (smoothstep peak = 1.5 x mean) at most 90 deg/s for the head and legs, 120 deg/s for the arms and 45 deg/s for the waist

```json
"validation": {"ok": false, "duration_s": 5.0, "joints": ["right_elbow_joint"], "max_vel_deg_s": {"right_elbow_joint": 75.0},
               "issues": [{"level": "error", "message": "…", "keyframe": 2, "joint": "right_elbow_joint"}],
               "real_eligible": false, "real_reasons": ["multi-joint playback on the real robot is not enabled (simulation and Unreal only)"]}
```

| endpoint | body / reply |
|---|---|
| `GET /motions` | `{"motions": [{"name", "title", "description", "author", "tags", "loop", "source": "builtin"\|"user", "keyframes": 8, "duration_s", "joints", "valid", "issues": [...], "sha256"}]}` |
| `GET /motions/{name}` | `{"clip": {...}, "validation": {...}}` (404 unknown) |
| `POST /motions/check` | `{"clip": {...}}` or `{"name": "..."}`, reply `{"ok": true, "validation": {...}}` |
| `POST /motions/save` | `{"clip": {...}, "overwrite": false}`. 409 if the name exists (or is built-in); 400 + `validation` if invalid |
| `POST /motions/delete` | `{"name": "..."}` (user clips only; 409 for built-ins) |
| `POST /motions/play` | `{"name": "..."}` or `{"clip": {...}}`, plus `"target": "practice"`, `"loop": false`. Job `motion`; 409 busy |
| `GET /player`, `POST /player/stop` | `{"running", "kind": "clip"\|"mode", "job", "name", "t", "duration_s", "loop", "mode", "joints", "ok", "reason", "recording": {...}}` |
| `POST /motions/record/start` | `{"source": "real"\|"practice"\|"virtual"}`. Teach mode: records that source's joint angles (read-only). On the real robot, run Monitor and move a limp arm by hand; on the twin, use the joint sliders |
| `POST /motions/record/stop` | `{"name": "my_take"}` returns `{"clip": {...}, "validation": {...}}`, keyframes every 0.5 s over the joints that moved at least 1 deg. **Not saved** until `/motions/save` |
| `GET /motions/{name}/export` | store package `{"format": "porobottrain.motion/1", "exported_at", "sha256", "validation": {"ok", "duration_s", "joints"}, "clip": {...}}` |
| `POST /motions/import` | `{"package": {...}, "overwrite": false}`: checks the format and that `sha256` matches the clip (name, keyframes, loop), then saves like `/motions/save` |

---

## 6. Setup

### Calibration wizard (zero offsets)

`POST /calibration/capture {"source": "real", "reference": "zero" | "home", "joints": ["head_yaw_joint"]}` reads
the source's latest joint angles. They must be less than 30 s old (take a Snapshot first; otherwise HTTP 409).
`zero` expects the joints at their zero marks; `home` expects the stand home pose. The default joints are the
waist, arms and head:

```json
{"ok": true, "source": "real", "reference": "zero", "captured_at": 1795274707.2,
 "rows": [{"joint": "head_yaw_joint", "measured_deg": 0.8, "reference_deg": 0.0, "offset_deg": 0.8, "verdict": "ok", "twin_offset_deg": 0.0}]}
```

The verdict is `ok` up to 1 deg, `warn` up to 3 deg and `check` above that. `twin_offset_deg` is the offset the
simulator's calibration assumes. `POST /calibration/save {"confirm": true, "offsets": {...optional edits...}}`
stores the offsets in `RobotControl/user_data/joint_zero_offsets.json` with `"applied": false`. Offsets are
**never applied** to a controller: that needs a validation session. An offset over 15 deg is refused.
`GET /calibration/offsets` returns `{"saved": {...} | null, "pending": {...} | null}`.

### Network

`GET /network` returns `{"interfaces": [{"name": "Ethernet 2", "ipv4": ["192.168.123.99"]}], "profiles": [{"name", "target", "host", "port", "interface", "note"}], "active": "sim"}`.
The defaults are `sim`, `urlab` and `real` (192.168.123.164:22). Saved profiles live in the git-ignored
`RobotControl/user_data/connection_profiles.json`. `POST /network/test {"host", "port", "icmp": true}` returns
`{"reachable", "tcp_ok", "tcp_ms", "ping_ok", "ping_ms", "detail"}`. `POST /network/profiles {"profile": {...}}`
adds or replaces a profile. `/network/profiles/delete {"name"}` and `/network/active {"name"}` remove one or
choose the active one. A profile is information only (which interface to pass to `--network`). It does not
reconfigure the service.

### Pre-flight checklist

`GET /preflight`:

```json
{"ok": true, "complete": false, "valid_for_s": null, "missing": ["Robot hangs on its stand, feet clear of the floor"],
 "items": [{"id": "on_stand", "label": "…", "kind": "manual", "ok": false, "detail": ""},
           {"id": "robot_reachable", "label": "Robot reachable (192.168.123.164:22)", "kind": "auto", "ok": false, "detail": "…"}]}
```

Manual items: `on_stand`, `estop_tested`, `area_clear`, `head_forward`, `battery_checked`. Each tick lasts
30 min. Automatic items:

- the Level 0 validation report passed
- the policy matches the joint map
- the robot is reachable
- robot state was read in the last 60 s
- motor temperatures are under 55 C
- there were no critical events in the last 10 min

`POST /preflight {"item": "on_stand", "checked": true}` and `POST /preflight/reset {}` return the same shape.

---

## 7. Help and assistant

`GET /docs` returns `{"docs": [{"name": "service_api", "title": "…", "bytes": 1234}]}` (the `docs/*.md` files).
`GET /docs/{name}` returns `{"name", "title", "markdown"}`. Names match `^[a-z0-9_]{1,60}$`; anything else gets
400 or 404. The clients open the document for the current page/tab:

| page | doc |
|---|---|
| Control | `behaviours` |
| Walk | `level2_walk` |
| Robustness | `robustness` |
| Health / Setup | `robot_connection_guide` |
| Motions | `service_api` |
| Real-robot steps | `sim_to_real_level0` |

`POST /assistant/chat {"text": "how hot are the motors?", "session": "blazor-1", "arm_unlocked": false, "walk_active": false}`
returns a **superset of `/assistant/interpret`** plus `answer`, `pose`, `session` and `turns`:

- `kind: "answer"`: a status answer (battery, temperatures, faults, what is running, robot connection, pre-flight,
  limits, versions, poses, motions, IMU, walk). Show `answer` and speak `say`.
- `kind: "pose"`: `pose` names a pose. Offer "Preview pose" (target practice or urlab); never start it unasked.
- `behaviour` / `joint` / `walk` / `unknown`: exactly as `/assistant/interpret`. Follow-ups use the session
  memory (40 turns): `again`, `bigger`, `smaller`, `what did I ask?`.

It stays suggest-only and runs locally (deterministic, no text leaves the machine).
`GET /assistant/chat/history?session=` returns `{"turns": [{"role": "user"|"assistant", "text", "kind", "at"}]}`.
`POST /assistant/chat/reset {"session"}` clears it.
