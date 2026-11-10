# Step 5: Unreal finalization and runtime profile (Level 0, virtual only)

Measured 2026-11-09 on the RTX 5070 Ti laptop, UE 5.8.4, URLab with MuJoCo 4.0.0, policy `r1_level0_upperbody`.

## What was finalized

| Area | Result |
|---|---|
| Camera | `R1TwinGameMode` spawns `R1FollowCamera` framing the stand; Play shows the robot without ejecting. |
| Rendering | Directional sun, real-time sky light, sky atmosphere, unbound post-process with manual exposure (bias -2.5), 400 m visual floor. MuJoCo still owns all contact. |
| Level setup | Fully scripted and idempotent: `urlab_setup_scene.py`, `ue_finalize_level.py`, `ue_import_policy.py`. Everything is saved. |
| In-engine policy | The NNE (ONNX Runtime CPU) policy drives the twin with no Python, 50 Hz, Level 0 sweep command. |
| Handoff | The in-engine policy releases the actuators while a Python client steps URLab (direct mode) and reclaims them afterwards. |
| Reset | Simulated targets auto-reset to `home` on a safety trip or when the root leaves the workspace (3 m radius, 0.25 m minimum height). Real hardware never auto-resets: it damps and exits. |
| Spawning | No runtime spawning or destruction of gameplay actors. The robot, stand, lights and camera start come from the saved level. |

## Runtime profile

Python control loop, 1000 ticks, budget 20 ms per 50 Hz tick:

| Target | Inference p50 | Tick p50 | Tick p99 | Tick max |
|---|---|---|---|---|
| Headless MuJoCo | 0.04 ms | 0.37 ms | 0.80 ms | 2.8 ms |
| Unreal (URLab direct) | 0.11 ms | 2.3 ms | 9.5 ms | 29 ms |

The Unreal tick is dominated by the physics-step RPC, which URLab serves on the editor game thread, so its tail follows
the editor frame time.

In-engine NNE policy (logged every 5 s by `R1PolicyComponent`):

| Metric | Value |
|---|---|
| Policy rate, editor in foreground | 49.4 to 50.1 Hz |
| NNE inference | 0.13 to 0.14 ms average, 0.42 to 0.84 ms max |
| Heap allocation in the inference step | None by construction: pre-allocated obs/action buffers and tensor bindings, cached component pointers, no string lookups. `GetImuGyro` allocates one small array but Level 0 never calls it. |

Python `OnnxPolicy.infer`: 7 retained blocks (799 bytes) over 2000 calls, i.e. no per-call allocation.

Unreal frame time in Play (12 s capture, `csvprofile`):

| Thread | Average | p95 |
|---|---|---|
| Frame | 33.2 ms (about 30 fps) | 63.7 ms |
| Game | 19.9 ms | 25.2 ms |
| Render | 33.0 ms | 66.4 ms |
| GPU | 26.1 ms | 63.5 ms |

Physics runs on URLab's own thread, so the render rate does not change the simulation. The frame is render-bound;
the robot head camera re-renders the scene every frame, and Lumen runs in the editor viewport.

## Known issues

* **Background throttling.** With the editor not in the foreground the game thread drops to a few frames per second and
  the in-engine policy falls to 7 to 35 Hz. `bThrottleCPUWhenNotForeground=False` is set in the project config but
  did not take effect; set *Editor Preferences, General, Performance, Use Less CPU when in Background* off by hand.
* **Policy is paced by the game thread.** At 30 fps the in-engine policy runs one or two steps per frame against the
  latest physics snapshot. Rates are correct, but for walking (Level 2) the policy should move to a URLab physics-thread
  callback so every step sees fresh state.
* **URLab crash with editor Python during live Play.** Running an editor Python script during live Play crashed URLab's
  telemetry publisher (`UURLabZmqPublishTransport::PostStep`, stale component pointer on the physics thread). Run
  editor scripts only with Play stopped. Report upstream to URLab.
* **Engine assertion when Play starts right after a level edit.** Starting Play seconds after `ue_finalize_level.py`
  re-saved the open level hit a UE 5.8 tick-manager assertion. Starting Play on the saved level works.
* **Head camera frames.** The Python bridge release is older than the plugin, so `set_camera_streaming` is unknown to
  the server and no head-camera frames reach Python yet. Needed for Level 3, not Level 0.
* **URLab HUD panels** cover part of the viewport during Play. They are URLab's debug UI.
