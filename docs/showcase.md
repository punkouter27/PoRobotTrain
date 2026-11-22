# Diagnostics overlay, idle demo mode and editor-exposed reward config

Po Game Standard C9 / C10 / C12 / C13. Code: `UnrealProject/Source/R1Twin/R1Showcase.*` and `R1RewardConfig.*` (this repo's own copy).

## Diagnostics overlay (Tab)
`Tab` (or the 44 px "i" button top-right on touch devices; `-ShowcaseTouch` shows it on desktop) toggles an opaque
panel (#121826, text #e6e9f2, ~14:1) with FPS, the current episode reward, inference latency (rolling mean of the last
60 policy steps, ms) and the curriculum stage. `-DiagOverlay` starts with it open.

Game hook: UR1PolicyComponent: the episode's measurable penalty terms (action rate + joint velocity, weighted by UR1RewardConfig) as the reward, NNE inference time per policy step, `L0_full (<model>)` as the curriculum stage. AR1FollowCamera hides the HUD, enables inference and orbits. The command-mode toggle moved from Tab to C.

## Idle demo mode
After `IdleSeconds` (30 s) without keyboard, mouse, touch or gamepad input: the HUD hides, every agent runs on its
trained policy (inference only) and the showcase cameras cycle every `CameraDwellSeconds` (7 s): Front, ThreeQuarter, Side (orbit presets of AR1FollowCamera). Any input
returns control and restores the HUD, mode and camera. A small "Demo - press any key" hint stays at the bottom.

## Editor-first config
- `UR1ShowcaseConfig` (DataAsset): idle threshold, camera list, dwell time, overlay key, latency window, touch button.
- `UR1RewardConfig` (DataAsset): penalty weights `TorqueSqWeight`, `JointAngVelSqWeight`, `ActionRateSqWeight`,
  `FootSlipWeight` and the curriculum stages (name, threshold, metric).
- Pick the assets in Project Settings > Game > R1 Showcase; empty = the C++ defaults.
- The defaults mirror `RobotControl/configs/reward_config.json`, which the Python trainer reads; a pytest (`test_unreal_reward_mirror.py`) fails when the
  two differ. Change both together.

The shipped r1_level0_upperbody policy was trained with the same weights before the config file existed: not retrained. Changing a weight only affects future training runs.

## Unattended check
`UnrealEditor.exe <uproject> -game -windowed -ResX=1600 -ResY=900 -ShowcaseCapture=<dir> -ShowcaseTouch` writes
`overlay_on.png`, `overlay_off.png`, `demo_a.png`, `demo_b.png` into `<dir>` (demo shots after 35 s and one dwell later),
then quits. `-ShowcaseIdle=<s>` overrides the idle threshold.
`-ShowcaseBench` presses the overlay's **Benchmark training speed** button first: it steps a private copy of the live MuJoCo
scene for 1 s on a worker thread and shows physics steps/s and x real time (one training environment's sim throughput on
this PC; 2026-11-22, RTX 5070 Ti laptop: ~20,000 steps/s, ~40x real time). The live simulation and the robot are untouched.
The idle clock ignores frames longer than 0.5 s (shader warm-up, level load), so they never count as idle time.
