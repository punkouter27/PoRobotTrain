# Robustness explorer

Shows how the deployed Level 0 policy copes when the twin is wrong, before anything runs on the robot. One
command runs the exact `policy.onnx` the robot uses on many perturbed copies of the headless MuJoCo stand twin at
once. Each copy runs the validation sweep (head yaw ±30° at 0.25 Hz, shoulder pitch sweep at 0.15 Hz) and is
scored like the verification gate.

```powershell
.venv\Scripts\python.exe -m robotcontrol.tools.robustness_sweep           # full: 91 twins, 8 s each, ~15 s
.venv\Scripts\python.exe -m robotcontrol.tools.robustness_sweep --quick   # 56 twins, 5 s each, ~8 s
```

The same run is available from both clients: the **Robustness** panel in the Blazor control UI and the
**Robustness** tab in the Unreal twin's panel (service job `robustness`, see `docs/service_api.md` section 2).

## What it varies

| factor | levels (full) | how |
|---|---|---|
| Body mass | 0.7 … 1.3 × | every body's mass and inertia |
| Joint friction + damping | 0.25 … 3 × | `dof_frictionloss` and `dof_damping` of every hinge |
| Motor gains kp/kd | 0.6 … 1.5 × | position-actuator gains (passive legs stay kp 0, kd 3) |
| Action latency | 0 … 80 ms | targets applied late, at 5 ms (physics-step) resolution |
| Sensor noise | 0 … 6 × training | uniform encoder noise, ±0.01 rad and ±0.5 rad/s per unit |
| Torque limit | 0.4 … 1 × | actuator force range |

Sensitivity rows change one factor at a time. The grid crosses gains with latency, the two biggest sim-to-real
unknowns.

## Verdict per cell

| verdict | rule |
|---|---|
| fail | mean tracking error > 3°, or max joint acceleration > 80 rad/s², or a joint more than 1° past its 0.9 soft limit, or non-finite state |
| warn | mean tracking error > 2°, or max joint acceleration > 60 rad/s² |
| pass | otherwise |

Each cell also records the joint with the largest acceleration (`acc_joint`) and the action jitter (mean |Δaction|
per step; the verification gate allows 0.05). Output: `logs/robustness/latest.json`, `report.md`, `heatmap.png`.

## Finding on 2026-11-21: the shipped Level 0 policy chatters on head yaw

In the current calibrated twin, 89 of 91 cells fail, nominal included. The cause is head-yaw acceleration of
about 290 rad/s² and action jitter of 0.25. `sim_calibration.yaml` (added after the policy was trained and
validated) lowered head-yaw armature 12× (0.01 → 0.0008) and also lowered its damping and friction. The policy,
trained on the old values, now drives head yaw into a fast oscillation. Re-running `validate_level0.py` against the
calibrated twin gives the same result: 6 of 14 checks fail (max acceleration 290 rad/s², jitter 0.24, head-yaw mean
error 4.6°). The `logs/level0_validation/report.md` that says ALL CHECKS PASSED predates the calibration.

Only latency 10 ms with gains 0.6–0.7× passes, which points to a timing resonance in the head-yaw loop rather than
a gross model error. Retraining Level 0 against the calibrated twin (the domain randomisation already covers
latency and gains) is the fix; until then, treat Level 0 policy runs on the real head with care. The stand
behaviours (`neck_turn` and friends) are separate controllers and are not affected by this policy.
