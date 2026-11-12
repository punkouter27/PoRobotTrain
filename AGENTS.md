# Agent guide

PoRobotTrain is an RL-trained Unitree R1 digital twin and sim-to-real pipeline.
Read `README.md` and the relevant procedure in `docs/` before changing the
training, simulator, deployment, or robot-control path. The file
`RobotControl/configs/r1_joint_map.yaml` is the source of truth for joint order,
motor slots, limits, gains, and defaults.

## Project contracts

- Keep training, MuJoCo simulation, Unreal / URLab, and hardware interfaces
  aligned with the same model, timing, joint map, and ONNX deployment contract.
  Do not silently reorder or reinterpret observations, actions, or motor slots.
- MuJoCo / URLab owns simulated physics; Unreal Chaos must not simulate the
  robot. Preserve the documented 5 ms physics step and 50 Hz policy rate.
- Keep the Level 0 scope bounded to the stand-mounted upper-body policy and
  its documented safety checks. Do not start a real-robot session or change
  hardware safety behavior unless explicitly requested.
- Treat `docs/ue5_setup.md` as historical UE 5.7 scaffold guidance. The current UE 5.8
  project and verified runtime reference are documented in `README.md` and `docs/step5_runtime.md`.
- Keep generated models, policies, and external reference checkouts in their
  documented locations. Do not replace or promote a policy without its
  validation gate.

## Workflow

- Preserve existing local work. Do not commit, push, or deploy unless explicitly
  requested.
- Run the smallest relevant Python tests or validation scripts for changed code.
  Report skipped Unreal, device, and hardware gates explicitly; the mock bridge
  is not evidence that URLab drives the robot.
- For UI changes, retain the five HUD anchors and document before/after
  screenshots and an HTML report under `SCREENSHOTS/`.
