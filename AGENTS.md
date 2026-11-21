# Agent guide — PoRobotTrain

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
- Level 2 walking exists only in the simulated walk sandbox (`RobotControl/walk/`, `docs/level2_walk.md`). It must
  never reach the robot: `robot_kit.py` excludes `walk/`, and the sandbox has no robot backend. A walking policy is
  loaded only from `RobotControl/models/r1_level2_velocity/` after `robotcontrol.training.validate_level2 --promote`
  passed (deploy.yaml `validation.policy_sha256` must match the ONNX). Training runs in Docker (`scripts/level2/`).
- Operator console (`RobotControl/behaviours/console.py`, `docs/service_api.md` sections 4-7): poses, motions and
  modes power several joints, so they stay simulation-only (the service refuses target `real`); the pre-flight
  checklist and session limits may only add restrictions; zero offsets are stored, never applied to a controller.
- New service endpoints (voice intent, robustness sweep, walk sandbox) are specified in `docs/service_api.md`; the
  Blazor UI and the Unreal panel implement them at parity, so change both when the contract changes.
- Reward penalty weights (torque², joint angular velocity², action rate, foot slip) and the curriculum
  thresholds live in `RobotControl/configs/reward_config.json`; `Level0Config` reads them and the Unreal
  DataAsset mirrors them (a pytest checks parity). Level 0 hangs on a stand, so `foot_slip` is inactive there.
  The shipped Level 0 policy predates the file; changing a weight does not retrain it.

## Workflow

- Preserve existing local work. Do not commit, push, or deploy unless explicitly
  requested.
- Checks before a push: `ruff check .`, `ruff format --check .`, `pytest` (repo root; collects
  `RobotControl/tests`), the `R1TwinEditor` build, and `python web/build.py`. Run the smallest relevant
  validation scripts for changed code.
  Report skipped Unreal, device, and hardware gates explicitly; the mock bridge
  is not evidence that URLab drives the robot.
- For UI changes, retain the five HUD anchors and document before/after
  screenshots and an HTML report under `SCREENSHOTS/`.

## CI/CD and Azure

- `.github/workflows/deploy.yml` (the only workflow): on push to `master` it runs the Python checks (ruff, pytest),
  builds the showcase site (`web/build.py`), deploys it to the Azure Static Web App `swa-porobottrain`
  (resource group `PoShared`, secret `AZURE_STATIC_WEB_APPS_API_TOKEN`) and checks `/` and `/diag` live.
- The Unreal twin is built and tested locally (GitHub's runners have no Unreal Engine). It compiles from a fresh
  clone because URLab is vendored in `UnrealProject/Plugins/`.

## Po Game Standard (Unity / Unreal RL games)

This repo is one of several independent Po games. It follows the conventions below, but it owns every file:
nothing is generated from, synced with, or required to stay identical to another repository (no "same file in
every game" headers, no sync scripts that copy into other repos). Third-party code it needs (ML-Agents, MuJoCo
plugins, URLab) is vendored INSIDE this repo at a pinned version. Azure resources may be shared through `PoShared`.

### Repo skeleton
```
README.md   AGENTS.md (this standard + repo notes)   CLAUDE.md ("Read AGENTS.md.")
docs/       lowercase, all prose (no DOCS/, no root DESIGN.md/SPEC.md/tasks.md — move them into docs/)
scripts/    build/publish/helper scripts (Unity: Tools/ is renamed scripts/ unless Unity itself needs it)
.github/workflows/<one file>.yml   the ONLY workflow
pyproject.toml with a [tool.ruff] section for any Python in the repo
```
No absolute local paths (C:\Users\...) committed anywhere. No build outputs or node_modules tracked.

### Web page around the game (WebGL template / web viewer)
- The page chrome is drawn ONCE, by the HTML page, OUTSIDE the game canvas, so it can never overlap the HUD:
  a 44px top bar (game name top-left, settings gear / fullscreen top-right) and the version `vYYMMDD` bottom-right
  in the bar's counterpart or a 20px footer strip. The canvas fills the remaining viewport.
- The in-game HUD does NOT repeat the game name or version (it may show them inside menus, not as corner chrome).
- Same tokens as the web apps: dark background `#0b0f17`, surface `#121826`, text `#e6e9f2`, muted `#9aa3b5`,
  accent `#6366f1`, Inter/system-ui font. Text ≥ 4.5:1 contrast.

### Orientation (C11)
The canvas fills any viewport (no fixed portrait aspect, no pillarboxing). UI is anchored and DPI-scaled
(Unity: CanvasScaler "Scale With Screen Size" or UI Toolkit PanelSettings scale-with-screen; Unreal: UMG anchors +
DPI curve). Must be checked at 1600x900 (landscape) and 450x800 (portrait) — screenshots in the PR/commit notes.

### Version
One stamp, UTC `yyMMdd`, computed once by the publish/build script and injected into: index.html, diag.html,
and the player (`PlayerSettings.bundleVersion` / Unreal ProjectVersion) so `Application.version` matches the page.

### /diag (games)
`/diag` (static `diag.html` generated by the publish script) titled **System status**, same layout as the web apps:
overall banner, then **API / Data / Services** sections. It must LIVE-probe (fetch from the browser, with latency)
every external dependency the game really calls (leaderboard tables/functions, weather APIs, AI APIs, replay bundles,
model files), plus the WebGL2 capability and that the build bundle files exist. Nothing hard-coded as "Ready".
The checks are listed in the repo's own config (e.g. `scripts/webgl.json` → `diagChecks`).

### In-game diagnostics overlay (C12)
`Tab` (and a 44px corner button on touch devices) toggles an overlay showing: FPS, current episode reward,
inference latency (ms, rolling average), curriculum stage. Readable text (≥ 4.5:1 on an opaque panel).

### Demo mode (C13)
After 30 s with no input (keyboard, mouse, touch, gamepad) from load or from last input: hide non-essential HUD,
all agents are driven by the trained policy (inference only), showcase cameras cycle (~6–8 s each); any input
returns control and the HUD. Idle threshold, camera list and dwell time are editor-tunable assets.

### Rewards + editor-first config (C9, C10)
Reward includes penalties for joint torque², joint angular velocity², action rate ‖a_t − a_{t−1}‖², and foot sliding
(foot planar speed while in contact), each with its own weight. Reward weights, curriculum thresholds, cameras and UI
references live in ScriptableObjects / `[SerializeField]` (Unity) or DataAssets / `UPROPERTY(EditAnywhere,
BlueprintReadWrite)` (Unreal); Python trainers read the same values from the repo's training config. Changing a
penalty does not retrain shipped policies — note this in docs when shipped models predate the change.

### Workflow
- Unity: the WebGL player is built locally with the EXACT pinned editor (Unity cannot be licensed in CI) by
  `scripts/publish_webgl.ps1`, which runs EditMode tests first, then builds, zips and publishes a `webgl-*` release.
  The one workflow: on release → deploy to the game's Static Web App → check the live page AND /diag; on push →
  Python checks (`ruff check`, `ruff format --check`, `pytest`) only.
- Unreal: the one workflow builds and deploys the repo's web front (viewer / showcase + /diag) to its Static Web App,
  after Python checks. The Unreal project must compile from a fresh clone (all plugins vendored in `Plugins/`).

### Tests
Python: real pytest tests (collected by pytest, not ad-hoc scripts), ruff clean. Unity: EditMode tests pass before
every publish. Tests never write into tracked files.
