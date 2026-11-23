# Agent guide — PoRobotTrain

PoRobotTrain is an RL-trained Unitree R1 digital twin and sim-to-real pipeline.
Read `README.md` and the relevant procedure in `docs/` before changing the
training, simulator, deployment, or robot-control path. The file
`RobotControl/configs/r1_joint_map.yaml` is the source of truth for joint order,
motor slots, limits, gains, and defaults.

The guide has three parts: **1. Coding standard** (project contracts, the .NET control UI, the Po Game Standard),
**2. Concurrent work** and **3. Verify the repo** (exact commands).

## 1. Coding standard

### Project contracts

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

### Blazor control UI (`src/`, Profile A)

- Radzen is the only component library (`Radzen.Blazor` pinned in `Directory.Packages.props`; owner decision, Radzen
  wins). Buttons are `RadzenButton class="po-plain ..."`: Radzen's theme sits in a low CSS layer (`app.css`
  `@layer radzen, po-reset`), `po-plain` drops the theme look, so the app's `po-chip` / `po-btn` styling stays. Inputs
  are `RadzenTextBox` / `RadzenNumeric` / `RadzenDropDown` / `RadzenCheckBox` / `RadzenSlider` / `RadzenFileInput`.
  Allowed exceptions (no other raw form controls): read-only data tables (`<table class="diag-table">` on Health,
  Setup, Diagnostics, Robustness, the keyframe grid). The three.js / WebGPU twin `<canvas>`, the joint-history
  `<canvas>` and SVG charts are drawings, not form controls.
- Joint sliders move a joint once, on release (pointer up / key up over the slider), never on every drag step.
- Form rules are FluentValidation validators in `src/PoRobotTrain.Shared/Validation` (same rules as the behaviour
  service), registered BY TYPE in the Client `Program.cs` (no assembly scan, trimming-safe) and shown inline with
  `FieldError`. No `alert()` / `confirm()`.
- One layout (`MainLayout`: title top-left, E-STOP top-right, version stamp in the footer after the content), a
  NotFound page, loading / empty / error states. API endpoints live in `Features/*` modules (`MapXEndpoints`), Serilog,
  `/health`. `/diag` probes the behaviour service and the browser runtime live; capabilities that are off on purpose
  (WebGPU, speech recognition) show a grey OFF and do not fail the page.
- Sign-in: none today (local operator console next to the robot). Profile A asks for Microsoft Entra; that needs an
  owner decision because the console also hosts the software E-STOP and may run offline next to the robot.
- The control UI is NOT deployed; only the static showcase (`web/`) is.

### CI/CD and Azure

- `.github/workflows/deploy.yml` (the only workflow): on push to `master` it runs the Python checks (ruff, pytest),
  the .NET checks (format, Release build with 0 warnings, Unit, Integration), builds the showcase site
  (`web/build.py`), deploys it to the Azure Static Web App `swa-porobottrain` (resource group `PoShared`, secret
  `AZURE_STATIC_WEB_APPS_API_TOKEN`) and checks `/`, `/diag` and the `X-Commit` header (= the pushed commit) live.
- The Unreal twin is built and tested locally (GitHub's runners have no Unreal Engine). URLab is not tracked:
  `scripts/setup_urlab.ps1` recreates it (pinned upstream commit + MuJoCo 4.0.0 + `scripts/patches/`).
- **Hosting:** Azure Static Web App `swa-porobottrain` (RG `PoShared`), Free tier; the control UI is not hosted. Never
  run test suites against the live site; live checks = at most 6 page loads + /diag + X-Commit.

### Deliberate deviations

| Deviation | Why | Commit sha / date | How to undo |
|---|---|---|---|
| No Entra sign-in on the localhost robot console (it holds the software E-STOP and may run offline) — proposed 2026-11-22, awaiting owner decision | Profile A asks for Microsoft Entra | — | add Entra sign-in if the owner decides so |
| URLab gitlink from owner commit `f03c5e7` replaced by `scripts/setup_urlab.ps1` (same pinned upstream + patches, MuJoCo 4.0.0); folder git-ignored | the gitlink pointed at a local-only commit, so fresh clones could not build; v3 forbids submodules | `9dc0eb7` 2026-11-22 (local) | revert `9dc0eb7`'s `.gitignore`/index change for a true vendored copy |
| Radzen exceptions: read-only `<table class="diag-table">` tables; the twin and joint-history `<canvas>` and SVG charts are drawings | allowed kinds (Profile A) | `03ff3b2` 2026-11-22 (local) | — |
| /diag (control UI and showcase): capabilities off on purpose (WebGPU, speech recognition; the showcase's switched-off row) show a grey **Off**, not counted | not faults | `03ff3b2`, `8b5dd01` 2026-11-22 (local) | — |
| Control UI: version stamp in the footer after the content; showcase: sticky 20 px footer on desktop, static on phones | a fixed stamp covered content on phones | `03ff3b2`, `8b5dd01` 2026-11-22 (local) | `git revert` the stamp part |
| `bRunLevel0PolicyOnPlay=False` kept: on a normal Play the overlay shows reward/latency as n/a | owner decision 2026-11-15 | — | set it to True |
| The Blazor control UI is not deployed (local operator console); only `web/` is | it drives the robot | — | — |

### Unreal content layout

`/Game/Project/{Agents,DataAssets,Environments,RL_Policies}` (the R1 MuJoCo import, the reward + showcase DataAssets
selected in `DefaultGame.ini`, the R1_Stand level, the NNE policy). Core and UI are C++ (`Source/R1Twin`), so those
folders hold no assets. Move or rename content only in the editor (AssetTools rename + fix up redirectors), never on
disk. The overlay's "Benchmark sim speed" button steps a private copy of the MuJoCo scene for 1 s and reports
steps/s; the robot and the live simulation are untouched.

### Po Game Standard (Unity / Unreal RL games)

This repo is one of several independent Po games. It follows the conventions below, but it owns every file:
nothing is generated from, synced with, or required to stay identical to another repository (no "same file in
every game" headers, no sync scripts that copy into other repos). Third-party code it needs (ML-Agents, MuJoCo
plugins, URLab) is vendored INSIDE this repo at a pinned version. Azure resources may be shared through `PoShared`.

#### Repo skeleton
```
README.md   AGENTS.md (this standard + repo notes)   CLAUDE.md ("Read AGENTS.md.")
docs/       lowercase, all prose (no DOCS/, no root DESIGN.md/SPEC.md/tasks.md — move them into docs/)
scripts/    build/publish/helper scripts (Unity: Tools/ is renamed scripts/ unless Unity itself needs it)
.github/workflows/<one file>.yml   the ONLY workflow
pyproject.toml with a [tool.ruff] section for any Python in the repo
```
No absolute local paths (C:\Users\...) committed anywhere. No build outputs or node_modules tracked.

#### Web page around the game (WebGL template / web viewer)
- The page chrome is drawn ONCE, by the HTML page, OUTSIDE the game canvas, so it can never overlap the HUD:
  a 44px top bar (game name top-left, settings gear / fullscreen top-right) and the version `vYYMMDD` bottom-right
  in the bar's counterpart or a 20px footer strip. The canvas fills the remaining viewport.
- The in-game HUD does NOT repeat the game name or version (it may show them inside menus, not as corner chrome).
- Same tokens as the web apps: dark background `#0b0f17`, surface `#121826`, text `#e6e9f2`, muted `#9aa3b5`,
  accent `#6366f1`, Inter/system-ui font. Text ≥ 4.5:1 contrast.

#### Orientation (C11)
The canvas fills any viewport (no fixed portrait aspect, no pillarboxing). UI is anchored and DPI-scaled
(Unity: CanvasScaler "Scale With Screen Size" or UI Toolkit PanelSettings scale-with-screen; Unreal: UMG anchors +
DPI curve). Must be checked at 1600x900 (landscape) and 450x800 (portrait) — screenshots in the PR/commit notes.

#### Version
One stamp, UTC `yyMMdd`, computed once by the publish/build script and injected into: index.html, diag.html,
and the player (`PlayerSettings.bundleVersion` / Unreal ProjectVersion) so `Application.version` matches the page.

#### /diag (games)
`/diag` (static `diag.html` generated by the publish script) titled **System status**, same layout as the web apps:
overall banner, then **API / Data / Services** sections. It must LIVE-probe (fetch from the browser, with latency)
every external dependency the game really calls (leaderboard tables/functions, weather APIs, AI APIs, replay bundles,
model files), plus the WebGL2 capability and that the build bundle files exist. Nothing hard-coded as "Ready".
The checks are listed in the repo's own config (e.g. `scripts/webgl.json` → `diagChecks`).

#### In-game diagnostics overlay (C12)
`Tab` (and a 44px corner button on touch devices) toggles an overlay showing: FPS, current episode reward,
inference latency (ms, rolling average), curriculum stage. Readable text (≥ 4.5:1 on an opaque panel).

#### Demo mode (C13)
After 30 s with no input (keyboard, mouse, touch, gamepad) from load or from last input: hide non-essential HUD,
all agents are driven by the trained policy (inference only), showcase cameras cycle (~6–8 s each); any input
returns control and the HUD. Idle threshold, camera list and dwell time are editor-tunable assets.

#### Rewards + editor-first config (C9, C10)
Reward includes penalties for joint torque², joint angular velocity², action rate ‖a_t − a_{t−1}‖², and foot sliding
(foot planar speed while in contact), each with its own weight. Reward weights, curriculum thresholds, cameras and UI
references live in ScriptableObjects / `[SerializeField]` (Unity) or DataAssets / `UPROPERTY(EditAnywhere,
BlueprintReadWrite)` (Unreal); Python trainers read the same values from the repo's training config. Changing a
penalty does not retrain shipped policies — note this in docs when shipped models predate the change.

#### Workflow
- Unity: the WebGL player is built locally with the EXACT pinned editor (Unity cannot be licensed in CI) by
  `scripts/publish_webgl.ps1`, which runs EditMode tests first, then builds, zips and publishes a `webgl-*` release.
  The one workflow: on release → deploy to the game's Static Web App → check the live page AND /diag; on push →
  Python checks (`ruff check`, `ruff format --check`, `pytest`) only.
- Unreal: the one workflow builds and deploys the repo's web front (viewer / showcase + /diag) to its Static Web App,
  after Python checks. The Unreal project must compile from a fresh clone (all plugins vendored in `Plugins/`).

#### Tests
Python: real pytest tests (collected by pytest, not ad-hoc scripts), ruff clean. Unity: EditMode tests pass before
every publish. Tests never write into tracked files.

## 2. Concurrent work

- Preserve existing local work. Do not commit, push, or deploy unless explicitly requested.
- One Unreal editor or headless browser at a time on this PC; close the editor when done. Editor Python only with
  Play stopped (URLab crash, `docs/step5_runtime.md`).
- `scripts/run-e2eui.ps1` owns ports 5080 (published app) and 8765 (behaviour service, only if nothing answers there);
  stop your own app / service first. It stops everything it started.
- Report skipped Unreal, device, and hardware gates explicitly; the mock bridge is not evidence that URLab drives
  the robot. Never start a real-robot session from tests.
- For UI changes, retain the five HUD anchors and document before/after screenshots and an HTML report under
  `SCREENSHOTS/`.

## 3. Verify the repo (exact commands)

The sequence below verifies the repo; for the non-Unreal part in one line:
`.venv\Scripts\python.exe -m ruff check . && .venv\Scripts\python.exe -m pytest && dotnet build PoRobotTrain.slnx -c Release && dotnet test tests\PoRobotTrain.Unit -c Release --no-build && dotnet test tests\PoRobotTrain.Integration -c Release --no-build && powershell -ExecutionPolicy Bypass -File scripts\run-e2eui.ps1`.
Last full run (2026-11-22): pytest 150 passed; Release build 0 warnings; Unit 25/25; Integration 27/27; E2EUI 19/19
(4 runs in a row); R1TwinEditor build succeeded.

```powershell
# Python (repo root; .venv as in README)
.venv\Scripts\python.exe -m ruff check .
.venv\Scripts\python.exe -m ruff format --check .
.venv\Scripts\python.exe -m pytest
.venv\Scripts\python.exe web\build.py --check
# .NET control UI
dotnet format PoRobotTrain.slnx --verify-no-changes
dotnet build PoRobotTrain.slnx -c Release            # 0 warnings (TreatWarningsAsErrors)
dotnet test tests\PoRobotTrain.Unit -c Release --no-build
dotnet test tests\PoRobotTrain.Integration -c Release --no-build
powershell -ExecutionPolicy Bypass -File scripts\run-e2eui.ps1   # browser tier on the trimmed Release publish
# Unreal: URLab once per clone, then the editor target
powershell -ExecutionPolicy Bypass -File scripts\setup_urlab.ps1
& "C:\Program Files\Epic Games\UE_5.8\Engine\Build\BatchFiles\Build.bat" R1TwinEditor Win64 Development "-Project=$PWD\UnrealProject\R1Twin.uproject" -WaitMutex -MaxParallelActions=6
# Unreal runtime check: overlay + benchmark + demo mode screenshots (also -ResX=450 -ResY=800), docs/showcase.md
& "C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe" UnrealProject\R1Twin.uproject -game -windowed -ResX=1600 -ResY=900 -ForceRes -ShowcaseCapture=$env:TEMP\r1cap -ShowcaseTouch -ShowcaseBench -unattended
```
