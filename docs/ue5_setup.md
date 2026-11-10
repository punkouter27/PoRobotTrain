# UE 5.7 setup for the R1 digital twin (`UnrealProject/`)

> **Status: scaffold only - NOT YET COMPILED.** Unreal 5.7 and the Visual Studio C++ workload are not
> installed on this machine (only the UE 5.8 launcher build is present, and URLab is tested on 5.7 only).
> Every C++ file under `UnrealProject/Source/` was written against the UE 5.7 public API from memory of
> the 5.4-5.6 headers and must be compiled and fixed up once the toolchain exists. See
> [What is not yet compiled / verified](#what-is-not-yet-compiled--verified) before trusting anything.

Project root: `C:\Users\punko\Downloads\CODE_MATT\PoRobotTrain`. Paths below are relative to it.

## 0. What is in `UnrealProject/`

| Path | Purpose |
|---|---|
| `R1Twin.uproject` | EngineAssociation `5.7`; module `R1Twin` (Runtime); plugins `UnrealRoboticsLab`, `NNERuntimeORT`, `EnhancedInput`, `PythonScriptPlugin` |
| `Source/R1Twin.Target.cs`, `Source/R1TwinEditor.Target.cs` | Game / Editor targets |
| `Source/R1Twin/R1Twin.Build.cs` | Deps: Core, CoreUObject, Engine, InputCore, EnhancedInput, NNE (+ `UnrealRoboticsLab` when the plugin folder exists). Defines `R1_WITH_URLAB`, `R1_URLAB_API_VERIFIED` |
| `Source/R1Twin/Public/MjUeConvert.h` | MuJoCo -> UE frame/unit conversion with `static_assert` identities |
| `Source/R1Twin/Public,Private/R1ArticulationBridge.*` | `IR1ArticulationBridge`, `FR1UrlabBridge` (URLab, guesses marked `TODO verify`), `FR1MockBridge` |
| `Source/R1Twin/Public,Private/R1DeployConfig.*` | `FR1DeployConfig` (deploy.yaml twin) + `UR1DeployConfigAsset` data asset |
| `Source/R1Twin/Public,Private/R1PolicyComponent.*` | NNE/ORT inference at fixed 50 Hz, Level 0 observation builder, pre-allocated buffers |
| `Source/R1Twin/Public,Private/R1Level0SweepCommand.*` | Level 0 command source (sinusoid / keyboard / hold) |
| `Source/R1Twin/Public,Private/R1TwinGameMode.*` | Spawns the policy host, disables Chaos simulation, default pawn |
| `Source/R1Twin/Public,Private/R1FollowCamera.*` | Orbit camera following the pelvis, `bLockToStand` for Level 0 |
| `Config/DefaultEngine.ini`, `DefaultGame.ini`, `DefaultInput.ini` | Project settings (physics neutered, default map, Enhanced Input) |
| `MJCF/*.xml`, `meshes/*.STL` | Generated MJCF scenes (`scene_stand.xml` = Level 0 gantry stand, pelvis z = 1.0 m) |
| `Plugins/` | Empty; URLab is cloned here (git-ignored) |

NNE itself is **not** listed as a plugin in the `.uproject`: since UE 5.3 it is an engine runtime module
(`Engine/Source/Runtime/NNE`), so it is a module dependency in `R1Twin.Build.cs`. Only the backend
`NNERuntimeORT` is a plugin (it registers the runtime named `NNERuntimeORTCpu`).

## 1. Install the toolchain (this machine)

### 1.1 Visual Studio 2022 Community (17.10 or newer)

1. Download the VS 2022 Community installer from <https://visualstudio.microsoft.com/vs/community/>.
2. In the installer select the workloads:
   - **Game development with C++** (keep its optional components: *C++ profiling tools*,
     *C++ AddressSanitizer*, *Windows 11 SDK (10.0.22621 or newer)*; tick **Unreal Engine installer**
     only if you also want the launcher-less install, not needed here).
   - **.NET desktop development** (UnrealBuildTool / the Target.cs compiler need the .NET SDK).
   - **Desktop development with C++** (brings *MSVC v143 - VS 2022 C++ x64/x86 build tools (Latest)*).
3. In *Individual components* make sure these are ticked: *MSVC v143 x64/x86 build tools (Latest)*,
   *Windows 11 SDK*, *.NET 8 SDK* (or whatever the 5.7 release notes require), *Git for Windows* (optional).
4. Install, reboot if asked. Verify: `"C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\devenv.exe" /?`
   and that `cl.exe` exists under `VC\Tools\MSVC\<ver>\bin\Hostx64\x64`.

If VS 2022 is already installed without the workload: *Visual Studio Installer -> Modify -> tick the
workloads above*.

### 1.2 Unreal Engine 5.7 (alongside 5.8)

1. Open the **Epic Games Launcher -> Unreal Engine -> Library**.
2. Click the **+** next to *Engine Versions*, pick **5.7.x** (latest hotfix), choose an install path
   (default `C:\Program Files\Epic Games\UE_5.7`). Keep 5.8 installed; `R1Twin.uproject` pins
   `"EngineAssociation": "5.7"` so the two never mix.
3. Optional (saves ~20 GB): *Options* on the 5.7 tile -> untick every target platform except Windows,
   untick *Starter Content* / *Templates and Feature Packs*; keep *Engine Source* ticked (needed to step
   into engine code when the NNE/URLab guesses below fail to compile).
4. Install. Verify `C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe` exists.

### 1.3 CMake 3.24+, Git, uv

```powershell
winget install --id Kitware.CMake -e        # URLab third_party/build_all.ps1 needs CMake >= 3.24
winget install --id Git.Git -e              # if missing; needs submodule support
cmake --version                             # >= 3.24
```
`uv` is already used by `RobotControl/` (the `.venv` exists). `uv run urlab-ping` below is executed from
`external/urlab_bridge`.

## 2. Get the URLab plugin

The plugin directory is git-ignored (`UnrealProject/Plugins/UnrealRoboticsLab/`), so clone it in place:

```powershell
cd C:\Users\punko\Downloads\CODE_MATT\PoRobotTrain\UnrealProject\Plugins
git clone --recurse-submodules https://github.com/URLab-Sim/UnrealRoboticsLab.git UnrealRoboticsLab
```
(If you later want it tracked as a real submodule instead, remove the `.gitignore` line and use
`git submodule add https://github.com/URLab-Sim/UnrealRoboticsLab.git UnrealProject/Plugins/UnrealRoboticsLab`.)

Build the native dependencies (MuJoCo, ZeroMQ, msgpack, ...):

```powershell
cd C:\Users\punko\Downloads\CODE_MATT\PoRobotTrain\UnrealProject\Plugins\UnrealRoboticsLab
pwsh -ExecutionPolicy Bypass -File .\third_party\build_all.ps1
```
Expect this to take several minutes; it needs `cmake` and the VS 2022 C++ toolchain on `PATH`
(run it from a *Developer PowerShell for VS 2022* if it cannot find the compiler).

Then **check the module name** URLab exports:

```powershell
Get-Content .\UnrealRoboticsLab.uplugin | Select-String -Pattern '"Name"'
```
`Source/R1Twin/R1Twin.Build.cs` adds `"UnrealRoboticsLab"` as a private dependency. If the runtime
module in the `.uplugin` `Modules` list is spelled differently, change it there (do **not** add any
`*Editor` module to the Runtime module).

## 3. Generate project files and build

1. In Explorer right-click `UnrealProject\R1Twin.uproject`:
   - if the context menu offers **Switch Unreal Engine version...**, pick **5.7** (it rewrites
     `EngineAssociation` only if it differs);
   - then **Generate Visual Studio project files**.
   Command-line equivalent:
   ```powershell
   & "C:\Program Files\Epic Games\UE_5.7\Engine\Build\BatchFiles\Build.bat" -projectfiles -project="C:\Users\punko\Downloads\CODE_MATT\PoRobotTrain\UnrealProject\R1Twin.uproject" -game -engine -progress
   ```
2. Open `UnrealProject\R1Twin.sln` in VS 2022. Set configuration **Development Editor**, platform
   **Win64**, startup project **R1Twin**. Build (Ctrl+Shift+B). Or:
   ```powershell
   & "C:\Program Files\Epic Games\UE_5.7\Engine\Build\BatchFiles\Build.bat" R1TwinEditor Win64 Development -Project="C:\Users\punko\Downloads\CODE_MATT\PoRobotTrain\UnrealProject\R1Twin.uproject" -WaitMutex -FromMsBuild
   ```
3. **Expect compile errors on the first attempt** in `R1PolicyComponent.cpp` (NNE enum/header names) and
   possibly `R1FollowCamera.cpp` (Enhanced Input). Fix against the real 5.7 headers; the section at the
   end lists every guess. URLab calls are compiled out until you set `R1_URLAB_API_VERIFIED=1`.
4. Launch: F5 from VS, or double-click `R1Twin.uproject`.

## 4. First run inside the editor

### 4.1 Create the default level
`Config/DefaultEngine.ini` points at `/Game/Environments/Stadium_Pitch`, which does not exist yet.
*File -> New Level -> Basic*, delete the floor's physics (select the Floor -> Details -> Physics ->
*Simulate Physics* off, it is off by default), *File -> Save Current Level As...* ->
`Content/Environments/Stadium_Pitch`. The game mode comes from `GlobalDefaultGameMode`
(`AR1TwinGameMode`), so *World Settings -> GameMode Override* can stay `None`.

### 4.2 Import the MJCF
1. Content Browser -> navigate to `MJCF`. The `.xml` and `meshes/` already live there
   on disk; URLab's importer runs when you **drag `scene_stand.xml` from Explorer into that folder**
   of the Content Browser (dragging the file onto itself is fine; what matters is triggering the import
   factory). It converts the STL meshes to GLB and produces an **Articulation Blueprint** (name derived
   from the MJCF model name, `scene_stand`).
2. Check the Output Log for importer warnings (missing meshes, unsupported elements). The scene uses
   `<equality><weld>` for the stand, `<position>` actuators with `kp/kv`, sites, one `<camera>` and
   `<sensor>` entries; all are plain MuJoCo and should import.

### 4.3 Add the PD controller
Open the Articulation Blueprint -> *Components* panel -> **Add -> MjPDController** -> *Compile* -> *Save*.
Without it URLab ignores position targets (confirmed by the urlab_bridge README). If the component
exposes per-joint `kp`/`kv` you can leave them at the MJCF values; `UR1PolicyComponent` pushes the
deploy gains through `IR1ArticulationBridge::SetActuatorGains` on BeginPlay once the bridge is verified.

### 4.4 Place it and tag it
Drag the Articulation Blueprint into the level at the origin (MuJoCo `(0,0,0)` = UE `(0,0,0)`; the
MJCF already places the pelvis at z = 1.0 m, i.e. 100 cm). In *Details -> Actor -> Tags* add
**`R1Articulation`** so `R1Urlab::FindArticulationActor` finds it unambiguously.

Make sure a URLab manager actor (`AAMjManager`) exists - URLab either spawns one automatically or asks
you to place one; check the plugin docs. On it set **`Options.Timestep = 0.005`** (matches
`<option timestep="0.005">` and the 50 Hz policy with decimation 4), `SimSpeedPercent = 100`,
`NumWorkerThreads` to taste.

### 4.5 Policy assets (optional for the first smoke test)
- Drag the Level 0 `.onnx` into `Content/Humanoid/Policies`; NNE creates a `UNNEModelData` asset.
- Right-click in `Content/Humanoid/Policies` -> *Miscellaneous -> Data Asset -> R1DeployConfigAsset*,
  name it `DA_Level0Deploy`, fill `Config` from the deploy.yaml (joint_names, default_joint_pos, kp,
  kd, action_scale, num_obs = 23, num_actions = 7, num_commands = 2, policy_hz = 50) and set
  `PolicyModel`. The planned `scripts/deploy_yaml_to_uasset.py` (UE-Python; not written yet) will do this.
- Either assign both on the `AR1TwinGameMode` class defaults (`DefaultDeployConfig`,
  `DefaultPolicyModel`) by making a Blueprint child of it and setting it as the level's game mode, or
  place an empty actor with a `UR1PolicyComponent` + `UR1Level0SweepCommand` yourself.
  Without any asset the component uses the built-in Level 0 defaults from `r1_joint_map.yaml` and, with
  no model, simply **holds the default pose**.

### 4.6 Press Play
Expected Output Log (category `LogR1Twin`):
```
R1Twin module starting (URLab compiled in: 1, URLab API verified: 0)
R1TwinGameMode: Chaos simulation disabled on N primitive component(s); MuJoCo (URLab) is the physics authority.
R1TwinGameMode: spawned policy host (config: ..., model: ...).
R1Policy: articulation actor candidate 'scene_stand_C_0' (class scene_stand_C)
FR1UrlabBridge: articulation actor 'scene_stand_C_0' found but URLab bindings are compiled out ...   <- until verified
FR1MockBridge active with 7 joints - the robot you see (if any) is NOT being driven.               <- until verified
Level0 command source ready: mode=0, head yaw 30 deg @ 0.25 Hz, shoulder 0.35..-0.85 rad @ 0.15 Hz
R1Policy: loaded 'level0_policy' on NNERuntimeORTCpu: obs(1,23) -> actions(1,7)
R1Policy: state=3, bridge=..., 7 joints, obs=23, actions=7, 50 Hz
```
Camera: hold **right mouse** to orbit, **wheel** to zoom, **L** toggles the stand lock, **P** pauses
inference, **Tab** cycles the command mode (sinusoid -> manual -> hold), **W/S A/D** drive the manual
command, **R** resets the policy/command clock.

### 4.7 Verify the ZMQ endpoints
With PIE running:
```powershell
cd C:\Users\punko\Downloads\CODE_MATT\PoRobotTrain\external\urlab_bridge
uv run urlab-ping            # RPC REQ/REP on tcp://127.0.0.1:5559 (msgpack)
```
Ports: **5559** RPC, **5555** state PUB (joints + sensors), **5556** control, **5558+** camera PUB.
`uv run urlab-dashboard` (if the extras are installed) shows joints/sensors/cameras live; the
`head_rgbd` camera (fovy 58, pitched 20 deg down) should appear once URLab maps MJCF cameras to its
camera components (Real/Depth/Segmentation modes).

## 5. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `.uproject` opens in 5.8 | Right-click -> *Switch Unreal Engine version* -> 5.7. Delete `Intermediate/`, `Binaries/`, `.vs/`, `R1Twin.sln` and regenerate. |
| "The following modules are missing or built with a different engine version: R1Twin" | Normal on first open; answer **Yes** to rebuild, or build *Development Editor* from VS first. |
| "Plugin 'UnrealRoboticsLab' failed to load because module 'X' could not be found" | `third_party/build_all.ps1` did not run, or the plugin was cloned without `--recurse-submodules`. Re-run both, regenerate project files. |
| `R1Twin.Build.cs` error: could not find module `UnrealRoboticsLab` | Module name differs; read the `.uplugin` `Modules` list and fix the name in `Build.cs`. |
| Compile errors in `R1PolicyComponent.cpp` about `ESetInputTensorShapesStatus` / `ERunSyncStatus` / `FTensorShape::Make` | NNE API drift; open `Engine/Source/Runtime/NNE/Public/NNERuntimeRunSync.h` and `NNETypes.h` and adapt (5.3 returned `int` 0 for success; 5.4+ nested enums). |
| `NNE runtime 'NNERuntimeORTCpu' not found` | `NNERuntimeORT` plugin disabled or failed to load; check *Edit -> Plugins -> NNE*. Also happens in `-nullrhi` commandlets before plugins load. |
| `SetInputTensorShapes((1,23)) failed` | The exported ONNX has a fixed batch or a different obs size; re-export with dynamic batch, or fix `NumObs` in the data asset. |
| Robot imports but does not move | No `MjPDController` on the Articulation Blueprint (4.3), or `R1_URLAB_API_VERIFIED=0` (mock bridge). Check the log for `FR1MockBridge active`. |
| Robot explodes / drifts on Play | Chaos simulating a component: look for the `Chaos simulation disabled on N` line; `N` should be 0 for URLab actors. Also confirm `Options.Timestep = 0.005` and that the stand weld (`stand_weld`) imported. |
| Camera does not orbit | Viewport needs focus (click once); `DefaultInput.ini` must keep `DefaultInputComponentClass=EnhancedInputComponent`. Check for the `input component is not UEnhancedInputComponent` error. |
| `urlab-ping` times out | PIE not running, or URLab's ZMQ server disabled/port changed in the manager actor; firewall prompt on first run. |
| Importer: meshes missing | `meshdir="meshes"` is relative to the `.xml`; keep `scene_stand.xml` and `meshes/` side by side (they are). |

## 6. What is not yet compiled / verified

Nothing in `UnrealProject/Source` has been compiled. Beyond that, these are explicit assumptions:

**UE / toolchain**
1. `BuildSettingsVersion.Latest` / `EngineIncludeOrderVersion.Latest` are used instead of the concrete
   5.7 enum values the template would write (`V6` / `Unreal5_7`, unconfirmed).
2. NNE is an engine module, not a plugin, in 5.7 (true for 5.3-5.6). If 5.7 moved it back to a plugin,
   add `{"Name": "NNE", "Enabled": true}` to the `.uproject`.
3. Enhanced Input runtime-created assets: `UInputMappingContext::MapKey()` returning
   `FEnhancedActionKeyMapping&`, `UInputModifierNegate`, `EInputActionValueType`, `ETriggerEvent`
   (all present since 5.1; spelling unverified against 5.7).
4. `UObject::IsDataValid(FDataValidationContext&)` override signature (5.2+).
5. `DefaultViewportMouseCaptureMode=CaptureDuringRightMouseDown` + `bShowMouseCursor` is enough for
   orbit + free cursor without an explicit `FInputModeGameAndUI`.
6. INI keys under `[/Script/Engine.PhysicsSettings]` (`bSubsteppingAsync`, `bTickPhysicsAsync`,
   `bDisableCCD`) exist with those names in 5.7; unknown keys are ignored silently, so verify in
   *Project Settings -> Physics*.
7. `AGameModeBase::StartPlay()` runs before actors receive BeginPlay, so the spawned policy host gets its
   BeginPlay in the normal sweep (true for 5.x so far).

**NNE (R1PolicyComponent.cpp)**
8. `UE::NNE::GetRuntime<INNERuntimeCPU>(FString)` -> `TWeakInterfacePtr<INNERuntimeCPU>` (`NNE.h`).
9. `INNERuntimeCPU::CanCreateModelCPU` returns `INNERuntimeCPU::ECanCreateModelCPUStatus::Ok`.
10. `IModelInstanceRunSync::ESetInputTensorShapesStatus::Ok` and `::ERunSyncStatus::Ok` are nested enums
    (5.4+); header `NNERuntimeRunSync.h`.
11. `UE::NNE::FTensorShape::Make(TConstArrayView<uint32>)`, `FTensorShape::Volume()` (uint64),
    `FTensorDesc::GetName()`/`GetDataType()`, `ENNETensorDataType::Float`.
12. `FTensorBindingCPU` is `{ void* Data; uint64 SizeInBytes; }`.
13. The ORT CPU runtime accepts a symbolic batch dim resolved to 1 via `SetInputTensorShapes`.

**URLab (R1ArticulationBridge.cpp, all behind `R1_URLAB_API_VERIFIED`)**
14. Plugin runtime module is named `UnrealRoboticsLab` and its headers are `MjArticulation.h`,
    `MjComponent.h`, `MjPDController.h`, `MjManager.h`.
15. Script path of the articulation class is `/Script/UnrealRoboticsLab.MjArticulation`.
16. Guessed accessors on `AMjArticulation`: `FindJointIndex(FName)`, `FindActuatorIndex(FName)`,
    `GetJointPosition(int32)`, `GetJointVelocity(int32)`, `SetActuatorControl(int32, double)`,
    `GetBodyWorldPose(FName, FVector&, FQuat&)`, `GetSensorData(FName, TArray<double>&)`; on
    `UMjPDController`: `SetGains(FName, double, double)`. **None of these names are known to exist.**
    The real API may instead live on the `UMjComponent` tree or on the published `FMjRenderSnapshot`.
17. `SetActuatorControl` semantics = position target consumed by `UMjPDController` (not a torque), and
    the controller accepts per-step updates from the game thread.
18. URLab's `GetSensorData("imu_quat")` returns MuJoCo order `(w, x, y, z)`.
19. URLab imports `<equality><weld>` (gantry stand) and `<position kp kv>` actuators as-is.
20. `AAMjManager` exposes `Options.Timestep`, `SimSpeedPercent`, `NumWorkerThreads` in Details (from
    the URLab docs; not seen in code).

**Policy contract**
21. Level 0 observation layout `[joint_pos_rel(7) | joint_vel(7) | last_action(7) | command(2)]` with
    unit scales; `command = [head_yaw_target_rad, right_shoulder_pitch_target_rad]` as absolute joint
    angles. Confirm against the training env in `RobotControl/` and set `ObsJointPosScale` /
    `ObsJointVelScale` in the data asset if the env scales them.
22. `last_action` stores the clipped raw action (before `action_scale`), matching the usual mjlab /
    IsaacLab convention.
23. Policy stepping is driven by game time (fixed 50 Hz accumulator, catch-up cap 4), not by a MuJoCo
    step counter; with `SimSpeedPercent = 100` these agree on average but are not phase-locked.
24. Joint-name convention: deploy.yaml may carry either `right_shoulder_pitch` or
    `right_shoulder_pitch_joint`; both are normalised (`FR1DeployConfig::ToJointName/ToActuatorName`).

**Frame conversion (MjUeConvert.h)** - the maths is checked by `static_assert` and `MjUe::RunSelfTest()`,
but URLab's own convention for placing imported bodies has not been compared against it. If URLab
mirrors differently (e.g. flips X instead of Y), poses read through `GetBodyPoseMj` must be converted
with URLab's convention instead, and the camera focus will be mirrored until that is fixed.
