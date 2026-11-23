# Browser twin in the Blazor control UI

The R1 Stand Explore page (`src/PoRobotTrain.Client`) shows a live 3D twin of the robot on its stand. It also
runs a set of checks in the browser. Everything here displays or checks poses. MuJoCo still owns physics, and
nothing in the browser commands the robot. Slider writes and behaviour runs go through the same API, gates and
confirm dialogs as before.

## Build the assets

```
.venv\Scripts\python.exe scripts\build_web_twin.py --compress     # needs Node (npx) for Draco
```

This writes `src/PoRobotTrain.Client/wwwroot/twin/` from the compiled `UnrealProject/MJCF/scene_stand.xml`:

| file | what | checked by |
|---|---|---|
| `r1_stand.glb` | MuJoCo's own visual meshes, one node per body. Welded, simplified and Draco-compressed (10.7 MB → 0.8 MB) | `RobotControl/tests/test_web_twin.py` |
| `twin_manifest.json` | joints in `r1_joint_map.yaml` policy order, axes, limits, defaults, body tree, MuJoCo collision capsules, stand geoms, golden FK poses | Python + C# tests, `/diag` |
| `policy.onnx` | byte copy of the validated Level 0 policy (sha256 in the manifest) | test fails if it differs from `RobotControl/models/...` |
| `policy_golden.json` | 16 baseline-rollout observations and the actions Python onnxruntime gives for them | `/diag` policy parity |

Rerun the script whenever the MJCF, the joint map or the policy changes. `test_web_twin.py` fails until you do.

## What each library does here

| library | used for |
|---|---|
| three.js (`WebGPURenderer`, WebGL2 fallback) | the live pose view, replacing the server PNG (kept as the fallback). Same camera presets as `robot_view.py` |
| glTF 2.0 + Draco | the robot asset above, decoded in the browser by the three.js Draco loader |
| TSL node materials | per-link status tint with a pulsing rim: near/at limit (same thresholds as the joint sliders), sim ≠ real > 2°, motor ≥ 55 °C, proposed-pose contact. Ghost shader for the sim pose and the slider preview |
| three-nebula (WebGPU `GPURenderer`) | heat haze over hot motors, sparks at a proposed-pose contact, a red burst at the primary joint when a job ends failed |
| WebGPU compute (WGSL, CPU fallback) | reach cloud: 16k random configurations of the behaviour's joint chain inside the joint limits (dim), plus the band the selected sweep covers (bright). Head behaviours show the gaze envelope |
| Rapier (WASM) | shape-level contact queries only, with no world and no dynamics. While a slider is dragged, the proposed pose is checked against MuJoCo's collision capsules (contacts already present in the default pose are ignored) and against the stand frame, which MuJoCo does not collide with. The result is a display-only warning |
| Tone.js | optional spatial audio cues: job start (a harder sound for powered physical runs), done/failed, a pulse while the physical robot is powered, ticks that speed up near a joint limit while dragging, a collision buzz, hot-motor chime |
| `<canvas>` + `js/heatmap.js` (one pixel per cell, CSS-scaled) | joint-history heatmap: 26 joints × the last 240 samples, coloured by position-in-range, limit margin, or virtual-minus-physical error |
| Silk.NET.Maths | C# forward kinematics (`PoRobotTrain.Shared/Kinematics`) for the hand and face read-out, unit-tested against MuJoCo |
| ONNX Runtime Web | `/diag` runs the deployed policy in the browser and compares it with the golden actions (tolerance 1e-4), plus a sha256 check and per-step latency |
| Transformers.js (MiniLM, local) | the "Ask in plain words" box suggests a behaviour and sweep size, or points at a single joint's slider. It never starts anything. A one-time 23 MB model download happens on first use |

Silk.NET's WebGL and OpenAL bindings are not used: in Blazor WebAssembly they would need Emscripten-linked native
GL/AL alongside three.js and Tone.js, which already cover rendering and audio.

## Requirements and fallbacks

- The libraries load from jsDelivr at pinned versions (import map in `wwwroot/index.html`). Offline, the 3D view
  falls back to the server PNG render. The rest of the page is unaffected.
- WebGPU is optional. Without it the renderer uses WebGL2 and the reach cloud is computed on the CPU, with
  bit-identical random samples.
- `/diag` live-probes every module, asset, GPU path and self-test listed above.

## Tests

```
.venv\Scripts\python.exe -m pytest RobotControl/tests/test_web_twin.py
dotnet test tests/PoRobotTrain.Unit          # Silk.NET FK vs MuJoCo, twin joint order vs RobotJointCatalog
```
