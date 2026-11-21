// ONNX Runtime Web: run the *deployed* Level 0 policy in the browser and check it against the golden actions that
// Python onnxruntime produced (scripts/build_web_twin.py). A third runtime agreeing on the same bytes is evidence the
// ONNX deployment contract (obs[23] -> actions[7], float32) holds; it never drives the robot.
const ORT_DIST = "https://cdn.jsdelivr.net/npm/onnxruntime-web@1.31.1/dist/";

async function sha256Hex(buf) {
  const h = await crypto.subtle.digest("SHA-256", buf);
  return [...new Uint8Array(h)].map((b) => b.toString(16).padStart(2, "0")).join("");
}

export async function runParity(base = "twin/") {
  const t0 = performance.now();
  const ort = await import("onnxruntime-web");
  ort.env.wasm.wasmPaths = ORT_DIST;
  ort.env.wasm.numThreads = 1; // no cross-origin isolation on this page; single-threaded is deterministic anyway

  const [manifest, golden, bytes] = await Promise.all([
    fetch(base + "twin_manifest.json").then((r) => r.json()),
    fetch(base + "policy_golden.json").then((r) => r.json()),
    fetch(base + "policy.onnx").then((r) => {
      if (!r.ok) throw new Error(`policy.onnx HTTP ${r.status}`);
      return r.arrayBuffer();
    }),
  ]);
  const sha = await sha256Hex(bytes);
  const shaOk = sha === manifest.policy.sha256 && sha === manifest.policy.sourceSha256;

  const session = await ort.InferenceSession.create(bytes, { executionProviders: ["wasm"] });
  const n = golden.obs.length;
  const flat = Float32Array.from(golden.obs.flat());
  const out = await session.run({ [golden.input]: new ort.Tensor("float32", flat, [n, golden.obsDim]) });
  const act = out[golden.output].data;
  let maxAbs = 0;
  for (let i = 0; i < n; i++) {
    for (let k = 0; k < golden.actDim; k++) maxAbs = Math.max(maxAbs, Math.abs(act[i * golden.actDim + k] - golden.actions[i][k]));
  }

  // single-observation latency, the shape the 50 Hz loop uses (20 ms budget per step)
  const one = new ort.Tensor("float32", flat.slice(0, golden.obsDim), [1, golden.obsDim]);
  for (let i = 0; i < 5; i++) await session.run({ [golden.input]: one });
  const reps = 50;
  const t1 = performance.now();
  for (let i = 0; i < reps; i++) await session.run({ [golden.input]: one });
  const msPerStep = (performance.now() - t1) / reps;
  await session.release?.();

  const parityOk = maxAbs <= golden.tolerance;
  return {
    ok: parityOk && shaOk,
    parityOk,
    shaOk,
    sha256: sha,
    maxAbsDiff: maxAbs,
    tolerance: golden.tolerance,
    samples: n,
    obsDim: golden.obsDim,
    actDim: golden.actDim,
    msPerStep,
    totalMs: performance.now() - t0,
    backend: "onnxruntime-web wasm",
  };
}
