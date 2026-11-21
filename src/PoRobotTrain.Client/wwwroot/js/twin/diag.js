// Live browser-side probes for /diag: every module, model file and GPU capability the twin really uses.
// Nothing is hard-coded as ready; each row is fetched or exercised now, with its latency.
const CDN = "https://cdn.jsdelivr.net/npm/";
const FILES = [
  ["Twin manifest", "twin/twin_manifest.json"],
  ["R1 stand glTF (Draco)", "twin/r1_stand.glb"],
  ["Level 0 policy (ONNX)", "twin/policy.onnx"],
  ["Policy golden vectors", "twin/policy_golden.json"],
  ["Draco decoder (WASM)", CDN + "three@0.187.0/examples/jsm/libs/draco/gltf/draco_decoder.wasm"],
  ["ONNX Runtime WASM", CDN + "onnxruntime-web@1.31.1/dist/ort-wasm-simd-threaded.wasm"],
];
const MODULES = [
  ["three.js WebGPU build", "three/webgpu"],
  ["three.js TSL", "three/tsl"],
  ["three-nebula", "three-nebula"],
  ["three-nebula WebGPU renderer", "three-nebula/webgpu"],
  ["Rapier 3D (WASM)", "@dimforge/rapier3d-compat"],
  ["Tone.js", "tone"],
  ["ONNX Runtime Web", "onnxruntime-web"],
  ["Transformers.js", "@huggingface/transformers"],
];

async function timed(fn) {
  const t = performance.now();
  try {
    const detail = await fn();
    return { ok: true, ms: performance.now() - t, detail: detail ?? "" };
  } catch (e) {
    return { ok: false, ms: performance.now() - t, detail: String(e?.message ?? e) };
  }
}

export async function capabilities() {
  const rows = [];
  rows.push({
    section: "GPU",
    name: "WebGL2",
    ...(await timed(async () => {
      const gl = document.createElement("canvas").getContext("webgl2");
      if (!gl) throw new Error("not available");
      const ext = gl.getExtension("WEBGL_debug_renderer_info");
      return ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : "available";
    })),
  });
  rows.push({
    section: "GPU",
    name: "WebGPU (compute + renderer)",
    ...(await timed(async () => {
      if (!navigator.gpu) throw new Error("navigator.gpu missing: twin uses WebGL2 + CPU reach sampling");
      const a = await navigator.gpu.requestAdapter();
      if (!a) throw new Error("no adapter: twin uses WebGL2 + CPU reach sampling");
      return a.info?.description || a.info?.vendor || "adapter ok";
    })),
  });
  return rows;
}

export async function files() {
  return Promise.all(
    FILES.map(async ([name, url]) => ({
      section: "Data",
      name,
      ...(await timed(async () => {
        const r = await fetch(url, { cache: "no-store" });
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const n = (await r.arrayBuffer()).byteLength;
        return `${(n / 1024).toFixed(0)} KB`;
      })),
    }))
  );
}

export async function modules() {
  return Promise.all(
    MODULES.map(async ([name, spec]) => ({
      section: "Services",
      name,
      ...(await timed(async () => {
        const m = await import(spec);
        return `${Object.keys(m).length} exports`;
      })),
    }))
  );
}

export async function selfTests() {
  const rows = [];
  const manifest = await fetch("twin/twin_manifest.json").then((r) => r.json());
  const fk = await import("./fk.js");
  const model = fk.buildModel(manifest);
  rows.push({
    section: "Checks",
    name: "JS forward kinematics vs MuJoCo",
    ...(await timed(async () => {
      const r = fk.selfTest(model);
      if (!r.ok) throw new Error(`max ${(r.maxPosM * 1000).toFixed(3)} mm`);
      return `${r.poses} poses, max ${(r.maxPosM * 1e6).toFixed(1)} µm`;
    })),
  });
  rows.push({
    section: "Checks",
    name: "Rapier pose check (default pose clear)",
    ...(await timed(async () => {
      const { createChecker } = await import("./collide.js");
      const c = await createChecker(model);
      const r = c.check(null);
      if (r.self.length || r.stand.length) throw new Error(`${r.self.length + r.stand.length} contacts at default pose`);
      return `${c.pairs.length} collider pairs`;
    })),
  });
  rows.push({
    section: "Checks",
    name: "Reach kernel (WebGPU vs CPU)",
    ...(await timed(async () => {
      const reach = await import("./reach.js");
      const { root, links } = fk.chainTo(model, "right_wrist_roll_link");
      const spec = {
        root: fk.forward(model, null).get(root),
        links: links.map((b) => ({ pos: b.pos, quat: b.quat, axis: b.axis, lo: model.joints[b.joint].minDeg, hi: model.joints[b.joint].maxDeg })),
        tip: fk.tipOffset(model, "right_wrist_roll_link"),
        count: 256,
        seed: 3,
      };
      const r = await reach.selfTest(spec);
      if (!r.ok) throw new Error(`GPU/CPU differ by ${(r.maxErrM * 1000).toFixed(3)} mm`);
      return r.backend === "webgpu" ? `WebGPU = CPU within ${(r.maxErrM * 1e6).toFixed(1)} µm` : r.note;
    })),
  });
  return rows;
}

export async function policyParity() {
  const { runParity } = await import("./policy.js");
  const r = await timed(() => runParity("twin/"));
  return { section: "Checks", name: "Policy parity (ONNX Runtime Web vs Python)", ...r };
}

// JSON-string entry points for Blazor
export const capabilitiesJson = async () => JSON.stringify(await capabilities());
export const filesJson = async () => JSON.stringify(await files());
export const modulesJson = async () => JSON.stringify(await modules());
export const selfTestsJson = async () => JSON.stringify(await selfTests());
export async function policyParityJson() {
  const row = await policyParity();
  if (row.ok && row.detail && typeof row.detail === "object") {
    const d = row.detail;
    row.ok = d.ok;
    row.detail =
      `max |Δ| ${d.maxAbsDiff.toExponential(2)} (tol ${d.tolerance}) on ${d.samples} obs · sha256 ${d.shaOk ? "matches" : "MISMATCH"} · ` +
      `${d.msPerStep.toFixed(2)} ms/step (budget 20 ms @ 50 Hz)`;
  } else if (row.detail && typeof row.detail === "object") {
    row.detail = JSON.stringify(row.detail);
  }
  return JSON.stringify([row]);
}
