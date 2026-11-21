// Reach envelope on a WebGPU compute shader (CPU fallback with bit-identical random numbers).
// For a joint chain (e.g. torso -> right wrist) it samples N joint configurations inside the r1_joint_map.yaml
// limits and returns the fingertip/face point for each one, so the 3D view can show where a behaviour can
// possibly reach (dim cloud) and the narrow band the selected behaviour actually sweeps (bright cloud).
import { DEG, qAxisAngle, qMul, qRotate } from "./fk.js";

const MAX_LINKS = 8;
const WGSL = /* wgsl */ `
struct Link { pos: vec4f, quat: vec4f, axis: vec4f, range: vec4f };
struct Params { rootPos: vec4f, rootQuat: vec4f, tip: vec4f, count: u32, nlinks: u32, seed: u32, pad: u32 };
@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> links: array<Link>;
@group(0) @binding(2) var<storage, read_write> outPts: array<vec4f>;

fn pcg(v: u32) -> u32 {
  let s = v * 747796405u + 2891336453u;
  let w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
  return (w >> 22u) ^ w;
}
fn u01(i: u32, j: u32) -> f32 { return f32(pcg(i * 16u + j + P.seed * 2654435769u) >> 8u) / 16777216.0; }
fn qmul(a: vec4f, b: vec4f) -> vec4f {
  return vec4f(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz), a.w * b.w - dot(a.xyz, b.xyz));
}
fn qrot(q: vec4f, v: vec3f) -> vec3f {
  let t = 2.0 * cross(q.xyz, v);
  return v + q.w * t + cross(q.xyz, t);
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3u) {
  let i = gid.x;
  if (i >= P.count) { return; }
  var pos = P.rootPos.xyz;
  var rot = P.rootQuat;
  var first = 0.0;
  for (var j = 0u; j < P.nlinks; j++) {
    let L = links[j];
    let u = u01(i, j);
    if (j == 0u) { first = u; }
    let a = (L.range.x + u * (L.range.y - L.range.x)) * 0.017453292519943295;
    let h = 0.5 * a;
    let jq = vec4f(L.axis.xyz * sin(h), cos(h));
    pos = pos + qrot(rot, L.pos.xyz);
    rot = qmul(rot, qmul(L.quat, jq));
  }
  outPts[i] = vec4f(pos + qrot(rot, P.tip.xyz), first);
}`;

function pcg(v) {
  const s = (Math.imul(v >>> 0, 747796405) + 2891336453) >>> 0;
  const w = Math.imul(((s >>> ((s >>> 28) + 4)) ^ s) >>> 0, 277803737) >>> 0;
  return ((w >>> 22) ^ w) >>> 0;
}
const u01 = (i, j, seed) => (pcg((Math.imul(i, 16) + j + Math.imul(seed, 2654435769)) >>> 0) >>> 8) / 16777216;

/** spec: { root: {pos, quat}, links: [{pos, quat, axis, lo, hi}] (deg), tip: [x,y,z], count, seed } */
export function reachCpu(spec) {
  const out = new Float32Array(spec.count * 4);
  for (let i = 0; i < spec.count; i++) {
    let pos = spec.root.pos;
    let rot = spec.root.quat;
    let first = 0;
    spec.links.forEach((L, j) => {
      const u = u01(i, j, spec.seed);
      if (j === 0) first = u;
      const p = qRotate(rot, L.pos);
      pos = [pos[0] + p[0], pos[1] + p[1], pos[2] + p[2]];
      rot = qMul(rot, qMul(L.quat, qAxisAngle(L.axis, (L.lo + u * (L.hi - L.lo)) * DEG)));
    });
    const t = qRotate(rot, spec.tip);
    out.set([pos[0] + t[0], pos[1] + t[1], pos[2] + t[2], first], i * 4);
  }
  return out;
}

let gpu = null;
async function getGpu() {
  if (gpu !== null) return gpu;
  gpu = false;
  try {
    if (!globalThis.navigator?.gpu) return gpu;
    const adapter = await navigator.gpu.requestAdapter();
    if (!adapter) return gpu;
    const device = await adapter.requestDevice();
    const module = device.createShaderModule({ code: WGSL });
    const pipeline = await device.createComputePipelineAsync({ layout: "auto", compute: { module, entryPoint: "main" } });
    gpu = { device, pipeline };
  } catch (e) {
    console.warn("[twin] WebGPU compute unavailable, using CPU reach sampling", e);
    gpu = false;
  }
  return gpu;
}

async function reachGpu(g, spec) {
  const { device, pipeline } = g;
  const params = new ArrayBuffer(64);
  const f = new Float32Array(params);
  const u = new Uint32Array(params);
  f.set([...spec.root.pos, 0], 0);
  f.set(spec.root.quat, 4);
  f.set([...spec.tip, 0], 8);
  u[12] = spec.count;
  u[13] = spec.links.length;
  u[14] = spec.seed;
  const linkData = new Float32Array(MAX_LINKS * 16);
  spec.links.forEach((L, j) => linkData.set([...L.pos, 0, ...L.quat, ...L.axis, 0, L.lo, L.hi, 0, 0], j * 16));
  const mk = (size, usage, data) => {
    const b = device.createBuffer({ size, usage, mappedAtCreation: !!data });
    if (data) {
      new Uint8Array(b.getMappedRange()).set(new Uint8Array(data));
      b.unmap();
    }
    return b;
  };
  const S = GPUBufferUsage;
  const pBuf = mk(64, S.UNIFORM, params);
  const lBuf = mk(linkData.byteLength, S.STORAGE, linkData.buffer);
  const oBuf = mk(spec.count * 16, S.STORAGE | S.COPY_SRC);
  const rBuf = mk(spec.count * 16, S.MAP_READ | S.COPY_DST);
  const bind = device.createBindGroup({
    layout: pipeline.getBindGroupLayout(0),
    entries: [
      { binding: 0, resource: { buffer: pBuf } },
      { binding: 1, resource: { buffer: lBuf } },
      { binding: 2, resource: { buffer: oBuf } },
    ],
  });
  const enc = device.createCommandEncoder();
  const pass = enc.beginComputePass();
  pass.setPipeline(pipeline);
  pass.setBindGroup(0, bind);
  pass.dispatchWorkgroups(Math.ceil(spec.count / 64));
  pass.end();
  enc.copyBufferToBuffer(oBuf, 0, rBuf, 0, spec.count * 16);
  device.queue.submit([enc.finish()]);
  await rBuf.mapAsync(GPUMapMode.READ);
  const out = new Float32Array(rBuf.getMappedRange().slice(0));
  rBuf.unmap();
  for (const b of [pBuf, lBuf, oBuf, rBuf]) b.destroy();
  return out;
}

/** Returns { points: Float32Array (x,y,z,u)*count in MuJoCo frame, backend, ms }. */
export async function computeReach(spec) {
  if (spec.links.length > MAX_LINKS) throw new Error(`reach chain longer than ${MAX_LINKS}`);
  const t0 = performance.now();
  const g = await getGpu();
  if (g) {
    try {
      return { points: await reachGpu(g, spec), backend: "webgpu", ms: performance.now() - t0 };
    } catch (e) {
      console.warn("[twin] WebGPU reach failed, falling back to CPU", e);
    }
  }
  return { points: reachCpu(spec), backend: "cpu", ms: performance.now() - t0 };
}

/** GPU vs CPU agreement on the first `n` samples (only meaningful when WebGPU is present). */
export async function selfTest(spec, n = 256) {
  const g = await getGpu();
  if (!g) return { ok: true, backend: "cpu", note: "WebGPU not available; CPU path only" };
  const small = { ...spec, count: n };
  const a = await reachGpu(g, small);
  const b = reachCpu(small);
  let maxErr = 0;
  for (let i = 0; i < n; i++) {
    for (let k = 0; k < 3; k++) maxErr = Math.max(maxErr, Math.abs(a[i * 4 + k] - b[i * 4 + k]));
  }
  return { ok: maxErr < 1e-4, backend: "webgpu", maxErrM: maxErr, samples: n };
}
