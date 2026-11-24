// node --test: the per-adapter WebGPU reach tolerance (gpu_tolerance.js). Run by tests/test_reach_tolerance.py.
import { test } from "node:test";
import assert from "node:assert/strict";
import { classifyAdapter, toleranceFor, DISCRETE_TOL_M, INTEGRATED_TOL_M } from "./gpu_tolerance.js";

test("discrete GPUs keep the 0.1 mm tolerance", () => {
  for (const info of [
    { vendor: "nvidia", architecture: "turing", description: "NVIDIA GeForce RTX 2060" },
    { vendor: "amd", architecture: "rdna-3", description: "AMD Radeon RX 7900 XT" },
    { vendor: "intel", architecture: "xe-hpg", description: "Intel(R) Arc(TM) A770 Graphics" },
    { type: "discrete GPU", vendor: "intel" },
  ]) {
    const c = classifyAdapter(info);
    assert.equal(c.kind, "discrete", JSON.stringify(info));
    assert.equal(toleranceFor(c.kind), 1e-4);
  }
});

test("integrated GPUs get up to 0.5 mm", () => {
  for (const info of [
    { vendor: "intel", architecture: "gen-12lp", description: "Intel(R) UHD Graphics" },
    { vendor: "apple", architecture: "metal-3" },
    { vendor: "qualcomm", architecture: "adreno-7xx" },
    { vendor: "amd", description: "AMD Radeon(TM) Graphics" },
    { type: "integrated GPU", vendor: "nvidia" },
  ]) {
    const c = classifyAdapter(info);
    assert.equal(c.kind, "integrated", JSON.stringify(info));
    assert.equal(toleranceFor(c.kind), 5e-4);
  }
});

test("an adapter it cannot classify keeps the strict tolerance", () => {
  for (const info of [undefined, null, {}, { vendor: "", description: "" }, { vendor: "mystery" }]) {
    const c = classifyAdapter(info);
    assert.equal(c.kind, "unknown");
    assert.equal(toleranceFor(c.kind), DISCRETE_TOL_M);
  }
  assert.equal(classifyAdapter({}).label, "unknown adapter");
});

test("the label names the adapter for /diag", () => {
  assert.equal(classifyAdapter({ vendor: "nvidia", architecture: "turing" }).label, "nvidia turing");
});

test("the bounds are the owner's: 0.1 mm discrete, 0.5 mm integrated", () => {
  assert.equal(DISCRETE_TOL_M, 0.0001);
  assert.equal(INTEGRATED_TOL_M, 0.0005);
  // a 0.3 mm GPU/CPU difference passes only on an integrated adapter
  assert.ok(0.0003 < toleranceFor("integrated"));
  assert.ok(!(0.0003 < toleranceFor("discrete")));
});
