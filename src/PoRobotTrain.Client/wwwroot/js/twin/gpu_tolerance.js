// How closely the WebGPU reach kernel must agree with the CPU path, per adapter (owner decision 2026-11-24):
// 0.1 mm on a discrete GPU (and whenever the adapter cannot be classified), up to 0.5 mm on an integrated GPU, whose
// shader compilers fuse and reorder f32 math more freely. /diag shows which adapter ran and which tolerance applied.
// No imports: node --test runs it (reach_tolerance.test.mjs, from tests/test_reach_tolerance.py).

export const DISCRETE_TOL_M = 1e-4; // 0.1 mm
export const INTEGRATED_TOL_M = 5e-4; // 0.5 mm

/**
 * Classify a WebGPU adapter from its GPUAdapterInfo (vendor, architecture, device, description, and Chromium's
 * optional `type`, e.g. "discrete GPU" / "integrated GPU"). Returns { kind: "discrete"|"integrated"|"unknown", label }.
 */
export function classifyAdapter(info) {
  const i = info ?? {};
  const label =
    [i.vendor, i.architecture, i.device, i.description].filter((s) => typeof s === "string" && s.trim()).join(" ").trim() ||
    "unknown adapter";
  const type = String(i.type ?? "").toLowerCase();
  if (type.includes("integrated")) return { kind: "integrated", label };
  if (type.includes("discrete")) return { kind: "discrete", label };
  const text = label.toLowerCase();
  const vendor = String(i.vendor ?? "").toLowerCase();
  // Intel Arc and AMD/NVIDIA cards are discrete; Intel UHD/Iris/Xe, Apple, Qualcomm/Adreno, ARM/Mali, AMD APU
  // "Radeon Graphics"/Vega iGPUs are integrated.
  if (/\barc\b|alchemist|battlemage|xe-hpg/.test(text)) return { kind: "discrete", label };
  if (vendor === "intel" || /\bintel\b|\buhd\b|\biris\b/.test(text)) return { kind: "integrated", label };
  if (/apple|qualcomm|adreno|\barm\b|mali|powervr|imagination/.test(vendor + " " + text)) return { kind: "integrated", label };
  if (/radeon\(tm\) graphics|radeon graphics|\bvega \d+\b|780m|680m|760m|890m/.test(text)) return { kind: "integrated", label };
  if (/nvidia|geforce|rtx|quadro|tesla/.test(vendor + " " + text)) return { kind: "discrete", label };
  if (/\bamd\b|\bati\b|radeon/.test(vendor + " " + text)) return { kind: "discrete", label };
  return { kind: "unknown", label };
}

/** The GPU/CPU agreement tolerance (metres) for an adapter kind. Unknown adapters get the strict discrete value. */
export function toleranceFor(kind) {
  return kind === "integrated" ? INTEGRATED_TOL_M : DISCRETE_TOL_M;
}
