// Plain-language behaviour picker with Transformers.js (MiniLM sentence embeddings, runs locally in the browser
// after a one-time ~23 MB model download from Hugging Face). It only *suggests* a selection: a behaviour + sweep
// size, or "use this joint's slider". It never starts a run; Steps 1 + 2 and the confirm dialog still apply.
// A request about one joint is answered with that joint's slider, never with a multi-joint behaviour.
const MODEL = "Xenova/all-MiniLM-L6-v2";
const SYNONYMS = {
  neck_turn: "turn the head left and right, look left, look right, pan the head, shake head no, neck yaw",
  neck_nod: "nod the head up and down, look up, look down, nod yes, tilt head, neck pitch",
  arm_raise: "raise the right arm forward, lift the right arm up, arm up, reach forward, right shoulder",
  hand_wave: "wave the right hand, wave hello, greet, say hi, wave goodbye",
};
const LEFT_RE = /\b(left|l\.)\b/i;
const RIGHT_RE = /\b(right|r\.)\b/i;

let extractor = null;
let loading = null;

export function isLoaded() {
  return extractor !== null;
}

export async function load(onProgress) {
  loading ??= (async () => {
    const tf = await import("@huggingface/transformers");
    tf.env.allowLocalModels = false;
    extractor = await tf.pipeline("feature-extraction", MODEL, {
      dtype: "q8",
      progress_callback: (p) => onProgress?.(p),
    });
    return true;
  })();
  return loading;
}

async function embed(texts) {
  const t = await extractor(texts, { pooling: "mean", normalize: true });
  return t.tolist();
}

const cos = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

function parseDegrees(text) {
  const m = text.match(/(-?\d+(?:\.\d+)?)\s*(°|deg\b|degs\b|degree|degrees)?/i);
  return m ? Math.abs(parseFloat(m[1])) : null;
}

/**
 * behaviours: [{ id, label, desc, maxDeg }], joints: [{ name, label, group, minDeg, maxDeg }], sizes: [10, 20, 30]
 * Returns { kind: 'behaviour'|'joint'|'unknown', ... , score, note }.
 */
export async function interpret(text, behaviours, joints, sizes, armUnlocked) {
  if (!extractor) throw new Error("assistant model not loaded");
  const q = text.trim();
  if (!q) return { kind: "unknown", note: "Type what you want the robot to do." };
  const bTexts = behaviours.map((b) => `${b.label}. ${b.desc}. ${SYNONYMS[b.id] ?? ""}`);
  const jTexts = joints.map((j) => `move the ${j.label.replace(/\bL\b/, "left").replace(/\bR\b/, "right")} joint (${j.name.replaceAll("_", " ")})`);
  const [qe, ...rest] = await embed([q, ...bTexts, ...jTexts]);
  const bScores = behaviours.map((_, i) => cos(qe, rest[i]));
  const mentionsLeft = LEFT_RE.test(q) && !RIGHT_RE.test(q);
  const mentionsRight = RIGHT_RE.test(q) && !LEFT_RE.test(q);
  // Sentence embeddings barely separate "left" from "right", so the side is taken from the words themselves.
  const sideOk = (name) =>
    (!mentionsLeft || !name.startsWith("right_")) && (!mentionsRight || !name.startsWith("left_"));
  const jScores = joints.map((j, i) => (sideOk(j.name) ? cos(qe, rest[behaviours.length + i]) : -1));
  const bi = bScores.indexOf(Math.max(...bScores));
  const ji = jScores.indexOf(Math.max(...jScores));
  const deg = parseDegrees(q);

  const jointWins = jScores[ji] > bScores[bi] + 0.04 || /\b(elbow|wrist|knee|hip|ankle|waist|slider)\b/i.test(q);
  if (jointWins && jScores[ji] >= 0.3) {
    const j = joints[ji];
    const locked = /leg|waist/i.test(j.group);
    return {
      kind: "joint",
      joint: j.name,
      label: j.label,
      deg: deg != null && !locked ? Math.min(j.maxDeg, Math.max(j.minDeg, deg)) : null,
      score: jScores[ji],
      note: locked
        ? `${j.label} is stand-locked (read-only); it can be watched, not moved.`
        : `Single joint: use the ${j.label} slider (arm write must be enabled; only that joint is powered).`,
    };
  }
  if (bScores[bi] < 0.3) {
    return { kind: "unknown", score: bScores[bi], note: "No stand-safe behaviour matches that. Try 'turn the head 20 degrees'." };
  }
  const b = behaviours[bi];
  const isArm = b.id === "arm_raise" || b.id === "hand_wave";
  const allowed = sizes.filter((s) => s <= b.maxDeg + 1e-6);
  let amplitude = null;
  if (deg != null && allowed.length) {
    amplitude = allowed.reduce((best, s) => (Math.abs(s - deg) < Math.abs(best - deg) ? s : best), allowed[0]);
  }
  let note = `${b.label}${amplitude ? ` at ${amplitude}°` : ""}. Still needs Step 1 (virtual) and Step 2 (check) before any run.`;
  if (deg != null && amplitude != null && Math.abs(amplitude - deg) > 0.5) note += ` (${deg}° snapped to the nearest allowed size.)`;
  if (isArm && mentionsLeft) note = `Only right-arm behaviours exist. For the left arm use the left-arm sliders. Closest: ${b.label}.`;
  if (isArm && !armUnlocked) note += " Tick 'Unlock arm motions' first.";
  return { kind: "behaviour", id: b.id, label: b.label, amplitude, score: bScores[bi], blocked: isArm && !armUnlocked, note };
}

/** JSON-string entry point for Blazor: request { text, behaviours, joints, sizes, armUnlocked } -> suggestion. */
export async function interpretJson(json) {
  const r = JSON.parse(json);
  return JSON.stringify(await interpret(r.text, r.behaviours, r.joints, r.sizes, r.armUnlocked));
}
