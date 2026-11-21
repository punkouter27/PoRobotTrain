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
// Spoken replies (`say`) and walk intents follow docs/service_api.md §1 so the Unreal twin (service matcher) and
// this page answer the same way. Walk intents apply only while the walk sandbox (simulation only) is active.
const WALK_STEP = { vx: 0.5, vxFast: 1.0, vxSlow: 0.25, vy: 0.3, wz: 0.6 };
const WALK_RULES = [
  [/\b(stop|halt|freeze|stand still)\b/i, { stop: true }, "Stopping."],
  [/\b(turn|turning|rotate|spin)\s+(to\s+the\s+)?left\b/i, { vx: 0, vy: 0, wz: WALK_STEP.wz }, "Turning left."],
  [/\b(turn|turning|rotate|spin)\s+(to\s+the\s+)?right\b/i, { vx: 0, vy: 0, wz: -WALK_STEP.wz }, "Turning right."],
  [/\b(faster|run|speed up)\b/i, { vx: WALK_STEP.vxFast, vy: 0, wz: 0 }, "Walking faster."],
  [/\b(slower|slow down)\b/i, { vx: WALK_STEP.vxSlow, vy: 0, wz: 0 }, "Walking slower."],
  [/\b(back|backward|backwards|reverse)\b/i, { vx: -WALK_STEP.vx, vy: 0, wz: 0 }, "Walking backward."],
  [/\bleft\b/i, { vx: 0, vy: WALK_STEP.vy, wz: 0 }, "Stepping left."],
  [/\bright\b/i, { vx: 0, vy: -WALK_STEP.vy, wz: 0 }, "Stepping right."],
  [/\b(forward|forwards|ahead|go|walk|straight)\b/i, { vx: WALK_STEP.vx, vy: 0, wz: 0 }, "Walking forward."],
];

/** Walk intent (lexical, no model needed) or null. */
export function interpretWalk(text) {
  const q = (text ?? "").trim();
  for (const [re, walk, say] of WALK_RULES) {
    if (re.test(q)) return { kind: "walk", walk, score: 1, blocked: false, note: say, say };
  }
  return null;
}

/** Walk sandbox entry point for Blazor: text -> walk intent or an "unknown" reply (never loads the MiniLM model). */
export function interpretWalkJson(text) {
  return JSON.stringify(
    interpretWalk(text) ?? {
      kind: "unknown",
      blocked: false,
      note: "Walk commands: forward, back, left, right, turn left, turn right, faster, slower, stop.",
      say: "Try: walk forward, turn left, or stop.",
    }
  );
}

const ONES = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen"];
const TENS = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"];
const NUM_WORD_RE = new RegExp(`\\b(${[...ONES, ...TENS.slice(2)].join("|")})(?:[\\s-]+(${ONES.slice(1, 10).join("|")}))?\\b`, "gi");

/** 20 -> "twenty", 25 -> "twenty five" (spoken replies avoid digits and symbols). */
export function numberWords(n) {
  const v = Math.round(Math.abs(n));
  if (v < 20) return ONES[v];
  if (v < 100) return TENS[Math.floor(v / 10)] + (v % 10 ? " " + ONES[v % 10] : "");
  return String(v);
}

/** "twenty five degrees" -> "25 degrees" so speech-recognised numbers parse like typed ones. */
export function wordsToDigits(text) {
  return text.replace(NUM_WORD_RE, (m, a, b) => {
    const t = TENS.indexOf(a.toLowerCase());
    if (t >= 2) return String(t * 10 + (b ? ONES.indexOf(b.toLowerCase()) : 0));
    const o = ONES.indexOf(a.toLowerCase());
    return b || o < 10 ? m : String(o); // "one joint" stays words; only angle-sized numbers (10+) are converted
  });
}

/** "L Shld Pitch" -> "left shoulder pitch" for speech. */
function spokenLabel(label) {
  return (label ?? "")
    .replace(/\bL\b\.?/g, "left")
    .replace(/\bR\b\.?/g, "right")
    .replace(/\bShld\b/gi, "shoulder")
    .replace(/\s+/g, " ")
    .trim();
}

function sayFor(r) {
  if (r.kind === "behaviour") {
    if (r.blocked) return `${spokenLabel(r.label)} needs arm motions unlocked first.`;
    const amp = r.amplitude != null ? `, ${numberWords(r.amplitude)} degrees` : "";
    return `${spokenLabel(r.label)}${amp}, selected. Run it on the virtual robot first.`;
  }
  if (r.kind === "joint") return r.locked ? `${spokenLabel(r.label)} is locked on the stand.` : `Use the ${spokenLabel(r.label)} slider.`;
  return "Sorry, I did not catch a safe motion. Try: turn the head twenty degrees.";
}

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
export async function interpret(text, behaviours, joints, sizes, armUnlocked, walkActive = false) {
  const raw = (text ?? "").trim();
  if (walkActive) {
    const w = interpretWalk(raw);
    if (w) return w;
  }
  const r = await interpretSelection(wordsToDigits(raw), behaviours, joints, sizes, armUnlocked);
  return { ...r, say: sayFor(r) };
}

async function interpretSelection(text, behaviours, joints, sizes, armUnlocked) {
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
      locked,
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

/** JSON-string entry point for Blazor: request { text, behaviours, joints, sizes, armUnlocked, walkActive } -> suggestion. */
export async function interpretJson(json) {
  const r = JSON.parse(json);
  return JSON.stringify(await interpret(r.text, r.behaviours, r.joints, r.sizes, r.armUnlocked, r.walkActive));
}
