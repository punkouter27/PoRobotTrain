// Procedural audio cues with Tone.js (Web Audio): no sound files, nothing to download beyond the library.
// Cues are spatialised with HRTF panners placed at the joint they are about, so with the robot in front of you
// "left arm near its limit" comes from the left. Off until the operator turns Sound on (browsers require a gesture).
let Tone = null;
let enabled = false;
let bus = null;
let poweredLoop = null;
let lastTick = 0;
const lastCue = new Map();

async function load() {
  Tone ??= await import("tone");
  return Tone;
}

export async function setEnabled(on) {
  const T = await load();
  if (on) {
    await T.start();
    if (!bus) {
      bus = new T.Volume(-8).toDestination();
    }
  } else {
    setPowered(false);
  }
  enabled = on;
  return enabled;
}

export const isEnabled = () => enabled;

/** Listener follows the 3D camera (positions in three.js world units, y up). */
export function setListener(pos, forward, up) {
  if (!enabled || !Tone) return;
  const l = Tone.getListener();
  const set = (p, v) => (p.value !== undefined ? (p.value = v) : null);
  set(l.positionX, pos.x);
  set(l.positionY, pos.y);
  set(l.positionZ, pos.z);
  set(l.forwardX, forward.x);
  set(l.forwardY, forward.y);
  set(l.forwardZ, forward.z);
  set(l.upX, up.x);
  set(l.upY, up.y);
  set(l.upZ, up.z);
}

function voice(at, type = "sine", envelope = { attack: 0.005, decay: 0.15, sustain: 0.1, release: 0.25 }) {
  const out = at
    ? new Tone.Panner3D({ panningModel: "HRTF", positionX: at.x, positionY: at.y, positionZ: at.z, rolloffFactor: 0.3 }).connect(bus)
    : bus;
  const synth = new Tone.Synth({ oscillator: { type }, envelope }).connect(out);
  return {
    synth,
    dispose: (afterS) =>
      setTimeout(() => {
        synth.dispose();
        if (out !== bus) out.dispose();
      }, afterS * 1000),
  };
}

function seq(notes, { at, type, step = 0.11, dur = "16n", vel = 0.7 } = {}) {
  const v = voice(at, type);
  const now = Tone.now();
  notes.forEach((n, i) => v.synth.triggerAttackRelease(n, dur, now + i * step, vel));
  v.dispose(notes.length * step + 1.2);
}

const throttle = (key, ms) => {
  const t = performance.now();
  if (t - (lastCue.get(key) ?? -1e9) < ms) return false;
  lastCue.set(key, t);
  return true;
};

/**
 * kind: virtualStart | practiceStart | realStart | ok | fail | stop | collision | hot | limit
 * at: optional three.js world position; value: 0..1 for 'limit' (1 = at the limit).
 */
export function cue(kind, at = null, value = 0) {
  if (!enabled || !Tone) return;
  switch (kind) {
    case "virtualStart":
      return seq(["C5", "G5"], { at, type: "sine" });
    case "practiceStart":
      return seq(["C5", "E5", "G5"], { at, type: "triangle" });
    case "realStart":
      // distinct, harder timbre: something physical is about to move
      return seq(["A4", "E5", "A4", "E5"], { at, type: "square", step: 0.14, vel: 0.5 });
    case "ok":
      return seq(["C5", "E5", "G5", "C6"], { at, type: "sine", step: 0.08 });
    case "fail":
      return seq(["E4", "C4", "A3"], { at, type: "sawtooth", step: 0.18, dur: "8n", vel: 0.45 });
    case "stop":
      return seq(["A3", "A3"], { at, type: "square", step: 0.2, dur: "8n", vel: 0.5 });
    case "collision":
      if (throttle("collision", 350)) seq(["F#3"], { at, type: "sawtooth", dur: "16n", vel: 0.5 });
      return;
    case "hot":
      if (throttle("hot", 4000)) seq(["D6", "A5"], { at, type: "sine", step: 0.15 });
      return;
    case "limit": {
      // geiger-style ticks that speed up and rise in pitch as a dragged joint nears its limit
      const now = performance.now();
      const gap = 260 - 200 * value;
      if (now - lastTick < gap) return;
      lastTick = now;
      const v = voice(at, "triangle", { attack: 0.001, decay: 0.04, sustain: 0, release: 0.03 });
      v.synth.triggerAttackRelease(400 + 900 * value, "64n", Tone.now(), 0.35 + 0.4 * value);
      v.dispose(0.4);
      return;
    }
    default:
  }
}

/** A low pulse every 1.5 s while the physical robot is powered and moving (job 'run' on real hardware). */
export function setPowered(on) {
  if (!Tone) return;
  if (on && !poweredLoop && enabled) {
    const v = voice(null, "sine", { attack: 0.02, decay: 0.3, sustain: 0, release: 0.2 });
    poweredLoop = new Tone.Loop((time) => v.synth.triggerAttackRelease("A2", "8n", time, 0.6), 1.5).start(0);
    poweredLoop._voice = v;
    Tone.getTransport().start();
  } else if (!on && poweredLoop) {
    poweredLoop.stop();
    poweredLoop.dispose();
    poweredLoop._voice.dispose(0.5);
    poweredLoop = null;
  }
}
