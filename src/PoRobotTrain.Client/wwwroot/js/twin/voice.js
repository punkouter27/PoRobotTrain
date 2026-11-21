// Voice input + spoken replies for the control UI (idea #11, docs/service_api.md §1).
// Speech to text: the Web Speech API (Chrome / Edge) when present, else Whisper tiny.en through Transformers.js,
// recorded locally at 16 kHz (one-time ~40 MB model download). Text to speech: speechSynthesis.
// This module only turns speech into text and text into speech. What the text may do is decided by the caller,
// and a robot behaviour is only ever *selected* by voice, never started.
const WHISPER_MODEL = "Xenova/whisper-tiny.en";
const MAX_RECORD_MS = 6000;

const SR = globalThis.SpeechRecognition ?? globalThis.webkitSpeechRecognition ?? null;
let muted = false;
let active = null; // { stop(): void }
let whisper = null;
let whisperLoading = null;

export function availability() {
  return {
    recognition: !!SR,
    synthesis: "speechSynthesis" in globalThis,
    recorder: !!(navigator.mediaDevices?.getUserMedia && globalThis.MediaRecorder),
  };
}

export const availabilityJson = () => JSON.stringify(availability());

/** Which engine startListening will use: "webspeech" | "whisper" | "none". */
export function engine(forceWhisper = false) {
  const a = availability();
  if (a.recognition && !forceWhisper && !new URLSearchParams(location.search).has("whisper")) return "webspeech";
  return a.recorder ? "whisper" : "none";
}

/**
 * Start one utterance. Callbacks on the .NET object: OnVoiceState(state, detail), OnVoicePartial(text),
 * OnVoiceFinal(text). state: "listening" | "loading" | "transcribing" | "idle" | "error".
 */
export async function startListening(dotnet, forceWhisper = false) {
  stopListening();
  const which = engine(forceWhisper);
  const call = (m, ...a) => dotnet.invokeMethodAsync(m, ...a).catch(() => {});
  if (which === "webspeech") return startWebSpeech(call);
  if (which === "whisper") return startWhisper(call);
  call("OnVoiceState", "error", "No speech input in this browser (no Web Speech API, no microphone recorder).");
  return "none";
}

export function stopListening() {
  const a = active;
  active = null;
  a?.stop();
}

function startWebSpeech(call) {
  const rec = new SR();
  rec.lang = "en-US";
  rec.interimResults = true;
  rec.continuous = false;
  rec.maxAlternatives = 1;
  let final = "";
  let done = false;
  const finish = () => {
    if (done) return;
    done = true;
    if (active?.rec === rec) active = null;
    if (final.trim()) call("OnVoiceFinal", final.trim());
    call("OnVoiceState", "idle", "");
  };
  rec.onresult = (e) => {
    let interim = "";
    for (let i = e.resultIndex; i < e.results.length; i++) {
      const r = e.results[i];
      if (r.isFinal) final += r[0].transcript;
      else interim += r[0].transcript;
    }
    call("OnVoicePartial", (final + interim).trim());
  };
  rec.onerror = (e) => {
    if (done) return;
    done = true;
    if (active?.rec === rec) active = null;
    const msg = e.error === "not-allowed" ? "Microphone permission denied." : e.error === "no-speech" ? "No speech heard." : `Speech recognition: ${e.error}`;
    call("OnVoiceState", "error", msg);
  };
  rec.onend = finish;
  active = { rec, stop: () => rec.stop() };
  rec.start();
  call("OnVoiceState", "listening", "Web Speech API");
  return "webspeech";
}

async function loadWhisper(onProgress) {
  whisperLoading ??= (async () => {
    const tf = await import("@huggingface/transformers");
    tf.env.allowLocalModels = false;
    whisper = await tf.pipeline("automatic-speech-recognition", WHISPER_MODEL, {
      dtype: "q8",
      progress_callback: (p) => onProgress?.(p),
    });
    return whisper;
  })();
  try {
    return await whisperLoading;
  } catch (e) {
    whisperLoading = null; // allow a retry after a failed download
    throw e;
  }
}

async function startWhisper(call) {
  let stream;
  try {
    stream = await navigator.mediaDevices.getUserMedia({ audio: { channelCount: 1, echoCancellation: true, noiseSuppression: true } });
  } catch (e) {
    call("OnVoiceState", "error", `Microphone unavailable: ${e?.message ?? e}`);
    return "whisper";
  }
  // load the model while the user speaks
  const modelReady = loadWhisper((p) => {
    if (p?.status === "progress" && p.total) call("OnVoiceState", "loading", `Whisper model ${Math.round((100 * p.loaded) / p.total)}%`);
  });
  modelReady.catch(() => {});
  const chunks = [];
  const recorder = new MediaRecorder(stream);
  recorder.ondataavailable = (e) => e.data.size && chunks.push(e.data);
  const timer = setTimeout(() => recorder.state === "recording" && recorder.stop(), MAX_RECORD_MS);
  recorder.onstop = async () => {
    clearTimeout(timer);
    stream.getTracks().forEach((t) => t.stop());
    if (active?.recorder === recorder) active = null;
    try {
      call("OnVoiceState", "transcribing", "Whisper (local)");
      const audio = await decode16k(new Blob(chunks, { type: recorder.mimeType }));
      const asr = await modelReady;
      const out = await asr(audio);
      const text = (Array.isArray(out) ? out[0]?.text : out?.text)?.trim() ?? "";
      if (text && !/^\[.*\]$/.test(text)) call("OnVoiceFinal", text);
      call("OnVoiceState", text ? "idle" : "error", text ? "" : "No speech heard.");
    } catch (e) {
      call("OnVoiceState", "error", `Whisper: ${e?.message ?? e}`);
    }
  };
  active = { recorder, stop: () => recorder.state === "recording" && recorder.stop() };
  recorder.start();
  call("OnVoiceState", "listening", `Whisper (local), up to ${MAX_RECORD_MS / 1000} s`);
  return "whisper";
}

async function decode16k(blob) {
  const buf = await blob.arrayBuffer();
  const ctx = new AudioContext({ sampleRate: 16000 });
  try {
    const audio = await ctx.decodeAudioData(buf);
    return audio.getChannelData(0).slice();
  } finally {
    ctx.close();
  }
}

export function setMuted(m) {
  muted = !!m;
  if (muted) globalThis.speechSynthesis?.cancel();
  return muted;
}

export function speak(text) {
  if (muted || !text || !("speechSynthesis" in globalThis)) return false;
  const synth = globalThis.speechSynthesis;
  synth.cancel(); // newest reply wins
  const u = new SpeechSynthesisUtterance(text);
  u.lang = "en-US";
  u.rate = 1.05;
  const voice = synth.getVoices().find((v) => v.lang?.startsWith("en") && /natural|online/i.test(v.name)) ?? synth.getVoices().find((v) => v.lang?.startsWith("en"));
  if (voice) u.voice = voice;
  synth.speak(u);
  return true;
}
