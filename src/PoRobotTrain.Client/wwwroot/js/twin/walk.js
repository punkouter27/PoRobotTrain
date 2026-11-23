// Level 2 walk sandbox driver for the control UI (idea #20, docs/service_api.md §3). Simulation only.
// - polls GET /api/robot/walk/state at ~20 Hz while the page is visible (2 Hz while the sandbox is idle) and draws
//   it on the three.js twin in walk mode; the .NET page gets a throttled copy for its HUD
// - turns keyboard (WASD / arrows / Q E, Shift = full speed), gamepad (left stick vx/vy, right stick wz) and the
//   on-screen pad into a velocity command, POSTed to /api/robot/walk/command at ~20 Hz while held, then zeros once
//   on release. The service watchdog zeroes the command after 0.5 s without one, so a lost page stops the robot.
// - a voice command is held (re-sent) until "stop" or any manual input
import { create as createTwin } from "./twin.js";

const SEND_MS = 50;
const POLL_ACTIVE_MS = 50;
const POLL_IDLE_MS = 500;
const HUD_MS = 200;
const DEADZONE = 0.15;
const DEFAULT_LIMITS = { vx: [-0.5, 1.0], vy: [-0.5, 0.5], wz: [-1.0, 1.0] };
// keyboard / pad speeds as a fraction of the limit; Shift = 1.0
const KEY_FRACTION = 0.6;
const KEYS = {
  KeyW: ["vx", 1], ArrowUp: ["vx", 1],
  KeyS: ["vx", -1], ArrowDown: ["vx", -1],
  KeyA: ["vy", 1], KeyD: ["vy", -1],
  KeyQ: ["wz", 1], ArrowLeft: ["wz", 1],
  KeyE: ["wz", -1], ArrowRight: ["wz", -1],
};

const isTyping = () => {
  const el = document.activeElement;
  return el && (el.tagName === "INPUT" || el.tagName === "TEXTAREA" || el.isContentEditable);
};
const dz = (v) => (Math.abs(v) < DEADZONE ? 0 : Math.sign(v) * (Math.abs(v) - DEADZONE) / (1 - DEADZONE));
const scale = (u, [lo, hi]) => (u >= 0 ? u * hi : -u * lo);
const isZero = (c) => Math.abs(c.vx) < 1e-3 && Math.abs(c.vy) < 1e-3 && Math.abs(c.wz) < 1e-3;
const round = (v) => Math.round(v * 1000) / 1000;

class WalkDriver {
  constructor(dotnet) {
    this.dotnet = dotnet;
    this.keys = new Set();
    this.pad = new Set(); // on-screen buttons held: "vx+", "wz-", ...
    this.stick = { vx: 0, vy: 0, wz: 0 }; // on-screen thumbsticks (analog, -1..1)
    this.shift = false;
    this.voice = null; // held {vx, vy, wz}
    this.driveEnabled = false;
    this.limits = DEFAULT_LIMITS;
    this.lastSent = { vx: 0, vy: 0, wz: 0 };
    this.zeroRepeats = 0;
    this.sendBusy = false;
    this.pollBusy = false;
    this.state = null;
    this.error = null;
    this.input = { vx: 0, vy: 0, wz: 0, source: "none", gamepad: null };
    this.lastHud = 0;
    this.disposed = false;
  }

  async init(canvas) {
    try {
      this.twin = await createTwin(canvas, { base: "twin/", mode: "walk" });
    } catch (e) {
      this.twinError = String(e?.message ?? e);
      console.warn("[walk] 3D view unavailable", e);
    }
    this._onKey = (e) => this._key(e);
    window.addEventListener("keydown", this._onKey);
    window.addEventListener("keyup", this._onKey);
    this._onBlur = () => {
      this.keys.clear();
      this.pad.clear();
    };
    window.addEventListener("blur", this._onBlur);
    this._onVis = () => {
      if (document.hidden) {
        this._onBlur();
        this.voice = null;
        this._post({ vx: 0, vy: 0, wz: 0 }); // stop driving while hidden; polling pauses below
      }
    };
    document.addEventListener("visibilitychange", this._onVis);
    this.sendTimer = setInterval(() => this._sendTick(), SEND_MS);
    this._pollLoop();
    return this.twinError ?? "";
  }

  _key(e) {
    this.shift = e.shiftKey;
    if (!(e.code in KEYS) || isTyping()) return;
    if (!this.driveEnabled) return;
    e.preventDefault();
    if (e.type === "keydown") {
      this.keys.add(e.code);
      this.voice = null; // manual input overrides a held voice command
    } else this.keys.delete(e.code);
  }

  _gamepad() {
    const pads = navigator.getGamepads?.() ?? [];
    for (const g of pads) {
      if (!g || !g.connected) continue;
      const [lx = 0, ly = 0, rx = 0] = [g.axes[0], g.axes[1], g.axes[2]];
      const u = { vx: dz(-ly), vy: dz(-lx), wz: dz(-rx) };
      return { id: g.id, u };
    }
    return null;
  }

  _compute() {
    const L = this.limits;
    const gp = this._gamepad();
    if (gp && !isZero(gp.u)) {
      this.voice = null;
      return { vx: scale(gp.u.vx, L.vx), vy: scale(gp.u.vy, L.vy), wz: scale(gp.u.wz, L.wz), source: "gamepad", gamepad: gp.id };
    }
    if (!isZero(this.stick)) {
      this.voice = null;
      const s = this.stick;
      return { vx: scale(s.vx, L.vx), vy: scale(s.vy, L.vy), wz: scale(s.wz, L.wz), source: "touch sticks", gamepad: gp?.id ?? null };
    }
    const u = { vx: 0, vy: 0, wz: 0 };
    for (const code of this.keys) {
      const [axis, sign] = KEYS[code];
      u[axis] = Math.max(-1, Math.min(1, u[axis] + sign));
    }
    for (const p of this.pad) u[p.slice(0, 2)] = p.endsWith("+") ? 1 : -1;
    if (!isZero(u)) {
      const f = this.shift ? 1 : KEY_FRACTION;
      const src = this.pad.size ? "on-screen pad" : "keyboard";
      return { vx: f * scale(u.vx, L.vx), vy: f * scale(u.vy, L.vy), wz: f * scale(u.wz, L.wz), source: src, gamepad: gp?.id ?? null };
    }
    if (this.voice) return { ...this.voice, source: "voice", gamepad: gp?.id ?? null };
    return { vx: 0, vy: 0, wz: 0, source: "none", gamepad: gp?.id ?? null };
  }

  _sendTick() {
    if (this.disposed || document.hidden) return;
    const c = this._compute();
    this.input = c;
    if (!this.driveEnabled) return;
    const cmd = { vx: round(c.vx), vy: round(c.vy), wz: round(c.wz) };
    if (!isZero(cmd)) {
      this.zeroRepeats = 2; // after release, send zeros twice
      this._post(cmd);
    } else if (this.zeroRepeats > 0) {
      this.zeroRepeats--;
      this._post(cmd);
    }
  }

  async _post(cmd) {
    if (this.sendBusy && !isZero(cmd)) return; // never queue stale motion; zeros may overlap
    this.sendBusy = true;
    try {
      const r = await fetch("/api/robot/walk/command", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(cmd) });
      this.lastSent = cmd;
      if (!r.ok && r.status >= 500) this.error = `command HTTP ${r.status}`;
    } catch (e) {
      this.error = `command: ${e?.message ?? e}`;
    } finally {
      this.sendBusy = false;
    }
  }

  async _pollLoop() {
    while (!this.disposed) {
      const running = !!this.state?.running;
      if (!document.hidden) await this._pollOnce();
      await new Promise((res) => setTimeout(res, running ? POLL_ACTIVE_MS : POLL_IDLE_MS));
    }
  }

  async _pollOnce() {
    if (this.pollBusy) return;
    this.pollBusy = true;
    let error = null;
    try {
      const r = await fetch("/api/robot/walk/state", { cache: "no-store" });
      if (r.ok) {
        this.state = await r.json();
        if (this.state?.cmd_limits) this.limits = { ...DEFAULT_LIMITS, ...this.state.cmd_limits };
        this.twin?.setWalk(this.state);
      } else {
        let detail = `HTTP ${r.status}`;
        try {
          detail = (await r.json()).detail ?? detail;
        } catch {
          /* not problem+json */
        }
        error = detail;
      }
    } catch (e) {
      error = `walk state: ${e?.message ?? e}`;
    } finally {
      this.pollBusy = false;
    }
    if (error) this.state = null;
    this.error = error ?? (this.error?.startsWith("command") ? this.error : null);
    const now = performance.now();
    if (now - this.lastHud >= HUD_MS || error) {
      this.lastHud = now;
      const input = { vx: round(this.input.vx), vy: round(this.input.vy), wz: round(this.input.wz), source: this.input.source, gamepad: this.input.gamepad };
      this.dotnet
        .invokeMethodAsync("OnWalkFrame", this.state ? JSON.stringify(this.state) : "", JSON.stringify(input), this.error ?? "")
        .catch(() => {});
    }
  }

  // ---- API used from Blazor
  setDriveEnabled(on) {
    this.driveEnabled = !!on;
    if (!on) {
      this.keys.clear();
      this.pad.clear();
      this.stick = { vx: 0, vy: 0, wz: 0 };
      this.voice = null;
    }
  }

  /**
   * Feature #9: two on-screen thumbsticks (pointer/touch). Left stick: up/down = vx, left/right = vy;
   * right stick: left/right = wz (turn). Releasing a stick springs it back to zero.
   */
  attachSticks(leftEl, rightEl) {
    const bind = (el, apply) => {
      if (!el) return;
      const knob = el.querySelector(".stick__knob");
      let id = null;
      const move = (e) => {
        if (e.pointerId !== id) return;
        const r = el.getBoundingClientRect();
        const R = r.width / 2;
        let dx = (e.clientX - (r.left + R)) / R, dy = (e.clientY - (r.top + R)) / R;
        const m = Math.hypot(dx, dy);
        if (m > 1) { dx /= m; dy /= m; }
        const dead = (v) => (Math.abs(v) < 0.12 ? 0 : v);
        if (knob) knob.style.transform = `translate(${dx * R * 0.6}px, ${dy * R * 0.6}px)`;
        if (this.driveEnabled) apply(dead(dx), dead(dy));
      };
      const end = (e) => {
        if (e.pointerId !== id) return;
        id = null;
        if (knob) knob.style.transform = "";
        apply(0, 0);
      };
      el.addEventListener("pointerdown", (e) => {
        if (!this.driveEnabled) return;
        id = e.pointerId;
        el.setPointerCapture(id);
        move(e);
      });
      el.addEventListener("pointermove", move);
      el.addEventListener("pointerup", end);
      el.addEventListener("pointercancel", end);
    };
    bind(leftEl, (x, y) => { this.stick.vx = -y; this.stick.vy = -x; });
    bind(rightEl, (x) => { this.stick.wz = -x; });
  }

  setPad(dir, down) {
    if (!this.driveEnabled) return;
    if (down) {
      this.pad.add(dir);
      this.voice = null;
    } else this.pad.delete(dir);
  }

  /** Voice intent {vx, vy, wz} or {stop: true}; held until stop or manual input. */
  setVoice(json) {
    const w = JSON.parse(json || "null");
    if (!w || w.stop) {
      this.voice = null;
      return;
    }
    const L = this.limits;
    const clamp = (v, [lo, hi]) => Math.max(lo, Math.min(hi, v ?? 0));
    this.voice = { vx: clamp(w.vx, L.vx), vy: clamp(w.vy, L.vy), wz: clamp(w.wz, L.wz) };
  }

  refresh() {
    return this._pollOnce();
  }

  /** #2: draw CoM, capture point, support polygon and contact forces on the 3D view. */
  setPhysicsOverlay(on) {
    this.twin?.setPhysicsOverlay(!!on);
  }

  dispose() {
    this.disposed = true;
    clearInterval(this.sendTimer);
    window.removeEventListener("keydown", this._onKey);
    window.removeEventListener("keyup", this._onKey);
    window.removeEventListener("blur", this._onBlur);
    document.removeEventListener("visibilitychange", this._onVis);
    if (this.driveEnabled && !isZero(this.lastSent)) this._post({ vx: 0, vy: 0, wz: 0 });
    this.twin?.dispose();
  }


  /** "" when the 3D view is up, else why it is not (the HUD and driving still work). */
  twinStatus() {
    return this.twinError ?? "";
  }
}

export async function createDriver(canvas, dotnet) {
  const d = new WalkDriver(dotnet);
  await d.init(canvas);
  return d;
}
