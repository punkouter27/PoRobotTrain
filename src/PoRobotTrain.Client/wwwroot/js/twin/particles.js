// three-nebula particles rendered by its WebGPU/TSL GPURenderer (one instanced draw call).
// Only state-driven effects, nothing decorative:
//   * heat haze rising from motors reported hot by the physical robot (temps_c), rate scales with temperature;
//   * a short spark burst where the Rapier pose check finds a contact for a proposed pose;
//   * a red burst at the primary joint when a job ends in a safety stop / failure.
import System, { Alpha, Color, Emitter, Force, Life, Mass, Position, RadialVelocity, Radius, Rate, Scale, Span, SphereZone, Vector3D } from "three-nebula";
import { GPURenderer } from "three-nebula/webgpu";

export class TwinParticles {
  constructor(THREE, container) {
    this.system = new System();
    this.renderer = new GPURenderer(container, THREE, { maxParticles: 4000 });
    this.system.addRenderer(this.renderer);
    this.haze = new Map(); // joint index -> emitter
  }

  _emitter(rate, life, radius, colorA, colorB, speed, upForce) {
    const e = new Emitter();
    e.setRate(rate)
      .addInitializers([
        new Mass(1),
        new Life(life[0], life[1]),
        new Radius(radius[0], radius[1]),
        new Position(new SphereZone(0, 0, 0, 0.012)),
        new RadialVelocity(speed, new Vector3D(0, 1, 0), 35),
      ])
      .addBehaviours([new Alpha(0.85, 0), new Scale(1, 2.2), new Color(colorA, colorB), new Force(0, upForce, 0)]);
    return e;
  }

  /** temps: per policy-index °C (or null); positions: per policy-index three.js world position (Vector3). */
  setHeat(temps, positions, warnC = 55) {
    const hot = new Set();
    if (temps) {
      temps.forEach((t, i) => {
        if (t == null || t < warnC || !positions[i]) return;
        hot.add(i);
        const k = Math.min(1, (t - warnC) / 20);
        let e = this.haze.get(i);
        if (!e) {
          e = this._emitter(new Rate(new Span(1, 2), new Span(0.08, 0.14)), [0.8, 1.4], [0.018, 0.03], "#ffb347", "#ff3b30", 0.08, 0.6);
          e.emit();
          this.system.addEmitter(e);
          this.haze.set(i, e);
        }
        const level = Math.round(3 * k);
        if (e._level !== level) {
          e.setRate(new Rate(new Span(1, 1 + level), new Span(0.05, 0.12)));
          e._level = level;
        }
        e.setPosition(positions[i]);
      });
    }
    for (const [i, e] of this.haze) {
      if (!hot.has(i)) {
        e.stopEmit();
        this.system.removeEmitter(e);
        this.haze.delete(i);
      }
    }
  }

  _burst(pos, colorA, colorB, count, speed) {
    const e = this._emitter(new Rate(new Span(count, count), new Span(0.01)), [0.3, 0.6], [0.008, 0.016], colorA, colorB, speed, -1.5);
    e.setPosition(pos);
    e.emit(1);
    this.system.addEmitter(e);
    setTimeout(() => this.system.removeEmitter(e), 1200);
  }

  sparks(pos) {
    this._burst(pos, "#fff3b0", "#ff6a00", 24, 0.6);
  }

  fault(pos) {
    this._burst(pos, "#ff5c57", "#7a0000", 60, 0.9);
  }

  tick(dt) {
    this.system.tick(dt);
  }

  dispose() {
    this.system.destroy();
  }
}
