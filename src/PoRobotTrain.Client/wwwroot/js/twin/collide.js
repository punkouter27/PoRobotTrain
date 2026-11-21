// Rapier (WASM) used purely as a geometry library: no world, no stepping, no dynamics. MuJoCo owns physics.
// It answers one question for a *proposed* pose (e.g. a slider being dragged): would any of MuJoCo's own
// collision capsules interpenetrate, and how close does the arm come to the stand frame (which MuJoCo does not
// collide with)? Pairs that already touch in the default pose are treated as by-design and ignored.
import RAPIER from "@dimforge/rapier3d-compat";
import { compose, forward, qMul } from "./fk.js";

const Y_TO_Z = [Math.SQRT1_2, 0, 0, Math.SQRT1_2]; // Rapier capsules/cylinders run along Y, MuJoCo's along Z

let ready = null;
export function init() {
  ready ??= RAPIER.init().then(() => RAPIER);
  return ready;
}

function makeShape(R, c) {
  switch (c.type) {
    case "sphere":
      return R.Ball ? new R.Ball(c.size[0]) : null;
    case "capsule":
      return new R.Capsule(c.size[1], c.size[0]);
    case "cylinder":
      return new R.Cylinder(c.size[1], c.size[0]);
    case "box":
      return new R.Cuboid(c.size[0], c.size[1], c.size[2]);
    default:
      return null;
  }
}

const needsAxisFix = (t) => t === "capsule" || t === "cylinder";
const vec = (p) => ({ x: p[0], y: p[1], z: p[2] });
const rot = (q) => ({ x: q[0], y: q[1], z: q[2], w: q[3] });

export class PoseChecker {
  constructor(R, model) {
    this.R = R;
    this.model = model;
    this.robot = model.manifest.colliders
      .map((c, i) => ({ ...c, id: i, shape: makeShape(R, c) }))
      .filter((c) => c.shape);
    this.stand = model.manifest.stand.map((c, i) => ({ ...c, id: i, shape: makeShape(R, c) })).filter((c) => c.shape);
    const parentOf = new Map(model.bodies.map((b) => [b.name, b.parent]));
    this.pairs = [];
    for (let i = 0; i < this.robot.length; i++) {
      for (let j = i + 1; j < this.robot.length; j++) {
        const a = this.robot[i];
        const b = this.robot[j];
        if (a.body === b.body || parentOf.get(a.body) === b.body || parentOf.get(b.body) === a.body) continue;
        if (!((a.contype & b.conaffinity) || (b.contype & a.conaffinity))) continue;
        this.pairs.push([a, b]);
      }
    }
    const home = forward(model, null);
    this.byDesign = new Set();
    for (const hit of this._selfHits(home, 0)) this.byDesign.add(hit.key);
    // Stand contacts already present in the default pose (e.g. head collider vs. top bar) are by design:
    // only report them when the proposed pose gets more than 5 mm closer than the default pose does.
    this.standBaseline = new Map();
    for (const h of this._standHits(home, 0.05)) this.standBaseline.set(h.key, h.distanceM);
  }

  _standHits(fk, clearanceM) {
    const hits = [];
    for (const c of this.robot) {
      const pc = this._world(c, fk.get(c.body));
      for (const s of this.stand) {
        const ps = { pos: s.pos, quat: needsAxisFix(s.type) ? qMul(s.quat, Y_TO_Z) : s.quat };
        const ct = c.shape.contactShape(vec(pc.pos), rot(pc.quat), s.shape, vec(ps.pos), rot(ps.quat), clearanceM);
        if (ct && ct.distance < clearanceM) {
          hits.push({
            key: `${c.id}:s${s.id}`,
            a: c.body,
            b: "stand",
            distanceM: ct.distance,
            point: [ct.point1.x, ct.point1.y, ct.point1.z],
          });
        }
      }
    }
    return hits;
  }

  _world(c, bodyPose) {
    const w = compose(bodyPose, { pos: c.pos, quat: c.quat });
    return { pos: w.pos, quat: needsAxisFix(c.type) ? qMul(w.quat, Y_TO_Z) : w.quat };
  }

  _selfHits(fk, margin) {
    const hits = [];
    const cache = new Map();
    const pose = (c) => {
      if (!cache.has(c.id)) cache.set(c.id, this._world(c, fk.get(c.body)));
      return cache.get(c.id);
    };
    for (const [a, b] of this.pairs) {
      const pa = pose(a);
      const pb = pose(b);
      const ct = a.shape.contactShape(vec(pa.pos), rot(pa.quat), b.shape, vec(pb.pos), rot(pb.quat), margin);
      if (ct && ct.distance < margin) {
        hits.push({
          key: `${a.id}:${b.id}`,
          a: a.body,
          b: b.body,
          distanceM: ct.distance,
          point: [ct.point1.x, ct.point1.y, ct.point1.z],
        });
      }
    }
    return hits;
  }

  /** Check a pose (policy-order degrees). `clearanceM` flags robot colliders this close to the stand frame. */
  check(qDeg, clearanceM = 0.02) {
    const fk = forward(this.model, qDeg);
    const self = this._selfHits(fk, 0).filter((h) => !this.byDesign.has(h.key));
    const stand = this._standHits(fk, clearanceM).filter((h) => {
      const base = this.standBaseline.get(h.key);
      return base === undefined || h.distanceM < base - 0.005;
    });
    return { self, stand, fk };
  }
}

export async function createChecker(model) {
  const R = await init();
  return new PoseChecker(R, model);
}
