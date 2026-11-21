// Forward kinematics for the R1 stand twin, in MuJoCo's frame (z up, metres, quaternions [x, y, z, w]).
// Pure math, no three.js: shared by the Rapier pose check, the reach-cloud kernel set-up and the self-test that
// compares against MuJoCo's own xpos/xquat (twin_manifest.json -> fkGolden).

export const DEG = Math.PI / 180;

export function qMul(a, b) {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

export function qAxisAngle(axis, rad) {
  const s = Math.sin(rad / 2);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(rad / 2)];
}

export function qRotate(q, v) {
  const [x, y, z, w] = q;
  // t = 2 * cross(q.xyz, v)
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ];
}

export function compose(parent, local) {
  const p = qRotate(parent.quat, local.pos);
  return {
    pos: [parent.pos[0] + p[0], parent.pos[1] + p[1], parent.pos[2] + p[2]],
    quat: qMul(parent.quat, local.quat),
  };
}

/** Index the manifest once: bodies in parent-first order with their joint (policy index) if any. */
export function buildModel(manifest) {
  const joints = manifest.joints;
  const bodies = manifest.bodies.map((b) => ({
    name: b.name,
    parent: b.parent,
    pos: b.pos,
    quat: b.quat,
    joint: b.joint ?? null,
    axis: b.joint != null ? joints[b.joint].axis : null,
  }));
  const byName = new Map(bodies.map((b) => [b.name, b]));
  return { joints, bodies, byName, manifest };
}

/** World pose of every body for joint angles qDeg (policy order, degrees). Missing/null entries use defaults. */
export function forward(model, qDeg) {
  const out = new Map();
  for (const b of model.bodies) {
    let quat = b.quat;
    if (b.joint != null) {
      const deg = qDeg && Number.isFinite(qDeg[b.joint]) ? qDeg[b.joint] : model.joints[b.joint].defaultDeg;
      quat = qMul(b.quat, qAxisAngle(b.axis, deg * DEG));
    }
    const local = { pos: b.pos, quat };
    out.set(b.name, b.parent == null ? local : compose(out.get(b.parent), local));
  }
  return out;
}

/** Compare forward() with MuJoCo's golden body poses. Returns the worst position (m) and rotation (rad) error. */
export function selfTest(model) {
  let maxPos = 0;
  let maxRot = 0;
  for (const g of model.manifest.fkGolden) {
    const fk = forward(model, g.qDeg);
    for (const [name, ref] of Object.entries(g.bodies)) {
      const got = fk.get(name);
      const dp = Math.hypot(got.pos[0] - ref.pos[0], got.pos[1] - ref.pos[1], got.pos[2] - ref.pos[2]);
      const dot = Math.abs(got.quat.reduce((s, v, i) => s + v * ref.quat[i], 0));
      maxPos = Math.max(maxPos, dp);
      maxRot = Math.max(maxRot, 2 * Math.acos(Math.min(1, dot)));
    }
  }
  return { ok: maxPos < 1e-4 && maxRot < 1e-3, maxPosM: maxPos, maxRotRad: maxRot, poses: model.manifest.fkGolden.length };
}

/** Joint chain (policy indices) from the root down to `endBody`, plus the end body's tip offset. */
export function chainTo(model, endBody, stopAtBody = "torso_link") {
  const chain = [];
  let b = model.byName.get(endBody);
  while (b && b.name !== stopAtBody && b.parent != null) {
    chain.unshift(b);
    b = model.byName.get(b.parent);
  }
  return { root: b ? b.name : null, links: chain };
}

/** Farthest collider end point from the body origin, in the body frame (approximate fingertip / face point). */
export function tipOffset(model, bodyName) {
  let best = [0, 0, 0];
  let bestD = -1;
  for (const c of model.manifest.colliders) {
    if (c.body !== bodyName) continue;
    const pts = [];
    if (c.type === "capsule" || c.type === "cylinder") {
      const half = c.size[1];
      const ext = c.type === "capsule" ? half + c.size[0] : half;
      for (const s of [-1, 1]) {
        const p = qRotate(c.quat, [0, 0, s * ext]);
        pts.push([c.pos[0] + p[0], c.pos[1] + p[1], c.pos[2] + p[2]]);
      }
    } else {
      pts.push(c.pos);
    }
    for (const p of pts) {
      const dd = Math.hypot(p[0], p[1], p[2]);
      if (dd > bestD) {
        bestD = dd;
        best = p;
      }
    }
  }
  return best;
}
