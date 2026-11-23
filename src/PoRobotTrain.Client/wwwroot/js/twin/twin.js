// Live 3D R1 stand twin for the Blazor control UI.
// three.js WebGPURenderer (falls back to WebGL2 automatically) + TSL node materials, Draco glTF exported from the
// compiled MuJoCo model, Rapier pose checks, WebGPU-compute reach clouds, three-nebula particles, Tone.js cues.
// It only *displays* poses that MuJoCo / the robot produced (and checks proposed ones); it never simulates.
// Walk mode (opts.mode = "walk", Level 2 sandbox, docs/service_api.md §3): no stand, the floating base follows the
// pelvis pose the service's headless MuJoCo reports, the course (boxes, checkpoint rings) is drawn from its geometry
// and the camera follows the robot.
import * as THREE from "three/webgpu";
import { abs, color, dot, float, mix, normalView, positionViewDirection, pow, sin, time, uniform } from "three/tsl";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { DRACOLoader } from "three/addons/loaders/DRACOLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import * as fk from "./fk.js";

const DRACO_PATH = "https://cdn.jsdelivr.net/npm/three@0.187.0/examples/jsm/libs/draco/gltf/";
const LOOKAT_MJ = [-0.08, 0.0, 0.78];
// same presets as the server PNG renderer (RobotControl/behaviours/robot_view.py): azimuth, elevation, distance
const CAMERA_PRESETS = { iso: [136, -14, 2.35], front: [180, -10, 2.25], side: [90, -10, 2.3], back: [25, -14, 2.35], chase: [-30, -24, 5.2] };
const WALK_IDLE_PELVIS = [0, 0, 0.73]; // standing pelvis height on the ground scene, shown until the sandbox reports a pose
// Head behaviours: the envelope that matters is where the robot can *look*, so the tip is a gaze point 0.3 m in
// front of the face (+x is forward in the R1 head frame). Arm behaviours use the wrist collider's far end.
const GAZE_TIP = [0.3, 0, 0];
const BEHAVIOUR_CHAINS = {
  neck_turn: { end: "head_yaw_link", primary: "head_yaw_joint", tip: GAZE_TIP },
  neck_nod: { end: "head_yaw_link", primary: "head_pitch_joint", tip: GAZE_TIP },
  arm_raise: { end: "right_wrist_roll_link", primary: "right_shoulder_pitch_joint" },
  hand_wave: { end: "right_wrist_roll_link", primary: "right_shoulder_roll_joint" },
};
// Status palette, shared with the joint panel (JointTelemetryPanel.razor)
const C = {
  amber: new THREE.Color("#ffbd42"),
  red: new THREE.Color("#ff5c57"),
  heat: new THREE.Color("#ff7a1a"),
  error: new THREE.Color("#b18cff"),
  hit: new THREE.Color("#ff2d55"),
  ghostSim: new THREE.Color("#3dc7f2"),
  ghostPreview: new THREE.Color("#ffe066"),
};

const mjToThree = (p) => new THREE.Vector3(p[0], p[2], -p[1]);

function limitStatus(j, q) {
  const span = Math.max(1, j.maxDeg - j.minDeg);
  const nv = Math.min(1, Math.max(0, (q - j.minDeg) / span));
  const margin = Math.min(nv, 1 - nv);
  const degMargin = Math.min(q - j.minDeg, j.maxDeg - q);
  if (margin < 0.03 && degMargin < 4) return { level: 2, degMargin };
  if (margin < 0.07 && degMargin < 8) return { level: 1, degMargin };
  return { level: 0, degMargin };
}

class Twin {
  constructor(canvas, opts) {
    this.canvas = canvas;
    this.base = opts.base ?? "twin/";
    this.mode = opts.mode === "walk" ? "walk" : "stand";
    this.walk = null; // walk mode: { rootPos, rootQuat, angles target, geometry key, ... }
    this.state = {};
    this.angles = null; // displayed (smoothed) angles
    this.target = null;
    this.ghostTarget = null;
    this.prevJob = null;
    this.ghostOn = true;
    this.reachOn = false;
    this.feedback = { self: [], stand: [] };
    this.clock = new THREE.Clock();
  }

  async init() {
    const r = new THREE.WebGPURenderer({ canvas: this.canvas, antialias: true });
    await r.init();
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    r.shadowMap.enabled = true;
    r.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer = r;
    this.backend = r.backend?.isWebGPUBackend ? "webgpu" : "webgl2";

    const scene = (this.scene = new THREE.Scene());
    scene.background = new THREE.Color("#0b0f17");
    scene.fog = this.mode === "walk" ? new THREE.Fog("#0b0f17", 12, 34) : new THREE.Fog("#0b0f17", 6, 14);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.02, 50);
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.target.copy(mjToThree(LOOKAT_MJ));

    scene.add(new THREE.HemisphereLight("#cfe3ff", "#1a2030", 1.1));
    const sun = new THREE.DirectionalLight("#ffffff", 2.2);
    sun.position.set(2.5, 4, 2);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024);
    Object.assign(sun.shadow.camera, { left: -1.5, right: 1.5, top: 2.5, bottom: -0.5, near: 0.5, far: 10 });
    scene.add(sun);
    scene.add(sun.target);
    this.sun = sun;
    const floorSize = this.mode === "walk" ? 80 : 30;
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(floorSize, floorSize), new THREE.MeshStandardNodeMaterial({ color: "#151c2b", roughness: 0.95 }));
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    scene.add(floor);
    const grid = this.mode === "walk" ? new THREE.GridHelper(40, 80, "#2c3954", "#1d2638") : new THREE.GridHelper(12, 48, "#2c3954", "#1d2638");
    grid.position.y = 0.001;
    scene.add(grid);

    this.zUp = new THREE.Group();
    this.zUp.rotation.x = -Math.PI / 2; // MuJoCo z-up -> three.js y-up
    scene.add(this.zUp);

    const [manifest, gltf] = await Promise.all([
      fetch(this.base + "twin_manifest.json").then((x) => {
        if (!x.ok) throw new Error(`twin_manifest.json HTTP ${x.status}`);
        return x.json();
      }),
      this._loadGlb(this.base + "r1_stand.glb"),
    ]);
    this.manifest = manifest;
    this.model = fk.buildModel(manifest);
    this.fkSelfTest = fk.selfTest(this.model);
    this.joints = manifest.joints;

    this.robot = this._prepareRobot(gltf.scene);
    this.zUp.add(this.robot.root);
    this.ghost = this._makeGhost(gltf.scene);
    this.zUp.add(this.ghost.root);
    this._buildStand(manifest.stand);
    this._buildReachMeshes();

    this.angles = this.joints.map((j) => j.defaultDeg);
    this._applyPose(this.robot, this.angles);
    if (this.mode === "walk") {
      this.robot.bodies.get("pelvis")?.node.position.set(...WALK_IDLE_PELVIS);
      this.setCamera("chase");
    } else this.setCamera("iso");

    this._resize = new ResizeObserver(() => this._fit());
    this._resize.observe(this.canvas.parentElement ?? this.canvas);
    this._fit();

    // optional subsystems load in the background; the robot view does not wait for them
    this.extras = this._loadExtras();
    r.setAnimationLoop(() => this._frame());
    return this.info();
  }

  async _loadExtras() {
    const out = {};
    const tasks = [
      import("./particles.js").then((m) => (this.particles = new m.TwinParticles(THREE, this.scene))),
      import("./collide.js").then((m) => m.createChecker(this.model)).then((c) => (this.checker = c)),
      import("./audio.js").then((m) => (this.audio = m)),
      import("./reach.js").then((m) => (this.reach = m)),
    ];
    const names = ["particles", "rapier", "audio", "reach"];
    (await Promise.allSettled(tasks)).forEach((r, i) => {
      out[names[i]] = r.status === "fulfilled" ? "ok" : String(r.reason?.message ?? r.reason);
      if (r.status === "rejected") console.warn(`[twin] ${names[i]} unavailable`, r.reason);
    });
    this.extrasStatus = out;
    return out;
  }

  _loadGlb(url) {
    const draco = new DRACOLoader().setDecoderPath(DRACO_PATH);
    const loader = new GLTFLoader().setDRACOLoader(draco);
    return loader.loadAsync(url);
  }

  _statusMaterial(src) {
    const base = src.color?.clone() ?? new THREE.Color("#c8ccd4");
    const uStatus = uniform(new THREE.Color(0, 0, 0));
    const uAmt = uniform(0);
    const m = new THREE.MeshStandardNodeMaterial({ metalness: src.metalness ?? 0.35, roughness: src.roughness ?? 0.45 });
    const rim = pow(float(1).sub(abs(dot(normalView, positionViewDirection))), 2.5);
    const pulse = sin(time.mul(5)).mul(0.2).add(0.8);
    m.colorNode = mix(color(base), uStatus, uAmt.mul(0.6));
    m.emissiveNode = uStatus.mul(uAmt).mul(rim.mul(1.4).add(0.25)).mul(pulse);
    return { m, uStatus, uAmt };
  }

  _prepareRobot(root) {
    const bodies = new Map();
    for (const b of this.manifest.bodies) {
      const node = root.getObjectByName(b.name);
      if (!node) throw new Error(`glb is missing body node ${b.name}`);
      const status = { uStatus: null, uAmt: null, mats: [] };
      node.children.forEach((child) => {
        child.traverse((o) => {
          if (!o.isMesh) return;
          const s = this._statusMaterial(o.material);
          o.material = s.m;
          o.castShadow = true;
          status.mats.push(s);
        });
      });
      bodies.set(b.name, { node, q0: node.quaternion.clone(), joint: b.joint, status });
    }
    return { root, bodies };
  }

  _makeGhost(src) {
    const root = src.clone(true);
    const uColor = uniform(C.ghostSim.clone());
    const m = new THREE.MeshBasicNodeMaterial({ transparent: true, depthWrite: false });
    const rim = pow(float(1).sub(abs(dot(normalView, positionViewDirection))), 2.0);
    m.colorNode = uColor;
    m.opacityNode = rim.mul(0.75).add(0.08);
    root.traverse((o) => {
      if (o.isMesh) {
        o.material = m;
        o.castShadow = false;
        o.renderOrder = 2;
      }
    });
    const bodies = new Map();
    for (const b of this.manifest.bodies) {
      const node = root.getObjectByName(b.name);
      bodies.set(b.name, { node, q0: node.quaternion.clone(), joint: b.joint });
    }
    root.visible = false;
    return { root, bodies, uColor };
  }

  _buildStand(stand) {
    this.standGroup = new THREE.Group();
    this.standGroup.visible = this.mode !== "walk";
    this.zUp.add(this.standGroup);
    const mat = new THREE.MeshStandardNodeMaterial({ color: "#5b6273", metalness: 0.6, roughness: 0.4 });
    for (const s of stand) {
      let g = null;
      if (s.type === "box") g = new THREE.BoxGeometry(2 * s.size[0], 2 * s.size[1], 2 * s.size[2]);
      else if (s.type === "capsule") g = new THREE.CapsuleGeometry(s.size[0], 2 * s.size[1], 4, 12).rotateX(Math.PI / 2);
      else if (s.type === "cylinder") g = new THREE.CylinderGeometry(s.size[0], s.size[0], 2 * s.size[1], 16).rotateX(Math.PI / 2);
      else if (s.type === "sphere") g = new THREE.SphereGeometry(s.size[0], 16, 12);
      if (!g) continue;
      const mesh = new THREE.Mesh(g, mat);
      mesh.position.set(...s.pos);
      mesh.quaternion.set(...s.quat);
      mesh.castShadow = true;
      this.standGroup.add(mesh);
    }
  }

  _buildReachMeshes() {
    const geo = new THREE.IcosahedronGeometry(0.0045, 0);
    const mk = (n, col, opacity) => {
      const m = new THREE.MeshBasicNodeMaterial({ color: col, transparent: true, opacity, depthWrite: false });
      const mesh = new THREE.InstancedMesh(geo, m, n);
      mesh.count = 0;
      mesh.frustumCulled = false;
      mesh.visible = false;
      this.zUp.add(mesh);
      return mesh;
    };
    this.reachFull = mk(16384, "#6366f1", 0.22);
    this.reachSweep = mk(1024, "#ffe066", 0.95);
  }

  _applyPose(rig, qDeg) {
    const tmp = new THREE.Quaternion();
    const axis = new THREE.Vector3();
    for (const b of rig.bodies.values()) {
      if (b.joint == null) continue;
      const j = this.joints[b.joint];
      axis.set(...j.axis);
      tmp.setFromAxisAngle(axis, (qDeg[b.joint] ?? j.defaultDeg) * fk.DEG);
      b.node.quaternion.copy(b.q0).multiply(tmp);
    }
  }

  _fit() {
    const el = this.canvas.parentElement ?? this.canvas;
    const w = Math.max(1, el.clientWidth);
    const h = Math.max(1, el.clientHeight);
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  _jointWorld(i, rig = this.robot) {
    const body = this.joints[i]?.body;
    const b = body && rig.bodies.get(body);
    return b ? b.node.getWorldPosition(new THREE.Vector3()) : null;
  }

  _frame() {
    const dt = Math.min(0.1, this.clock.getDelta());
    if (this.walk) {
      this._walkFrame(dt);
    } else if (this.target) {
      const k = 1 - Math.exp(-dt / 0.12); // poll is ~350 ms; ease so motion reads as continuous
      this.angles = this.angles.map((a, i) => (this.target[i] == null ? a : a + (this.target[i] - a) * k));
      this._applyPose(this.robot, this.angles);
    }
    this.controls.update();
    this.particles?.tick(dt);
    if (this.audio?.isEnabled()) {
      const fwd = this.camera.getWorldDirection(new THREE.Vector3());
      this.audio.setListener(this.camera.position, fwd, this.camera.up);
    }
    this.renderer.render(this.scene, this.camera);
  }

  // ------------------------------------------------------------------ walk mode (Level 2 sandbox)
  _walkFrame(dt) {
    const w = this.walk;
    const k = 1 - Math.exp(-dt / 0.05); // state arrives at ~20 Hz; short ease hides the poll steps
    if (w.target) {
      this.angles = this.angles.map((a, i) => (w.target[i] == null ? a : a + (w.target[i] - a) * k));
      this._applyPose(this.robot, this.angles);
    }
    const pelvis = this.robot.bodies.get("pelvis")?.node;
    if (pelvis && w.rootPos) {
      if (!w.shown) {
        pelvis.position.copy(w.rootPos);
        pelvis.quaternion.copy(w.rootQuat);
        w.shown = true;
      } else {
        pelvis.position.lerp(w.rootPos, k);
        pelvis.quaternion.slerp(w.rootQuat, k);
      }
      // follow camera: keep the user's orbit offset, move the target with the pelvis (in three.js y-up world)
      const p = this.zUp.localToWorld(pelvis.position.clone());
      p.y = Math.max(0.6, p.y - 0.1);
      const delta = p.clone().sub(this.controls.target);
      this.controls.target.add(delta);
      this.camera.position.add(delta);
      this.sun.position.set(p.x + 2.5, 4, p.z + 2);
      this.sun.target.position.copy(p);
    }
    if (this.course) {
      const t = performance.now() / 1000;
      this.course.rings.forEach((r, i) => {
        const next = i === w.nextIndex;
        r.material.opacity = next ? 0.75 + 0.25 * Math.sin(t * 5) : i < w.nextIndex && w.running ? 0.25 : 0.45;
        r.scale.setScalar(next ? 1 + 0.06 * Math.sin(t * 5) : 1);
        r.userData.beacon.visible = next;
      });
    }
  }

  _buildCourse(geometry) {
    if (this.course) {
      this.zUp.remove(this.course.group);
      this.course.group.traverse((o) => {
        o.geometry?.dispose();
        o.material?.dispose?.();
      });
      this.course = null;
    }
    if (!geometry) return;
    const group = new THREE.Group();
    for (const b of geometry.boxes ?? []) {
      const [r, g, bl] = b.rgba ?? [0.4, 0.45, 0.55, 1];
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(2 * b.size[0], 2 * b.size[1], 2 * b.size[2]),
        new THREE.MeshStandardNodeMaterial({ color: new THREE.Color(r, g, bl), roughness: 0.8, metalness: 0.1 })
      );
      mesh.position.set(...b.pos);
      if (b.quat) mesh.quaternion.set(b.quat[1], b.quat[2], b.quat[3], b.quat[0]); // MuJoCo wxyz -> three xyzw
      mesh.castShadow = mesh.receiveShadow = true;
      mesh.name = b.name ?? "box";
      group.add(mesh);
    }
    const radius = geometry.checkpoint_radius ?? 0.5;
    const rings = (geometry.checkpoints ?? []).map((c) => {
      // torus lies in its local XY plane, which is the MuJoCo ground plane inside the z-up group
      const ring = new THREE.Mesh(
        new THREE.TorusGeometry(radius, 0.035, 8, 48),
        new THREE.MeshBasicNodeMaterial({ color: "#6366f1", transparent: true, opacity: 0.45, depthWrite: false })
      );
      ring.position.set(c[0], c[1], (c[2] ?? 0) + 0.03);
      const beacon = new THREE.Mesh(
        new THREE.CylinderGeometry(0.02, 0.02, 1.6, 8).rotateX(Math.PI / 2),
        new THREE.MeshBasicNodeMaterial({ color: "#ffe066", transparent: true, opacity: 0.8 })
      );
      beacon.position.set(c[0], c[1], (c[2] ?? 0) + 0.8);
      beacon.visible = false;
      ring.userData.beacon = beacon;
      group.add(ring, beacon);
      return ring;
    });
    this.zUp.add(group);
    this.course = { group, rings };
  }

  /**
   * Walk sandbox state from GET /walk/state: base {pos, quat (wxyz)}, q (26 rad, joint-map order), geometry,
   * checkpoint {index, total}. Display only.
   */
  setWalk(state) {
    if (this.mode !== "walk" || !state) return;
    const w = (this.walk ??= { shown: false, nextIndex: -1, geoKey: null });
    const geoKey = state.geometry ? JSON.stringify(state.geometry) : null;
    if (geoKey !== w.geoKey) {
      w.geoKey = geoKey;
      this._buildCourse(state.geometry);
    }
    if (Array.isArray(state.q) && state.q.length >= this.joints.length) w.target = state.q.map((v) => v / fk.DEG);
    if (state.base?.pos && state.base?.quat) {
      const [qw, qx, qy, qz] = state.base.quat;
      w.rootPos = new THREE.Vector3(...state.base.pos);
      w.rootQuat = new THREE.Quaternion(qx, qy, qz, qw).normalize();
    }
    w.nextIndex = state.checkpoint ? state.checkpoint.index : -1;
    w.running = !!state.running;
    try {
      this._syncProps(state.props);
      this._syncPhysics(state.physics);
    } catch (e) {
      console.warn("[twin] walk overlay", e); // display only; the robot keeps drawing
    }
  }

  /** #9 thrown objects: one mesh per prop id (ball: size [r]; box: half sizes), MuJoCo world, quat wxyz. */
  _syncProps(props) {
    const map = (this.propMeshes ??= new Map());
    const seen = new Set();
    for (const p of Array.isArray(props) ? props : []) {
      if (!Array.isArray(p?.pos)) continue;
      const key = `${p.id}:${p.kind}`;
      seen.add(key);
      let mesh = map.get(key);
      if (!mesh) {
        const s = p.size ?? [];
        const geo = p.kind === "box"
          ? new THREE.BoxGeometry(2 * (s[0] ?? 0.125), 2 * (s[1] ?? s[0] ?? 0.125), 2 * (s[2] ?? s[0] ?? 0.125))
          : new THREE.SphereGeometry(s[0] ?? 0.1, 20, 14);
        mesh = new THREE.Mesh(geo, new THREE.MeshStandardNodeMaterial({ color: p.kind === "box" ? "#d9822b" : "#e8505b", roughness: 0.6 }));
        mesh.castShadow = true;
        this.zUp.add(mesh);
        map.set(key, mesh);
      }
      mesh.position.set(...p.pos);
      if (Array.isArray(p.quat) && p.quat.length === 4) mesh.quaternion.set(p.quat[1], p.quat[2], p.quat[3], p.quat[0]);
    }
    for (const [key, mesh] of map) {
      if (seen.has(key)) continue;
      this.zUp.remove(mesh);
      mesh.geometry.dispose();
      mesh.material.dispose();
      map.delete(key);
    }
  }

  /** #2 physics overlay (toggle setPhysicsOverlay): CoM, capture point, support polygon, contact force arrows. */
  _syncPhysics(ph) {
    if (!this.physicsGroup) {
      const g = (this.physicsGroup = new THREE.Group());
      g.visible = !!this.physicsOn;
      const dot = (col, r) => new THREE.Mesh(new THREE.SphereGeometry(r, 16, 10), new THREE.MeshBasicNodeMaterial({ color: col, depthTest: false, transparent: true }));
      this.phCom = dot("#ffe066", 0.04);
      this.phComFloor = dot("#ffe066", 0.025);
      this.phCp = dot("#ff5c57", 0.035);
      const lines = (col) => {
        const l = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicNodeMaterial({ color: col, depthTest: false, transparent: true }));
        l.frustumCulled = false;
        l.renderOrder = 5;
        return l;
      };
      this.phSupport = lines("#52d984");
      this.phForces = lines("#3dc7f2");
      for (const o of [this.phCom, this.phComFloor, this.phCp]) o.renderOrder = 5;
      g.add(this.phCom, this.phComFloor, this.phCp, this.phSupport, this.phForces);
      this.zUp.add(g);
    }
    const set = (line, pts) => line.geometry.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    const com = ph?.com;
    this.phCom.visible = this.phComFloor.visible = Array.isArray(com);
    if (Array.isArray(com)) {
      this.phCom.position.set(...com);
      this.phComFloor.position.set(com[0], com[1], 0.01);
    }
    const cp = ph?.capture_point;
    this.phCp.visible = Array.isArray(cp);
    if (Array.isArray(cp)) this.phCp.position.set(cp[0], cp[1], 0.012);
    const hull = Array.isArray(ph?.support) ? ph.support : [];
    const seg = [];
    for (let i = 0; i < hull.length && hull.length > 1; i++) {
      const a = hull[i], b = hull[(i + 1) % hull.length];
      seg.push(a[0], a[1], 0.01, b[0], b[1], 0.01);
    }
    set(this.phSupport, seg);
    const f = [];
    for (const c of Array.isArray(ph?.contacts) ? ph.contacts : []) {
      if (!Array.isArray(c?.pos) || !Array.isArray(c?.force)) continue;
      const k = 0.0015; // m per N: the robot's weight (~250 N) draws about 0.4 m
      f.push(...c.pos, c.pos[0] + k * c.force[0], c.pos[1] + k * c.force[1], c.pos[2] + k * c.force[2]);
    }
    set(this.phForces, f);
  }

  setPhysicsOverlay(on) {
    this.physicsOn = !!on;
    if (this.physicsGroup) this.physicsGroup.visible = this.physicsOn;
  }

  walkJson(json) {
    this.setWalk(JSON.parse(json));
  }

  // ------------------------------------------------------------------ API used from Blazor
  setCamera(name) {
    const p = CAMERA_PRESETS[name] ?? CAMERA_PRESETS.iso;
    const az = p[0] * fk.DEG;
    const el = p[1] * fk.DEG;
    const fwd = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
    const pos = LOOKAT_MJ.map((v, i) => v - p[2] * fwd[i]);
    this.camera.position.copy(mjToThree(pos));
    this.controls.target.copy(mjToThree(LOOKAT_MJ));
    this.controls.update();
  }

  setGhost(on) {
    this.ghostOn = !!on;
    this._refreshGhost();
  }

  async setSound(on) {
    await this.extras;
    if (!this.audio) return false;
    return this.audio.setEnabled(!!on);
  }

  info() {
    return {
      backend: this.backend,
      fkOk: this.fkSelfTest?.ok ?? false,
      fkMaxPosMm: (this.fkSelfTest?.maxPosM ?? NaN) * 1000,
      extras: this.extrasStatus ?? null,
      glbBytes: this.manifest?.glb?.bytes ?? 0,
      draco: this.manifest?.glb?.draco ?? false,
    };
  }

  async ready() {
    await this.extras;
    return this.info();
  }

  /**
   * state: { actual: number[26]|null, sim: number[26]|null, temps: number[26]|null, mode: 'real'|'virtual',
   *          preview: { index, deg }|null, job: string|null, results: {job: {ok}}, primaryJoint: string|null,
   *          reach: { on, behaviour, amplitude }|null }
   * Returns feedback for the proposed (preview) pose: { self: [...], stand: [...], limit }.
   */
  update(state) {
    this.state = state ?? {};
    const actual = this.state.actual && this.state.actual.length >= this.joints.length ? this.state.actual : null;
    this.target = actual ?? this.joints.map((j) => j.defaultDeg);
    if (!actual && this.angles) this.target = [...this.angles];
    const cur = actual ?? this.angles;

    // proposed pose while a slider is dragged
    let preview = null;
    let feedback = { self: [], stand: [], limit: 0, checked: false };
    if (this.state.preview && Number.isInteger(this.state.preview.index)) {
      preview = [...cur];
      preview[this.state.preview.index] = this.state.preview.deg;
      if (this.checker) {
        const res = this.checker.check(preview);
        const toPoint = (h) => ({ ...h, point: undefined });
        feedback = { self: res.self.map(toPoint), stand: res.stand.map(toPoint), checked: true };
        const firstHit = res.self[0] ?? res.stand[0];
        if (firstHit) {
          const at = this.zUp.localToWorld(new THREE.Vector3(...firstHit.point));
          this.particles?.sparks(at);
          this.audio?.cue("collision", at);
        }
        this._hitBodies = new Set([...res.self.flatMap((h) => [h.a, h.b]), ...res.stand.map((h) => h.a)]);
      }
      const j = this.joints[this.state.preview.index];
      const ls = limitStatus(j, this.state.preview.deg);
      feedback.limit = ls.degMargin < 15 ? 1 - Math.max(0, ls.degMargin) / 15 : 0;
      if (feedback.limit > 0) this.audio?.cue("limit", this._jointWorld(this.state.preview.index), feedback.limit);
    } else {
      this._hitBodies = null;
    }
    this.previewPose = preview;
    this.feedback = feedback;
    this._refreshGhost();
    this._refreshStatus(cur);
    this._jobCues();
    this._refreshHeat();
    this._refreshReach(cur);
    return feedback;
  }

  _refreshGhost() {
    const g = this.ghost;
    if (!g) return;
    if (this.previewPose) {
      g.uColor.value.copy(C.ghostPreview);
      this._applyPose(g, this.previewPose);
      g.root.visible = true;
      return;
    }
    const sim = this.state.sim;
    const showSim = this.ghostOn && this.state.mode === "real" && sim && sim.length >= this.joints.length;
    if (showSim) {
      g.uColor.value.copy(C.ghostSim);
      this._applyPose(g, sim);
    }
    g.root.visible = !!showSim;
  }

  _refreshStatus(cur) {
    const temps = this.state.temps;
    const sim = this.state.mode === "real" ? this.state.sim : null;
    for (const [name, b] of this.robot.bodies) {
      let col = null;
      let amt = 0;
      if (this._hitBodies?.has(name)) {
        col = C.hit;
        amt = 1;
      } else if (b.joint != null) {
        const j = this.joints[b.joint];
        const q = cur[b.joint];
        const ls = limitStatus(j, q);
        const t = temps?.[b.joint];
        const err = sim && Number.isFinite(sim[b.joint]) ? Math.abs(sim[b.joint] - q) : 0;
        if (ls.level === 2) [col, amt] = [C.red, 0.9];
        else if (t != null && t >= 55) [col, amt] = [C.heat, Math.min(1, 0.4 + (t - 55) / 20)];
        else if (err > 2) [col, amt] = [C.error, Math.min(1, 0.3 + (err - 2) / 6)];
        else if (ls.level === 1) [col, amt] = [C.amber, 0.55];
      }
      for (const s of b.status.mats) {
        if (col) s.uStatus.value.copy(col);
        s.uAmt.value = amt;
      }
    }
  }

  _jobCues() {
    const job = this.state.job || null;
    const prev = this.prevJob;
    if (job === prev) return;
    this.prevJob = job;
    const real = this.state.realConnected;
    if (job && !prev) {
      if (job === "preview") this.audio?.cue("virtualStart");
      else if (job === "run") this.audio?.cue(real ? "realStart" : "practiceStart");
      this.audio?.setPowered(job === "run" && real);
    }
    if (prev && !job) {
      this.audio?.setPowered(false);
      if (!["preview", "run", "check"].includes(prev)) return;
      const res = this.state.results?.[prev];
      if (!res) return;
      const pj = this.joints.findIndex((j) => j.name === this.state.primaryJoint);
      const at = pj >= 0 ? this._jointWorld(pj) : null;
      if (res.ok) this.audio?.cue("ok", at);
      else {
        this.audio?.cue("fail", at);
        if (at) this.particles?.fault(at);
      }
    }
  }

  _refreshHeat() {
    if (!this.particles) return;
    const temps = this.state.mode === "real" ? this.state.temps : null;
    const pos = this.joints.map((_, i) => (temps?.[i] >= 55 ? this._jointWorld(i) : null));
    this.particles.setHeat(temps, pos, 55);
    if (temps?.some((t) => t >= 65)) this.audio?.cue("hot");
  }

  _refreshReach(cur) {
    const r = this.state.reach;
    const on = !!(r && r.on && BEHAVIOUR_CHAINS[r.behaviour] && this.reach);
    this.reachFull.visible = on;
    this.reachSweep.visible = on;
    if (!on) {
      this._reachKey = null;
      return;
    }
    const key = `${r.behaviour}|${r.amplitude}|${cur.map((v) => Math.round(v)).join(",")}`;
    if (key === this._reachKey || this._reachBusy) return;
    this._reachKey = key;
    this._reachBusy = true;
    this._computeReach(r, cur)
      .catch((e) => console.warn("[twin] reach failed", e))
      .finally(() => (this._reachBusy = false));
  }

  async _computeReach(r, cur) {
    const spec = BEHAVIOUR_CHAINS[r.behaviour];
    const { root, links } = fk.chainTo(this.model, spec.end);
    const rootPose = fk.forward(this.model, cur).get(root);
    const tip = spec.tip ?? fk.tipOffset(this.model, spec.end);
    const mkLinks = (sweepOnly) =>
      links.map((b) => {
        const j = this.joints[b.joint];
        let lo = j.minDeg;
        let hi = j.maxDeg;
        if (sweepOnly) {
          const c = cur[b.joint];
          if (j.name === spec.primary) {
            lo = Math.max(j.minDeg, c - r.amplitude);
            hi = Math.min(j.maxDeg, c + r.amplitude);
          } else {
            lo = hi = c;
          }
        }
        return { pos: b.pos, quat: b.quat, axis: b.axis, lo, hi };
      });
    const full = await this.reach.computeReach({ root: rootPose, links: mkLinks(false), tip, count: this.reachFull.instanceMatrix.count, seed: 1 });
    const sweep = await this.reach.computeReach({ root: rootPose, links: mkLinks(true), tip, count: this.reachSweep.instanceMatrix.count, seed: 2 });
    this._fillCloud(this.reachFull, full.points);
    this._fillCloud(this.reachSweep, sweep.points);
    this.reachInfo = { backend: full.backend, ms: full.ms + sweep.ms, samples: full.points.length / 4 + sweep.points.length / 4 };
    if (!this.reachChecked) {
      this.reachChecked = true;
      this.reachSelfTest = await this.reach.selfTest({ root: rootPose, links: mkLinks(false), tip, count: 256, seed: 3 });
    }
  }

  _fillCloud(mesh, pts) {
    const m = new THREE.Matrix4();
    const n = pts.length / 4;
    for (let i = 0; i < n; i++) {
      m.makeTranslation(pts[i * 4], pts[i * 4 + 1], pts[i * 4 + 2]);
      mesh.setMatrixAt(i, m);
    }
    mesh.count = n;
    mesh.instanceMatrix.needsUpdate = true;
  }

  diagnostics() {
    return {
      ...this.info(),
      reach: this.reachInfo ?? null,
      reachSelfTest: this.reachSelfTest ?? null,
      rapierPairs: this.checker?.pairs.length ?? 0,
    };
  }

  // JSON-string entry points for Blazor (source-generated, trim-safe serialisation on the .NET side)
  updateJson(json) {
    return JSON.stringify(this.update(JSON.parse(json)));
  }

  infoJson() {
    return JSON.stringify(this.info());
  }

  async readyJson() {
    return JSON.stringify(await this.ready());
  }

  dispose() {
    this.renderer?.setAnimationLoop(null);
    this._resize?.disconnect();
    this.audio?.setPowered(false);
    this.particles?.dispose();
    this.controls?.dispose();
    this.renderer?.dispose();
  }
}

export async function create(canvas, opts) {
  const t = new Twin(canvas, opts ?? {});
  await t.init();
  if (new URLSearchParams(location.search).has("twindebug")) globalThis.__poTwin = t; // test/debug hook only
  return t;
}
