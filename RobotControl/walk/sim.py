"""Walk sandbox physics: free-floating R1 on the ground scene, driven by the Level 2 policy (or a PD hold).

MuJoCo owns physics (5 ms step, implicitfast, 50 Hz policy = 4 substeps). The model is
``UnrealProject/MJCF/scene_ground.xml`` with the course boxes from ``course.py`` added at load time and the
calibrated actuator/joint dynamics from ``sim_calibration.yaml`` applied, like the other twins.
Pure and single-threaded: ``WalkSandbox`` (sandbox.py) runs it in real time for the service.

Simulation-only extras (docs/sim_features.md): physics overlay (CoM, capture point, support polygon, contacts with
forces), motor speed/torque, the motor heat + battery model, the action filter, throwable props (MuJoCo free bodies
added at load time and parked far away until thrown), a 10 s frame ring buffer for replays, and fall analysis.
"""

from __future__ import annotations

import collections
import math
import pathlib
from typing import Deque, List, Optional, Tuple

import mujoco
import numpy as np

from .. import PROJECT_ROOT
from ..configs.joint_map import JointMap
from ..configs.sim_calibration import SimCalibration
from ..sim_features.motor_model import ActionFilter, MotorModel, MotorModelConfig
from .course import course as get_course
from .physics import analyse_fall, capture_point, convex_hull, signed_margin
from .policy import NUM_POLICY_JOINTS, Level2Policy, ObsSignals, build_obs

SCENE_XML = PROJECT_ROOT / "UnrealProject/MJCF/scene_ground.xml"
POLICY_DT = 0.02
SUBSTEPS = 4
FALL_HEIGHT = 0.45  # pelvis below this = fallen
FALL_TILT_DEG = 60.0  # pelvis z axis this far from vertical = fallen
FALL_RESET_S = 1.0
MODES = ("policy", "hold", "free_hold")

# throwable props (docs/sim_features.md #9): MAX_PROPS slots per kind are pre-added, at most MAX_PROPS in the air
MAX_PROPS = 4
PROP_KINDS = {
    "ball": {"size": [0.10], "mass": 0.45, "rgba": [0.95, 0.45, 0.2, 1]},
    "box": {"size": [0.125, 0.125, 0.125], "mass": 2.0, "rgba": [0.85, 0.75, 0.3, 1]},
}
PROP_SPAWN_DIST = 2.0
PARK = (60.0, 60.0)  # far away, resting on the floor
MAX_CONTACTS = 24
REPLAY_BUFFER_S = 10.0
HISTORY_S = 3.0
HISTORY_HZ = 10


def _prop_body(kind: str, i: int) -> str:
    return f"prop_{kind}_{i}"


def _park_pos(kind: str, i: int) -> List[float]:
    k = list(PROP_KINDS).index(kind)
    return [PARK[0] + 2.0 * i, PARK[1] + 2.0 * k, PROP_KINDS[kind]["size"][0] + 0.001]


def load_model(
    course_name: str,
    scene: pathlib.Path = SCENE_XML,
    calibrated: bool = True,
    seed: int = 0,
    difficulty: float = 0.5,
) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(scene))
    for b in get_course(course_name, seed=seed, difficulty=difficulty)["boxes"]:
        spec.worldbody.add_geom(
            name=f"course_{b['name']}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=b["size"],
            pos=b["pos"],
            quat=b["quat"],
            rgba=b["rgba"],
            friction=[0.9, 0.005, 0.0001],
            condim=3,
        )
    # Hold mode's "virtual harness": a soft pelvis-to-world weld at the standing pose (qpos0 of the free joint is
    # the standing pelvis pose). The R1 cannot balance passively, so without it a PD hold tips over in ~1 s.
    # Only active in hold mode; a walking policy never runs with it.
    spec.add_equality(
        name="hold_harness",
        type=mujoco.mjtEq.mjEQ_WELD,
        objtype=mujoco.mjtObj.mjOBJ_BODY,
        name1="pelvis",
        name2="world",
        active=False,
        solref=[0.02, 1.0],
    )
    # Throwable props: free bodies parked far away on the floor (the keyframe keeps their body pose there).
    for kind, p in PROP_KINDS.items():
        gtype = mujoco.mjtGeom.mjGEOM_SPHERE if kind == "ball" else mujoco.mjtGeom.mjGEOM_BOX
        size = (p["size"] + [0.0, 0.0])[:3] if kind == "ball" else p["size"]
        for i in range(MAX_PROPS):
            body = spec.worldbody.add_body(name=_prop_body(kind, i), pos=_park_pos(kind, i))
            body.add_freejoint(name=_prop_body(kind, i) + "_free")
            body.add_geom(
                name=_prop_body(kind, i),
                type=gtype,
                size=size,
                mass=p["mass"],
                rgba=p["rgba"],
                friction=[0.8, 0.005, 0.0001],
                condim=3,
            )
    m = spec.compile()
    if calibrated:
        jm = JointMap.load()
        hinge = np.array(
            [m.jnt_dofadr[i] for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE], dtype=np.int64
        )
        SimCalibration.load(jm=jm).apply_to_model(mujoco, m, hinge)
    return m


class WalkSim:
    """mode 'policy': velocity policy; 'hold': every joint PD-held at its default pose in the hold harness (no
    walking); 'free_hold': the same PD hold WITHOUT the harness, so the robot tips over in about a second."""

    def __init__(
        self,
        course: str = "loop",
        mode: str = "policy",
        policy: Optional[Level2Policy] = None,
        calibrated: bool = True,
        seed: int = 0,
        difficulty: float = 0.5,
    ):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
        if mode == "policy" and policy is None:
            raise ValueError("mode 'policy' needs a Level2Policy")
        self.jm = JointMap.load()
        self.course_name = course
        self.seed, self.difficulty = int(seed), float(difficulty)
        self.geometry = get_course(course, seed=self.seed, difficulty=self.difficulty)
        self.mode = mode
        self.policy = policy
        self.m = load_model(course, calibrated=calibrated, seed=self.seed, difficulty=self.difficulty)
        self.d = mujoco.MjData(self.m)
        m = self.m
        assert abs(m.opt.timestep - POLICY_DT / SUBSTEPS) < 1e-9, "walk scene must keep the 5 ms physics step"
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
        self.pelvis_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        free = [
            i
            for i in range(m.njnt)
            if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE and m.body_rootid[m.jnt_bodyid[i]] == self.pelvis_id
        ]
        assert len(free) == 1, "expected one floating base"
        self.root_qadr = int(m.jnt_qposadr[free[0]])
        self.root_vadr = int(m.jnt_dofadr[free[0]])
        self.qadr = np.array([m.jnt_qposadr[names.index(n)] for n in self.jm.names])
        self.vadr = np.array([m.jnt_dofadr[names.index(n)] for n in self.jm.names])
        act_joint = [names[m.actuator_trnid[a, 0]] for a in range(m.nu)]
        self.act_idx = np.array([act_joint.index(n) for n in self.jm.names])  # actuator per joint-map joint
        self.default_all = np.asarray(self.jm.defaults, dtype=np.float64)
        self.home = m.key("home").qpos.copy()
        self.harness_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_harness")
        if policy is not None:
            dep = policy.deploy
            if list(dep.joint_names) != list(self.jm.names[:NUM_POLICY_JOINTS]):
                raise RuntimeError("Level 2 deploy.yaml joint order differs from r1_joint_map.yaml policy_index 0..23")
            # deploy gains are the policy's training gains; write them into the position actuators
            for k in range(NUM_POLICY_JOINTS):
                a = self.act_idx[k]
                m.actuator_gainprm[a, 0] = dep.kp[k]
                m.actuator_biasprm[a, 1] = -dep.kp[k]
                m.actuator_biasprm[a, 2] = -dep.kd[k]
        self._index_geoms()
        self._index_props()
        cfg = MotorModelConfig.load()
        self.motors = MotorModel(self.jm.names, self.jm.effort, cfg)
        self.filter = ActionFilter(NUM_POLICY_JOINTS, cfg.action_filter_alpha, cfg.jitter_window_s, POLICY_DT)
        self.frames: Deque[dict] = collections.deque(maxlen=int(REPLAY_BUFFER_S / POLICY_DT))
        self.margin_hist: Deque[dict] = collections.deque(maxlen=int(HISTORY_S / POLICY_DT) + 1)
        self.prop_hits: Deque[Tuple[float, str]] = collections.deque(maxlen=64)
        self.fall_analysis: Optional[dict] = None
        self.falls_log: Deque[dict] = collections.deque(maxlen=10)
        self.cmd = np.zeros(3)
        self.falls = 0
        self.laps = 0
        self.last_lap_s: Optional[float] = None
        self.best_lap_s: Optional[float] = None
        self.t = 0.0
        self.reset()

    def _index_geoms(self) -> None:
        """Contact labels: robot geoms by body (feet merged), floor, course boxes and props by name."""
        m = self.m
        self.robot_geom = np.array([m.body_rootid[m.geom_bodyid[g]] == self.pelvis_id for g in range(m.ngeom)])
        self.geom_label: List[str] = []
        self.foot_geom = np.zeros(m.ngeom, dtype=bool)
        for g in range(m.ngeom):
            gname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or "world"
            if gname.startswith(("left_foot", "right_foot")):
                label = gname.split("_foot")[0] + "_foot"
                self.foot_geom[g] = True
            elif self.robot_geom[g]:
                label = bname.replace("_link", "")
            else:
                label = gname or bname
            self.geom_label.append(label)

    def _index_props(self) -> None:
        m = self.m
        self.prop_slots = []  # per (kind, i): body id, qpos adr, dof adr, geom id
        for kind in PROP_KINDS:
            for i in range(MAX_PROPS):
                bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, _prop_body(kind, i))
                jid = int(m.body_jntadr[bid])
                gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, _prop_body(kind, i))
                self.prop_slots.append(
                    {"kind": kind, "i": i, "body": bid, "qadr": int(m.jnt_qposadr[jid]),
                     "vadr": int(m.jnt_dofadr[jid]), "geom": gid}
                )  # fmt: skip
        self.prop_geom_slot = {s["geom"]: s for s in self.prop_slots}
        self.active_props: List[dict] = []  # oldest first: {"slot", "id", "kind"}
        self.prop_seq = 0

    # ------------------------------------------------------------------ state
    def reset(self, keep_time: bool = False) -> None:
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key("home").id)
        self.d.eq_active[self.harness_id] = self.mode == "hold"
        self.d.ctrl[self.act_idx] = self.default_all
        mujoco.mj_forward(self.m, self.d)
        if not keep_time:
            self.t = 0.0
            self.frames.clear()
            self.motors.reset()
        self.phase_t = 0.0
        self.last_action = np.zeros(NUM_POLICY_JOINTS)
        self.filter.reset()
        self.fallen_at: Optional[float] = None
        self.cp_index = 0
        self.lap_start = self.t
        self.active_props = []
        self.margin_hist.clear()
        self._physics_cache: Optional[dict] = None
        self._record()

    @property
    def base_pos(self) -> np.ndarray:
        return self.d.qpos[self.root_qadr : self.root_qadr + 3].copy()

    @property
    def base_quat(self) -> np.ndarray:
        return self.d.qpos[self.root_qadr + 3 : self.root_qadr + 7].copy()

    @property
    def q(self) -> np.ndarray:
        return self.d.qpos[self.qadr].copy()

    @property
    def dq(self) -> np.ndarray:
        return self.d.qvel[self.vadr].copy()

    def base_ang_vel_body(self) -> np.ndarray:
        # MuJoCo free joint: qvel[3:6] is the angular velocity expressed in the body (local) frame
        return self.d.qvel[self.root_vadr + 3 : self.root_vadr + 6].copy()

    def base_lin_vel_world(self) -> np.ndarray:
        return self.d.qvel[self.root_vadr : self.root_vadr + 3].copy()

    def yaw(self) -> float:
        w, x, y, z = self.base_quat
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def tilt_deg(self) -> float:
        w, x, y, z = self.base_quat
        bz = 1 - 2 * (x * x + y * y)  # world z component of the body z axis
        return math.degrees(math.acos(max(-1.0, min(1.0, bz))))

    def is_fallen(self) -> bool:
        return self.base_pos[2] < FALL_HEIGHT or self.tilt_deg() > FALL_TILT_DEG

    def signals(self) -> ObsSignals:
        p = NUM_POLICY_JOINTS
        return ObsSignals(
            base_ang_vel=self.base_ang_vel_body(),
            base_quat=self.base_quat,
            command=self.cmd.copy(),
            phase_time=self.phase_t,
            q=self.q[:p],
            dq=self.dq[:p],
            last_action=self.last_action.copy(),
        )

    # ------------------------------------------------------------------ control
    def set_command(self, vx: float, vy: float, wz: float) -> None:
        lim = self.policy.deploy.cmd_limits if self.policy else {"vx": [0, 0], "vy": [0, 0], "wz": [0, 0]}
        self.cmd = np.array(
            [
                float(np.clip(vx, *lim["vx"])),
                float(np.clip(vy, *lim["vy"])),
                float(np.clip(wz, *lim["wz"])),
            ]
        )

    def step(self) -> None:
        """One 20 ms policy step (4 physics substeps), then falls / checkpoints / laps and the overlay data."""
        if self.mode == "policy":
            dep = self.policy.deploy
            obs = build_obs(dep, self.signals())
            raw = self.policy.act(obs)
            filt = self.filter.step(raw)
            a = filt if self.filter.apply else raw
            self.last_action = a
            target = dep.default + dep.action_scale * a
            lo, hi = self.jm.limits_lo[:NUM_POLICY_JOINTS], self.jm.limits_hi[:NUM_POLICY_JOINTS]
            self.d.ctrl[self.act_idx[:NUM_POLICY_JOINTS]] = np.clip(target, lo, hi)
            self.d.ctrl[self.act_idx[NUM_POLICY_JOINTS:]] = self.default_all[NUM_POLICY_JOINTS:]
        else:
            self.filter.step(np.zeros(NUM_POLICY_JOINTS))
            self.d.ctrl[self.act_idx] = self.default_all
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.m, self.d)
        self.t += POLICY_DT
        self.phase_t += POLICY_DT
        if not np.isfinite(self.d.qpos).all():
            self.falls += 1
            self.reset(keep_time=True)
            return
        self.motors.step(self.d.actuator_force[self.act_idx], self.dq, POLICY_DT)
        self._physics_cache = None
        phys = self.physics()
        self._note_prop_hits(phys)
        self.margin_hist.append(
            {"t": round(self.t, 3), "cp_margin_m": phys["cp_margin_m"], "tilt_deg": round(self.tilt_deg(), 2),
             "height": round(float(self.base_pos[2]), 4)}
        )  # fmt: skip
        self._score()
        self._record()

    def _score(self) -> None:
        if self.fallen_at is None and self.is_fallen():
            self.fallen_at = self.t
            self.falls += 1
            self._analyse_fall()
        if self.fallen_at is not None:
            if self.t - self.fallen_at >= FALL_RESET_S:
                self.reset(keep_time=True)
            return
        cps = self.geometry["checkpoints"]
        nxt = np.asarray(cps[self.cp_index][:2])
        if np.linalg.norm(self.base_pos[:2] - nxt) < self.geometry["checkpoint_radius"]:
            self.cp_index += 1
            if self.cp_index >= len(cps):
                lap = self.t - self.lap_start
                self.laps += 1
                self.last_lap_s = round(lap, 2)
                self.best_lap_s = self.last_lap_s if self.best_lap_s is None else min(self.best_lap_s, self.last_lap_s)
                self.cp_index = 0
                self.lap_start = self.t

    def _analyse_fall(self) -> None:
        hist = list(self.margin_hist)
        res = analyse_fall(self.t, hist, FALL_TILT_DEG, FALL_HEIGHT, list(self.prop_hits))
        step = max(1, int(round(1.0 / (HISTORY_HZ * POLICY_DT))))
        res["history"] = hist[::-1][::step][::-1]  # 10 Hz, ending at the fall
        self.fall_analysis = res
        self.falls_log.append({k: res[k] for k in ("t_fall", "t_unstable", "lead_s", "cause")})

    # ------------------------------------------------------------------ props
    def throw(self, kind: str = "ball", speed: float = 4.0, from_deg: float = 0.0) -> dict:
        """Spawn a prop PROP_SPAWN_DIST from the pelvis at pelvis height, flying at the pelvis. from_deg = direction
        it comes FROM relative to the robot heading (0 front, 90 left). At most MAX_PROPS: the oldest is recycled."""
        if kind not in PROP_KINDS:
            raise ValueError(f"kind must be one of {sorted(PROP_KINDS)}")
        speed = float(speed)
        if not 1.0 <= speed <= 8.0:
            raise ValueError("speed must be in [1, 8] m/s")
        if len(self.active_props) >= MAX_PROPS:
            self._park(self.active_props.pop(0)["slot"])
        used = {p["slot"]["geom"] for p in self.active_props}
        slot = next(s for s in self.prop_slots if s["kind"] == kind and s["geom"] not in used)
        ang = self.yaw() + math.radians(float(from_deg))
        pel = self.base_pos
        pos = pel + PROP_SPAWN_DIST * np.array([math.cos(ang), math.sin(ang), 0.0])
        pos[2] = max(pel[2], PROP_KINDS[kind]["size"][0] + 0.01)
        vel = (pel - pos) / max(np.linalg.norm(pel - pos), 1e-9) * speed
        qa, va = slot["qadr"], slot["vadr"]
        self.d.qpos[qa : qa + 3] = pos
        self.d.qpos[qa + 3 : qa + 7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qvel[va : va + 3] = vel
        self.d.qvel[va + 3 : va + 6] = 0.0
        mujoco.mj_forward(self.m, self.d)
        pid = self.prop_seq
        self.prop_seq += 1
        self.active_props.append({"slot": slot, "id": pid, "kind": kind})
        return {"ok": True, "id": pid}

    def _park(self, slot: dict) -> None:
        qa, va = slot["qadr"], slot["vadr"]
        self.d.qpos[qa : qa + 3] = _park_pos(slot["kind"], slot["i"])
        self.d.qpos[qa + 3 : qa + 7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qvel[va : va + 6] = 0.0

    def props(self) -> List[dict]:
        out = []
        for p in self.active_props:
            s = p["slot"]
            qa = s["qadr"]
            out.append(
                {
                    "id": p["id"],
                    "kind": p["kind"],
                    "pos": [round(float(v), 4) for v in self.d.qpos[qa : qa + 3]],
                    "quat": [round(float(v), 5) for v in self.d.qpos[qa + 3 : qa + 7]],
                    "size": list(PROP_KINDS[p["kind"]]["size"]),
                }
            )
        return out

    def _note_prop_hits(self, phys: dict) -> None:
        active = {p["slot"]["geom"]: p["kind"] for p in self.active_props}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in active and self.robot_geom[b]:
                    self.prop_hits.append((round(self.t, 3), active[a]))
                    return

    # ------------------------------------------------------------------ physics overlay
    def physics(self) -> dict:
        """CoM, capture point, foot support polygon, signed margin and robot-involved contacts (world frame)."""
        if self._physics_cache is not None:
            return self._physics_cache
        m, d = self.m, self.d
        mujoco.mj_subtreeVel(m, d)
        com = d.subtree_com[self.pelvis_id].copy()
        com_vel = d.subtree_linvel[self.pelvis_id].copy()
        cp = capture_point(com, com_vel)
        support_pts, contacts = [], []
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            r1, r2 = self.robot_geom[g1], self.robot_geom[g2]
            if not (r1 or r2) or c.efc_address < 0:
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            frame = np.asarray(c.frame).reshape(3, 3)
            fw = frame.T @ f6[:3]  # contact frame -> world (force geom1 exerts on geom2)
            if (self.foot_geom[g1] and not r2) or (self.foot_geom[g2] and not r1):
                if (g1 if not r1 else g2) not in self.prop_geom_slot:
                    support_pts.append((float(c.pos[0]), float(c.pos[1])))
            contacts.append(
                {"pos": [round(float(v), 4) for v in c.pos], "force": [round(float(v), 2) for v in fw],
                 "a": self.geom_label[g1], "b": self.geom_label[g2], "_n": float(abs(f6[0]))}
            )  # fmt: skip
        contacts.sort(key=lambda x: -x["_n"])
        for x in contacts:
            del x["_n"]
        hull = convex_hull(support_pts)
        margin = signed_margin(cp, hull)
        self._physics_cache = {
            "com": [round(float(v), 4) for v in com],
            "com_vel": [round(float(v), 4) for v in com_vel],
            "capture_point": [round(cp[0], 4), round(cp[1], 4)],
            "support": [[round(x, 4), round(y, 4)] for x, y in hull],
            "cp_margin_m": None if margin is None else round(margin, 4),
            "contacts": contacts[:MAX_CONTACTS],
        }
        return self._physics_cache

    # ------------------------------------------------------------------ replay frames + snapshot for /walk/state
    def _base_block(self) -> dict:
        return {
            "pos": [round(float(v), 4) for v in self.base_pos],
            "quat": [round(float(v), 5) for v in self.base_quat],
            "lin_vel": [round(float(v), 3) for v in self.base_lin_vel_world()],
            "yaw": round(self.yaw(), 4),
        }

    def _record(self) -> None:
        root, q = self.full_qpos()
        self.frames.append(
            {"t": round(self.t, 3), "root": root, "qarr": q, "base": self._base_block(),
             "q": [round(float(v), 4) for v in q], "props": self.props(), "physics": self.physics()}
        )  # fmt: skip

    def snapshot(self) -> dict:
        cps = self.geometry["checkpoints"]
        proc = self.course_name == "procedural"
        return {
            "mode": self.mode,
            "course": self.course_name,
            "seed": self.seed if proc else None,
            "difficulty": self.difficulty if proc else None,
            "t": round(self.t, 3),
            "cmd": [round(float(v), 3) for v in self.cmd],
            "base": self._base_block(),
            "q": [round(float(v), 4) for v in self.q],
            "fallen": self.fallen_at is not None,
            "falls": self.falls,
            "checkpoint": {"index": self.cp_index, "total": len(cps), "next": cps[self.cp_index]},
            "lap": {
                "current_s": round(self.t - self.lap_start, 2),
                "last_s": self.last_lap_s,
                "best_s": self.best_lap_s,
                "laps": self.laps,
            },
            "physics": self.physics(),
            "motors": self.motors.motors_block(),
            "thermal": self.motors.thermal_block(),
            "battery": self.motors.battery_block(),
            "action_filter": self.filter.block(),
            "props": self.props(),
            "fall_analysis": self.fall_analysis,
            "falls_log": list(self.falls_log),
        }

    def full_qpos(self) -> Tuple[np.ndarray, np.ndarray]:
        """(root qpos 7: x y z qw qx qy qz, 26 joint angles) for the Unreal puppet."""
        return self.d.qpos[self.root_qadr : self.root_qadr + 7].copy(), self.q
