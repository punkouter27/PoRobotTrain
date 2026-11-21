"""Walk sandbox physics: free-floating R1 on the ground scene, driven by the Level 2 policy (or a PD hold).

MuJoCo owns physics (5 ms step, implicitfast, 50 Hz policy = 4 substeps). The model is
``UnrealProject/MJCF/scene_ground.xml`` with the course boxes from ``course.py`` added at load time and the
calibrated actuator/joint dynamics from ``sim_calibration.yaml`` applied, like the other twins.
Pure and single-threaded: ``WalkSandbox`` (sandbox.py) runs it in real time for the service.
"""

from __future__ import annotations

import math
import pathlib
from typing import Optional

import mujoco
import numpy as np

from .. import PROJECT_ROOT
from ..configs.joint_map import JointMap
from ..configs.sim_calibration import SimCalibration
from .course import course as get_course
from .policy import NUM_POLICY_JOINTS, Level2Policy, ObsSignals, build_obs

SCENE_XML = PROJECT_ROOT / "UnrealProject/MJCF/scene_ground.xml"
POLICY_DT = 0.02
SUBSTEPS = 4
FALL_HEIGHT = 0.45  # pelvis below this = fallen
FALL_TILT_DEG = 60.0  # pelvis z axis this far from vertical = fallen
FALL_RESET_S = 1.0


def load_model(course_name: str, scene: pathlib.Path = SCENE_XML, calibrated: bool = True) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(scene))
    for b in get_course(course_name)["boxes"]:
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
    m = spec.compile()
    if calibrated:
        jm = JointMap.load()
        hinge = np.array(
            [m.jnt_dofadr[i] for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE], dtype=np.int64
        )
        SimCalibration.load(jm=jm).apply_to_model(mujoco, m, hinge)
    return m


class WalkSim:
    """mode 'policy': velocity policy; mode 'hold': every joint PD-held at its default pose (no walking)."""

    def __init__(
        self,
        course: str = "loop",
        mode: str = "policy",
        policy: Optional[Level2Policy] = None,
        calibrated: bool = True,
    ):
        if mode not in ("policy", "hold"):
            raise ValueError(f"mode must be 'policy' or 'hold', got {mode!r}")
        if mode == "policy" and policy is None:
            raise ValueError("mode 'policy' needs a Level2Policy")
        self.jm = JointMap.load()
        self.course_name = course
        self.geometry = get_course(course)
        self.mode = mode
        self.policy = policy
        self.m = load_model(course, calibrated=calibrated)
        self.d = mujoco.MjData(self.m)
        m = self.m
        assert abs(m.opt.timestep - POLICY_DT / SUBSTEPS) < 1e-9, "walk scene must keep the 5 ms physics step"
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
        free = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
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
        self.cmd = np.zeros(3)
        self.falls = 0
        self.laps = 0
        self.last_lap_s: Optional[float] = None
        self.best_lap_s: Optional[float] = None
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key("home").id)
        self.d.eq_active[self.harness_id] = self.mode == "hold"
        self.d.ctrl[self.act_idx] = self.default_all
        mujoco.mj_forward(self.m, self.d)
        self.t = 0.0
        self.phase_t = 0.0
        self.last_action = np.zeros(NUM_POLICY_JOINTS)
        self.fallen_at: Optional[float] = None
        self.cp_index = 0
        self.lap_start = 0.0

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
        """One 20 ms policy step (4 physics substeps), then falls / checkpoints / laps."""
        if self.mode == "policy":
            dep = self.policy.deploy
            obs = build_obs(dep, self.signals())
            a = self.policy.act(obs)
            self.last_action = a
            target = dep.default + dep.action_scale * a
            lo, hi = self.jm.limits_lo[:NUM_POLICY_JOINTS], self.jm.limits_hi[:NUM_POLICY_JOINTS]
            self.d.ctrl[self.act_idx[:NUM_POLICY_JOINTS]] = np.clip(target, lo, hi)
            self.d.ctrl[self.act_idx[NUM_POLICY_JOINTS:]] = self.default_all[NUM_POLICY_JOINTS:]
        else:
            self.d.ctrl[self.act_idx] = self.default_all
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.m, self.d)
        self.t += POLICY_DT
        self.phase_t += POLICY_DT
        if not np.isfinite(self.d.qpos).all():
            self.falls += 1
            self.reset()
            return
        self._score()

    def _score(self) -> None:
        if self.fallen_at is None and self.is_fallen():
            self.fallen_at = self.t
            self.falls += 1
        if self.fallen_at is not None:
            if self.t - self.fallen_at >= FALL_RESET_S:
                falls, laps, last, best = self.falls, self.laps, self.last_lap_s, self.best_lap_s
                self.reset()
                self.falls, self.laps, self.last_lap_s, self.best_lap_s = falls, laps, last, best
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

    # ------------------------------------------------------------------ snapshot for /walk/state
    def snapshot(self) -> dict:
        cps = self.geometry["checkpoints"]
        return {
            "mode": self.mode,
            "course": self.course_name,
            "t": round(self.t, 3),
            "cmd": [round(float(v), 3) for v in self.cmd],
            "base": {
                "pos": [round(float(v), 4) for v in self.base_pos],
                "quat": [round(float(v), 5) for v in self.base_quat],
                "lin_vel": [round(float(v), 3) for v in self.base_lin_vel_world()],
                "yaw": round(self.yaw(), 4),
            },
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
        }

    def full_qpos(self) -> tuple[np.ndarray, np.ndarray]:
        """(root qpos 7: x y z qw qx qy qz, 26 joint angles) for the Unreal puppet."""
        return self.d.qpos[self.root_qadr : self.root_qadr + 7].copy(), self.q
