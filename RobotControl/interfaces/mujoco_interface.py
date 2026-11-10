"""Headless MuJoCo digital twin of the R1 (pure Python, no UE5 required).

Loads ``scene_stand.xml`` / ``scene_ground.xml``. The model has one ``<position>``
actuator per joint (named like the joint without ``_joint``); the joint order in
the model equals the joint-map order. PD gains sent through ``send_low_cmd`` are
applied by rewriting the actuator gain/bias parameters each tick::

    gainprm[:,0] = kp      biasprm[:,1] = -kp      biasprm[:,2] = -kd

so ``force = kp*(ctrl - q) - kd*dq`` exactly like the real motor controller.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import numpy as np

from .. import MJCF_DIR
from ..configs.joint_map import JointMap, actuator_name_for
from .base_interface import LowStateData, MotorCommand, RobotHardwareInterface

log = logging.getLogger(__name__)

SCENES = {
    "stand": "scene_stand.xml",
    "ground": "scene_ground.xml",
}


def resolve_scene(scene: str | Path) -> Path:
    s = str(scene)
    if s in SCENES:
        return MJCF_DIR / SCENES[s]
    p = Path(s)
    if p.exists():
        return p
    cand = MJCF_DIR / s
    if cand.exists():
        return cand
    raise FileNotFoundError(f"scene {scene!r} not found (looked in {MJCF_DIR})")


class MujocoInterface(RobotHardwareInterface):
    def __init__(self, joint_map: Optional[JointMap] = None, scene: str | Path = "stand",
                 policy_dt: float = 0.02, render: bool = False, keyframe: Optional[str] = "home",
                 realtime: bool = False):
        super().__init__(joint_map)
        self.scene_path = resolve_scene(scene)
        self.policy_dt = float(policy_dt)
        self.render = bool(render)
        self.keyframe = keyframe
        self.realtime = realtime
        self.model = None
        self.data = None
        self._viewer = None
        self._tick = 0
        self._qpos_adr: np.ndarray
        self._dof_adr: np.ndarray
        self._act_ids: np.ndarray
        self._sens = {}

    # ------------------------------------------------------------- lifecycle
    def connect(self) -> None:
        import mujoco  # local import keeps the package importable without mujoco

        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        m = self.model

        # joints: verify order & build address tables
        names = []
        qpos_adr, dof_adr = [], []
        for j in range(m.njnt):
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            names.append(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j))
            qpos_adr.append(int(m.jnt_qposadr[j]))
            dof_adr.append(int(m.jnt_dofadr[j]))
        if names != self.joint_map.names:
            raise RuntimeError(
                "MJCF joint order differs from joint map:\n"
                f"  mjcf: {names}\n  map : {self.joint_map.names}"
            )
        self._qpos_adr = np.array(qpos_adr, dtype=np.int64)
        self._dof_adr = np.array(dof_adr, dtype=np.int64)

        # actuators: one per joint, looked up by name (joint name without _joint)
        act_ids = []
        for jn in self.joint_map.names:
            an = actuator_name_for(jn)
            aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, an)
            if aid < 0:
                raise RuntimeError(f"actuator {an!r} missing in {self.scene_path.name}")
            act_ids.append(aid)
        self._act_ids = np.array(act_ids, dtype=np.int64)
        # position actuators drive their joint directly: ctrl is a target in rad
        self._ctrl_lo = m.actuator_ctrlrange[self._act_ids, 0].copy()
        self._ctrl_hi = m.actuator_ctrlrange[self._act_ids, 1].copy()
        self._ctrl_limited = m.actuator_ctrllimited[self._act_ids].astype(bool)

        # sensors
        for sname in ("imu_quat", "imu_ang_vel", "imu_lin_acc"):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, sname)
            if sid < 0:
                raise RuntimeError(f"sensor {sname!r} missing in {self.scene_path.name}")
            adr, dim = int(m.sensor_adr[sid]), int(m.sensor_dim[sid])
            self._sens[sname] = slice(adr, adr + dim)

        self.sim_dt = float(m.opt.timestep)
        self.substeps = max(1, int(round(self.policy_dt / self.sim_dt)))
        self.reset()

        if self.render:
            try:
                import mujoco.viewer as mjv
                self._viewer = mjv.launch_passive(self.model, self.data)
            except Exception as exc:  # pragma: no cover - viewer optional
                log.warning("mujoco viewer unavailable (%s); running headless", exc)
                self._viewer = None
        self.connected = True
        log.info("MujocoInterface: %s dt=%.4f substeps=%d", self.scene_path.name, self.sim_dt, self.substeps)

    def reset(self, keyframe: Optional[str] = None) -> None:
        mujoco = self._mj
        key = keyframe if keyframe is not None else self.keyframe
        kid = -1
        if key is not None:
            kid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, key)
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, kid)
        else:
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[self._qpos_adr] = self.joint_map.defaults
        self.data.ctrl[self._act_ids] = self.data.qpos[self._qpos_adr]
        self.data.qfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._tick = 0

    def close(self) -> None:
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:  # pragma: no cover
                pass
            self._viewer = None
        self.connected = False

    # ----------------------------------------------------------------- state
    def get_low_state(self) -> LowStateData:
        d = self.data
        s = d.sensordata
        return LowStateData(
            q=d.qpos[self._qpos_adr].copy(),
            dq=d.qvel[self._dof_adr].copy(),
            tau=d.actuator_force[self._act_ids].copy(),
            imu_quat=s[self._sens["imu_quat"]].copy(),
            gyro=s[self._sens["imu_ang_vel"]].copy(),
            accel=s[self._sens["imu_lin_acc"]].copy(),
            timestamp=float(d.time),
            extras={"root_pos": d.qpos[:3].copy()},
            tick=self._tick,
        )

    # --------------------------------------------------------------- command
    def _send(self, cmd: MotorCommand) -> None:
        m, d = self.model, self.data
        m.actuator_gainprm[self._act_ids, 0] = cmd.kp
        m.actuator_biasprm[self._act_ids, 0] = 0.0
        m.actuator_biasprm[self._act_ids, 1] = -cmd.kp
        m.actuator_biasprm[self._act_ids, 2] = -cmd.kd
        q_cmd = cmd.q
        if np.any(self._ctrl_limited):
            q_cmd = np.where(self._ctrl_limited, np.clip(cmd.q, self._ctrl_lo, self._ctrl_hi), cmd.q)
        d.ctrl[self._act_ids] = q_cmd
        # feed-forward torque + velocity target (kd * dq_des) as an applied generalized force
        d.qfrc_applied[self._dof_adr] = cmd.tau + cmd.kd * cmd.dq

    # ---------------------------------------------------------------- timing
    def step(self, n: Optional[int] = None) -> None:
        """Advance physics by ``n`` sim steps (default: one policy tick = substeps)."""
        n = self.substeps if n is None else int(n)
        mujoco = self._mj
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)
        self._tick += 1
        if self._viewer is not None:
            if self._viewer.is_running():
                self._viewer.sync()
            else:
                self._viewer = None

    def tick(self) -> None:
        self.step()

    @property
    def is_simulation(self) -> bool:
        return True

    def now(self) -> float:
        return float(self.data.time)

    # ---------------------------------------------------------------- extras
    def set_joint_positions(self, q: np.ndarray, zero_velocity: bool = True) -> None:
        """Teleport joints (for tests / resets)."""
        self.data.qpos[self._qpos_adr] = np.asarray(q, dtype=np.float64)
        if zero_velocity:
            self.data.qvel[self._dof_adr] = 0.0
        self.data.ctrl[self._act_ids] = self.data.qpos[self._qpos_adr]
        self._mj.mj_forward(self.model, self.data)

    def base_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """(pos, quat_wxyz) of the pelvis (free joint) from qpos."""
        return self.data.qpos[0:3].copy(), self.data.qpos[3:7].copy()
