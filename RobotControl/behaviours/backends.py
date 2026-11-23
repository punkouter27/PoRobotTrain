"""Transports for behaviours. Each one powers ONLY the behaviour's ``powered`` joints.

    sim    headless MuJoCo stand scene (this PC)
    urlab  Unreal Engine twin through URLab (MuJoCo inside Unreal), Python steps physics in direct mode
    real   R1 low-level DDS commands (on the robot's Jetson)

Common API: ``read() -> (q, dq, age_s)`` as arrays in joint-map order, ``command(targets: {joint: rad})``,
``advance(t)``, ``release()``, ``close()``, plus ``rate_hz`` and ``realtime``.

What happens to the joints the behaviour does not power:
  real   motor mode 0 (disabled, zero torque) in every LowCmd - identical to the robot's zero-torque state.
         On the stand those geared joints hold their pose (checked: waist and arms do not drift).
  sim    actuators off. MuJoCo joints have no gearbox friction, so a limp waist topples and a limp head pitch
         creeps; those are locked where they settle. A second head-off copy of the scene is stepped
         alongside and ``read`` reports other joints as start + (with - without), i.e. only motion CAUSED by
         the behaviour.
  urlab  held at the pose they had when the run started (PD with joint-map gains). This stands in for the
         real geared joints holding still; it is a visual twin of the motion, not a test of the reaction
         torques. The real robot run checks those with the other-joint abort.
         The twin's MJCF gives the legs no stiffness (kp 0, built for the upper-body Level 0 policy), so after
         the home reset they hang and keep creeping (knee ~11 deg in 2 s, ankles ~0.7 deg/s after that). The
         backend lets the twin settle for SETTLE_S before taking the start pose, and joints the twin cannot
         hold (model kp 0) are left out of the other-joint check on this target only.
"""

from __future__ import annotations

import collections
import copy
import math
import time
from typing import Dict, Optional, Sequence

import numpy as np

from .. import MJCF_DIR
from ..configs.joint_map import JointMap
from ..configs.sim_calibration import SimCalibration
from ..interfaces.base_interface import quat_to_rpy


def extract_lowstate_telemetry(msg, jm: JointMap) -> dict:
    """Extract read-only IMU, motor temp, bus voltage, and mode telemetry from a unitree_hg LowState_."""
    slots = jm.motor_slots.tolist()
    temps = [int(max(msg.motor_state[s].temperature)) for s in slots]
    hot_idx = int(np.argmax(temps))
    vols = [float(msg.motor_state[s].vol) for s in slots if float(msg.motor_state[s].vol) > 0.5]
    modes = [int(msg.motor_state[s].mode) for s in slots]
    rpy = [round(math.degrees(float(x)), 1) for x in msg.imu_state.rpy]
    return {
        "imu_rpy_deg": rpy,
        "temps_c": temps,
        "max_temp_c": temps[hot_idx],
        "hot_joint": jm.names[hot_idx].replace("_joint", ""),
        "bus_v": round(float(np.median(vols)), 1) if vols else None,
        "mode_machine": int(msg.mode_machine),
        "active_motors": sum(1 for m in modes if m != 0),
    }


class _Base:
    rate_hz = 50.0
    realtime = False
    # joints this transport cannot hold still, so the other-joint check skips them (with a printed note)
    unchecked: Sequence[str] = ()

    def __init__(self, jm: JointMap, powered: Sequence[str]):
        self.jm = jm
        self.powered = list(powered)
        self.pidx = [jm.index_of(n) for n in self.powered]

    def telemetry(self) -> dict:
        return {}

    def advance(self, until_t: float) -> None:
        pass

    def release(self) -> None:
        pass

    def close(self) -> None:
        pass


# ----------------------------------------------------------------------------- MuJoCo (headless)
class SimBackend(_Base):
    rate_hz = 200.0

    LOCK_IF_UNPOWERED = ("waist_roll_joint", "waist_yaw_joint", "waist_pitch_joint", "head_pitch_joint")

    def __init__(
        self,
        jm: JointMap,
        powered: Sequence[str],
        scene: str = "scene_stand.xml",
        friction_nm: Optional[float] = None,
        start_q: Optional[Dict[str, float]] = None,
        settle_s: float = 6.0,
        calibration: Optional[SimCalibration] = None,
        realistic: bool = True,
        seed: int = 0,
    ):
        super().__init__(jm, powered)
        import mujoco

        self.mj = mujoco
        self.realistic = bool(realistic)
        self.cal = calibration if calibration is not None else SimCalibration.load(jm=jm)
        self.rng = np.random.default_rng(seed)

        m = self.m = mujoco.MjModel.from_xml_path(str(MJCF_DIR / scene))
        d = self.d = mujoco.MjData(m)
        k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
        if k >= 0:
            mujoco.mj_resetDataKeyframe(m, d, k)
        m.actuator_gainprm[:, 0] = 0.0  # every actuator off; powered joints get PD torque below
        m.actuator_biasprm[:, :3] = 0.0
        d.ctrl[:] = 0.0
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in jm.names]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jid])
        self.vadr = np.array([m.jnt_dofadr[j] for j in jid])
        pv = self.vadr[self.pidx]
        unpowered_mask = np.ones(jm.num_joints, dtype=bool)
        unpowered_mask[self.pidx] = False
        uv = self.vadr[unpowered_mask]

        if self.realistic:
            self.cal.apply_to_model(mujoco, m, self.vadr)
            u_fric = self.cal.unpowered_friction_nm[unpowered_mask]
            if friction_nm is not None:
                u_fric = np.maximum(u_fric, float(friction_nm))
            m.dof_frictionloss[uv] = np.maximum(m.dof_frictionloss[uv], u_fric)
            m.dof_solref[uv, 0] = max(0.01, 2.0 * m.opt.timestep)
            if self.cal.compliant_stand:
                wid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "stand_weld")
                if wid >= 0:
                    m.eq_solref[wid] = [self.cal.stand_timeconst_s, self.cal.stand_damping_ratio]
        else:
            fl = m.dof_frictionloss
            fl[6:] = np.maximum(fl[6:], float(friction_nm if friction_nm is not None else 0.5))
            fl[pv] = np.minimum(fl[pv], 0.05)

        for n, v in (start_q or {}).items():
            d.qpos[self.qadr[jm.index_of(n)]] = v
        mujoco.mj_forward(m, d)
        # settle the limp body with temporary damping; powered joints are held where they were placed
        damping = m.dof_damping.copy()
        m.dof_damping[6:] = np.maximum(damping[6:], 5.0)
        m.dof_damping[pv] = 1e3
        # Geared joints the real robot holds when unpowered (waist, head pitch) are held where they start, too:
        # letting them droop here and locking them there compounded run after run on the practice twin (head
        # pitch 0 -> 10 -> 35 deg), which no real robot does and which tripped the head-centred start check.
        for n in self.LOCK_IF_UNPOWERED:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            if j >= 0 and n not in self.powered:
                m.dof_damping[m.jnt_dofadr[j]] = 1e3
        for _ in range(int(settle_s / m.opt.timestep)):
            mujoco.mj_step(m, d)
        m.dof_damping[:] = damping
        for n in self.LOCK_IF_UNPOWERED:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            if j >= 0 and n not in self.powered:
                qv = float(d.qpos[m.jnt_qposadr[j]])
                m.jnt_limited[j] = 1
                m.jnt_range[j] = (qv - 1e-4, qv + 1e-4)
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)

        self.d_ref = copy.copy(d)  # behaviour-off reference
        self.q_start = d.qpos[self.qadr].copy()
        self.dt = m.opt.timestep
        self.t = 0.0
        self.targets: Dict[int, float] = {}
        self.kp = jm.kp[self.pidx]
        self.kd = jm.kd[self.pidx]
        self.eff = jm.effort[self.pidx]
        self._pelvis_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.on = False
        self.tq = d.qpos[self.qadr[self.pidx]].copy()
        self._tq_filt = self.tq.copy()
        self._dq_filt = np.zeros(jm.num_joints, dtype=np.float64)
        delay_steps = int(round((self.cal.control_delay_ms / 1000.0) / self.dt)) if self.realistic else 0
        self._cmd_delay = collections.deque(
            [self.tq.copy() for _ in range(max(0, delay_steps))],
            maxlen=max(1, delay_steps + 1),
        )
        self._delay_steps = max(0, delay_steps)

    def telemetry(self) -> dict:
        if self._pelvis_id < 0:
            return {}
        rpy = quat_to_rpy(self.d.xquat[self._pelvis_id])
        if self.realistic and self.cal.imu_noise_rad > 0:
            rpy = rpy + self.rng.normal(0.0, self.cal.imu_noise_rad, size=3)
        return {"imu_rpy_deg": [round(math.degrees(float(x)), 1) for x in rpy]}

    def read(self):
        q_raw = self.d.qpos[self.qadr]
        caused = self.q_start + (q_raw - self.d_ref.qpos[self.qadr])
        caused[self.pidx] = q_raw[self.pidx]
        dq_raw = self.d.qvel[self.vadr].copy()
        if not self.realistic:
            return caused, dq_raw, 0.0
        a = self.cal.vel_filter_alpha
        self._dq_filt = (1.0 - a) * self._dq_filt + a * dq_raw
        q_meas = caused + self.cal.zero_offset_rad
        if self.cal.pos_noise_rad > 0:
            q_meas = q_meas + self.rng.normal(0.0, self.cal.pos_noise_rad, size=q_meas.shape)
        dq_meas = self._dq_filt
        if self.cal.vel_noise_rad_s > 0:
            dq_meas = dq_meas + self.rng.normal(0.0, self.cal.vel_noise_rad_s, size=dq_meas.shape)
        age_s = self.cal.control_delay_ms / 1000.0
        return q_meas, dq_meas, age_s

    def command(self, targets: Dict[str, float]) -> None:
        cmd = np.array([targets[n] for n in self.powered], dtype=np.float64)
        if self.realistic:
            cmd = cmd - self.cal.zero_offset_rad[self.pidx]
        self.tq = cmd
        self.on = True

    def release(self) -> None:
        self.on = False

    def advance(self, until_t: float) -> None:
        pq, pv = self.qadr[self.pidx], self.vadr[self.pidx]
        v_scale = self.cal.bus_voltage_v / max(self.cal.nominal_bus_v, 1.0)
        lpf_alpha = self.dt / max(self.cal.lowpass_tau_s + self.dt, 1e-6)
        while self.t + 1e-9 < until_t:
            if self.on:
                if self.realistic:
                    self._cmd_delay.append(self.tq.copy())
                    tq_delayed = (
                        self._cmd_delay.popleft() if len(self._cmd_delay) > self._delay_steps else self._cmd_delay[0]
                    )
                    self._tq_filt = (1.0 - lpf_alpha) * self._tq_filt + lpf_alpha * tq_delayed
                    eff_dyn = np.maximum(0.0, self.eff * v_scale - self.cal.bemf_nm_per_rad_s * np.abs(self.d.qvel[pv]))
                    tau = self.kp * (self._tq_filt - self.d.qpos[pq]) - self.kd * self.d.qvel[pv]
                    self.d.qfrc_applied[pv] = np.clip(tau, -eff_dyn, eff_dyn)
                else:
                    tau = self.kp * (self.tq - self.d.qpos[pq]) - self.kd * self.d.qvel[pv]
                    self.d.qfrc_applied[pv] = np.clip(tau, -self.eff, self.eff)
            else:
                self.d.qfrc_applied[pv] = 0.0
            self.mj.mj_step(self.m, self.d)
            self.mj.mj_step(self.m, self.d_ref)
            self.t += self.dt


# ----------------------------------------------------------------------------- Unreal (URLab)
class URLabBackend(_Base):
    """Drives the Unreal twin through ``URLabInterface`` (direct mode: Python steps the physics, 50 Hz)."""

    rate_hz = 50.0
    SETTLE_S = 2.0

    def __init__(
        self,
        jm: JointMap,
        powered: Sequence[str],
        address: str = "tcp://localhost",
        realtime: bool = True,
        start_q: Optional[Dict[str, float]] = None,
    ):
        super().__init__(jm, powered)
        from ..interfaces.urlab_interface import URLabInterface

        head_only = set(self.powered) <= {"head_yaw_joint", "head_pitch_joint"}
        self.itf = URLabInterface(
            jm,
            address=address,
            step_mode="direct",
            policy_dt=1.0 / self.rate_hz,
            reset_on_connect=not head_only and start_q is None,
        )
        self.itf.connect()
        self.realtime = realtime
        self.kp = jm.kp.copy()
        self.kd = jm.kd.copy()
        self.hold = self.itf.get_low_state().q.copy()
        if start_q:
            for n, v in start_q.items():
                self.hold[jm.index_of(n)] = float(v)
        for _ in range(int(self.SETTLE_S * self.rate_hz)):  # let the limp legs hang and align upper body to start_q
            self.itf.send_low_cmd(self.hold, self.kp, self.kd)
            self.itf.tick()
        self.hold = self.itf.get_low_state().q.copy()
        model_kp = self.itf.model_kp()
        if model_kp is not None:
            self.unchecked = [n for i, n in enumerate(jm.names) if model_kp[i] <= 0.0 and n not in self.powered]
        print(f"[urlab] connected; powered: {self.powered}; every other joint held at its start pose")
        if self.unchecked:
            print(
                f"[urlab] twin has no stiffness on {len(self.unchecked)} joints ({', '.join(self.unchecked)}): "
                f"they hang freely here and are not part of the other-joint check on the twin"
            )

    def read(self):
        s = self.itf.get_low_state()
        return s.q, s.dq, 0.0

    def telemetry(self) -> dict:
        s = self.itf.get_low_state()
        rpy = quat_to_rpy(s.imu_quat)
        return {"imu_rpy_deg": [round(math.degrees(float(x)), 1) for x in rpy]}

    def command(self, targets: Dict[str, float]) -> None:
        q = self.hold.copy()
        for n, v in targets.items():
            q[self.jm.index_of(n)] = v
        self.itf.send_low_cmd(q, self.kp, self.kd)

    def advance(self, until_t: float) -> None:
        self.itf.tick()

    def release(self) -> None:
        # end in the same held pose; the in-engine policy reclaims the actuators when the client disconnects
        self.itf.send_low_cmd(self.itf.get_low_state().q, self.kp, self.kd)
        self.itf.tick()

    def close(self) -> None:
        self.itf.close()


# ----------------------------------------------------------------------------- real robot
class RealBackend(_Base):
    """unitree_sdk2py LowCmd with ONLY the powered slots enabled (mode 1); every other slot stays mode 0."""

    rate_hz = 500.0
    realtime = True

    def __init__(self, jm: JointMap, powered: Sequence[str], iface: Optional[str], publish: bool):
        super().__init__(jm, powered)
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        from ..interfaces.unitree_interface import detect_robot_iface, ensure_channel_factory

        iface = iface or detect_robot_iface()
        if iface is None:
            raise RuntimeError("no network interface with a 192.168.123.x address; pass --iface")
        ensure_channel_factory(0, iface)
        self._setup(jm, publish, unitree_hg_msg_dds__LowCmd_, CRC())
        self._sub = ChannelSubscriber(jm.lowstate_topic, LowState_)
        self._sub.Init(self._on_state, 10)
        t_end = time.monotonic() + 5.0
        while self._msg is None:
            if time.monotonic() > t_end:
                raise TimeoutError(f"no {jm.lowstate_topic} within 5 s on {iface}")
            time.sleep(0.01)
        if publish:
            self._release_motion_mode()
            self._pub = ChannelPublisher(jm.lowcmd_topic, LowCmd_)
            self._pub.Init()
        print(
            f"[real] interface {iface}, powered slots {self.pslots} ({', '.join(self.powered)}), "
            f"{'PUBLISHING' if publish else 'READ-ONLY, nothing is published'}"
        )

    def _setup(self, jm, publish, new_cmd, crc):
        self.slots = jm.motor_slots.tolist()
        self.pslots = [int(jm.motor_slots[i]) for i in self.pidx]
        self.kp = jm.kp[self.pidx]
        self.kd = jm.kd[self.pidx]
        self.publish = publish
        self._new_cmd = new_cmd
        self._crc = crc
        self._msg = None
        self._t_msg = 0.0
        self._sub = None
        self._pub = None

    @staticmethod
    def _release_motion_mode():
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        for _ in range(10):
            status, result = msc.CheckMode()
            if status != 0:
                raise RuntimeError(f"MotionSwitcherClient.CheckMode failed: {status}")
            if not result.get("name"):
                return
            print(f"[real] releasing active motion mode {result.get('name')!r}")
            status, _ = msc.ReleaseMode()
            if status != 0:
                raise RuntimeError(f"ReleaseMode failed: {status}")
            time.sleep(1.0)
        raise RuntimeError("could not release the active motion mode")

    def _on_state(self, msg):
        if msg.crc != self._crc.Crc(msg):
            return
        self._msg = msg
        self._t_msg = time.monotonic()

    def read(self):
        msg = self._msg
        q = np.array([msg.motor_state[s].q for s in self.slots])
        dq = np.array([msg.motor_state[s].dq for s in self.slots])
        return q, dq, time.monotonic() - self._t_msg

    def telemetry(self) -> dict:
        if self._msg is None:
            return {}
        return extract_lowstate_telemetry(self._msg, self.jm)

    def _write(self, mode, q, kp, kd):
        cmd = self._new_cmd()  # fresh message: every motor_cmd is mode 0 / kp 0 / kd 0
        cmd.mode_pr = 0
        cmd.mode_machine = int(self._msg.mode_machine)
        for k, s in enumerate(self.pslots):
            mc = cmd.motor_cmd[s]
            mc.mode, mc.q, mc.dq, mc.tau, mc.kp, mc.kd = mode, float(q[k]), 0.0, 0.0, float(kp[k]), float(kd[k])
        enabled = [i for i, m in enumerate(cmd.motor_cmd) if m.mode != 0 or m.kp != 0 or m.kd != 0]
        assert set(enabled) <= set(self.pslots), f"refusing to send: slots {enabled} enabled"
        cmd.crc = self._crc.Crc(cmd)
        if self._pub is not None:
            self._pub.Write(cmd)

    def command(self, targets: Dict[str, float]) -> None:
        self._write(1, [targets[n] for n in self.powered], self.kp, self.kd)

    def release(self) -> None:
        if self._pub is None:
            return
        q = self.read()[0][self.pidx]
        zeros = np.zeros(len(self.pidx))
        for _ in range(250):  # 0.5 s pure damping on the powered joints, then disable
            self._write(1, q, zeros, self.kd)
            time.sleep(0.002)
        for _ in range(50):
            self._write(0, zeros, zeros, zeros)
            time.sleep(0.002)

    def close(self) -> None:
        for h in (self._sub, self._pub):
            try:
                if h is not None:
                    h.Close()
            except Exception:
                pass
