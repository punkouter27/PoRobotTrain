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

import copy
import time
from typing import Dict, Optional, Sequence

import numpy as np

from .. import MJCF_DIR
from ..configs.joint_map import JointMap


class _Base:
    rate_hz = 50.0
    realtime = False
    # joints this transport cannot hold still, so the other-joint check skips them (with a printed note)
    unchecked: Sequence[str] = ()

    def __init__(self, jm: JointMap, powered: Sequence[str]):
        self.jm = jm
        self.powered = list(powered)
        self.pidx = [jm.index_of(n) for n in self.powered]

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

    def __init__(self, jm: JointMap, powered: Sequence[str], scene: str = "scene_stand.xml",
                 friction_nm: float = 0.5, start_q: Optional[Dict[str, float]] = None, settle_s: float = 6.0):
        super().__init__(jm, powered)
        import mujoco
        self.mj = mujoco
        m = self.m = mujoco.MjModel.from_xml_path(str(MJCF_DIR / scene))
        d = self.d = mujoco.MjData(m)
        k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
        if k >= 0:
            mujoco.mj_resetDataKeyframe(m, d, k)
        m.actuator_gainprm[:, 0] = 0.0          # every actuator off; powered joints get PD torque below
        m.actuator_biasprm[:, :3] = 0.0
        d.ctrl[:] = 0.0
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in jm.names]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jid])
        self.vadr = np.array([m.jnt_dofadr[j] for j in jid])
        pv = self.vadr[self.pidx]
        fl = m.dof_frictionloss
        fl[6:] = np.maximum(fl[6:], friction_nm)
        fl[pv] = np.minimum(fl[pv], 0.05)
        for n, v in (start_q or {}).items():
            d.qpos[self.qadr[jm.index_of(n)]] = v
        mujoco.mj_forward(m, d)
        # settle the limp body with temporary damping; powered joints are held where they were placed
        damping = m.dof_damping.copy()
        m.dof_damping[6:] = np.maximum(damping[6:], 5.0)
        m.dof_damping[pv] = 1e3
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
        self.d_ref = copy.copy(d)                # behaviour-off reference
        self.q_start = d.qpos[self.qadr].copy()
        self.dt = m.opt.timestep
        self.t = 0.0
        self.targets: Dict[int, float] = {}
        self.kp = jm.kp[self.pidx]
        self.kd = jm.kd[self.pidx]
        self.eff = jm.effort[self.pidx]
        self.on = False

    def read(self):
        q = self.d.qpos[self.qadr]
        caused = self.q_start + (q - self.d_ref.qpos[self.qadr])
        caused[self.pidx] = q[self.pidx]
        return caused, self.d.qvel[self.vadr].copy(), 0.0

    def command(self, targets: Dict[str, float]) -> None:
        self.tq = np.array([targets[n] for n in self.powered])
        self.on = True

    def release(self) -> None:
        self.on = False

    def advance(self, until_t: float) -> None:
        pq, pv = self.qadr[self.pidx], self.vadr[self.pidx]
        while self.t + 1e-9 < until_t:
            if self.on:
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

    def __init__(self, jm: JointMap, powered: Sequence[str], address: str = "tcp://localhost",
                 realtime: bool = True):
        super().__init__(jm, powered)
        from ..interfaces.urlab_interface import URLabInterface
        self.itf = URLabInterface(jm, address=address, step_mode="direct", policy_dt=1.0 / self.rate_hz)
        self.itf.connect()
        self.realtime = realtime
        self.kp = jm.kp.copy()
        self.kd = jm.kd.copy()
        self.hold = self.itf.get_low_state().q.copy()
        for _ in range(int(self.SETTLE_S * self.rate_hz)):     # let the limp legs hang before the start pose
            self.itf.send_low_cmd(self.hold, self.kp, self.kd)
            self.itf.tick()
        self.hold = self.itf.get_low_state().q.copy()
        model_kp = self.itf.model_kp()
        if model_kp is not None:
            self.unchecked = [n for i, n in enumerate(jm.names) if model_kp[i] <= 0.0 and n not in self.powered]
        print(f"[urlab] connected; powered: {self.powered}; every other joint held at its start pose")
        if self.unchecked:
            print(f"[urlab] twin has no stiffness on {len(self.unchecked)} joints ({', '.join(self.unchecked)}): "
                  f"they hang freely here and are not part of the other-joint check on the twin")

    def read(self):
        s = self.itf.get_low_state()
        return s.q, s.dq, 0.0

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
        print(f"[real] interface {iface}, powered slots {self.pslots} ({', '.join(self.powered)}), "
              f"{'PUBLISHING' if publish else 'READ-ONLY, nothing is published'}")

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

    def _write(self, mode, q, kp, kd):
        cmd = self._new_cmd()            # fresh message: every motor_cmd is mode 0 / kp 0 / kd 0
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
        for _ in range(250):                 # 0.5 s pure damping on the powered joints, then disable
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
