"""Real-robot interface over Unitree SDK2 (DDS), modelled on
``external/unitree_sdk2_python/example/r1/low_level/r1_low_level_example.py`` and
``r1_A5_low_level_example.py``.

Sequence (same as the examples)::

    ChannelFactoryInitialize(0, network_interface)
    MotionSwitcherClient: CheckMode / ReleaseMode until no mode is active
    ChannelPublisher("rt/lowcmd", LowCmd_)      ChannelSubscriber("rt/lowstate", LowState_)
    low_cmd.mode_pr = 0, low_cmd.mode_machine = low_state.mode_machine
    motor_cmd[slot].mode = 1, q/dq/kp/kd/tau   (only the 26 mapped slots; 14,20,21,27,28 stay mode 0)
    low_cmd.crc = CRC().Crc(low_cmd); publisher.Write(low_cmd)

``dry_run=True`` logs commands instead of publishing (and works without the SDK
installed, using an in-memory LowCmd stand-in; the fake state tracks the commanded
targets so the control loop can be exercised offline).
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

from ..configs.joint_map import JointMap
from .base_interface import LowStateData, MotorCommand, RobotHardwareInterface

log = logging.getLogger(__name__)

SDK_INSTALL_HINT = (
    "unitree_sdk2py is not installed. It needs cyclonedds; follow "
    "external/unitree_sdk2_python/README.md (set CYCLONEDDS_HOME, then "
    "`pip install -e external/unitree_sdk2_python`). Use dry_run=True to run without it."
)

_channel_factory_initialized = False
_channel_factory_lock = threading.Lock()


def detect_robot_iface(subnet_prefix: str = "192.168.123.") -> Optional[str]:
    """Return the Linux network interface that carries a 192.168.123.x address (the robot's internal LAN).

    On the R1's Jetson this is the NIC wired to the motion controller (192.168.123.161). Returns None if
    no such interface exists or the platform has no ``ip`` tool (e.g. Windows).
    """
    import subprocess
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == "inet" and parts[3].startswith(subnet_prefix):
            return parts[1]
    return None


def ensure_channel_factory(domain_id: int = 0, network_interface: Optional[str] = None) -> None:
    """``ChannelFactoryInitialize`` may only run once per process; share it between the
    low-level and high-level (LocoClient) wrappers."""
    global _channel_factory_initialized
    with _channel_factory_lock:
        if _channel_factory_initialized:
            return
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        except ImportError as exc:
            raise ImportError(SDK_INSTALL_HINT) from exc
        if network_interface:
            ChannelFactoryInitialize(domain_id, network_interface)
        else:
            ChannelFactoryInitialize(domain_id)
        _channel_factory_initialized = True


class _FakeMotorCmd:
    __slots__ = ("mode", "q", "dq", "tau", "kp", "kd", "reserve")

    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.reserve = 0


class FakeLowCmd:
    """Stand-in for ``unitree_hg.msg.dds_.LowCmd_`` used in dry-run mode."""

    def __init__(self, num_slots: int = 35):
        self.mode_pr = 0
        self.mode_machine = 0
        self.motor_cmd = [_FakeMotorCmd() for _ in range(num_slots)]
        self.crc = 0


class UnitreeSDK2Interface(RobotHardwareInterface):
    MODE_PR = 0
    MODE_AB = 1
    LOWCMD_SLOTS = 35  # size of the unitree_hg motor_cmd array

    def __init__(self, joint_map: Optional[JointMap] = None, network_interface: Optional[str] = None,
                 dry_run: bool = False, domain_id: int = 0, release_motion_mode: bool = True,
                 state_timeout_s: float = 5.0, log_every: int = 50, check_crc: bool = True,
                 observe_only: bool = False):
        super().__init__(joint_map)
        self.network_interface = network_interface
        self.dry_run = bool(dry_run)
        # observe_only: subscribe to rt/lowstate ONLY. No command publisher is created, the robot's
        # motion mode is never released, and commands are computed and logged but never sent.
        self.observe_only = bool(observe_only)
        self.last_shadow_cmd = None
        self.domain_id = int(domain_id)
        self.release_motion_mode = release_motion_mode
        self.state_timeout_s = float(state_timeout_s)
        self.log_every = int(log_every)
        self.check_crc = check_crc
        self.mode_machine = 0
        self._state_lock = threading.Lock()
        self._low_state = None
        self._state_time = 0.0
        self._state_count = 0
        self._crc_errors = 0
        self._cmd_count = 0
        self.low_cmd = None
        self._publisher = None
        self._subscriber = None
        self._crc = None
        self._fake_q = self.joint_map.defaults.copy()
        self._t0 = time.monotonic()

    # ------------------------------------------------------------- lifecycle
    def connect(self) -> None:
        if self.dry_run:
            self.low_cmd = FakeLowCmd(self.LOWCMD_SLOTS)
            self.connected = True
            log.info("UnitreeSDK2Interface DRY RUN: commands are logged, nothing is published")
            return

        try:
            from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.utils.crc import CRC
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        except ImportError as exc:
            raise ImportError(SDK_INSTALL_HINT) from exc

        ensure_channel_factory(self.domain_id, self.network_interface)
        self._crc = CRC()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

        if self.observe_only:
            self.low_cmd = FakeLowCmd(self.LOWCMD_SLOTS)
            self._subscriber = ChannelSubscriber(self.joint_map.lowstate_topic, LowState_)
            self._subscriber.Init(self._on_low_state, 10)
            t_end = time.monotonic() + self.state_timeout_s
            while self._low_state is None:
                if time.monotonic() > t_end:
                    raise TimeoutError(f"no {self.joint_map.lowstate_topic} message within {self.state_timeout_s}s "
                                       f"on interface {self.network_interface!r}")
                time.sleep(0.01)
            self.connected = True
            log.info("UnitreeSDK2Interface OBSERVE ONLY: reading %s, nothing is published, motion mode untouched",
                     self.joint_map.lowstate_topic)
            return

        if self.release_motion_mode:
            msc = MotionSwitcherClient()
            msc.SetTimeout(5.0)
            msc.Init()
            while True:
                status, result = msc.CheckMode()
                if status != 0:
                    raise RuntimeError(f"MotionSwitcherClient.CheckMode failed: {status}")
                if not result.get("name"):
                    break
                log.info("releasing active motion mode %r", result.get("name"))
                status, _ = msc.ReleaseMode()
                if status != 0:
                    raise RuntimeError(f"MotionSwitcherClient.ReleaseMode failed: {status}")
                time.sleep(1.0)

        self._publisher = ChannelPublisher(self.joint_map.lowcmd_topic, LowCmd_)
        self._publisher.Init()
        self._subscriber = ChannelSubscriber(self.joint_map.lowstate_topic, LowState_)
        self._subscriber.Init(self._on_low_state, 10)

        t_end = time.monotonic() + self.state_timeout_s
        while self._low_state is None:
            if time.monotonic() > t_end:
                raise TimeoutError(f"no {self.joint_map.lowstate_topic} message within {self.state_timeout_s}s "
                                   f"on interface {self.network_interface!r}")
            time.sleep(0.01)
        self.connected = True
        log.info("UnitreeSDK2Interface connected (mode_machine=%d)", self.mode_machine)

    def close(self) -> None:
        if self.connected and not self.dry_run and not self.observe_only:
            try:
                self.damping()
            except Exception:  # pragma: no cover
                pass
        if self._subscriber is not None:
            try:
                self._subscriber.Close()
            except Exception:  # pragma: no cover
                pass
        if self._publisher is not None:
            try:
                self._publisher.Close()
            except Exception:  # pragma: no cover
                pass
        self._subscriber = None
        self._publisher = None
        self.connected = False

    # -------------------------------------------------------------- callbacks
    def _on_low_state(self, msg) -> None:
        if self.check_crc and self._crc is not None and msg.crc != self._crc.Crc(msg):
            self._crc_errors += 1
            if self._crc_errors % 100 == 1:
                log.error("LowState CRC error (#%d)", self._crc_errors)
            return
        with self._state_lock:
            self._low_state = msg
            self._state_time = time.monotonic()
            self._state_count += 1
            self.mode_machine = int(msg.mode_machine)

    # ----------------------------------------------------------------- state
    def get_low_state(self) -> LowStateData:
        if self.dry_run:
            n = self.num_joints
            return LowStateData(
                q=self._fake_q.copy(), dq=np.zeros(n), tau=np.zeros(n),
                imu_quat=np.array([1.0, 0.0, 0.0, 0.0]), gyro=np.zeros(3), accel=np.zeros(3),
                timestamp=time.monotonic() - self._t0, tick=self._cmd_count,
            )
        with self._state_lock:
            msg = self._low_state
            ts = self._state_time
            count = self._state_count
        if msg is None:
            raise RuntimeError("no LowState received yet")
        slots = self.joint_map.motor_slots.tolist()
        q = np.array([msg.motor_state[s].q for s in slots], dtype=np.float64)
        dq = np.array([msg.motor_state[s].dq for s in slots], dtype=np.float64)
        tau = np.array([msg.motor_state[s].tau_est for s in slots], dtype=np.float64)
        imu = msg.imu_state
        return LowStateData(
            q=q, dq=dq, tau=tau,
            imu_quat=np.array(imu.quaternion, dtype=np.float64),      # (w, x, y, z)
            gyro=np.array(imu.gyroscope, dtype=np.float64),
            accel=np.array(imu.accelerometer, dtype=np.float64),
            timestamp=ts, tick=count,
            extras={"rpy": np.array(imu.rpy, dtype=np.float64)},
        )

    @property
    def state_age_s(self) -> float:
        return 0.0 if self.dry_run else time.monotonic() - self._state_time

    # --------------------------------------------------------------- command
    def _send(self, cmd: MotorCommand) -> None:
        cmd.fill_low_cmd(self.low_cmd, self.joint_map, mode_machine=self.mode_machine, mode_pr=self.MODE_PR)
        self._cmd_count += 1
        if self.observe_only:
            self.last_shadow_cmd = cmd
            if self.log_every and self._cmd_count % self.log_every == 1:
                log.info("[observe] would send #%d q=%s", self._cmd_count, np.round(cmd.q, 3).tolist())
            return
        if self.dry_run:
            self._fake_q = np.where(cmd.kp > 0, cmd.q, self._fake_q)
            if self.log_every and self._cmd_count % self.log_every == 1:
                log.info("[dry-run] cmd #%d q=%s kp=%s kd=%s", self._cmd_count,
                         np.round(cmd.q, 3).tolist(), np.round(cmd.kp, 1).tolist(), np.round(cmd.kd, 2).tolist())
            return
        self.low_cmd.crc = self._crc.Crc(self.low_cmd)
        self._publisher.Write(self.low_cmd)

    # ---------------------------------------------------------------- helpers
    def zero_torque(self) -> MotorCommand:
        """mode=1 with kp=kd=tau=0 on all mapped joints (motors enabled, no torque)."""
        return self.send_zero_torque()

    def damping(self, kd: Optional[float | np.ndarray] = None) -> MotorCommand:
        """kp=0, kd>0 on all mapped joints."""
        kd_arr = self.joint_map.kd if kd is None else np.broadcast_to(np.asarray(kd, dtype=np.float64), (self.num_joints,))
        state = self.get_low_state()
        return self.send_low_cmd(state.q, np.zeros(self.num_joints), kd_arr)

    def move_to(self, target_q: np.ndarray, duration_s: float = 3.0, kp: Optional[np.ndarray] = None,
                kd: Optional[np.ndarray] = None, rate_hz: float = 250.0, stop_event: Optional[threading.Event] = None) -> None:
        """Linearly interpolate from the current pose to ``target_q`` over ``duration_s``
        (the Stage-1 pattern of the Unitree examples)."""
        target_q = np.asarray(target_q, dtype=np.float64)
        kp_arr = self.joint_map.kp if kp is None else np.asarray(kp, dtype=np.float64)
        kd_arr = self.joint_map.kd if kd is None else np.asarray(kd, dtype=np.float64)
        q0 = self.get_low_state().q.copy()
        dt = 1.0 / float(rate_hz)
        steps = max(1, int(round(duration_s / dt)))
        t_next = time.monotonic()
        for k in range(1, steps + 1):
            if stop_event is not None and stop_event.is_set():
                return
            ratio = min(1.0, k / steps)
            self.send_low_cmd((1.0 - ratio) * q0 + ratio * target_q, kp_arr, kd_arr)
            t_next += dt
            slack = t_next - time.monotonic()
            if slack > 0 and not self.dry_run:
                time.sleep(slack)

    def stats(self) -> dict:
        return {"states": self._state_count, "crc_errors": self._crc_errors, "cmds": self._cmd_count,
                "mode_machine": self.mode_machine, "dry_run": self.dry_run}
