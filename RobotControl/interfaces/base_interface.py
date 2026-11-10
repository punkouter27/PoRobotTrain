"""Hardware abstraction shared by the MuJoCo twin, the URLab (UE5) bridge and the
real Unitree SDK2 robot.

All vectors are in *policy order* (see ``configs/r1_joint_map.yaml``), length
``joint_map.num_joints`` (26 for the R1 A5).
"""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence

import numpy as np

from ..configs.joint_map import JointMap


# ----------------------------------------------------------------------------
# quaternion helpers (wxyz convention everywhere: MuJoCo, Unitree IMU, URLab root_quat_w)
def quat_to_rpy(q_wxyz: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) -> roll, pitch, yaw [rad] (ZYX convention)."""
    w, x, y, z = (float(v) for v in q_wxyz)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return np.array([roll, pitch, yaw])


def quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate world vector ``v`` into the body frame described by ``q`` (w, x, y, z)."""
    q = np.asarray(q_wxyz, dtype=np.float64)
    w = q[0]
    xyz = q[1:4]
    v = np.asarray(v, dtype=np.float64)
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(xyz, v) * (2.0 * w)
    c = xyz * (2.0 * float(np.dot(xyz, v)))
    return a - b + c


def projected_gravity_from_quat(q_wxyz: np.ndarray) -> np.ndarray:
    """Gravity direction (0,0,-1 in world) expressed in the IMU/body frame."""
    return quat_rotate_inverse(q_wxyz, np.array([0.0, 0.0, -1.0]))


# ----------------------------------------------------------------------------
@dataclass
class LowStateData:
    """Snapshot of the robot state in policy order."""

    q: np.ndarray                      # joint positions [rad], (N,)
    dq: np.ndarray                     # joint velocities [rad/s], (N,)
    tau: np.ndarray                    # estimated joint torques [Nm], (N,)
    imu_quat: np.ndarray               # (w, x, y, z)
    gyro: np.ndarray                   # body angular velocity [rad/s], (3,)
    accel: np.ndarray                  # body linear acceleration [m/s^2], (3,)
    timestamp: float                   # seconds (sim time or monotonic wall clock)
    tick: int = 0                      # message / step counter
    extras: Dict[str, np.ndarray] = field(default_factory=dict)

    @classmethod
    def zeros(cls, n: int, timestamp: float = 0.0) -> "LowStateData":
        return cls(
            q=np.zeros(n), dq=np.zeros(n), tau=np.zeros(n),
            imu_quat=np.array([1.0, 0.0, 0.0, 0.0]), gyro=np.zeros(3), accel=np.zeros(3),
            timestamp=timestamp,
        )

    @property
    def num_joints(self) -> int:
        return int(self.q.shape[0])

    def rpy(self) -> np.ndarray:
        return quat_to_rpy(self.imu_quat)

    def projected_gravity(self) -> np.ndarray:
        return projected_gravity_from_quat(self.imu_quat)

    def copy(self) -> "LowStateData":
        return LowStateData(
            q=self.q.copy(), dq=self.dq.copy(), tau=self.tau.copy(),
            imu_quat=self.imu_quat.copy(), gyro=self.gyro.copy(), accel=self.accel.copy(),
            timestamp=self.timestamp, tick=self.tick,
            extras={k: v.copy() for k, v in self.extras.items()},
        )


# ----------------------------------------------------------------------------
@dataclass
class MotorCommand:
    """A full PD command in policy order plus the policy-order -> motor-slot mapping."""

    q: np.ndarray
    dq: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau: np.ndarray

    @classmethod
    def from_targets(cls, target_q: np.ndarray, kp: np.ndarray, kd: np.ndarray,
                     tau_ff: Optional[np.ndarray] = None, dq: Optional[np.ndarray] = None) -> "MotorCommand":
        target_q = np.asarray(target_q, dtype=np.float64)
        n = target_q.shape[0]
        kp = np.broadcast_to(np.asarray(kp, dtype=np.float64), (n,)).copy()
        kd = np.broadcast_to(np.asarray(kd, dtype=np.float64), (n,)).copy()
        tau = np.zeros(n) if tau_ff is None else np.asarray(tau_ff, dtype=np.float64).copy()
        dqv = np.zeros(n) if dq is None else np.asarray(dq, dtype=np.float64).copy()
        for name, arr in (("kp", kp), ("kd", kd), ("tau_ff", tau), ("dq", dqv)):
            if arr.shape != (n,):
                raise ValueError(f"{name} has shape {arr.shape}, expected ({n},)")
        return cls(q=target_q.copy(), dq=dqv, kp=kp, kd=kd, tau=tau)

    @classmethod
    def damping(cls, current_q: np.ndarray, kd: np.ndarray) -> "MotorCommand":
        n = len(current_q)
        return cls.from_targets(current_q, np.zeros(n), kd)

    @classmethod
    def zero_torque(cls, n: int) -> "MotorCommand":
        return cls.from_targets(np.zeros(n), np.zeros(n), np.zeros(n))

    @property
    def num_joints(self) -> int:
        return int(self.q.shape[0])

    def to_slot_arrays(self, joint_map: JointMap) -> Dict[str, np.ndarray]:
        """Scatter into ``lowcmd_num_slots``-long arrays. ``mode`` is 1 for mapped
        slots and 0 for unmapped ones (14, 20, 21, 27, 28 on the A5)."""
        if self.num_joints != joint_map.num_joints:
            raise ValueError(f"command has {self.num_joints} joints, joint map has {joint_map.num_joints}")
        mode = np.zeros(joint_map.lowcmd_num_slots, dtype=np.int64)
        mode[joint_map.motor_slots] = 1
        return {
            "mode": mode,
            "q": joint_map.to_slot_array(self.q),
            "dq": joint_map.to_slot_array(self.dq),
            "kp": joint_map.to_slot_array(self.kp),
            "kd": joint_map.to_slot_array(self.kd),
            "tau": joint_map.to_slot_array(self.tau),
        }

    def fill_low_cmd(self, low_cmd, joint_map: JointMap, mode_machine: int = 0, mode_pr: int = 0) -> None:
        """Write this command into a unitree_hg ``LowCmd_``-like object (anything with
        ``.mode_pr``, ``.mode_machine`` and ``.motor_cmd[slot].{mode,q,dq,tau,kp,kd}``).
        Only the 26 mapped slots are touched."""
        low_cmd.mode_pr = int(mode_pr)
        low_cmd.mode_machine = int(mode_machine)
        for i, slot in enumerate(joint_map.motor_slots.tolist()):
            mc = low_cmd.motor_cmd[slot]
            mc.mode = 1
            mc.q = float(self.q[i])
            mc.dq = float(self.dq[i])
            mc.tau = float(self.tau[i])
            mc.kp = float(self.kp[i])
            mc.kd = float(self.kd[i])


# ----------------------------------------------------------------------------
class RobotHardwareInterface(ABC):
    """Common interface. Sub-classes implement ``connect``, ``get_low_state``,
    ``_send`` and ``close``; ``send_low_cmd`` handles validation + passive joints."""

    def __init__(self, joint_map: Optional[JointMap] = None):
        self.joint_map: JointMap = joint_map if joint_map is not None else JointMap.load()
        self.num_joints: int = self.joint_map.num_joints
        self._passive_mask = np.zeros(self.num_joints, dtype=bool)
        self._passive_kd = np.zeros(self.num_joints)
        self.last_command: Optional[MotorCommand] = None
        self.connected: bool = False

    # ---- lifecycle -------------------------------------------------------
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    def __enter__(self) -> "RobotHardwareInterface":
        if not self.connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ---- state -----------------------------------------------------------
    @abstractmethod
    def get_low_state(self) -> LowStateData: ...

    # ---- command ---------------------------------------------------------
    @abstractmethod
    def _send(self, cmd: MotorCommand) -> None:
        """Transport-specific write of a validated command."""

    def send_low_cmd(self, target_q: np.ndarray, kp: np.ndarray, kd: np.ndarray,
                     tau_ff: Optional[np.ndarray] = None, dq: Optional[np.ndarray] = None) -> MotorCommand:
        """Send a PD command (policy order, length N). Joints marked passive via
        :meth:`set_passive` are forced to kp=0 / kd=passive_kd."""
        cmd = MotorCommand.from_targets(target_q, kp, kd, tau_ff, dq)
        if cmd.num_joints != self.num_joints:
            raise ValueError(f"target_q has {cmd.num_joints} joints, expected {self.num_joints}")
        if np.any(self._passive_mask):
            cmd.kp[self._passive_mask] = 0.0
            cmd.kd[self._passive_mask] = self._passive_kd[self._passive_mask]
            cmd.tau[self._passive_mask] = 0.0
        self.last_command = cmd
        self._send(cmd)
        return cmd

    def set_passive(self, joint_indices: Sequence[int] | np.ndarray, kd: float | np.ndarray | None = None) -> None:
        """Mark joints as damping-only (kp forced to 0 on every subsequent command).
        ``kd`` defaults to the joint map kd. Pass an empty list to clear."""
        self._passive_mask[:] = False
        idx = np.asarray(list(joint_indices), dtype=np.int64)
        if idx.size == 0:
            return
        self._passive_mask[idx] = True
        kd_arr = self.joint_map.kd[idx] if kd is None else np.broadcast_to(np.asarray(kd, dtype=np.float64), idx.shape)
        self._passive_kd[idx] = kd_arr

    @property
    def passive_mask(self) -> np.ndarray:
        return self._passive_mask.copy()

    def send_damping(self, kd: Optional[np.ndarray] = None) -> MotorCommand:
        """Damping-only command for all joints (kp=0, target=current q)."""
        state = self.get_low_state()
        kd_arr = self.joint_map.kd if kd is None else np.asarray(kd, dtype=np.float64)
        return self.send_low_cmd(state.q, np.zeros(self.num_joints), kd_arr)

    def send_zero_torque(self) -> MotorCommand:
        return self.send_low_cmd(np.zeros(self.num_joints), np.zeros(self.num_joints), np.zeros(self.num_joints))

    # ---- timing ----------------------------------------------------------
    def tick(self) -> None:
        """Advance simulated time by one control period. No-op on real hardware."""
        return None

    @property
    def is_simulation(self) -> bool:
        return False

    def now(self) -> float:
        return time.monotonic()
