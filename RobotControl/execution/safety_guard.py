"""Pure-numpy safety layer between a policy and the motors.

Checks, in order:

1. command staleness watchdog  (no new target within ``stale_after_s`` -> hold last)
2. per-joint soft-limit clip    (``soft_limit_fraction`` of the range about its centre)
3. slew-rate limiter            (max rad/s per joint; default 6 arms/head, 10 legs/waist)
4. torque estimate clip         (|kp * (q* - q)| <= effort  -> move q* towards q)
5. joint acceleration estimate  (> ``max_accel`` rad/s^2 -> flag; trip optional)
6. base tilt from the IMU       (roll or pitch > ``max_tilt_deg`` -> trip)

On a trip (latched until :meth:`reset`) the guard returns a damping-only command:
``target = current q``, ``kp = 0``, ``kd = trip_kd``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from ..configs.joint_map import JointMap, SOFT_LIMIT_FRACTION
from ..interfaces.base_interface import LowStateData, quat_to_rpy


@dataclass
class SafetyConfig:
    soft_limit_fraction: float = SOFT_LIMIT_FRACTION
    max_slew_arms_head: float = 6.0     # rad/s
    max_slew_legs_waist: float = 10.0   # rad/s
    max_accel: float = 80.0             # rad/s^2
    trip_on_accel: bool = False
    max_tilt_deg: float = 25.0
    check_tilt: bool = True
    torque_clip: bool = True
    stale_after_s: float = 0.1
    trip_on_stale: bool = False
    latch_trips: bool = True
    max_slew: Optional[np.ndarray] = None   # explicit per-joint override
    trip_kd: Optional[np.ndarray] = None    # damping used after a trip (default: joint map kd)


@dataclass
class SafetyReport:
    tripped: bool = False
    trip_reason: Optional[str] = None
    stale: bool = False
    limit_clipped: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    slew_limited: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    torque_clipped: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    accel_flag: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    max_accel: float = 0.0
    kp: np.ndarray = field(default_factory=lambda: np.zeros(0))
    kd: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dt: float = 0.0

    @property
    def any_flag(self) -> bool:
        return bool(self.tripped or self.stale or self.limit_clipped.any() or self.slew_limited.any()
                    or self.torque_clipped.any() or self.accel_flag.any())

    def summary(self) -> str:
        if self.tripped:
            return f"TRIP:{self.trip_reason}"
        parts = []
        if self.stale:
            parts.append("stale")
        if self.limit_clipped.any():
            parts.append(f"lim{np.flatnonzero(self.limit_clipped).tolist()}")
        if self.slew_limited.any():
            parts.append(f"slew{np.flatnonzero(self.slew_limited).tolist()}")
        if self.torque_clipped.any():
            parts.append(f"tau{np.flatnonzero(self.torque_clipped).tolist()}")
        if self.accel_flag.any():
            parts.append(f"acc{np.flatnonzero(self.accel_flag).tolist()}")
        return " ".join(parts) if parts else "ok"


class SafetyGuard:
    def __init__(self, joint_map: JointMap, config: Optional[SafetyConfig] = None):
        self.jm = joint_map
        self.cfg = config or SafetyConfig()
        n = joint_map.num_joints
        self.n = n
        self.soft_lo, self.soft_hi = joint_map.soft_limits(self.cfg.soft_limit_fraction)
        if self.cfg.max_slew is not None:
            self.max_slew = np.broadcast_to(np.asarray(self.cfg.max_slew, dtype=np.float64), (n,)).copy()
        else:
            self.max_slew = np.full(n, self.cfg.max_slew_arms_head)
            for g in ("left_leg", "right_leg", "waist"):
                if g in joint_map.groups:
                    self.max_slew[joint_map.groups[g]] = self.cfg.max_slew_legs_waist
        self.trip_kd = (np.asarray(self.cfg.trip_kd, dtype=np.float64) if self.cfg.trip_kd is not None
                        else joint_map.kd.copy())
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.tripped = False
        self.trip_reason: Optional[str] = None
        self._last_target: Optional[np.ndarray] = None
        self._last_fresh_t: Optional[float] = None
        self._last_t: Optional[float] = None
        self._last_dq: Optional[np.ndarray] = None

    def trip(self, reason: str) -> None:
        self.tripped = True
        self.trip_reason = reason

    # ------------------------------------------------------------------ apply
    def apply(self, target_q: Optional[np.ndarray], state: LowStateData,
              kp: Optional[np.ndarray] = None, kd: Optional[np.ndarray] = None,
              t: Optional[float] = None) -> Tuple[np.ndarray, SafetyReport]:
        """Filter ``target_q`` (policy order; ``None`` = no new target this tick).
        Returns ``(safe_target_q, report)``; ``report.kp`` / ``report.kd`` are the gains to
        send (zeros / trip_kd when tripped)."""
        n = self.n
        t = float(state.timestamp if t is None else t)
        kp_in = self.jm.kp if kp is None else np.asarray(kp, dtype=np.float64)
        kd_in = self.jm.kd if kd is None else np.asarray(kd, dtype=np.float64)
        rep = SafetyReport(
            limit_clipped=np.zeros(n, dtype=bool), slew_limited=np.zeros(n, dtype=bool),
            torque_clipped=np.zeros(n, dtype=bool), accel_flag=np.zeros(n, dtype=bool),
        )
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        rep.dt = dt
        q = np.asarray(state.q, dtype=np.float64)

        # 1. staleness watchdog
        if target_q is None:
            if self._last_target is None:
                target = q.copy()
            else:
                target = self._last_target.copy()
            if self._last_fresh_t is not None and (t - self._last_fresh_t) > self.cfg.stale_after_s:
                rep.stale = True
                if self.cfg.trip_on_stale and not self.tripped:
                    self.trip("stale_command")
        else:
            target = np.asarray(target_q, dtype=np.float64).copy()
            if target.shape != (n,):
                raise ValueError(f"target_q shape {target.shape} != ({n},)")
            if not np.all(np.isfinite(target)):
                self.trip("non_finite_target")
                target = q.copy()
            self._last_fresh_t = t

        # 5. acceleration estimate (from measured velocities)
        if self._last_dq is not None and dt > 1e-6:
            acc = (np.asarray(state.dq) - self._last_dq) / dt
            rep.accel_flag = np.abs(acc) > self.cfg.max_accel
            rep.max_accel = float(np.max(np.abs(acc)))
            if self.cfg.trip_on_accel and rep.accel_flag.any() and not self.tripped:
                self.trip(f"accel>{self.cfg.max_accel}")
        self._last_dq = np.asarray(state.dq, dtype=np.float64).copy()

        # 6. tilt
        if self.cfg.check_tilt:
            rpy = quat_to_rpy(state.imu_quat)
            rep.roll_deg = math.degrees(rpy[0])
            rep.pitch_deg = math.degrees(rpy[1])
            if (abs(rep.roll_deg) > self.cfg.max_tilt_deg or abs(rep.pitch_deg) > self.cfg.max_tilt_deg) and not self.tripped:
                self.trip(f"tilt roll={rep.roll_deg:.1f} pitch={rep.pitch_deg:.1f} deg")

        if self.tripped:
            rep.tripped = True
            rep.trip_reason = self.trip_reason
            rep.kp = np.zeros(n)
            rep.kd = self.trip_kd.copy()
            self._last_t = t
            self._last_target = q.copy()
            safe = q.copy()
            if not self.cfg.latch_trips:
                self.tripped = False
            return safe, rep

        # 2. soft limits
        clipped = np.clip(target, self.soft_lo, self.soft_hi)
        rep.limit_clipped = np.abs(clipped - target) > 1e-12
        target = clipped

        # 3. slew-rate limit relative to the previous (safe) target
        ref = self._last_target if self._last_target is not None else q
        if dt > 0.0:
            max_step = self.max_slew * dt
            delta = target - ref
            limited = np.clip(delta, -max_step, max_step)
            rep.slew_limited = np.abs(limited - delta) > 1e-12
            target = ref + limited

        # 4. torque clip: |kp (q* - q)| <= effort
        if self.cfg.torque_clip:
            with np.errstate(divide="ignore", invalid="ignore"):
                max_err = np.where(kp_in > 0, self.jm.effort / np.where(kp_in > 0, kp_in, 1.0), np.inf)
            err = target - q
            err_c = np.clip(err, -max_err, max_err)
            rep.torque_clipped = np.abs(err_c - err) > 1e-12
            target = q + err_c
            target = np.clip(target, self.soft_lo, self.soft_hi)

        # flags are only meaningful for driven joints (kp > 0); passive targets are ignored by the motors
        active = kp_in > 0
        rep.limit_clipped &= active
        rep.slew_limited &= active
        rep.torque_clipped &= active

        rep.kp = kp_in.copy()
        rep.kd = kd_in.copy()
        self._last_target = target.copy()
        self._last_t = t
        return target, rep

    # ---------------------------------------------------------------- helpers
    def damping_command(self, state: LowStateData) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return state.q.copy(), np.zeros(self.n), self.trip_kd.copy()
