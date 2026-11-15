"""Neck turn: the head turns left and right. Powers ONLY head_yaw_joint.

Motion: smooth move to straight ahead, smooth fade-in, ``A * sin(2*pi*f*t)``, smooth fade-out, release.
Defaults are gentle: A = 10 deg, f = 0.2 Hz (one left-right cycle per 5 s). The target is clipped so that
kp * |target - q| never exceeds the motor's effort limit (0.86 Nm for head yaw).

Aborts (returns None, see ``abort_reason``) on: start not within --start-tol of straight ahead (refuses to
start), tracking error > max_err for > 0.3 s, head beyond A + 15 deg, stale state, non-finite state.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

HEAD_YAW = "head_yaw_joint"


@dataclass
class SweepParams:
    amplitude_deg: float = 10.0
    freq_hz: float = 0.2
    duration_s: float = 20.0      # sweeping time, excluding centre and fades
    centre_s: float = 2.0         # move from the start angle to 0
    fade_s: float = 2.0           # amplitude fade in / out
    start_tol_deg: float = 10.0
    max_err_deg: float = 12.0
    max_err_time_s: float = 0.3
    stale_s: float = 0.1
    max_amplitude_deg: float = 30.0

    @property
    def total_s(self) -> float:
        return self.centre_s + self.duration_s + 2 * self.fade_s


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class HeadSweepController:
    """Pure logic for one joint: target trajectory + abort checks."""

    def __init__(self, params: SweepParams, kp: float, kd: float, effort: float, q_lo: float, q_hi: float,
                 check_tracking: bool = True):
        if not 0.0 < params.amplitude_deg <= params.max_amplitude_deg:
            raise ValueError(f"amplitude must be in (0, {params.max_amplitude_deg}] deg")
        self.p = params
        self.kp, self.kd, self.effort = float(kp), float(kd), float(effort)
        self.q_lo, self.q_hi = float(q_lo), float(q_hi)
        self.A = math.radians(params.amplitude_deg)
        self.check_tracking = check_tracking   # off in read-only runs, where the head cannot follow
        self.q0: Optional[float] = None
        self._err_since: Optional[float] = None
        self.abort_reason: Optional[str] = None

    def start(self, q0: float) -> None:
        if not math.isfinite(q0):
            raise RuntimeError("head yaw angle is not finite")
        if abs(q0) > math.radians(self.p.start_tol_deg):
            raise RuntimeError(
                f"head yaw is {math.degrees(q0):+.1f} deg; turn the head to face straight ahead by hand "
                f"(within +-{self.p.start_tol_deg:.0f} deg) and run again")
        self.q0 = q0

    def phase(self, t: float) -> str:
        p = self.p
        if t < p.centre_s:
            return "centre"
        if t < p.centre_s + p.fade_s:
            return "fade_in"
        if t < p.centre_s + p.fade_s + p.duration_s:
            return "sweep"
        if t < p.total_s:
            return "fade_out"
        return "done"

    def nominal_target(self, t: float) -> float:
        p = self.p
        if t < p.centre_s:
            return (1.0 - _smoothstep(t / p.centre_s)) * self.q0
        ts = t - p.centre_s
        gain = _smoothstep(ts / p.fade_s) * (1.0 - _smoothstep((ts - p.fade_s - p.duration_s) / p.fade_s))
        return gain * self.A * math.sin(2.0 * math.pi * p.freq_hz * ts)

    def command(self, t: float, q: float, dq: float, state_age_s: float) -> Optional[float]:
        """Target for this tick, or None when the run must abort (see ``abort_reason``)."""
        if not (math.isfinite(q) and math.isfinite(dq)):
            return self._abort("non-finite head state")
        if state_age_s > self.p.stale_s:
            return self._abort(f"stale state ({state_age_s * 1000:.0f} ms)")
        if self.check_tracking and abs(q) > self.A + math.radians(15.0) and t > self.p.centre_s:
            return self._abort(f"head yaw {math.degrees(q):+.1f} deg beyond the sweep range")
        target = self.nominal_target(t)
        if self.check_tracking and abs(target - q) > math.radians(self.p.max_err_deg):
            self._err_since = t if self._err_since is None else self._err_since
            if t - self._err_since > self.p.max_err_time_s:
                return self._abort(f"tracking error {math.degrees(target - q):+.1f} deg for "
                                   f">{self.p.max_err_time_s} s (head not following: wrong direction or blocked)")
        else:
            self._err_since = None
        band = self.effort / self.kp          # torque limit: kp * |target - q| <= effort
        target = min(max(target, q - band), q + band)
        return min(max(target, self.q_lo), self.q_hi)

    def _abort(self, reason: str) -> None:
        self.abort_reason = reason
        return None

    def tau_estimate(self, target: float, q: float, dq: float) -> float:
        return self.kp * (target - q) - self.kd * dq


class NeckTurn:
    """Behaviour wrapper: powers only the head yaw joint."""

    name = "neck_turn"
    powered = (HEAD_YAW,)

    def __init__(self, joint_map, params: SweepParams, check_tracking: bool = True):
        i = joint_map.index_of(HEAD_YAW)
        self.ctl = HeadSweepController(params, joint_map.kp[i], joint_map.kd[i], joint_map.effort[i],
                                       joint_map.limits_lo[i], joint_map.limits_hi[i], check_tracking)

    @property
    def duration_s(self) -> float:
        return self.ctl.p.total_s

    @property
    def abort_reason(self) -> Optional[str]:
        return self.ctl.abort_reason

    @property
    def check_tracking(self) -> bool:
        return self.ctl.check_tracking

    def start(self, q: Dict[str, float]) -> None:
        self.ctl.start(q[HEAD_YAW])

    def phase(self, t: float) -> str:
        return self.ctl.phase(t)

    def targets(self, t: float, q: Dict[str, float], dq: Dict[str, float], age: float) -> Optional[Dict[str, float]]:
        tg = self.ctl.command(t, q[HEAD_YAW], dq[HEAD_YAW], age)
        return None if tg is None else {HEAD_YAW: tg}

    def describe(self) -> str:
        p = self.ctl.p
        return (f"neck turn +-{p.amplitude_deg:.0f} deg at {p.freq_hz} Hz for {p.duration_s:.0f} s "
                f"(total {p.total_s:.0f} s)")
