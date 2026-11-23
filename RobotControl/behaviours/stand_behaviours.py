"""Stand-safe behaviours (upper body and head only; legs and waist remain unpowered).

All behaviours in this module share ``JointSweepChannel`` safety checks (finite state,
staleness, tracking error timeout, torque cap ``kp * |target - q| <= effort``, and
joint limit clamping) and smoothstep ramps so they start and end at the resting pose
without jumps.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence

from .neck_turn import HEAD_YAW, NeckTurn, SweepParams, _smoothstep

HEAD_PITCH = "head_pitch_joint"
R_SHOULDER_PITCH = "right_shoulder_pitch_joint"
R_SHOULDER_ROLL = "right_shoulder_roll_joint"
R_ELBOW = "right_elbow_joint"


@dataclass(frozen=True)
class BehaviourSpec:
    id: str
    label: str
    desc: str
    primary_joint: str
    primary_label: str
    max_deg: float
    cls: type
    kind: str = "scripted"  # "scripted" (code) | "trained" (validated ONNX policy, behaviours/head_skills.py)


class JointSafetyChannel:
    """Per-joint safety envelope: stale check, tracking-error timeout, torque cap, and limit clamp."""

    def __init__(
        self,
        kp: float,
        effort: float,
        q_lo: float,
        q_hi: float,
        params: SweepParams,
        check_tracking: bool = True,
        margin_rad: float = 0.02,
    ):
        self.kp = float(kp)
        self.effort = float(effort)
        self.q_lo = float(q_lo) + margin_rad
        self.q_hi = float(q_hi) - margin_rad
        self.p = params
        self.check_tracking = check_tracking
        self._err_since: Optional[float] = None
        self.abort_reason: Optional[str] = None

    def filter(self, name: str, t: float, target: float, q: float, dq: float, age: float) -> Optional[float]:
        if not (math.isfinite(q) and math.isfinite(dq)):
            self.abort_reason = f"non-finite {name} state"
            return None
        if age > self.p.stale_s:
            self.abort_reason = f"stale state ({age * 1000:.0f} ms)"
            return None
        if self.check_tracking and abs(target - q) > math.radians(self.p.max_err_deg):
            self._err_since = t if self._err_since is None else self._err_since
            if t - self._err_since > self.p.max_err_time_s:
                self.abort_reason = (
                    f"{name} tracking error {math.degrees(target - q):+.1f} deg for "
                    f">{self.p.max_err_time_s} s (blocked or wrong direction)"
                )
                return None
        else:
            self._err_since = None
        band = self.effort / max(self.kp, 1e-6)
        capped = min(max(target, q - band), q + band)
        return min(max(capped, self.q_lo), self.q_hi)


class _StandBehaviourBase:
    name: str = ""
    powered: Sequence[str] = ()
    primary_joint: str = ""
    primary_label: str = ""

    def __init__(self, joint_map, params: SweepParams, check_tracking: bool = True, max_amp_deg: float = 30.0):
        self.jm = joint_map
        self.eff_amp_deg = min(float(params.amplitude_deg), float(max_amp_deg))
        self.p = SweepParams(
            amplitude_deg=self.eff_amp_deg,
            freq_hz=params.freq_hz,
            duration_s=params.duration_s,
            centre_s=params.centre_s,
            fade_s=params.fade_s,
            start_tol_deg=params.start_tol_deg,
            max_err_deg=params.max_err_deg,
            max_err_time_s=params.max_err_time_s,
            stale_s=params.stale_s,
            max_amplitude_deg=max(params.max_amplitude_deg, max_amp_deg),
        )
        self.A = math.radians(self.eff_amp_deg)
        self.check_tracking = check_tracking
        self.abort_reason: Optional[str] = None
        self.q0: Dict[str, float] = {}
        self.channels: Dict[str, JointSafetyChannel] = {
            jn: JointSafetyChannel(
                joint_map.kp[joint_map.index_of(jn)],
                joint_map.effort[joint_map.index_of(jn)],
                joint_map.limits_lo[joint_map.index_of(jn)],
                joint_map.limits_hi[joint_map.index_of(jn)],
                self.p,
                check_tracking=check_tracking,
            )
            for jn in self.powered
        }

    @property
    def duration_s(self) -> float:
        return self.p.total_s

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

    def start(self, q: Dict[str, float]) -> None:
        for jn in self.powered:
            val = q[jn]
            if not math.isfinite(val):
                raise RuntimeError(f"{jn} angle is not finite")
            self.q0[jn] = float(val)

    def nominal_targets(self, t: float) -> Dict[str, float]:
        raise NotImplementedError

    def targets(self, t: float, q: Dict[str, float], dq: Dict[str, float], age: float) -> Optional[Dict[str, float]]:
        nom = self.nominal_targets(t)
        out: Dict[str, float] = {}
        for jn in self.powered:
            ch = self.channels[jn]
            val = ch.filter(jn, t, nom[jn], q[jn], dq[jn], age)
            if val is None:
                self.abort_reason = ch.abort_reason
                return None
            out[jn] = val
        return out


class NeckNod(_StandBehaviourBase):
    """Gentle head pitch nod (+-amplitude, capped at 20 deg inside +-36 deg limits)."""

    name = "neck_nod"
    powered = (HEAD_PITCH,)
    primary_joint = HEAD_PITCH
    primary_label = "HEAD PITCH"

    def __init__(self, joint_map, params: SweepParams, check_tracking: bool = True):
        super().__init__(joint_map, params, check_tracking=check_tracking, max_amp_deg=20.0)

    def start(self, q: Dict[str, float]) -> None:
        super().start(q)
        if abs(self.q0[HEAD_PITCH]) > math.radians(self.p.start_tol_deg):
            raise RuntimeError(
                f"head pitch is {math.degrees(self.q0[HEAD_PITCH]):+.1f} deg; level the head "
                f"(within +-{self.p.start_tol_deg:.0f} deg) and run again"
            )

    def nominal_targets(self, t: float) -> Dict[str, float]:
        p = self.p
        if t < p.centre_s:
            return {HEAD_PITCH: (1.0 - _smoothstep(t / p.centre_s)) * self.q0[HEAD_PITCH]}
        ts = t - p.centre_s
        gain = _smoothstep(ts / p.fade_s) * (1.0 - _smoothstep((ts - p.fade_s - p.duration_s) / p.fade_s))
        return {HEAD_PITCH: gain * self.A * math.sin(2.0 * math.pi * p.freq_hz * ts)}

    def describe(self) -> str:
        return (
            f"neck nod +-{self.eff_amp_deg:.0f} deg at {self.p.freq_hz} Hz for {self.p.duration_s:.0f} s "
            f"(total {self.p.total_s:.0f} s)"
        )


class ArmRaise(_StandBehaviourBase):
    """Gentle stand-safe right shoulder forward raise from the starting pose and back.

    Holds right_elbow_joint gently at its starting angle so gravity on the forearm does not
    swing the elbow when the shoulder pitches forward.
    """

    name = "arm_raise"
    powered = (R_SHOULDER_PITCH, R_ELBOW)
    primary_joint = R_SHOULDER_PITCH
    primary_label = "R.SHOULDER"

    def __init__(self, joint_map, params: SweepParams, check_tracking: bool = True):
        super().__init__(joint_map, params, check_tracking=check_tracking, max_amp_deg=30.0)

    def nominal_targets(self, t: float) -> Dict[str, float]:
        p = self.p
        q_rest = self.q0[R_SHOULDER_PITCH]
        el_hold = self.q0[R_ELBOW]
        if t < p.centre_s:
            return {R_SHOULDER_PITCH: q_rest, R_ELBOW: el_hold}
        ts = t - p.centre_s
        gain = _smoothstep(ts / p.fade_s) * (1.0 - _smoothstep((ts - p.fade_s - p.duration_s) / p.fade_s))
        # Negative shoulder pitch raises the right arm forward/up cleanly away from the stand.
        lift = 0.5 * (1.0 - math.cos(2.0 * math.pi * p.freq_hz * ts))
        return {R_SHOULDER_PITCH: q_rest - gain * self.A * lift, R_ELBOW: el_hold}

    def describe(self) -> str:
        return (
            f"right arm raise {self.eff_amp_deg:.0f} deg at {self.p.freq_hz} Hz for {self.p.duration_s:.0f} s "
            f"(total {self.p.total_s:.0f} s)"
        )


class HandWave(_StandBehaviourBase):
    """Stand-safe upper-body wave: lifts right arm slightly forward/out and waves gently."""

    name = "hand_wave"
    powered = (R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_ELBOW)
    primary_joint = R_SHOULDER_ROLL
    primary_label = "R.ARM WAVE"

    def __init__(self, joint_map, params: SweepParams, check_tracking: bool = True):
        super().__init__(joint_map, params, check_tracking=check_tracking, max_amp_deg=20.0)

    def nominal_targets(self, t: float) -> Dict[str, float]:
        p = self.p
        q_sp0 = self.q0[R_SHOULDER_PITCH]
        q_sr0 = self.q0[R_SHOULDER_ROLL]
        q_el0 = self.q0[R_ELBOW]
        # Ready pose: small forward shoulder lift, slight outward roll, bent elbow; smoothly returns to q0.
        sp_ready = max(q_sp0 - 0.35, -0.45)
        sr_ready = max(q_sr0 - 0.35, -0.75)
        el_ready = min(max(q_el0, 0.35), 1.10)

        if t < p.centre_s:
            u = _smoothstep(t / p.centre_s)
        elif t >= p.total_s - p.fade_s:
            u = 1.0 - _smoothstep((t - (p.total_s - p.fade_s)) / p.fade_s)
        else:
            u = 1.0

        sp = (1.0 - u) * q_sp0 + u * sp_ready
        sr_base = (1.0 - u) * q_sr0 + u * sr_ready
        el = (1.0 - u) * q_el0 + u * el_ready

        ts = max(0.0, t - p.centre_s)
        wave_gain = _smoothstep(ts / p.fade_s) * (1.0 - _smoothstep((ts - p.fade_s - p.duration_s) / p.fade_s))
        sr = sr_base - wave_gain * self.A * math.sin(2.0 * math.pi * p.freq_hz * ts)
        return {R_SHOULDER_PITCH: sp, R_SHOULDER_ROLL: sr, R_ELBOW: el}

    def describe(self) -> str:
        return (
            f"hand wave +-{self.eff_amp_deg:.0f} deg at {self.p.freq_hz} Hz for {self.p.duration_s:.0f} s "
            f"(total {self.p.total_s:.0f} s)"
        )


BEHAVIOUR_SPECS: Dict[str, BehaviourSpec] = {
    "neck_turn": BehaviourSpec(
        id="neck_turn",
        label="Neck Turn",
        desc="Turns head left/right. Only neck yaw motor moves.",
        primary_joint=HEAD_YAW,
        primary_label="HEAD YAW",
        max_deg=30.0,
        cls=NeckTurn,
    ),
    "neck_nod": BehaviourSpec(
        id="neck_nod",
        label="Neck Nod",
        desc="Nods head up/down (max 20 deg). Only neck pitch motor moves.",
        primary_joint=HEAD_PITCH,
        primary_label="HEAD PITCH",
        max_deg=20.0,
        cls=NeckNod,
    ),
    "arm_raise": BehaviourSpec(
        id="arm_raise",
        label="Arm Raise",
        desc="Gently raises/lowers right arm forward. Only shoulder pitch moves.",
        primary_joint=R_SHOULDER_PITCH,
        primary_label="R.SHOULDER",
        max_deg=30.0,
        cls=ArmRaise,
    ),
    "hand_wave": BehaviourSpec(
        id="hand_wave",
        label="Hand Wave",
        desc="Gentle stand-safe right-arm wave (max 20 deg). Legs/waist stay off.",
        primary_joint=R_SHOULDER_ROLL,
        primary_label="R.ARM WAVE",
        max_deg=20.0,
        cls=HandWave,
    ),
}


def _register_trained_head_skills() -> None:
    from .head_skills import SKILL_CLASSES, SKILLS

    for sid, sk in SKILLS.items():
        BEHAVIOUR_SPECS[sid] = BehaviourSpec(
            id=sid,
            label=sk.label,
            desc=sk.desc,
            primary_joint=sk.primary_joint,
            primary_label=sk.primary_label,
            max_deg=sk.max_deg,
            cls=SKILL_CLASSES[sid],
            kind="trained",
        )


_register_trained_head_skills()


def sync_trained_head_skills() -> None:
    """Re-sync after taught poses changed (behaviours/taught_poses.py): drop trained entries that left SKILLS."""
    from .head_skills import SKILLS

    for sid in [k for k, v in BEHAVIOUR_SPECS.items() if v.kind == "trained" and k not in SKILLS]:
        del BEHAVIOUR_SPECS[sid]
    _register_trained_head_skills()
