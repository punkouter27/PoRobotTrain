"""Loader and MuJoCo helpers for ``sim_calibration.yaml`` (sim-to-real digital twin fidelity)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml

from .joint_map import JointMap

DEFAULT_CALIBRATION_PATH = Path(__file__).resolve().parent / "sim_calibration.yaml"


def joint_class_for(name: str) -> str:
    if "ankle" in name or "knee" in name or "hip" in name:
        return "leg"
    if "waist" in name:
        return "waist"
    if "wrist" in name:
        return "wrist"
    if name == "head_pitch_joint":
        return "head_pitch"
    if name == "head_yaw_joint":
        return "head_yaw"
    return "arm"


@dataclass
class SimCalibration:
    armature: np.ndarray
    damping: np.ndarray
    frictionloss: np.ndarray
    unpowered_friction_nm: np.ndarray
    zero_offset_rad: np.ndarray
    control_delay_ms: float = 10.0
    lowpass_tau_s: float = 0.015
    nominal_bus_v: float = 48.0
    bus_voltage_v: float = 48.0
    bemf_nm_per_rad_s: float = 0.12
    pos_noise_rad: float = 0.0012
    vel_noise_rad_s: float = 0.03
    vel_filter_alpha: float = 0.35
    imu_noise_rad: float = 0.003
    body_mass_scale: float = 1.0
    torso_extra_mass_kg: float = 0.0
    compliant_stand: bool = True
    stand_timeconst_s: float = 0.03
    stand_damping_ratio: float = 1.0
    last_calibrated_at: Optional[str] = None
    parity_rms_deg: Optional[float] = None
    parity_max_deg: Optional[float] = None
    raw: Dict[str, Any] = field(default_factory=dict)
    source_path: Optional[Path] = None

    @classmethod
    def load(cls, path: Optional[str | Path] = None, jm: Optional[JointMap] = None) -> SimCalibration:
        jm = jm or JointMap.load()
        p = Path(path) if path is not None else DEFAULT_CALIBRATION_PATH
        raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        groups = raw.get("groups") or {}
        j_ovr = raw.get("joints") or {}

        n = jm.num_joints
        arm = np.zeros(n, dtype=np.float64)
        damp = np.zeros(n, dtype=np.float64)
        fric = np.zeros(n, dtype=np.float64)
        ufric = np.zeros(n, dtype=np.float64)
        zoff = np.zeros(n, dtype=np.float64)

        for i, name in enumerate(jm.names):
            g = groups.get(joint_class_for(name), {})
            ov = j_ovr.get(name, {})
            arm[i] = float(ov.get("armature", g.get("armature", 0.01)))
            damp[i] = float(ov.get("damping", g.get("damping", 0.05)))
            fric[i] = float(ov.get("frictionloss", g.get("frictionloss", 0.2)))
            ufric[i] = float(ov.get("unpowered_friction_nm", g.get("unpowered_friction_nm", 0.5)))
            zoff[i] = math.radians(float(ov.get("zero_offset_deg", 0.0)))

        timing = raw.get("timing") or {}
        act = raw.get("actuator") or {}
        sens = raw.get("sensors") or {}
        mass = raw.get("mass") or {}
        stand = raw.get("stand") or {}

        return cls(
            armature=arm,
            damping=damp,
            frictionloss=fric,
            unpowered_friction_nm=ufric,
            zero_offset_rad=zoff,
            control_delay_ms=float(timing.get("control_delay_ms", 10.0)),
            lowpass_tau_s=float(act.get("lowpass_tau_s", 0.015)),
            nominal_bus_v=float(act.get("nominal_bus_v", 48.0)),
            bus_voltage_v=float(act.get("bus_voltage_v", 48.0)),
            bemf_nm_per_rad_s=float(act.get("bemf_nm_per_rad_s", 0.12)),
            pos_noise_rad=float(sens.get("pos_noise_rad", 0.0012)),
            vel_noise_rad_s=float(sens.get("vel_noise_rad_s", 0.03)),
            vel_filter_alpha=float(sens.get("vel_filter_alpha", 0.35)),
            imu_noise_rad=float(sens.get("imu_noise_rad", 0.003)),
            body_mass_scale=float(mass.get("body_mass_scale", 1.0)),
            torso_extra_mass_kg=float(mass.get("torso_extra_mass_kg", 0.0)),
            compliant_stand=bool(stand.get("compliant", True)),
            stand_timeconst_s=float(stand.get("timeconst_s", 0.03)),
            stand_damping_ratio=float(stand.get("damping_ratio", 1.0)),
            last_calibrated_at=raw.get("last_calibrated_at"),
            parity_rms_deg=float(raw["parity_rms_deg"]) if raw.get("parity_rms_deg") is not None else None,
            parity_max_deg=float(raw["parity_max_deg"]) if raw.get("parity_max_deg") is not None else None,
            raw=raw,
            source_path=p,
        )

    def apply_to_model(self, mujoco, model, vadr: np.ndarray) -> None:
        """Apply calibrated per-joint armature, damping, frictionloss, and body mass scaling to an MjModel."""
        model.dof_armature[vadr] = self.armature
        model.dof_damping[vadr] = self.damping
        model.dof_frictionloss[vadr] = self.frictionloss
        if abs(self.body_mass_scale - 1.0) > 1e-6:
            model.body_mass[1:] *= self.body_mass_scale
            model.body_inertia[1:] *= self.body_mass_scale
        if abs(self.torso_extra_mass_kg) > 1e-6:
            tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
            if tid > 0 and model.body_mass[tid] > 0:
                ratio = (model.body_mass[tid] + self.torso_extra_mass_kg) / model.body_mass[tid]
                model.body_mass[tid] *= ratio
                model.body_inertia[tid] *= ratio
        mujoco.mj_setConst(model, mujoco.MjData(model))

    def summary(self) -> dict:
        return {
            "control_delay_ms": self.control_delay_ms,
            "lowpass_tau_s": self.lowpass_tau_s,
            "bus_voltage_v": self.bus_voltage_v,
            "bemf_nm_per_rad_s": self.bemf_nm_per_rad_s,
            "pos_noise_deg": round(math.degrees(self.pos_noise_rad), 3),
            "compliant_stand": self.compliant_stand,
            "body_mass_scale": self.body_mass_scale,
            "last_calibrated_at": self.last_calibrated_at,
            "parity_rms_deg": self.parity_rms_deg,
            "parity_max_deg": self.parity_max_deg,
        }
