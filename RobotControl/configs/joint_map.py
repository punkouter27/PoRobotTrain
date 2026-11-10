"""Loader for ``r1_joint_map.yaml`` -- the single source of truth for joint order,
motor slots, limits and gains.

Everything downstream (MJCF twin, URLab bridge, Unitree SDK2 interface, ONNX
validator, safety guard) indexes joints in *policy order* (``policy_index``)
and uses this class to translate to motor slots / actuator names.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

DEFAULT_JOINT_MAP_PATH = Path(__file__).resolve().parent / "r1_joint_map.yaml"
SOFT_LIMIT_FRACTION = 0.9


def actuator_name_for(joint_name: str) -> str:
    """MJCF/URLab actuator name convention: joint name without the ``_joint`` suffix."""
    return joint_name[:-6] if joint_name.endswith("_joint") else joint_name


@dataclass
class JointMap:
    """Array view of the joint map. All arrays are in policy order."""

    names: List[str]
    policy_indices: np.ndarray
    motor_slots: np.ndarray
    limits_lo: np.ndarray
    limits_hi: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    effort: np.ndarray
    action_scale: np.ndarray
    defaults: np.ndarray
    axes: np.ndarray
    groups: Dict[str, List[int]] = field(default_factory=dict)
    robot: str = "unitree_r1_a5"
    lowcmd_num_slots: int = 31
    lowcmd_topic: str = "rt/lowcmd"
    lowstate_topic: str = "rt/lowstate"
    msg_type: str = "unitree_hg"
    imu_body: str = "pelvis"
    source_path: Optional[Path] = None

    # ------------------------------------------------------------------ loading
    @classmethod
    def load(cls, path: Optional[str | Path] = None) -> "JointMap":
        path = Path(path) if path is not None else DEFAULT_JOINT_MAP_PATH
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        return cls.from_dict(raw, source_path=path)

    @classmethod
    def from_dict(cls, raw: dict, source_path: Optional[Path] = None) -> "JointMap":
        joints = sorted(raw["joints"], key=lambda j: int(j["policy_index"]))
        n = len(joints)
        pidx = np.array([int(j["policy_index"]) for j in joints], dtype=np.int64)
        if not np.array_equal(pidx, np.arange(n)):
            raise ValueError(f"policy_index must be 0..{n-1} without gaps, got {pidx.tolist()}")
        if "num_joints" in raw and int(raw["num_joints"]) != n:
            raise ValueError(f"num_joints={raw['num_joints']} but {n} joints listed")
        names = [str(j["name"]) for j in joints]
        if len(set(names)) != n:
            raise ValueError("duplicate joint names in joint map")
        slots = np.array([int(j["motor_slot"]) for j in joints], dtype=np.int64)
        if len(set(slots.tolist())) != n:
            raise ValueError("duplicate motor_slot values in joint map")
        limits = np.array([j["limits"] for j in joints], dtype=np.float64)
        if np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("joint limits must satisfy lo < hi")
        jm = cls(
            names=names,
            policy_indices=pidx,
            motor_slots=slots,
            limits_lo=limits[:, 0].copy(),
            limits_hi=limits[:, 1].copy(),
            kp=np.array([float(j["kp"]) for j in joints]),
            kd=np.array([float(j["kd"]) for j in joints]),
            effort=np.array([float(j["effort"]) for j in joints]),
            action_scale=np.array([float(j["action_scale"]) for j in joints]),
            defaults=np.array([float(j["default"]) for j in joints]),
            axes=np.array([j.get("axis", [0, 0, 1]) for j in joints], dtype=np.float64),
            groups={str(k): [int(i) for i in v] for k, v in (raw.get("groups") or {}).items()},
            robot=str(raw.get("robot", "unitree_r1_a5")),
            lowcmd_num_slots=int(raw.get("lowcmd_num_slots", 31)),
            lowcmd_topic=str(raw.get("lowcmd_topic", "rt/lowcmd")),
            lowstate_topic=str(raw.get("lowstate_topic", "rt/lowstate")),
            msg_type=str(raw.get("msg_type", "unitree_hg")),
            imu_body=str(raw.get("imu_body", "pelvis")),
            source_path=source_path,
        )
        for g, idx in jm.groups.items():
            bad = [i for i in idx if i < 0 or i >= n]
            if bad:
                raise ValueError(f"group {g!r} references invalid indices {bad}")
        return jm

    # --------------------------------------------------------------- properties
    @property
    def num_joints(self) -> int:
        return len(self.names)

    def __len__(self) -> int:
        return len(self.names)

    @property
    def limits(self) -> np.ndarray:
        """(N, 2) array of [lo, hi]."""
        return np.stack([self.limits_lo, self.limits_hi], axis=1)

    @property
    def actuator_names(self) -> List[str]:
        return [actuator_name_for(n) for n in self.names]

    @property
    def unmapped_slots(self) -> List[int]:
        """LowCmd slots not driven by any mapped joint (left at mode 0)."""
        used = set(self.motor_slots.tolist())
        return [s for s in range(self.lowcmd_num_slots) if s not in used]

    @property
    def slot_to_index(self) -> Dict[int, int]:
        return {int(s): i for i, s in enumerate(self.motor_slots)}

    # ------------------------------------------------------------------ lookups
    def index_of(self, name: str) -> int:
        try:
            return self.names.index(name)
        except ValueError:
            raise KeyError(f"joint {name!r} not in joint map") from None

    def indices_of(self, names: Iterable[str]) -> np.ndarray:
        return np.array([self.index_of(n) for n in names], dtype=np.int64)

    def group(self, name: str) -> np.ndarray:
        return np.array(self.groups[name], dtype=np.int64)

    def group_indices(self, *group_names: str) -> np.ndarray:
        out: List[int] = []
        for g in group_names:
            out.extend(self.groups[g])
        return np.array(out, dtype=np.int64)

    def soft_limits(self, fraction: float = SOFT_LIMIT_FRACTION) -> Tuple[np.ndarray, np.ndarray]:
        """Soft limits: the joint range shrunk to ``fraction`` of its width around its
        centre (so asymmetric ranges stay centred)."""
        c = 0.5 * (self.limits_lo + self.limits_hi)
        h = 0.5 * (self.limits_hi - self.limits_lo) * float(fraction)
        return c - h, c + h

    # --------------------------------------------------------------- sub-maps
    def select(self, indices: Sequence[int] | np.ndarray) -> "JointMap":
        """Build a sub-map (policy_indices renumbered 0..k-1, groups filtered/remapped)."""
        idx = np.asarray(indices, dtype=np.int64)
        remap = {int(old): new for new, old in enumerate(idx.tolist())}
        groups = {}
        for g, members in self.groups.items():
            kept = [remap[m] for m in members if m in remap]
            if kept:
                groups[g] = kept
        return JointMap(
            names=[self.names[i] for i in idx],
            policy_indices=np.arange(len(idx), dtype=np.int64),
            motor_slots=self.motor_slots[idx].copy(),
            limits_lo=self.limits_lo[idx].copy(),
            limits_hi=self.limits_hi[idx].copy(),
            kp=self.kp[idx].copy(),
            kd=self.kd[idx].copy(),
            effort=self.effort[idx].copy(),
            action_scale=self.action_scale[idx].copy(),
            defaults=self.defaults[idx].copy(),
            axes=self.axes[idx].copy(),
            groups=groups,
            robot=self.robot,
            lowcmd_num_slots=self.lowcmd_num_slots,
            lowcmd_topic=self.lowcmd_topic,
            lowstate_topic=self.lowstate_topic,
            msg_type=self.msg_type,
            imu_body=self.imu_body,
            source_path=self.source_path,
        )

    def select_names(self, names: Iterable[str]) -> "JointMap":
        return self.select(self.indices_of(names))

    # ---------------------------------------------------------------- helpers
    def to_slot_array(self, values: np.ndarray, fill: float = 0.0) -> np.ndarray:
        """Scatter a policy-order vector into a ``lowcmd_num_slots``-long slot array."""
        out = np.full(self.lowcmd_num_slots, fill, dtype=np.float64)
        out[self.motor_slots] = values
        return out

    def from_slot_array(self, slots: np.ndarray) -> np.ndarray:
        """Gather a slot array (len >= lowcmd_num_slots) into policy order."""
        return np.asarray(slots, dtype=np.float64)[self.motor_slots]

    def describe(self) -> str:
        lines = [f"{self.robot}: {self.num_joints} joints, {self.lowcmd_num_slots} slots, "
                 f"unmapped slots {self.unmapped_slots}"]
        for i, n in enumerate(self.names):
            lines.append(
                f"  [{i:2d}] slot {int(self.motor_slots[i]):2d} {n:30s} "
                f"lim [{self.limits_lo[i]:+.3f},{self.limits_hi[i]:+.3f}] "
                f"kp {self.kp[i]:6.1f} kd {self.kd[i]:4.1f} eff {self.effort[i]:5.1f} "
                f"scale {self.action_scale[i]:.4f} def {self.defaults[i]:+.3f}"
            )
        return "\n".join(lines)


def load_joint_map(path: Optional[str | Path] = None) -> JointMap:
    return JointMap.load(path)


if __name__ == "__main__":  # pragma: no cover
    print(JointMap.load().describe())
