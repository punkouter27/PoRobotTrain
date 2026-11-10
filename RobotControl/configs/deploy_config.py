"""``deploy.yaml`` sidecar loader (the ONNX policy contract).

Every policy directory holds ``policy.onnx`` + ``deploy.yaml``::

    policy_name, step_dt, joint_names, motor_slots, default_joint_pos, kp, kd,
    action_scale, action_clip, observations [{name, size, ...}], commands {name: size},
    num_obs, num_actions, passive_joints, metadata
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import yaml

from .joint_map import JointMap, SOFT_LIMIT_FRACTION

KNOWN_OBS_TERMS = (
    "joint_pos_rel",
    "joint_vel",
    "joint_vel_rel",
    "last_action",
    "command",
    "base_ang_vel",
    "projected_gravity",
    "gait_phase",
)


@dataclass
class ObsTerm:
    name: str
    size: int
    params: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_entry(cls, entry: Any) -> "ObsTerm":
        if isinstance(entry, str):
            return cls(name=entry, size=-1)
        if not isinstance(entry, dict) or "name" not in entry:
            raise ValueError(f"bad observation entry {entry!r}")
        params = {k: v for k, v in entry.items() if k not in ("name", "size")}
        return cls(name=str(entry["name"]), size=int(entry.get("size", -1)), params=params)

    def to_entry(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {"name": self.name, "size": int(self.size)}
        d.update(self.params)
        return d


@dataclass
class DeployConfig:
    policy_name: str
    step_dt: float
    joint_names: List[str]
    motor_slots: List[int]
    default_joint_pos: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    action_scale: np.ndarray
    action_clip: Optional[np.ndarray]
    observations: List[ObsTerm]
    commands: Dict[str, int]
    num_obs: int
    num_actions: int
    passive_joints: List[str] = field(default_factory=list)
    passive_kd: Optional[float] = None          # damping for passive joints (None -> joint-map kd)
    metadata: Dict[str, Any] = field(default_factory=dict)
    source_path: Optional[Path] = None

    # ------------------------------------------------------------------ I/O
    @classmethod
    def load(cls, path: str | Path) -> "DeployConfig":
        path = Path(path)
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        return cls.from_dict(raw, source_path=path)

    @classmethod
    def from_dict(cls, raw: Dict[str, Any], source_path: Optional[Path] = None) -> "DeployConfig":
        joint_names = [str(n) for n in raw.get("joint_names", [])]
        a = len(joint_names)

        def arr(key: str, default: Optional[float] = None) -> np.ndarray:
            v = raw.get(key)
            if v is None:
                if default is None:
                    raise ValueError(f"deploy.yaml missing {key!r}")
                return np.full(a, float(default))
            if np.isscalar(v):
                return np.full(a, float(v))
            return np.asarray(v, dtype=np.float64)

        clip_raw = raw.get("action_clip")
        if clip_raw is None:
            action_clip = None
        elif np.isscalar(clip_raw):
            action_clip = np.full(a, float(clip_raw))
        else:
            action_clip = np.asarray(clip_raw, dtype=np.float64)

        observations = [ObsTerm.from_entry(e) for e in (raw.get("observations") or [])]
        commands_raw = raw.get("commands") or {}
        commands = {str(k): int(v) for k, v in commands_raw.items()}
        num_actions = int(raw.get("num_actions", a))
        num_obs = int(raw.get("num_obs", sum(max(t.size, 0) for t in observations)))

        return cls(
            policy_name=str(raw.get("policy_name", source_path.parent.name if source_path else "policy")),
            step_dt=float(raw.get("step_dt", 0.02)),
            joint_names=joint_names,
            motor_slots=[int(s) for s in raw.get("motor_slots", [])],
            default_joint_pos=arr("default_joint_pos"),
            kp=arr("kp"),
            kd=arr("kd"),
            action_scale=arr("action_scale"),
            action_clip=action_clip,
            observations=observations,
            commands=commands,
            num_obs=num_obs,
            num_actions=num_actions,
            passive_joints=[str(n) for n in (raw.get("passive_joints") or [])],
            passive_kd=(float(raw["passive_kd"]) if isinstance(raw.get("passive_kd"), (int, float)) else None),
            metadata=dict(raw.get("metadata") or {}),
            source_path=source_path,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "policy_name": self.policy_name,
            "step_dt": float(self.step_dt),
            "joint_names": list(self.joint_names),
            "motor_slots": [int(s) for s in self.motor_slots],
            "default_joint_pos": [float(x) for x in self.default_joint_pos],
            "kp": [float(x) for x in self.kp],
            "kd": [float(x) for x in self.kd],
            "action_scale": [float(x) for x in self.action_scale],
            "action_clip": None if self.action_clip is None else [float(x) for x in self.action_clip],
            "observations": [t.to_entry() for t in self.observations],
            "commands": dict(self.commands),
            "num_obs": int(self.num_obs),
            "num_actions": int(self.num_actions),
            "passive_joints": list(self.passive_joints),
            "passive_kd": self.passive_kd,
            "metadata": dict(self.metadata),
        }

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)
        return path

    # ------------------------------------------------------------- derived
    @property
    def command_dim(self) -> int:
        return int(sum(self.commands.values()))

    @property
    def obs_dim_from_terms(self) -> int:
        return int(sum(max(t.size, 0) for t in self.observations))

    def policy_indices(self, joint_map: JointMap) -> np.ndarray:
        return joint_map.indices_of(self.joint_names)

    def passive_indices(self, joint_map: JointMap) -> np.ndarray:
        return joint_map.indices_of(self.passive_joints)

    # ----------------------------------------------------------- validation
    def validate(self, joint_map: Optional[JointMap] = None,
                 soft_fraction: float = SOFT_LIMIT_FRACTION) -> List[str]:
        """Return a list of human-readable problems (empty list == OK)."""
        errs: List[str] = []
        a = len(self.joint_names)
        if a == 0:
            errs.append("joint_names is empty")
        if self.num_actions != a:
            errs.append(f"num_actions={self.num_actions} != len(joint_names)={a}")
        for key in ("default_joint_pos", "kp", "kd", "action_scale"):
            v = getattr(self, key)
            if len(v) != a:
                errs.append(f"{key} has length {len(v)}, expected {a}")
        if self.action_clip is not None and len(self.action_clip) != a:
            errs.append(f"action_clip has length {len(self.action_clip)}, expected {a}")
        if self.motor_slots and len(self.motor_slots) != a:
            errs.append(f"motor_slots has length {len(self.motor_slots)}, expected {a}")
        if self.step_dt <= 0:
            errs.append(f"step_dt must be > 0, got {self.step_dt}")
        if np.any(self.kp < 0) or np.any(self.kd < 0):
            errs.append("kp/kd must be non-negative")
        if np.any(self.action_scale <= 0):
            errs.append("action_scale must be > 0")

        obs_sum = self.obs_dim_from_terms
        if any(t.size < 0 for t in self.observations):
            errs.append("every observation term needs a size")
        elif obs_sum != self.num_obs:
            errs.append(f"sum(observations.size)={obs_sum} != num_obs={self.num_obs}")
        for t in self.observations:
            if t.name not in KNOWN_OBS_TERMS:
                errs.append(f"unknown observation term {t.name!r} (known: {KNOWN_OBS_TERMS})")
            if t.name in ("joint_pos_rel", "joint_vel", "joint_vel_rel", "last_action") and t.size not in (-1, a):
                joints = t.params.get("joints")
                if not joints or len(joints) != t.size:
                    errs.append(f"observation {t.name} size {t.size} != num_actions {a} and no explicit joints list")
            if t.name == "command" and t.size != self.command_dim and t.size >= 0:
                errs.append(f"command term size {t.size} != sum(commands)={self.command_dim}")
            if t.name in ("base_ang_vel", "projected_gravity") and t.size not in (-1, 3):
                errs.append(f"{t.name} must have size 3")
            if t.name == "gait_phase" and t.size not in (-1, 2):
                errs.append("gait_phase must have size 2")
        if len(set(self.joint_names)) != a:
            errs.append("duplicate joint_names")
        if set(self.passive_joints) & set(self.joint_names):
            errs.append("passive_joints overlaps joint_names")

        if joint_map is not None:
            for n in self.joint_names + self.passive_joints:
                if n not in joint_map.names:
                    errs.append(f"joint {n!r} not in joint map")
            if not errs:
                idx = joint_map.indices_of(self.joint_names)
                if self.motor_slots:
                    exp = joint_map.motor_slots[idx].tolist()
                    if [int(s) for s in self.motor_slots] != exp:
                        errs.append(f"motor_slots {self.motor_slots} != joint map slots {exp}")
                lo, hi = joint_map.soft_limits(soft_fraction)
                for k, i in enumerate(idx):
                    d = float(self.default_joint_pos[k])
                    if not (lo[i] <= d <= hi[i]):
                        errs.append(f"default_joint_pos[{k}]={d:.3f} outside soft limits "
                                    f"[{lo[i]:.3f},{hi[i]:.3f}] of {self.joint_names[k]}")
                    full_range = joint_map.limits_hi[i] - joint_map.limits_lo[i]
                    if self.action_scale[k] > full_range:
                        errs.append(f"action_scale[{k}]={self.action_scale[k]:.3f} larger than the full "
                                    f"joint range of {self.joint_names[k]}")
        return errs


# ----------------------------------------------------------------------------
# Canonical Level 0 layout (right arm + head, hanging on the stand).
LEVEL0_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
]
LEVEL0_COMMANDS = {"head_yaw_target": 1, "shoulder_pitch_target": 1}


def level0_deploy_config(joint_map: JointMap, policy_name: str = "r1_level0_upperbody",
                         metadata: Optional[Dict[str, Any]] = None) -> DeployConfig:
    """Build the Level 0 deploy config from the joint map (23 obs, 7 actions)."""
    idx = joint_map.indices_of(LEVEL0_JOINTS)
    passive = [n for n in joint_map.names if n not in LEVEL0_JOINTS]
    obs = [
        ObsTerm("joint_pos_rel", 7),
        ObsTerm("joint_vel", 7),
        ObsTerm("last_action", 7),
        ObsTerm("command", 2),
    ]
    return DeployConfig(
        policy_name=policy_name,
        step_dt=0.02,
        joint_names=list(LEVEL0_JOINTS),
        motor_slots=joint_map.motor_slots[idx].tolist(),
        default_joint_pos=joint_map.defaults[idx].copy(),
        kp=joint_map.kp[idx].copy(),
        kd=joint_map.kd[idx].copy(),
        action_scale=joint_map.action_scale[idx].copy(),
        action_clip=np.full(7, 1.0),
        observations=obs,
        commands=dict(LEVEL0_COMMANDS),
        num_obs=23,
        num_actions=7,
        passive_joints=passive,
        metadata=dict(metadata or {}),
    )


def find_deploy_yaml(onnx_path: str | Path) -> Path:
    p = Path(onnx_path)
    cand = p.with_name("deploy.yaml")
    if cand.exists():
        return cand
    cand2 = p.with_suffix(".yaml")
    if cand2.exists():
        return cand2
    raise FileNotFoundError(f"no deploy.yaml next to {p}")
