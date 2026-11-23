"""Taught head poses (docs/sim_features.md #14): the operator sets a head yaw / pitch, saves it, and the pose becomes
an ordinary trained head skill ``pose_<id>`` (hold that head angle; head pitch + yaw only).

Store: ``RobotControl/configs/taught_poses.json`` = ``{"poses": [{"id", "name", "yaw_deg", "pitch_deg"}]}``.
``head_skills`` adds one ``HeadSkill`` per pose at import (command [0, 0, 0] like Hold Centre, reference = the fixed
pose), so ``/skills``, ``/skills/train``, the training env, the validation gate, the runner and the Unreal lab treat
it like every other skill: it runs only after its own validation passed for its exact ONNX file.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Dict, List

from .. import CONFIGS_DIR

POSES_PATH = CONFIGS_DIR / "taught_poses.json"
NAME_RE = re.compile(r"^[A-Za-z0-9 _-]{1,32}$")
YAW_RANGE = (-30.0, 30.0)  # deg, left +
PITCH_RANGE = (-15.0, 20.0)  # deg, down +
MAX_POSES = 20
PREFIX = "pose_"


def load_poses(path: Path = None) -> List[Dict]:
    """Stored poses (an unreadable or malformed file counts as no poses; bad entries are skipped)."""
    p = Path(path or POSES_PATH)
    try:
        raw = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    out = []
    for e in raw.get("poses", []) if isinstance(raw, dict) else []:
        try:
            out.append(_check(e.get("name"), e.get("yaw_deg"), e.get("pitch_deg"), e.get("id")))
        except (ValueError, AttributeError, TypeError):
            continue
    return out


def _save(poses: List[Dict], path: Path = None) -> None:
    p = Path(path or POSES_PATH)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps({"poses": poses}, indent=1), encoding="utf-8")
    tmp.replace(p)


def slug(name: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    return s[:24] or "pose"


def _soft_limits_deg() -> Dict[str, tuple]:
    from ..configs.joint_map import JointMap
    from .head_skills import HEAD_PITCH, HEAD_YAW, SOFT_LIMIT_FACTOR

    jm = JointMap.load()
    out = {}
    for j in (HEAD_YAW, HEAD_PITCH):
        i = jm.index_of(j)
        mid = 0.5 * (jm.limits_lo[i] + jm.limits_hi[i])
        half = 0.5 * (jm.limits_hi[i] - jm.limits_lo[i]) * SOFT_LIMIT_FACTOR
        out[j] = (math.degrees(mid - half), math.degrees(mid + half))
    return out


def _check(name, yaw_deg, pitch_deg, pid=None) -> Dict:
    if not isinstance(name, str) or not NAME_RE.match(name) or not name.strip():
        raise ValueError("name must be 1..32 characters of letters, digits, space, _ or -")
    try:
        yaw, pitch = float(yaw_deg), float(pitch_deg)
    except (TypeError, ValueError):
        raise ValueError("yaw_deg and pitch_deg must be numbers") from None
    if not (math.isfinite(yaw) and YAW_RANGE[0] <= yaw <= YAW_RANGE[1]):
        raise ValueError(f"yaw_deg must be in [{YAW_RANGE[0]:g}, {YAW_RANGE[1]:g}]")
    if not (math.isfinite(pitch) and PITCH_RANGE[0] <= pitch <= PITCH_RANGE[1]):
        raise ValueError(f"pitch_deg must be in [{PITCH_RANGE[0]:g}, {PITCH_RANGE[1]:g}]")
    from .head_skills import HEAD_PITCH, HEAD_YAW

    soft = _soft_limits_deg()
    for j, v in ((HEAD_YAW, yaw), (HEAD_PITCH, pitch)):
        lo, hi = soft[j]
        if not lo <= v <= hi:
            raise ValueError(f"{j} {v:+.1f} deg is outside the soft limits [{lo:.1f}, {hi:.1f}]")
    pid = slug(name) if pid is None else str(pid)
    if not re.match(r"^[a-z0-9_]{1,32}$", pid):
        raise ValueError("bad pose id")
    return {"id": pid, "name": name.strip(), "yaw_deg": round(yaw, 2), "pitch_deg": round(pitch, 2)}


def view(p: Dict) -> Dict:
    return dict(p, skill=PREFIX + p["id"])


def list_poses(path: Path = None) -> List[Dict]:
    return [view(p) for p in load_poses(path)]


def add_pose(name, yaw_deg, pitch_deg, path: Path = None) -> Dict:
    """Validate and store a new pose; refreshes the skill registry. Raises ValueError on a bad or duplicate pose."""
    pose = _check(name, yaw_deg, pitch_deg)
    poses = load_poses(path)
    if any(p["id"] == pose["id"] for p in poses):
        raise ValueError(f"a pose named {pose['name']!r} already exists (id {pose['id']}); delete it first")
    if len(poses) >= MAX_POSES:
        raise ValueError(f"at most {MAX_POSES} taught poses")
    poses.append(pose)
    _save(poses, path)
    refresh(path)
    return view(pose)


def delete_pose(pid: str, path: Path = None) -> bool:
    """Remove a pose from the store (its trained policy files stay in models/ but the skill is no longer listed)."""
    poses = load_poses(path)
    keep = [p for p in poses if p["id"] != str(pid)]
    if len(keep) == len(poses):
        return False
    _save(keep, path)
    refresh(path)
    return True


def refresh(path: Path = None) -> None:
    """Re-sync head_skills.SKILLS / SKILL_CLASSES and the behaviour registry with the store (service process)."""
    from . import head_skills

    head_skills.sync_taught_poses(load_poses(path))
    from . import stand_behaviours

    stand_behaviours.sync_trained_head_skills()
