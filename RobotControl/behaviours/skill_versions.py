"""Version history for the trained head skills (PC side only; the robot only ever sees the active copy).

    RobotControl/models/head_skills/<skill>/                 the ACTIVE version (what every runtime loads)
    RobotControl/models/head_skills/<skill>/versions/v<N>/   every trained version, passed or not

A retrain stores a new version and activates it only when its validation passed, so a failed retrain never replaces a
working skill. Rolling back activates an older version that passed (its files are copied into the active folder, so its
sha256 still matches its own validation). deploy.yaml / deploy.json carry ``version: N``.
"""

from __future__ import annotations

import json
import re
import shutil
from pathlib import Path
from typing import List, Optional

import yaml

from .. import MODELS_DIR
from .head_skills import SKILLS, SKILLS_SUBDIR, sha256_file

# what makes a version: the policy, its contract + gate, and the training record
FILES = ("policy.onnx", "deploy.yaml", "deploy.json", "validation.json", "model.pt", "train_log.csv")
_V = re.compile(r"^v(\d+)$")


def active_dir(skill_id: str) -> Path:
    return MODELS_DIR / SKILLS_SUBDIR / skill_id


def versions_dir(skill_id: str) -> Path:
    return active_dir(skill_id) / "versions"


def _read_yaml(path: Path) -> dict:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return {}


def set_version(folder: Path, version: int) -> None:
    """Stamp ``version`` into deploy.yaml and deploy.json (no other change: the gate's sha covers policy.onnx only)."""
    dep_path = folder / "deploy.yaml"
    if dep_path.exists():
        dep = _read_yaml(dep_path)
        dep["version"] = version
        dep_path.write_text(yaml.safe_dump(dep, sort_keys=False), encoding="utf-8")
    js_path = folder / "deploy.json"
    if js_path.exists():
        js = json.loads(js_path.read_text(encoding="utf-8"))
        js["version"] = version
        js_path.write_text(json.dumps(js, indent=1), encoding="utf-8")


def migrate(skill_id: str) -> None:
    """A skill trained before version history existed becomes v1 (once)."""
    act, vd = active_dir(skill_id), versions_dir(skill_id)
    if (act / "policy.onnx").exists() and not vd.exists():
        set_version(act, int(_read_yaml(act / "deploy.yaml").get("version") or 1))
        dst = vd / "v1"
        dst.mkdir(parents=True)
        for f in FILES:
            if (act / f).exists():
                shutil.copy2(act / f, dst / f)


def _numbers(skill_id: str) -> List[int]:
    vd = versions_dir(skill_id)
    if not vd.is_dir():
        return []
    return sorted(int(m.group(1)) for p in vd.iterdir() if p.is_dir() and (m := _V.match(p.name)))


def next_version(skill_id: str) -> int:
    migrate(skill_id)
    nums = _numbers(skill_id)
    return (nums[-1] + 1) if nums else 1


def active_version(skill_id: str) -> Optional[int]:
    dep = active_dir(skill_id) / "deploy.yaml"
    if not dep.exists():
        return None
    return int(_read_yaml(dep).get("version") or 1)


def _summary(folder: Path, version: int, active: Optional[int]) -> dict:
    dep = _read_yaml(folder / "deploy.yaml")
    val = dep.get("validation") or {}
    onnx = folder / "policy.onnx"
    sha = sha256_file(onnx) if onnx.exists() else None
    passed = bool(val.get("passed")) and sha is not None and val.get("policy_sha256") == sha
    meta = dep.get("metadata") or {}
    iters, envs, horizon = meta.get("iters"), meta.get("num_envs"), meta.get("horizon")
    steps = int(iters * envs * horizon) if iters and envs and horizon else None
    return {
        "version": version,
        "active": version == active,
        "iters": iters,
        "num_envs": envs,
        "steps": steps,  # policy steps of experience (each 20 ms of simulated time)
        "sim_hours": round(steps * 0.02 / 3600.0, 1) if steps else None,
        "train_time_s": meta.get("train_time_s"),
        "device": meta.get("device"),
        "onnx_bytes": onnx.stat().st_size if onnx.exists() else None,
        "passed": passed,
        "trained_at": (dep.get("metadata") or {}).get("trained_at"),
        "validated_at": val.get("validated_at"),
        "track_err_deg": (val.get("metrics") or {}).get("track_err_deg"),
        "failures": val.get("failures") or [],
        "sha256": sha,
    }


def list_versions(skill_id: str) -> List[dict]:
    """Newest first."""
    migrate(skill_id)
    act = active_version(skill_id)
    return [_summary(versions_dir(skill_id) / f"v{n}", n, act) for n in reversed(_numbers(skill_id))]


def store(skill_id: str, stage: Path, version: int) -> Path:
    """Move a freshly trained + validated folder into the history as v<version>."""
    set_version(stage, version)
    dst = versions_dir(skill_id) / f"v{version}"
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(stage), str(dst))
    return dst


def activate(skill_id: str, version: int) -> dict:
    """Make a stored version the active one. Refused unless its validation passed for its exact ONNX file."""
    if skill_id not in SKILLS:
        raise ValueError(f"{skill_id!r} is not a trained skill")
    migrate(skill_id)
    src = versions_dir(skill_id) / f"v{int(version)}"
    if not src.is_dir():
        raise ValueError(f"{SKILLS[skill_id].label} has no version {version}")
    info = _summary(src, int(version), None)
    if not info["passed"]:
        raise ValueError(f"{SKILLS[skill_id].label} v{version} did not pass its validation gate")
    act = active_dir(skill_id)
    act.mkdir(parents=True, exist_ok=True)
    for f in FILES:
        if (src / f).exists():
            shutil.copy2(src / f, act / f)
        elif (act / f).exists():
            (act / f).unlink()
    return info
