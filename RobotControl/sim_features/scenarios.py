"""Scenario scripts (docs/sim_features.md #18): ``RobotControl/scenarios/<id>.json``, loaded and validated here.

Walk scenarios (``"target": "walk"``) are played by the walk sandbox on sim time; lab scenarios (``"target": "lab"``)
by the Unreal lab brain and by ``tools/skill_duel.py``.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Dict, List, Optional

from .. import PACKAGE_DIR

SCENARIOS_DIR = PACKAGE_DIR / "scenarios"
ID_RE = re.compile(r"^[a-z0-9_]{1,40}$")
WALK_STEPS = ("command", "throw", "reset", "replay", "filter")
LAB_STEPS = ("target", "push", "noise", "delay", "strength")
WALK_MODES = ("policy", "hold", "free_hold")


def _num(step: dict, key: str, lo: float, hi: float, default=None) -> float:
    v = step.get(key, default)
    if v is None or isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"step {step!r}: {key} must be a number")
    if not lo <= float(v) <= hi:
        raise ValueError(f"step {step!r}: {key} must be in [{lo}, {hi}]")
    return float(v)


def _check_step(step: dict, target: str, duration: float) -> None:
    if not isinstance(step, dict):
        raise ValueError("every step must be an object")
    _num(step, "t", 0.0, duration)
    kind = step.get("do")
    if target == "walk":
        if kind not in WALK_STEPS:
            raise ValueError(f"walk step kind must be one of {WALK_STEPS}, got {kind!r}")
        if kind == "command":
            for k in ("vx", "vy", "wz"):
                _num(step, k, -2.0, 2.0, 0.0)
        elif kind == "throw":
            if step.get("kind", "ball") not in ("ball", "box"):
                raise ValueError("throw kind must be ball or box")
            _num(step, "speed", 1.0, 8.0, 4.0)
            _num(step, "from_deg", -360.0, 360.0, 0.0)
        elif kind == "replay":
            _num(step, "seconds", 1.0, 10.0, 3.0)
            _num(step, "speed", 0.1, 1.0, 0.25)
        elif kind == "filter":
            _num(step, "alpha", 0.01, 1.0, 0.5)
    else:
        if kind not in LAB_STEPS:
            raise ValueError(f"lab step kind must be one of {LAB_STEPS}, got {kind!r}")
        if kind == "target":
            _num(step, "yaw_deg", -60.0, 60.0)
            _num(step, "pitch_deg", -30.0, 30.0)
        elif kind == "push":
            _num(step, "yaw", -1.0, 1.0, 0.0)
            _num(step, "pitch", -1.0, 1.0, 0.0)
        elif kind == "noise":
            _num(step, "deg", 0.0, 5.0)
        elif kind == "delay":
            v = step.get("steps")
            if not isinstance(v, int) or isinstance(v, bool) or not 0 <= v <= 4:
                raise ValueError("delay steps must be an integer in [0, 4]")
        elif kind == "strength":
            _num(step, "pct", 30.0, 100.0)


def validate(sc: dict, sid: Optional[str] = None) -> dict:
    """Check a scenario dict; returns it with defaults filled in. Raises ValueError with the first problem."""
    if not isinstance(sc, dict):
        raise ValueError("scenario must be a JSON object")
    if not ID_RE.match(str(sc.get("id", ""))):
        raise ValueError("id must be 1..40 chars of [a-z0-9_]")
    if sid is not None and sc["id"] != sid:
        raise ValueError(f"id {sc['id']!r} does not match the file name {sid!r}")
    for k in ("name", "description"):
        if not isinstance(sc.get(k), str) or not sc[k]:
            raise ValueError(f"{k} must be a non-empty string")
    target = sc.get("target")
    if target not in ("walk", "lab"):
        raise ValueError("target must be walk or lab")
    duration = _num(sc, "duration_s", 0.5, 300.0)
    if target == "walk":
        from ..walk.course import COURSES

        sc.setdefault("course", "flat")
        sc.setdefault("mode", "hold")
        sc.setdefault("seed", 0)
        sc.setdefault("difficulty", 0.5)
        if sc["course"] not in COURSES:
            raise ValueError(f"course must be one of {sorted(COURSES)}")
        for k in ("mode", "fallback_mode"):
            if k in sc and sc[k] not in WALK_MODES:
                raise ValueError(f"{k} must be one of {WALK_MODES}")
        _num(sc, "difficulty", 0.0, 1.0)
    else:
        if not isinstance(sc.get("skill"), str):
            raise ValueError("a lab scenario needs a skill")
        sc.setdefault("size_deg", 15.0)
        sc.setdefault("seed", 0)
        _num(sc, "size_deg", 1.0, 30.0)
    if not isinstance(sc.get("seed"), int) or isinstance(sc.get("seed"), bool):
        raise ValueError("seed must be an integer")
    steps = sc.get("steps")
    if not isinstance(steps, list):
        raise ValueError("steps must be a list")
    for st in steps:
        _check_step(st, target, duration)
    ts = [float(st["t"]) for st in steps]
    if ts != sorted(ts):
        raise ValueError("steps must be in time order")
    return sc


def load(sid: str, folder: Path = SCENARIOS_DIR) -> dict:
    if not ID_RE.match(str(sid)):
        raise ValueError(f"bad scenario id {sid!r}")
    path = folder / f"{sid}.json"
    if not path.is_file():
        raise ValueError(f"no scenario {sid!r}")
    return validate(json.loads(path.read_text(encoding="utf-8")), sid)


def list_scenarios(folder: Path = SCENARIOS_DIR) -> List[Dict]:
    """[{id, name, target, description, duration_s}] of every valid scenario (invalid files are skipped)."""
    out = []
    for p in sorted(folder.glob("*.json")):
        try:
            sc = load(p.stem, folder)
        except (ValueError, OSError):
            continue
        out.append({k: sc[k] for k in ("id", "name", "target", "description", "duration_s")})
    return out
