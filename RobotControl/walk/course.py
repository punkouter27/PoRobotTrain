"""Walk sandbox courses: checkpoints and static boxes, in the MuJoCo world frame (metres, z up, quat w x y z).

Single source of truth for the course: the simulator adds these boxes to ``scene_ground.xml`` at load time, and
``GET /walk/state`` sends the same geometry to the Blazor and Unreal clients, which only draw it.
The robot starts at the origin facing +x. A lap is the checkpoints in order; the last one is back at the start.

``procedural`` (docs/sim_features.md #3) is generated from a seed and a difficulty in [0, 1]: the same seed always gives
the same boxes. Pieces: stairs, ramps, stepping stones, clutter and low planks; the first 1.5 m around the origin
stays clear and only low planks (traversable) lie on the checkpoint path. A training scene can ``<include>`` the
same terrain:

    python -m robotcontrol.walk.course --export procedural --seed 3 --difficulty 0.7 --out terrain.xml
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path
from typing import List, Optional, Tuple
from xml.sax.saxutils import quoteattr

CHECKPOINT_RADIUS = 0.5
CLEAR_RADIUS = 1.5  # nothing within this distance of the start
PATH_CLEARANCE = 0.35  # gap between a non-traversable piece and the checkpoint path


def _yaw_quat(deg: float) -> List[float]:
    h = math.radians(deg) / 2
    return [round(math.cos(h), 6), 0.0, 0.0, round(math.sin(h), 6)]


def _quat_mul(a: List[float], b: List[float]) -> List[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


_LOOP_CHECKPOINTS = [
    [3.0, 0.0, 0.0],
    [5.0, 2.5, 0.0],
    [3.0, 5.0, 0.0],
    [0.0, 5.0, 0.0],
    [-2.0, 2.5, 0.0],
    [0.0, 0.0, 0.0],
]

COURSES: dict = {
    # Open floor, checkpoints only.
    "flat": {"checkpoints": _LOOP_CHECKPOINTS, "boxes": []},
    # Same loop with things to steer around and one low plank to step over (2 cm).
    "loop": {
        "checkpoints": _LOOP_CHECKPOINTS,
        "boxes": [
            {"name": "plank", "pos": [1.5, 0.0, 0.01], "size": [0.12, 0.6, 0.01], "quat": [1, 0, 0, 0],
             "rgba": [0.85, 0.62, 0.25, 1]},
            {"name": "pillar_a", "pos": [4.1, 3.9, 0.5], "size": [0.15, 0.15, 0.5], "quat": _yaw_quat(20),
             "rgba": [0.39, 0.4, 0.95, 1]},
            {"name": "pillar_b", "pos": [1.5, 5.3, 0.5], "size": [0.15, 0.15, 0.5], "quat": [1, 0, 0, 0],
             "rgba": [0.39, 0.4, 0.95, 1]},
            {"name": "pillar_c", "pos": [-1.1, 3.9, 0.5], "size": [0.15, 0.15, 0.5], "quat": _yaw_quat(-15),
             "rgba": [0.39, 0.4, 0.95, 1]},
            {"name": "wall", "pos": [1.5, 2.5, 0.3], "size": [1.2, 0.08, 0.3], "quat": [1, 0, 0, 0],
             "rgba": [0.55, 0.6, 0.7, 1]},
        ],
    },
    # Seeded generator (seed, difficulty); see procedural().
    "procedural": {"procedural": True},
}  # fmt: skip

PIECE_RGBA = {
    "stairs": [0.75, 0.55, 0.3, 1],
    "ramp": [0.45, 0.7, 0.5, 1],
    "stones": [0.6, 0.6, 0.65, 1],
    "clutter": [0.39, 0.4, 0.95, 1],
    "plank": [0.85, 0.62, 0.25, 1],
}


# ----------------------------------------------------------------------------- geometry helpers
def _seg_dist(p: Tuple[float, float], a: List[float], b: List[float]) -> float:
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + u * dx), p[1] - (ay + u * dy))


def _path_dist(p: Tuple[float, float], path: List[List[float]]) -> float:
    return min(_seg_dist(p, path[i], path[i + 1]) for i in range(len(path) - 1))


def _box(name: str, kind: str, pos, size, quat) -> dict:
    return {
        "name": name,
        "pos": [round(float(v), 4) for v in pos],
        "size": [round(float(v), 4) for v in size],
        "quat": [round(float(v), 6) for v in quat],
        "rgba": list(PIECE_RGBA[kind]),
    }


def _local(cx: float, cy: float, yaw: float, u: float, v: float) -> Tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return cx + c * u - s * v, cy + s * u + c * v


def _piece(kind: str, k: int, cx: float, cy: float, yaw_deg: float, d: float, rnd: random.Random) -> List[dict]:
    yaw = math.radians(yaw_deg)
    q = _yaw_quat(yaw_deg)
    out = []
    if kind == "stairs":
        rise = 0.02 + 0.10 * d
        n = 3
        for j in range(n):
            h = rise * (j + 1)
            x, y = _local(cx, cy, yaw, (j - (n - 1) / 2) * 0.3, 0.0)
            out.append(_box(f"proc{k}_stairs_{j}", kind, [x, y, h / 2], [0.15, 0.5, h / 2], q))
    elif kind == "ramp":
        ang = rnd.uniform(0.5, 1.0) * (4.0 + 8.0 * d)
        h = math.radians(ang) / 2
        pitch = [math.cos(h), 0.0, -math.sin(h), 0.0]  # rises towards local +x
        out.append(_box(f"proc{k}_ramp", kind, [cx, cy, 0.0], [0.6, 0.45, 0.05], _quat_mul(q, pitch)))
    elif kind == "stones":
        n = 3 + int(round(2 * d))
        for j in range(n):
            h = 0.02 + 0.06 * d * rnd.random()
            x, y = _local(cx, cy, yaw, (j - (n - 1) / 2) * 0.32, rnd.uniform(-0.08, 0.08))
            out.append(_box(f"proc{k}_stone_{j}", kind, [x, y, h / 2], [0.12, 0.12, h / 2], q))
    elif kind == "clutter":
        s = rnd.uniform(0.08, 0.15 + 0.1 * d)
        out.append(_box(f"proc{k}_clutter", kind, [cx, cy, s], [s, s, s], q))
    elif kind == "plank":
        h = 0.01 + 0.01 * d
        out.append(_box(f"proc{k}_plank", kind, [cx, cy, h / 2], [0.1, 0.6, h / 2], q))
    return out


PIECE_RADIUS = {"stairs": 0.65, "ramp": 0.75, "stones": 0.9, "clutter": 0.4, "plank": 0.62}


def procedural(seed: int = 0, difficulty: float = 0.5) -> dict:
    """Deterministic course for (seed, difficulty): checkpoints on a jittered loop + pieces beside it."""
    d = max(0.0, min(1.0, float(difficulty)))
    rnd = random.Random(int(seed) * 7919 + 17)
    rx, ry = rnd.uniform(3.0, 4.5), rnd.uniform(2.2, 3.0)
    n_cp = rnd.randint(5, 7)
    cps: List[List[float]] = []
    for k in range(1, n_cp):
        th = -math.pi / 2 + 2 * math.pi * k / n_cp
        jit = 0.3 + 0.4 * d
        x = rx * math.cos(th) + rnd.uniform(-jit, jit)
        y = ry + ry * math.sin(th) + rnd.uniform(-jit, jit)
        cps.append([round(x, 3), round(y, 3), 0.0])
    cps.append([0.0, 0.0, 0.0])
    path = [[0.0, 0.0]] + [c[:2] for c in cps]
    xs, ys = [p[0] for p in path], [p[1] for p in path]
    lo_x, hi_x, lo_y, hi_y = min(xs) - 1.5, max(xs) + 1.5, min(ys) - 1.5, max(ys) + 1.5
    n_pieces = 4 + int(round(8 * d))
    kinds = ["stairs", "ramp", "stones", "clutter", "clutter", "plank"]
    placed: List[Tuple[float, float, float]] = []  # (x, y, radius)
    boxes: List[dict] = []
    for k in range(n_pieces):
        kind = kinds[k % len(kinds)] if k < len(kinds) else rnd.choice(kinds)
        r = PIECE_RADIUS[kind]
        for _ in range(300):
            if kind == "plank":  # across the path, between two checkpoints
                i = rnd.randrange(len(path) - 1)
                a, b = path[i], path[i + 1]
                u = rnd.uniform(0.35, 0.65)
                cx, cy = a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])
                yaw_deg = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
                if min(math.hypot(cx - c[0], cy - c[1]) for c in cps) < CHECKPOINT_RADIUS + 0.3:
                    continue
            else:
                cx, cy = rnd.uniform(lo_x, hi_x), rnd.uniform(lo_y, hi_y)
                yaw_deg = rnd.uniform(-180.0, 180.0)
                if _path_dist((cx, cy), path) < r + PATH_CLEARANCE:
                    continue
            if math.hypot(cx, cy) < CLEAR_RADIUS + r:
                continue
            if any(math.hypot(cx - px, cy - py) < r + pr + 0.1 for px, py, pr in placed):
                continue
            placed.append((cx, cy, r))
            boxes.extend(_piece(kind, k, cx, cy, yaw_deg, d, rnd))
            break
    return {"checkpoints": cps, "boxes": boxes}


def course(name: str, seed: int = 0, difficulty: float = 0.5) -> dict:
    if name not in COURSES:
        raise ValueError(f"unknown course {name!r}; choose one of {sorted(COURSES)}")
    c = procedural(seed, difficulty) if COURSES[name].get("procedural") else COURSES[name]
    return {"checkpoints": c["checkpoints"], "checkpoint_radius": CHECKPOINT_RADIUS, "boxes": c["boxes"]}


# ----------------------------------------------------------------------------- MJCF export
def to_mjcf(geometry: dict) -> str:
    """MJCF fragment with the course boxes as static world geoms (same friction as the walk sim)."""

    def f(v) -> str:
        return " ".join(f"{float(x):.6g}" for x in v)

    lines = ["<mujoco>", "  <worldbody>"]
    for b in geometry["boxes"]:
        lines.append(
            f'    <geom name={quoteattr("course_" + b["name"])} type="box" size="{f(b["size"])}" pos="{f(b["pos"])}" '
            f'quat="{f(b["quat"])}" rgba="{f(b["rgba"])}" friction="0.9 0.005 0.0001" condim="3"/>'
        )
    lines += ["  </worldbody>", "</mujoco>", ""]
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Walk courses: export a course as an MJCF fragment")
    ap.add_argument("--export", required=True, choices=sorted(COURSES))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--difficulty", type=float, default=0.5)
    ap.add_argument("--out", type=Path, default=None, help="output file (default: stdout)")
    args = ap.parse_args(argv)
    if not 0.0 <= args.difficulty <= 1.0:
        ap.error("difficulty must be in [0, 1]")
    xml = to_mjcf(course(args.export, seed=args.seed, difficulty=args.difficulty))
    if args.out is None:
        sys.stdout.write(xml)
    else:
        args.out.write_text(xml, encoding="utf-8")
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
