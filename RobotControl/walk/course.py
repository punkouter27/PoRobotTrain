"""Walk sandbox courses: checkpoints and static boxes, in the MuJoCo world frame (metres, z up, quat w x y z).

Single source of truth for the course: the simulator adds these boxes to ``scene_ground.xml`` at load time, and
``GET /walk/state`` sends the same geometry to the Blazor and Unreal clients, which only draw it.
The robot starts at the origin facing +x. A lap is the checkpoints in order; the last one is back at the start.
"""

from __future__ import annotations

import math

CHECKPOINT_RADIUS = 0.5


def _yaw_quat(deg: float) -> list[float]:
    h = math.radians(deg) / 2
    return [round(math.cos(h), 6), 0.0, 0.0, round(math.sin(h), 6)]


_LOOP_CHECKPOINTS = [
    [3.0, 0.0, 0.0],
    [5.0, 2.5, 0.0],
    [3.0, 5.0, 0.0],
    [0.0, 5.0, 0.0],
    [-2.0, 2.5, 0.0],
    [0.0, 0.0, 0.0],
]

COURSES: dict[str, dict] = {
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
}  # fmt: skip


def course(name: str) -> dict:
    if name not in COURSES:
        raise ValueError(f"unknown course {name!r}; choose one of {sorted(COURSES)}")
    c = COURSES[name]
    return {"checkpoints": c["checkpoints"], "checkpoint_radius": CHECKPOINT_RADIUS, "boxes": c["boxes"]}
