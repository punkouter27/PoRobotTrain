"""Procedural walk course (docs/sim_features.md #3): seeded, deterministic, clear start, MJCF export."""

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from robotcontrol.walk import course as C
from robotcontrol.walk.physics import convex_hull, signed_margin


def _footprint(b):
    """Floor footprint corners of a (yawed) box."""
    w, x, y, z = b["quat"]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = b["size"][0], b["size"][1]
    return convex_hull(
        [(b["pos"][0] + c * u - s * v, b["pos"][1] + s * u + c * v) for u in (-hx, hx) for v in (-hy, hy)]
    )


def _path(c):
    return [[0.0, 0.0]] + [p[:2] for p in c["checkpoints"]]


@pytest.mark.parametrize("seed", [0, 1, 7, 42])
@pytest.mark.parametrize("difficulty", [0.0, 0.5, 1.0])
def test_procedural_course_rules(seed, difficulty):
    c = C.course("procedural", seed=seed, difficulty=difficulty)
    assert c == C.course("procedural", seed=seed, difficulty=difficulty)  # same seed, same course
    assert c["checkpoints"][-1] == [0.0, 0.0, 0.0] and c["checkpoint_radius"] == C.CHECKPOINT_RADIUS
    path = _path(c)
    names = set()
    for b in c["boxes"]:
        assert b["name"] not in names
        names.add(b["name"])
        assert np.isclose(np.linalg.norm(b["quat"]), 1.0, atol=1e-4)
        assert -signed_margin((0.0, 0.0), _footprint(b)) >= C.CLEAR_RADIUS - 1e-6  # first 1.5 m stays clear
        if "plank" not in b["name"]:  # only low planks lie on the path
            assert C._path_dist(tuple(b["pos"][:2]), path) > 0.3
        else:
            assert b["size"][2] * 2 <= 0.02 + 1e-9
    stairs = [b for b in c["boxes"] if "stairs" in b["name"]]
    for b in stairs:
        assert b["size"][2] * 2 <= 3 * (0.02 + 0.10 * difficulty) + 1e-6


def test_difficulty_and_seed_change_the_course():
    easy, hard = C.course("procedural", 3, 0.0), C.course("procedural", 3, 1.0)
    assert len(hard["boxes"]) > len(easy["boxes"])
    assert C.course("procedural", 3, 0.5) != C.course("procedural", 4, 0.5)
    assert "procedural" in C.COURSES and C.course("flat") == C.course("flat", seed=9)


def test_mjcf_export_compiles(tmp_path):
    out = tmp_path / "terrain.xml"
    assert C.main(["--export", "procedural", "--seed", "5", "--difficulty", "0.7", "--out", str(out)]) == 0
    root = ET.parse(out).getroot()
    geoms = root.findall("./worldbody/geom")
    assert root.tag == "mujoco" and len(geoms) == len(C.course("procedural", 5, 0.7)["boxes"])
    m = mujoco.MjModel.from_xml_path(str(out))
    assert m.ngeom == len(geoms)
    with pytest.raises(SystemExit):
        C.main(["--export", "procedural", "--difficulty", "2"])
