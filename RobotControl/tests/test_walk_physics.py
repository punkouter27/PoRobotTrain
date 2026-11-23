"""Support polygon, capture point margin and fall analysis helpers (docs/sim_features.md #2, #19)."""

import math

import pytest

from robotcontrol.walk.physics import analyse_fall, capture_point, convex_hull, signed_margin

SQUARE = [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0.2, 0.8)]


def test_convex_hull_and_signed_margin():
    hull = convex_hull(SQUARE)
    assert sorted(hull) == [(0, 0), (0, 1), (1, 0), (1, 1)]
    assert signed_margin((0.5, 0.5), hull) == pytest.approx(0.5)
    assert signed_margin((0.9, 0.5), hull) == pytest.approx(0.1)
    assert signed_margin((1.3, 0.5), hull) == pytest.approx(-0.3)
    assert signed_margin((2.0, 2.0), hull) == pytest.approx(-math.sqrt(2))
    assert signed_margin((0, 0), []) is None
    assert signed_margin((1, 0), convex_hull([(0, 0)])) == pytest.approx(-1)
    assert signed_margin((0.5, 1), convex_hull([(0, 0), (1, 0)])) == pytest.approx(-1)


def test_capture_point():
    assert capture_point([0, 0, 0.981], [0, 0, 0]) == (0.0, 0.0)
    x, y = capture_point([1.0, 2.0, 0.981], [0.5, -1.0, 0.0])  # omega = sqrt(9.81 / 0.981) = sqrt(10)
    assert x == pytest.approx(1.0 + 0.5 / math.sqrt(10)) and y == pytest.approx(2.0 - 1.0 / math.sqrt(10))


def _hist(margins, tilts=None, heights=None, dt=0.1):
    n = len(margins)
    tilts = tilts or [1.0] * n
    heights = heights or [0.7] * n
    return [{"t": round(k * dt, 3), "cp_margin_m": m, "tilt_deg": a, "height": h}
            for k, (m, a, h) in enumerate(zip(margins, tilts, heights))]  # fmt: skip


def test_fall_analysis_capture_point_run():
    h = _hist([0.05, 0.03, -0.01, 0.02, 0.01, -0.06, -0.1, -0.2, None, -0.5])
    res = analyse_fall(0.9, h, 60.0, 0.45)
    assert res["cause"] == "cp_outside" and res["t_unstable"] == 0.5 and res["lead_s"] == pytest.approx(0.4)
    assert "capture point left the feet (20 cm outside)" == res["reason"]  # worst in the first 0.3 s


def test_fall_analysis_tilt_height_and_prop_hit():
    h = _hist([0.05] * 10, tilts=[1, 1, 5, 20, 30, 40, 50, 55, 58, 61])
    res = analyse_fall(0.9, h, 60.0, 0.45)
    assert res["cause"] == "tilt" and res["t_unstable"] == 0.3
    h2 = _hist([0.05] * 5, heights=[0.7, 0.6, 0.5, 0.46, 0.4])
    assert analyse_fall(0.4, h2, 60.0, 0.45)["cause"] == "height"
    hit = analyse_fall(0.9, _hist([0.05, 0.05, 0.05, -0.02, -0.1, -0.3, -0.4, -0.5, -0.6, -0.7]), 60.0, 0.45,
                       prop_hits=[(0.1, "ball")])  # fmt: skip
    assert hit["cause"] == "prop_hit" and hit["reason"].startswith("hit by a ball 0.2 s before")
    old = analyse_fall(0.9, _hist([0.05, 0.05, 0.05, -0.02, -0.1, -0.3, -0.4, -0.5, -0.6, -0.7]), 60.0, 0.45,
                       prop_hits=[(-1.5, "box")])  # fmt: skip
    assert old["cause"] == "cp_outside"  # more than 1 s before t_unstable: not the cause
