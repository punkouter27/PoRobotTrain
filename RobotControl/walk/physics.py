"""Pure helpers for the walk sandbox's physics overlay and fall analysis (docs/sim_features.md #2, #19).

No MuJoCo here: support polygon (convex hull), signed capture-point margin, and the fall analysis over a recorded
history, so they are unit-testable on plain numbers.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

GRAVITY = 9.81

Point = Tuple[float, float]


def capture_point(com: Sequence[float], com_vel: Sequence[float]) -> Tuple[float, float]:
    """Instantaneous capture point of the linear inverted pendulum: com_xy + com_vel_xy / sqrt(g / com_z)."""
    z = max(float(com[2]), 1e-3)
    w = math.sqrt(GRAVITY / z)
    return float(com[0]) + float(com_vel[0]) / w, float(com[1]) + float(com_vel[1]) / w


def convex_hull(points: Sequence[Sequence[float]]) -> List[Point]:
    """Counter-clockwise convex hull (Andrew's monotone chain); 0, 1 or 2 points come back as they are (deduped)."""
    pts = sorted({(round(float(p[0]), 5), round(float(p[1]), 5)) for p in points})
    if len(pts) <= 2:
        return pts

    def cross(o: Point, a: Point, b: Point) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: List[Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: List[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _seg_dist(p: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - (a[0] + u * dx), p[1] - (a[1] + u * dy))


def signed_margin(p: Sequence[float], hull: Sequence[Point]) -> Optional[float]:
    """Signed distance from p to the support hull: + inside (distance to the nearest edge), - outside. None if the
    hull is empty. A point or segment support has no inside, so the margin is minus the distance to it."""
    if not hull:
        return None
    q = (float(p[0]), float(p[1]))
    if len(hull) == 1:
        return -math.hypot(q[0] - hull[0][0], q[1] - hull[0][1])
    if len(hull) == 2:
        return -_seg_dist(q, hull[0], hull[1])
    n = len(hull)
    d = min(_seg_dist(q, hull[i], hull[(i + 1) % n]) for i in range(n))
    inside = all(
        (hull[(i + 1) % n][0] - hull[i][0]) * (q[1] - hull[i][1])
        - (hull[(i + 1) % n][1] - hull[i][1]) * (q[0] - hull[i][0])
        >= 0
        for i in range(n)
    )
    return d if inside else -d


def analyse_fall(
    t_fall: float,
    samples: Sequence[dict],
    tilt_limit_deg: float,
    height_limit: float,
    prop_hits: Sequence[Tuple[float, str]] = (),
) -> dict:
    """Why did it fall? ``samples``: chronological dicts {t, cp_margin_m (None = no support), tilt_deg, height} up to
    the fall (any rate). ``prop_hits``: (t, kind) of prop-robot contacts.

    t_unstable = start of the final continuous run of cp_margin < 0 (no support counts as outside) that reaches the
    fall; without one, the start of the final run of tilt > 15 deg (cause tilt) or the fall time (cause height).
    A prop that touched the robot within 1 s before t_unstable makes the cause prop_hit.
    """
    s = [x for x in samples if x["t"] <= t_fall + 1e-9]
    cause, reason, t_unstable = "", "", t_fall

    def outside(x: dict) -> bool:
        return x.get("cp_margin_m") is None or x["cp_margin_m"] < 0

    if s and outside(s[-1]):
        k = len(s) - 1
        while k > 0 and outside(s[k - 1]):
            k -= 1
        t_unstable = s[k]["t"]
        early = [x["cp_margin_m"] for x in s[k:] if x["t"] <= t_unstable + 0.3 and x.get("cp_margin_m") is not None]
        cm = abs(min(early)) * 100.0 if early else None
        cause = "cp_outside"
        reason = (
            f"capture point left the feet ({cm:.0f} cm outside)"
            if cm is not None
            else "the feet lost contact with the ground"
        )
    else:
        tilted = s and s[-1]["tilt_deg"] > 15.0
        if tilted:
            k = len(s) - 1
            while k > 0 and s[k - 1]["tilt_deg"] > 15.0:
                k -= 1
            t_unstable = s[k]["t"]
        last = s[-1] if s else {"tilt_deg": 0.0, "height": 1.0}
        if last["tilt_deg"] >= tilt_limit_deg or (tilted and last["height"] >= height_limit):
            cause, reason = "tilt", f"the body tipped over ({last['tilt_deg']:.0f} deg) with the capture point inside"
        else:
            cause, reason = "height", f"the pelvis dropped to {last['height']:.2f} m (legs gave way)"
    hit = [(t, k) for t, k in prop_hits if t_unstable - 1.0 <= t <= t_unstable + 0.1]
    if hit:
        t_hit, kind = hit[-1]
        cause = "prop_hit"
        lead = max(0.0, t_unstable - t_hit)
        reason = f"hit by a {kind} {lead:.1f} s before it became unstable; " + reason
    return {
        "t_fall": round(float(t_fall), 3),
        "t_unstable": round(float(t_unstable), 3),
        "lead_s": round(float(t_fall - t_unstable), 3),
        "reason": reason,
        "cause": cause,
    }
