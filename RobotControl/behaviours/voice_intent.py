"""Plain-words intent matcher for voice commands (docs/service_api.md section 1).

Deterministic and lexical, so the Unreal panel (which has no browser-side MiniLM) gets the same kinds of
suggestion as the Blazor assistant: a behaviour + sweep size, a single joint's slider, a walk-sandbox velocity
command, or "unknown". It only *suggests*. Nothing here starts, checks or runs a behaviour; the caller fills in a
selection, and Steps 1 + 2 + the confirm dialog still gate any robot run. A request about one joint is answered
with that joint's slider, never with a multi-joint behaviour.
"""

from __future__ import annotations

import math
import re
from typing import Optional

from ..configs.joint_map import JointMap
from .stand_behaviours import BEHAVIOUR_SPECS

SIZES = (10.0, 20.0, 30.0)

# Walk sandbox velocity steps (simulation only), see docs/service_api.md section 1.
WALK_VX, WALK_VX_FAST, WALK_VX_SLOW, WALK_VY, WALK_WZ = 0.5, 1.0, 0.25, 0.3, 0.6

NUMBER_WORDS = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
    "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
    "seventeen": 17, "eighteen": 18, "nineteen": 19, "twenty": 20, "thirty": 30, "forty": 40, "fifty": 50,
}  # fmt: skip
SAY_NUMBER = {10.0: "ten", 20.0: "twenty", 30.0: "thirty"}

# Words that point at each behaviour. Weighted: a 2 is a near-certain cue, a 1 is supporting evidence.
BEHAVIOUR_CUES: dict[str, dict[str, int]] = {
    "neck_turn": {"turn": 1, "pan": 2, "shake": 2, "yaw": 2, "no": 1, "side": 1, "look left": 2, "look right": 2,
                  "left and right": 2, "head left": 2, "head right": 2, "neck turn": 3, "swivel": 2, "rotate": 1},
    "neck_nod": {"nod": 3, "yes": 1, "tilt": 2, "pitch": 1, "look up": 2, "look down": 2, "up and down": 2,
                 "neck nod": 3, "bow": 1},
    "arm_raise": {"raise": 2, "lift": 2, "arm up": 2, "reach": 2, "forward": 1, "shoulder": 1, "arm raise": 3,
                  "point": 1},
    "hand_wave": {"wave": 3, "hello": 2, "hi": 2, "greet": 2, "goodbye": 2, "bye": 2, "hand": 1, "hand wave": 3},
}  # fmt: skip
HEAD_WORDS = ("head", "neck", "look")
ARM_WORDS = ("arm", "hand", "shoulder", "wave", "raise", "lift", "reach")
JOINT_WORDS = ("elbow", "wrist", "knee", "hip", "ankle", "waist", "slider")
WALK_WORDS = ("walk", "go", "forward", "ahead", "back", "backward", "backwards", "reverse", "faster", "slower",
              "run", "stop", "halt", "left", "right", "turn", "strafe", "sidestep", "step")  # fmt: skip


def _words(text: str) -> str:
    return " " + re.sub(r"[^a-z0-9.\- ]+", " ", text.lower()).strip() + " "


def _has(t: str, phrase: str) -> bool:
    return f" {phrase} " in t


def parse_degrees(text: str) -> Optional[float]:
    """First number in the text, digits or words ("twenty five" = 25)."""
    m = re.search(r"(-?\d+(?:\.\d+)?)", text)
    if m:
        return abs(float(m.group(1)))
    toks = _words(text).split()
    for i, w in enumerate(toks):
        if w in NUMBER_WORDS:
            v = NUMBER_WORDS[w]
            if v >= 20 and i + 1 < len(toks) and toks[i + 1] in NUMBER_WORDS and NUMBER_WORDS[toks[i + 1]] < 10:
                v += NUMBER_WORDS[toks[i + 1]]
            return float(v)
    return None


def _strip_joint(name: str) -> str:
    return name[: -len("_joint")] if name.endswith("_joint") else name  # Python 3.8: no str.removesuffix


def joint_label(name: str) -> str:
    base = _strip_joint(name).replace("_", " ")
    return base[0].upper() + base[1:]


def _result(kind: str, note: str, say: str, **kw) -> dict:
    out = {
        "ok": True,
        "kind": kind,
        "id": None,
        "label": None,
        "amplitude": None,
        "joint": None,
        "deg": None,
        "walk": None,
        "blocked": False,
        "score": 0.0,
        "note": note,
        "say": say,
    }
    out.update(kw)
    return out


def _unknown(score: float = 0.0) -> dict:
    return _result(
        "unknown",
        "No stand-safe behaviour matches that. Try 'turn the head 20 degrees'.",
        "Sorry, I did not catch a safe motion. Try: turn the head twenty degrees.",
        score=score,
    )


def _walk_intent(t: str) -> Optional[dict]:
    """Velocity command for the walk sandbox, or None when the words are not about walking."""
    if not any(_has(t, w) for w in WALK_WORDS):
        return None
    if _has(t, "stop") or _has(t, "halt") or _has(t, "stand still"):
        return _result("walk", "Walk sandbox: stop.", "Stopping.", walk={"stop": True}, score=1.0)
    vx = vy = wz = 0.0
    say = []
    if _has(t, "turn") or _has(t, "rotate"):
        if _has(t, "left"):
            wz, say = WALK_WZ, ["Turning left."]
        elif _has(t, "right"):
            wz, say = -WALK_WZ, ["Turning right."]
    elif _has(t, "left"):
        vy, say = WALK_VY, ["Stepping left."]
    elif _has(t, "right"):
        vy, say = -WALK_VY, ["Stepping right."]
    if any(_has(t, w) for w in ("back", "backward", "backwards", "reverse")):
        vx, say = -WALK_VX, say + ["Walking backward."]
    elif _has(t, "faster") or _has(t, "run"):
        vx, say = WALK_VX_FAST, say + ["Walking faster."]
    elif _has(t, "slower") or _has(t, "slowly"):
        vx, say = WALK_VX_SLOW, say + ["Walking slowly."]
    elif any(_has(t, w) for w in ("forward", "ahead", "walk", "go")) and wz == 0.0 and vy == 0.0:
        vx, say = WALK_VX, ["Walking forward."]
    if vx == vy == wz == 0.0:
        return None
    note = f"Walk sandbox: vx {vx:+.2f} m/s, vy {vy:+.2f} m/s, turn {wz:+.2f} rad/s."
    return _result("walk", note, " ".join(say), walk={"vx": vx, "vy": vy, "wz": wz}, score=1.0)


def _joint_intent(t: str, jm: JointMap) -> Optional[dict]:
    """A single named joint: '<side> <part> <axis>' (e.g. 'left elbow', 'right shoulder roll', 'waist yaw')."""
    side = "left" if _has(t, "left") and not _has(t, "right") else "right" if _has(t, "right") else None
    best, best_score = None, 0
    for name in jm.names:
        parts = _strip_joint(name).split("_")
        score = 0
        body = [p for p in parts if p not in ("left", "right")]
        if not _has(t, body[0]):
            continue
        score += 2
        score += sum(1 for p in body[1:] if _has(t, p))
        if parts[0] in ("left", "right"):
            if side is None:
                score -= 1  # ambiguous side: still allowed, but prefer the right side below
            elif parts[0] != side:
                continue
            else:
                score += 1
        if score > best_score or (score == best_score and best is not None and name.startswith("right_")):
            best, best_score = name, score
    if best is None:
        return None
    i = jm.names.index(best)
    label = joint_label(best)
    locked = i in set(jm.groups.get("left_leg", []) + jm.groups.get("right_leg", []) + jm.groups.get("waist", []))
    deg = parse_degrees(t)
    if deg is not None and not locked:
        lo, hi = math.degrees(float(jm.limits_lo[i])), math.degrees(float(jm.limits_hi[i]))
        deg = round(min(hi, max(lo, deg)), 1)
    if locked:
        return _result(
            "joint",
            f"{label} is stand-locked (read-only); it can be watched, not moved.",
            f"{label} is locked on the stand.",
            joint=best,
            label=label,
            score=0.6,
        )
    return _result(
        "joint",
        f"Single joint: use the {label} slider (arm write must be enabled; only that joint is powered).",
        f"Use the {label} slider.",
        joint=best,
        label=label,
        deg=deg,
        score=0.6,
    )


def interpret(text: str, arm_unlocked: bool = False, walk_active: bool = False, jm: Optional[JointMap] = None) -> dict:
    """Plain words -> one suggestion dict (docs/service_api.md section 1). Never starts anything."""
    t = _words(text or "")
    if not t.strip():
        return _result("unknown", "Say or type what you want the robot to do.", "I did not hear anything.")
    jm = jm or JointMap.load()

    about_body = any(_has(t, w) for w in HEAD_WORDS + ARM_WORDS + JOINT_WORDS + ("nod",))
    if walk_active and not about_body:
        w = _walk_intent(t)
        if w is not None:
            return w

    if any(_has(t, w) for w in JOINT_WORDS) or re.search(r"shoulder (roll|yaw)", t):
        j = _joint_intent(t, jm)
        if j is not None:
            return j

    scores = {bid: sum(wt for cue, wt in cues.items() if _has(t, cue)) for bid, cues in BEHAVIOUR_CUES.items()}
    if any(_has(t, w) for w in HEAD_WORDS):
        scores["neck_turn"] += 1 if not _has(t, "nod") else 0
        scores["arm_raise"] -= 1
        scores["hand_wave"] -= 1
    if _has(t, "arm") or _has(t, "hand"):
        scores["neck_turn"] -= 1
        scores["neck_nod"] -= 1
    bid = max(scores, key=lambda k: scores[k])
    if scores[bid] < 2:
        return _unknown(score=max(0.0, scores[bid] / 5.0))

    spec = BEHAVIOUR_SPECS[bid]
    is_arm = bid in ("arm_raise", "hand_wave")
    deg = parse_degrees(t)
    allowed = [s for s in SIZES if s <= spec.max_deg + 1e-6]
    amplitude = min(allowed, key=lambda s: abs(s - deg)) if deg is not None and allowed else None
    note = f"{spec.label}{f' at {amplitude:g}°' if amplitude else ''}. "
    note += "Still needs Step 1 (virtual) and Step 2 (check) before any run."
    if deg is not None and amplitude is not None and abs(amplitude - deg) > 0.5:
        note += f" ({deg:g}° snapped to the nearest allowed size.)"
    left_only = _has(t, "left") and not _has(t, "right")
    if is_arm and left_only:
        note = f"Only right-arm behaviours exist. For the left arm use the left-arm sliders. Closest: {spec.label}."
    blocked = is_arm and not arm_unlocked
    if blocked:
        note += " Tick 'Unlock arm motions' first."
        say = f"{spec.label} needs arm motions unlocked first."
    else:
        size = f", {SAY_NUMBER.get(amplitude, f'{amplitude:g}')} degrees" if amplitude else ""
        say = f"{spec.label}{size}, selected. Run it on the virtual robot first."
    return _result(
        "behaviour",
        note,
        say,
        id=bid,
        label=spec.label,
        amplitude=amplitude,
        blocked=blocked,
        score=min(1.0, scores[bid] / 5.0),
    )
