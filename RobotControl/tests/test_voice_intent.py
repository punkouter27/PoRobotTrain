"""Voice intent matcher: suggest-only, single joint -> slider, walk words only in the walk sandbox."""

import pytest

from robotcontrol.behaviours.voice_intent import interpret, parse_degrees


@pytest.mark.parametrize(
    "text,kw,kind,target,amp",
    [
        ("nod the head twenty degrees", {}, "behaviour", "neck_nod", 20.0),
        ("turn the head left and right 30", {}, "behaviour", "neck_turn", 30.0),
        ("shake your head no", {}, "behaviour", "neck_turn", None),
        ("wave hello", {"arm_unlocked": True}, "behaviour", "hand_wave", None),
        ("raise the arm 10 degrees", {"arm_unlocked": True}, "behaviour", "arm_raise", 10.0),
        ("move the right elbow 40 degrees", {}, "joint", "right_elbow_joint", None),
        ("right shoulder roll", {}, "joint", "right_shoulder_roll_joint", None),
        ("make me a sandwich", {}, "unknown", None, None),
    ],
)
def test_suggestions(text, kw, kind, target, amp):
    r = interpret(text, **kw)
    assert r["ok"] and r["kind"] == kind
    assert (r["id"] or r["joint"]) == target
    assert r["amplitude"] == amp
    assert r["say"] and r["note"]


def test_amplitude_snaps_to_behaviour_max():
    r = interpret("nod thirty degrees")  # neck_nod is capped at 20 deg
    assert r["id"] == "neck_nod" and r["amplitude"] == 20.0 and "snapped" in r["note"]


def test_arm_behaviour_blocked_until_unlocked():
    r = interpret("wave hello")
    assert r["blocked"] and "unlocked" in r["say"]
    assert not interpret("wave hello", arm_unlocked=True)["blocked"]


def test_stand_locked_joint_is_read_only():
    r = interpret("bend the left knee 20 degrees")
    assert r["kind"] == "joint" and r["joint"] == "left_knee_joint"
    assert r["deg"] is None and "locked" in r["say"]


def test_joint_slider_target_clamped_to_limits():
    r = interpret("right elbow 400 degrees")
    assert r["joint"] == "right_elbow_joint" and r["deg"] == pytest.approx(125.2, abs=0.1)


def test_walk_words_only_when_walk_active():
    assert interpret("walk forward")["kind"] != "walk"
    assert interpret("walk forward", walk_active=True)["walk"] == {"vx": 0.5, "vy": 0.0, "wz": 0.0}
    assert interpret("turn left", walk_active=True)["walk"]["wz"] > 0
    assert interpret("step right", walk_active=True)["walk"]["vy"] < 0
    assert interpret("stop", walk_active=True)["walk"] == {"stop": True}
    # body words still mean a behaviour suggestion, even in the sandbox
    assert interpret("nod your head", walk_active=True)["kind"] == "behaviour"


def test_parse_degrees_words_and_digits():
    assert parse_degrees("twenty five degrees") == 25
    assert parse_degrees("15°") == 15
    assert parse_degrees("no number") is None
