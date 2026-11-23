"""Walk sandbox extras (docs/sim_features.md #2, #5, #9, #12, #15, #18, #19): free_hold, physics overlay, motors,
props, replay, scenarios, fall analysis, and the service endpoints. Deterministic: the sandbox loop is ticked by hand."""

import json

import numpy as np
import pytest

from robotcontrol.behaviours import service as S
from robotcontrol.walk.sandbox import WalkSandbox
from robotcontrol.walk.sim import MAX_CONTACTS, MAX_PROPS, WalkSim

STATE_KEYS = ("seed", "difficulty", "physics", "motors", "thermal", "battery", "action_filter", "props",
              "fall_analysis", "falls_log", "replay", "scenario")  # fmt: skip


@pytest.fixture(scope="module")
def toppled():
    """free_hold on flat ground, a ball from the left at t=0: the robot falls within ~1.5 s."""
    sim = WalkSim("flat", mode="free_hold")
    assert sim.throw("ball", 6.0, 90.0)["ok"]
    for _ in range(90):
        sim.step()
        if sim.falls:
            break
    return sim


def test_free_hold_topples_and_the_fall_is_analysed(toppled):
    sim = toppled
    assert sim.falls == 1 and sim.fall_analysis is not None
    fa = sim.fall_analysis
    assert fa["cause"] in ("cp_outside", "prop_hit", "tilt", "height")
    assert 0.0 <= fa["lead_s"] <= fa["t_fall"] and fa["t_unstable"] <= fa["t_fall"]
    assert fa["history"] and fa["history"][-1]["t"] <= fa["t_fall"] and len(fa["history"]) <= 31
    assert set(fa["history"][0]) == {"t", "cp_margin_m", "tilt_deg", "height"}
    assert list(sim.falls_log)[-1]["cause"] == fa["cause"]


def test_physics_motor_and_prop_blocks(toppled):
    snap = toppled.snapshot()
    ph = snap["physics"]
    assert len(ph["com"]) == 3 and len(ph["com_vel"]) == 3 and len(ph["capture_point"]) == 2
    assert len(ph["contacts"]) <= MAX_CONTACTS
    for c in ph["contacts"]:
        assert set(c) == {"pos", "force", "a", "b"} and len(c["force"]) == 3
    assert len(snap["motors"]["names"]) == 26 and len(snap["motors"]["torque_frac"]) == 26
    assert len(snap["thermal"]["temp_c"]) == 26 and snap["thermal"]["hottest"] in snap["motors"]["names"]
    assert 0 < snap["battery"]["soc_pct"] <= 100 and snap["battery"]["power_w"] >= 60.0
    assert len(snap["action_filter"]["raw"]) == 24
    json.dumps(snap)  # JSON-ready


def test_standing_support_polygon_contains_the_capture_point():
    sim = WalkSim("flat", mode="hold")
    sim.step()
    ph = sim.physics()
    assert len(ph["support"]) >= 3 and ph["cp_margin_m"] is not None and ph["cp_margin_m"] > 0
    feet = {c["a"] for c in ph["contacts"]} | {c["b"] for c in ph["contacts"]}
    assert {"left_foot", "right_foot", "floor"} <= feet
    fz = sum(c["force"][2] for c in ph["contacts"] if "floor" in (c["a"], c["b"]))
    assert abs(fz) == pytest.approx(9.81 * float(sim.m.body_subtreemass[sim.pelvis_id]), rel=0.35)


def test_props_are_capped_and_recycled():
    sim = WalkSim("flat", mode="hold")
    ids = [sim.throw("box" if k % 2 else "ball", 3.0, 45.0 * k)["id"] for k in range(MAX_PROPS + 2)]
    props = sim.props()
    assert len(props) == MAX_PROPS and [p["id"] for p in props] == ids[2:]  # the two oldest were recycled
    ball = next(p for p in props if p["kind"] == "ball")
    assert ball["size"] == [0.1] and len(ball["quat"]) == 4
    assert np.linalg.norm(np.array(ball["pos"][:2]) - sim.base_pos[:2]) == pytest.approx(2.0, abs=0.05)
    with pytest.raises(ValueError):
        sim.throw("piano")
    with pytest.raises(ValueError):
        sim.throw("ball", speed=20)


def test_replay_pauses_the_sim_and_serves_replayed_frames():
    sb = WalkSandbox(mirror_factory=None, log=lambda s: None)
    sb.sim, sb.running = WalkSim("flat", mode="free_hold"), True
    assert sb.state()["replay"]["active"] is False
    for _ in range(100):
        sb._tick_locked()
    t_live = sb.sim.t
    res = sb.start_replay(1.0, 0.5)
    assert res["ok"] and res["frames"] >= 50
    for _ in range(10):
        root, q = sb._tick_locked()
        assert root.shape == (7,) and q.shape == (26,)
    st = sb.state()
    assert sb.sim.t == t_live  # paused
    assert st["replay"]["active"] and st["replay"]["speed"] == 0.5
    assert st["t"] < t_live and st["t"] == pytest.approx(t_live - 1.0 + 10 * 0.02 * 0.5, abs=0.03)
    for _ in range(120):
        sb._tick_locked()
    assert not sb.state()["replay"]["active"] and sb.sim.t > t_live  # resumed
    assert not sb.start_replay(20, 0.5)["ok"] and not sb.start_replay(2, 3.0)["ok"]


def test_auto_replay_on_fall_and_scenario_steps():
    sb = WalkSandbox(mirror_factory=None, log=lambda s: None)
    sb.sim, sb.running = WalkSim("flat", mode="free_hold"), True
    assert sb.replay_post({"auto_on_fall": True}) == {"ok": True, "auto_on_fall": True}
    sb.scenario = {"id": "t", "steps": [{"t": 0.1, "do": "throw", "kind": "box", "speed": 5, "from_deg": 0},
                                        {"t": 0.2, "do": "filter", "alpha": 0.3}],
                   "idx": 0, "t0": 0.0, "duration_s": 0.5, "done": False, "cmd": None}  # fmt: skip
    started = False
    for _ in range(250):
        sb._tick_locked()
        if sb.replay is not None:
            started = True
            break
    assert started and sb.sim.falls >= 1
    st = sb.state()
    assert st["replay"]["auto_on_fall"] and 1.5 < st["replay"]["duration_s"] <= 3.0  # 3 s or all there is
    assert st["scenario"]["steps"] == 2 and st["scenario"]["step"] == 2 and st["scenario"]["done"]
    assert st["action_filter"]["alpha"] == 0.3


@pytest.fixture
def svc():
    s = S.Service(mirror_enabled=False)
    yield s
    if s.walk is not None:
        s.walk.stop()


def test_service_walk_extras(svc):
    st = svc.walk_state()
    assert all(st[k] is None for k in STATE_KEYS) and "procedural" in st["courses"] and "free_hold" in st["modes"]
    assert not svc.walk_post("throw", {"kind": "ball"})["ok"]  # not running
    res = svc.walk_post("start", {"course": "procedural", "mode": "free_hold", "seed": 4, "difficulty": 0.3})
    assert res["ok"] and svc.job == "walk"
    st = svc.walk_state()
    assert st["seed"] == 4 and st["difficulty"] == 0.3 and st["geometry"]["boxes"]
    assert all(k in st for k in STATE_KEYS) and st["physics"] is not None and st["scenario"] is None
    assert svc.walk_post("throw", {"kind": "box", "speed": 3, "from_deg": 90})["ok"]
    assert not svc.walk_post("throw", {"kind": "box", "speed": 30})["ok"]
    assert svc.walk_post("filter", {"alpha": 0.4, "apply": True})["action_filter"]["apply"] is True
    assert not svc.walk_post("filter", {"alpha": 5})["ok"]
    assert svc.walk_post("replay", {"stop": True})["ok"]
    assert svc.walk_post("scenario", {"id": "lab_stress"})["_code"] == 400  # a lab scenario
    assert svc.walk_post("scenario", {"id": "../x"})["_code"] == 404
    res = svc.walk_post("scenario", {"id": "topple"})
    assert res["ok"] and res["scenario"] == "topple" and svc.walk_state()["scenario"]["id"] == "topple"
    assert svc.walk_post("scenario", {"stop": True})["ok"]
    assert svc.walk_state()["scenario"]["done"]
    assert svc.walk_post("start", {"course": "flat", "mode": "fly"})["ok"] is False and svc.job == "walk"  # still on
    assert svc.stop()["ok"] and svc.job is None
