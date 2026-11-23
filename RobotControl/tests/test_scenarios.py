"""Scenario scripts (docs/sim_features.md #18): the shipped files are valid; the validator refuses bad ones."""

import copy
import json

import pytest

from robotcontrol.sim_features import scenarios as S

SHIPPED = {"push_test": "walk", "topple": "walk", "procedural_walk": "walk", "lab_stress": "lab", "lab_calm": "lab"}


def test_shipped_scenarios_load():
    listed = {s["id"]: s for s in S.list_scenarios()}
    assert set(SHIPPED) <= set(listed)
    for sid, target in SHIPPED.items():
        sc = S.load(sid)
        assert sc["target"] == target and listed[sid]["duration_s"] == sc["duration_s"]
        assert set(listed[sid]) == {"id", "name", "target", "description", "duration_s"}
    assert S.load("topple")["mode"] == "free_hold"
    assert S.load("lab_stress")["skill"] == "look_at"


def _bad(sc, **change):
    out = copy.deepcopy(sc)
    out.update(change)
    return out


def test_validator_refuses_bad_scripts(tmp_path):
    good = S.load("push_test")
    cases = [
        _bad(good, target="moon"),
        _bad(good, course="mars"),
        _bad(good, mode="dance"),
        _bad(good, steps=[{"t": 1.0, "do": "fly"}]),
        _bad(good, steps=[{"t": 99.0, "do": "reset"}]),
        _bad(good, steps=[{"t": 2.0, "do": "reset"}, {"t": 1.0, "do": "reset"}]),
        _bad(good, steps=[{"t": 1.0, "do": "throw", "kind": "ball", "speed": 50}]),
        _bad(good, steps=[{"t": 1.0, "do": "replay", "seconds": 30}]),
        _bad(good, seed="x"),
        _bad(good, id="Bad Id"),
    ]
    lab = S.load("lab_stress")
    cases += [
        _bad(lab, steps=[{"t": 0, "do": "delay", "steps": 9}]),
        _bad(lab, steps=[{"t": 0, "do": "strength", "pct": 10}]),
        _bad(lab, steps=[{"t": 0, "do": "throw"}]),
        {k: v for k, v in lab.items() if k != "skill"},
    ]
    for sc in cases:
        with pytest.raises(ValueError):
            S.validate(sc)
    (tmp_path / "broken.json").write_text("{", encoding="utf-8")
    (tmp_path / "other.json").write_text(json.dumps(_bad(good, id="mismatch")), encoding="utf-8")
    (tmp_path / "ok_one.json").write_text(json.dumps(_bad(good, id="ok_one")), encoding="utf-8")
    assert [s["id"] for s in S.list_scenarios(tmp_path)] == ["ok_one"]
    with pytest.raises(ValueError):
        S.load("../push_test")
    with pytest.raises(ValueError):
        S.load("missing", tmp_path)
