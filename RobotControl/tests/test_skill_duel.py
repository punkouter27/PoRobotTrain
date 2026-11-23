"""Policy-vs-policy duel (docs/sim_features.md #10): headless stand scene, lab disturbances, scores, endpoint."""

import numpy as np
import pytest

from robotcontrol.behaviours import service as S
from robotcontrol.tools import skill_duel as D

SHORT = {"id": "t", "target": "lab", "skill": "look_at", "size_deg": 15, "duration_s": 1.6, "seed": 3,
         "steps": [{"t": 0, "do": "target", "yaw_deg": 10, "pitch_deg": 5}, {"t": 0.2, "do": "push", "yaw": 1, "pitch": 0},
                   {"t": 0.4, "do": "noise", "deg": 1.0}, {"t": 0.6, "do": "delay", "steps": 2},
                   {"t": 0.8, "do": "strength", "pct": 50}]}  # fmt: skip


def _p_controller(gain):
    """A stand-in policy: action steering the head towards the look_at command (obs layout of head_skills)."""

    def pol(obs):
        k = 2
        q_rel, cmd = obs[:k], obs[3 * k : 3 * k + 3]
        want = np.array([cmd[1], cmd[0]])  # powered order: pitch, yaw
        return gain * (want - q_rel) / np.array([0.30, 0.25])

    return pol


def test_run_one_scores_and_disturbances_matter():
    calm = dict(SHORT, steps=SHORT["steps"][:1])
    good = D.run_one("look_at", None, calm, policy=_p_controller(1.0))
    assert set(good) == {"version", "err_mean_deg", "err_max_deg", "torque_rms", "jitter", "energy_j", "would_stop",
                         "score"}  # fmt: skip
    lazy = D.run_one("look_at", None, calm, policy=lambda obs: np.zeros(2))
    assert lazy["err_mean_deg"] > good["err_mean_deg"]  # a policy that does nothing misses the target
    pushed = D.run_one("look_at", None, SHORT, policy=_p_controller(1.0))
    assert pushed["err_max_deg"] > good["err_max_deg"]  # the 0.25 s shove moves the head
    again = D.run_one("look_at", None, SHORT, policy=_p_controller(1.0))
    assert again == pushed  # seeded: identical noise, identical result


def test_duel_winner_and_why(monkeypatch):
    scores = iter([{"score": 1.0}, {"score": 2.0}])
    base = {"version": 1, "err_mean_deg": 1.0, "err_max_deg": 2.0, "torque_rms": 0.1, "jitter": 0.1, "energy_j": 1.0,
            "would_stop": ""}  # fmt: skip
    monkeypatch.setattr(D, "run_one", lambda sid, v, sc: dict(base, **next(scores), err_mean_deg=1.0 + (v or 0)))
    res = D.duel("look_at", None, 2, "lab_calm")
    assert res["winner"] == "a" and res["why"].startswith("a tracks 2.00 deg closer")
    with pytest.raises(ValueError):
        D.duel("look_at", None, None, "topple")  # a walk scenario


@pytest.mark.skipif(not (D.policy_dir("look_at", None) / "policy.onnx").exists(), reason="no trained look_at")
def test_service_duel_current_vs_current():
    svc = S.Service(mirror_enabled=False)
    res = svc.skill_duel({"skill": "look_at", "a": None, "b": None, "scenario": "lab_calm"})
    assert res["ok"] and res["result"]["winner"] == "tie" and res["result"]["a"] == res["result"]["b"]
    assert svc.skill_duel({"skill": "dance"})["_code"] == 400
    assert svc.skill_duel({"skill": "look_at", "a": 999})["_code"] == 400  # no such version
