"""Taught head poses (docs/sim_features.md #14): store, validation, pose_<id> skills everywhere, endpoints.
The store is redirected to tmp_path, and the skill registry is restored after each test."""

import math

import numpy as np
import pytest

from robotcontrol.behaviours import head_skills as hs
from robotcontrol.behaviours import service as S
from robotcontrol.behaviours import stand_behaviours as SB
from robotcontrol.behaviours import taught_poses as TP
from robotcontrol.behaviours.neck_turn import SweepParams
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.training import validate_head_skill as V

BUILTIN = ["hold_centre", "look_at", "nod_yes", "shake_no", "glance_around"]


@pytest.fixture
def store(tmp_path, monkeypatch):
    monkeypatch.setattr(TP, "POSES_PATH", tmp_path / "taught_poses.json")
    TP.refresh()
    yield tmp_path / "taught_poses.json"
    TP.refresh()  # the tmp store is gone after the test: back to the built-in skills
    monkeypatch.undo()
    TP.refresh()


def test_builtin_skills_unchanged(store):
    assert list(hs.SKILLS)[:5] == BUILTIN and all(hs.SKILLS[s].pose is None for s in BUILTIN)
    assert hs.BUILTIN_SKILLS == tuple(BUILTIN)
    np.testing.assert_allclose(hs.reference(hs.SKILLS["hold_centre"], np.zeros((1, 3))), [[0.0, 0.0]])


def test_add_list_delete_and_registry(store):
    p = TP.add_pose("Look left-up", 20, -10)
    assert p == {"id": "look_left_up", "name": "Look left-up", "yaw_deg": 20.0, "pitch_deg": -10.0,
                 "skill": "pose_look_left_up"}  # fmt: skip
    assert store.exists() and TP.list_poses() == [p]
    sk = hs.SKILLS["pose_look_left_up"]
    assert sk.powered == (hs.HEAD_PITCH, hs.HEAD_YAW) and sk.pose == pytest.approx(
        (math.radians(20), math.radians(-10))
    )
    assert "pose_look_left_up" in hs.SKILL_CLASSES and SB.BEHAVIOUR_SPECS["pose_look_left_up"].kind == "trained"
    ref = hs.reference(sk, np.zeros((3, 3)))
    np.testing.assert_allclose(ref, np.tile([math.radians(-10), math.radians(20)], (3, 1)))  # pitch, yaw order
    assert S.parse_params({"behaviour": "pose_look_left_up"})["behaviour"] == "pose_look_left_up"
    with pytest.raises(ValueError, match="already exists"):
        TP.add_pose("look left up", 0, 0)
    assert TP.delete_pose("look_left_up") and not TP.delete_pose("look_left_up")
    assert "pose_look_left_up" not in hs.SKILLS and "pose_look_left_up" not in SB.BEHAVIOUR_SPECS
    assert list(hs.SKILLS) == BUILTIN


@pytest.mark.parametrize(
    "name,yaw,pitch",
    [("", 0, 0), ("x" * 33, 0, 0), ("bad/name", 0, 0), ("ok", 31, 0), ("ok", 0, -16), ("ok", 0, 21), ("ok", "a", 0)],
)
def test_validation(store, name, yaw, pitch):
    with pytest.raises(ValueError):
        TP.add_pose(name, yaw, pitch)
    assert TP.load_poses() == []


def test_runtime_behaviour_holds_the_pose(store):
    TP.add_pose("right", -25, 5)
    jm = JointMap.load()
    beh = hs.SKILL_CLASSES["pose_right"](jm, SweepParams(amplitude_deg=10, duration_s=3), policy=lambda o: np.zeros(2))
    np.testing.assert_allclose(beh.command_at(1.0), 0.0)  # command-driven like hold_centre
    np.testing.assert_allclose(beh.reference_at(1.0), [math.radians(5), math.radians(-25)])
    np.testing.assert_allclose(beh.reference_at(beh.run_s + 0.5), [math.radians(5), math.radians(-25)])  # settles there
    assert beh.p.max_err_deg == 15.0  # wide tracking window: the pose starts with a move
    assert "size" not in beh.describe()
    assert V.bars("pose_right") == V.POSE_BARS and V.bars("look_at") == V.BARS["look_at"]
    assert V._duration(hs.SKILLS["pose_right"]) == 4.0


def test_training_env_accepts_a_pose_skill(store):
    from robotcontrol.training.head_skill_env import HeadSkillConfig, HeadSkillEnv

    TP.add_pose("down", 0, 15)
    env = HeadSkillEnv(HeadSkillConfig(skill="pose_down", num_envs=2, nthread=1))
    try:
        np.testing.assert_allclose(env.cmd, 0.0)
        np.testing.assert_allclose(env.reference(), np.tile([math.radians(15), 0.0], (2, 1)))
        obs, rew, done, info = env.step(np.zeros((2, 2)))
        assert obs.shape == (2, hs.obs_size(hs.SKILLS["pose_down"])) and np.isfinite(rew).all()
    finally:
        env.close()


def test_service_pose_endpoints(store):
    svc = S.Service(mirror_enabled=False)
    presets = svc.poses_get()["poses"]
    assert all("id" not in p for p in presets)  # the console's preset poses, unchanged
    res = svc.poses_post({"name": "Nod down", "yaw_deg": 0, "pitch_deg": 12})
    assert res["ok"] and res["pose"]["skill"] == "pose_nod_down"
    assert svc.poses_post({"name": "Nod down", "yaw_deg": 0, "pitch_deg": 12})["_code"] == 400
    got = svc.poses_get()["poses"]
    taught = [p for p in got if "id" in p]
    assert len(got) == len(presets) + 1 and taught[0]["q_deg"] == {"head_yaw_joint": 0.0, "head_pitch_joint": 12.0}
    assert any(s["id"] == "pose_nod_down" for s in svc.skills()["skills"])
    assert svc.taught_pose_preview({"name": "unknown_pose"}) is None
    assert svc.poses_delete({"id": "nod_down"})["ok"]
    assert svc.poses_delete({"id": "nod_down"})["_code"] == 404
