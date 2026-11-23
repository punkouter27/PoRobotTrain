"""Trained head skills: shared training/runtime contract, safety envelope, validation gate, service and kit wiring."""

import io
import math
import tarfile

import numpy as np
import pytest
import yaml

from robotcontrol.behaviours import head_skills as hs
from robotcontrol.behaviours import service as S
from robotcontrol.behaviours.neck_turn import SweepParams
from robotcontrol.behaviours.stand_behaviours import BEHAVIOUR_SPECS
from robotcontrol.configs.joint_map import JointMap

HEAD = {hs.HEAD_PITCH, hs.HEAD_YAW}


def _q0(jm):
    return dict(zip(jm.names, jm.defaults.tolist()))


class FakePolicy:
    """Returns a constant action and counts calls (the runtime must call it at 50 Hz, not at the backend rate)."""

    def __init__(self, k, a=0.0):
        self.k, self.a, self.calls, self.last_obs = k, a, 0, None

    def __call__(self, obs):
        self.calls += 1
        self.last_obs = obs
        return np.full(self.k, self.a, dtype=np.float32)


def _beh(skill_id, amp=10.0, dur=4.0, a=0.0, check_tracking=True):
    jm = JointMap.load()
    sk = hs.SKILLS[skill_id]
    pol = FakePolicy(len(sk.powered), a)
    beh = hs.SKILL_CLASSES[skill_id](
        jm, SweepParams(amplitude_deg=amp, duration_s=dur), check_tracking=check_tracking, policy=pol
    )
    return jm, beh, pol


def test_five_trained_skills_are_registered_and_head_only():
    trained = {k for k, s in BEHAVIOUR_SPECS.items() if s.kind == "trained"}
    assert trained == {"hold_centre", "look_at", "nod_yes", "shake_no", "glance_around"}
    for sid in trained:
        sk = hs.SKILLS[sid]
        assert set(sk.powered) <= HEAD and sk.powered  # stand phase: head motors only
        assert BEHAVIOUR_SPECS[sid].cls.powered == sk.powered
    assert hs.SKILLS["nod_yes"].powered == (hs.HEAD_PITCH,)  # single-joint skills power exactly one motor
    assert hs.SKILLS["shake_no"].powered == (hs.HEAD_YAW,)
    assert {k for k, s in BEHAVIOUR_SPECS.items() if s.kind == "scripted"} == {
        "neck_turn",
        "neck_nod",
        "arm_raise",
        "hand_wave",
    }


@pytest.mark.parametrize("sid", ["nod_yes", "shake_no", "glance_around"])
def test_periodic_reference_starts_and_ends_at_centre_within_size(sid):
    sk = hs.SKILLS[sid]
    t = np.linspace(0.0, sk.period_s, 601)
    for size in (0.3, 1.0):
        ref = hs.reference(sk, hs.phase_command(sk, t, np.full_like(t, size)))
        assert np.abs(ref[0]).max() < 1e-6 and np.abs(ref[-1]).max() < 1e-6
        assert np.abs(ref).max() <= math.radians(sk.max_deg) * size + 1e-9
        assert np.abs(np.diff(ref, axis=0)).max() < math.radians(2.0)  # smooth: no jumps between samples


def test_reference_recovers_phase_from_command():
    sk = hs.SKILLS["glance_around"]
    t = np.array([1.0, 3.7, 8.2])
    cmd = hs.phase_command(sk, t, np.ones(3))
    np.testing.assert_allclose(hs.reference(sk, cmd), hs.reference(sk, cmd, (t / sk.period_s) % 1.0), atol=1e-9)


def test_look_at_reference_and_demo_targets():
    sk = hs.SKILLS["look_at"]
    ref = hs.reference(sk, np.array([[0.3, -0.1, 0.0]]))
    assert ref.tolist() == [[-0.1, 0.3]]  # powered order (pitch, yaw)
    assert hs.look_at_script(0.0, 20.0) == (0.0, 0.0)
    yaws = [abs(hs.look_at_script(t, 20.0)[0]) for t in np.arange(0, 15, 0.1)]
    assert max(yaws) == pytest.approx(math.radians(20.0))


def test_observation_layout_matches_training():
    jm, beh, pol = _beh("look_at", amp=20.0)
    q = _q0(jm)
    q[hs.HEAD_YAW] = 0.05
    beh.start(q)
    beh.targets(0.0, q, {n: 0.0 for n in jm.names}, 0.0)
    assert pol.last_obs.shape == (hs.obs_size(hs.SKILLS["look_at"]),) == (9,)
    assert pol.last_obs[1] == pytest.approx(0.05)  # q - default for (pitch, yaw)
    assert pol.last_obs[6:].tolist() == [0.0, 0.0, 0.0]  # first look_at target is straight ahead


def test_policy_runs_at_50hz_and_only_powered_joints_are_commanded():
    jm, beh, pol = _beh("nod_yes", dur=2.0)
    q, dq = _q0(jm), {n: 0.0 for n in jm.names}
    beh.start(q)
    for k in range(500):  # 1 s at the real backend's 500 Hz; the (fake) head follows the nod reference
        q[hs.HEAD_PITCH] = float(beh.reference_at(k * 0.002)[0])
        out = beh.targets(k * 0.002, q, dq, 0.0)
        assert out is not None and set(out) == {hs.HEAD_PITCH}
    assert pol.calls == 50


def test_torque_cap_and_limits_apply_to_policy_targets():
    jm, beh, _ = _beh("shake_no", a=5.0)  # saturating action
    q, dq = _q0(jm), {n: 0.0 for n in jm.names}
    beh.start(q)
    out = beh.targets(0.0, q, dq, 0.0)
    i = jm.index_of(hs.HEAD_YAW)
    assert abs(out[hs.HEAD_YAW] - q[hs.HEAD_YAW]) <= jm.effort[i] / jm.kp[i] + 1e-9


def test_aborts_when_head_does_not_follow_the_skill():
    jm, beh, _ = _beh("shake_no", amp=20.0, dur=6.0)
    q, dq = _q0(jm), {n: 0.0 for n in jm.names}
    beh.start(q)
    res = [beh.targets(k * 0.02, q, dq, 0.0) for k in range(int(3.0 / 0.02))]  # head stuck at 0
    assert None in res and "tracking error" in beh.abort_reason


def test_refuses_to_start_off_centre_and_on_stale_state():
    jm, beh, _ = _beh("nod_yes")
    q = _q0(jm)
    q[hs.HEAD_YAW] = math.radians(25.0)  # the unpowered head joint must be centred too (camera faces forward)
    with pytest.raises(RuntimeError, match="straight ahead"):
        beh.start(q)
    q[hs.HEAD_YAW] = 0.0
    beh.start(q)
    assert beh.targets(0.0, q, {n: 0.0 for n in jm.names}, 0.5) is None and "stale" in beh.abort_reason


def test_periodic_duration_rounds_to_whole_cycles_plus_settle():
    _, beh, _ = _beh("glance_around", dur=5.0)
    assert beh.run_s == 12.0 and beh.duration_s == 13.0
    assert beh.command_at(12.5).tolist() == [0.0, 0.0, 0.0]  # settle = Hold Centre


# ----------------------------------------------------------------------------- validation gate
def _fake_skill(tmp_path, passed=True, tamper=False):
    d = tmp_path / "look_at"
    d.mkdir()
    (d / "policy.onnx").write_bytes(b"onnx-bytes")
    dep = {"joint_names": list(hs.SKILLS["look_at"].powered), "metadata": {"trained_at": "2026-11-22T10:00:00"}}
    if passed is not None:
        dep["validation"] = {
            "passed": passed,
            "failures": [] if passed else ["nominal mean error 9 deg >= 2 deg"],
            "policy_sha256": hs.sha256_file(d / "policy.onnx"),
            "metrics": {"track_err_deg": 0.8},
        }
    (d / "deploy.yaml").write_text(yaml.safe_dump(dep))
    if tamper:
        (d / "policy.onnx").write_bytes(b"other-bytes")
    return d


@pytest.fixture
def skill_dir(monkeypatch, tmp_path):
    def use(**kw):
        d = _fake_skill(tmp_path, **kw)
        monkeypatch.setattr(hs, "skill_dirs", lambda sid: [d] if sid == "look_at" else [tmp_path / "none"])
        return d

    return use


def test_skill_status_states(skill_dir, monkeypatch, tmp_path):
    monkeypatch.setattr(hs, "skill_dirs", lambda sid: [tmp_path / "missing"])
    assert hs.skill_status("look_at")["state"] == "untrained"
    with pytest.raises(RuntimeError, match="train it first"):
        hs.load_validated("look_at")


def test_unvalidated_failed_and_tampered_policies_are_refused(skill_dir):
    skill_dir(passed=None)
    assert hs.skill_status("look_at")["state"] == "unvalidated"


def test_failed_validation_refused(skill_dir):
    skill_dir(passed=False)
    st = hs.skill_status("look_at")
    assert st["state"] == "failed" and "nominal mean error" in st["reason"]


def test_tampered_onnx_refused(skill_dir):
    skill_dir(passed=True, tamper=True)
    assert "sha256" in hs.skill_status("look_at")["reason"]


def test_validated_policy_is_ready(skill_dir):
    skill_dir(passed=True)
    st = hs.skill_status("look_at")
    assert st["state"] == "ready" and st["metrics"]["track_err_deg"] == 0.8


# ----------------------------------------------------------------------------- service + kit
@pytest.fixture
def svc(monkeypatch):
    s = S.Service(mirror_enabled=False)
    started = []

    def fake_start(job, cmd, p, pty=False):
        started.append((job, cmd, pty))
        s.job = job

    monkeypatch.setattr(s, "_start", fake_start)
    monkeypatch.setattr(s.console, "preflight_why_not", lambda: None)
    s.started = started
    return s


P = {"behaviour": "look_at", "amplitude": 20.0, "freq": 0.2, "duration": 20.0}


def test_service_refuses_untrained_skill_everywhere(svc, monkeypatch):
    monkeypatch.setattr(hs, "skill_status", lambda sid: {"state": "untrained", "reason": "not trained yet"})
    for res in (svc.preview(P), svc.check(P), svc.run(P, confirm=True)):
        assert not res["ok"] and "train it first" in res["error"]
    assert svc.started == []
    beh = {b["id"]: b for b in svc.status()["behaviours"]}
    assert beh["look_at"]["kind"] == "trained" and beh["look_at"]["ready"] is False
    assert beh["neck_turn"]["ready"] is True


def test_service_check_pushes_validated_skill_before_read_only_check(svc, monkeypatch):
    monkeypatch.setattr(hs, "skill_status", lambda sid: {"state": "ready", "reason": "", "dir": "x", "metrics": {}})
    assert svc.check(P) == {"ok": True}
    job, cmd, _ = svc.started[-1]
    assert job == "check" and "robotcontrol.behaviours.skill_push" in cmd and "look_at" in cmd
    assert "--target real" in cmd[-1] and "--go" not in cmd[-1]  # still read-only


def test_train_job_runs_on_this_pc_only(svc):
    assert svc.train_skill({"behaviour": "dance"})["ok"] is False
    assert svc.train_skill({"behaviour": "nod_yes", "iters": 5}) == {"ok": True}
    job, cmd, _ = svc.started[-1]
    assert job == "train" and cmd[-4:] == ["robotcontrol.training.train_head_skill", "nod_yes", "--iters", "5"]
    assert not any("ssh" in str(c) for c in cmd)
    assert svc.train_skill({"behaviour": "all"})["_code"] == 409  # busy


def test_robot_kit_ships_only_validated_skills(monkeypatch, tmp_path):
    from robotcontrol.tools import robot_kit

    d = _fake_skill(tmp_path, passed=True)
    states = {sid: {"state": "untrained"} for sid in hs.SKILLS}
    states["look_at"] = {"state": "ready", "dir": str(d)}
    monkeypatch.setattr(hs, "skill_status", lambda sid: states[sid])
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tar:
        assert robot_kit._add_head_skills(tar) == ["look_at"]
    with tarfile.open(fileobj=io.BytesIO(buf.getvalue())) as tar:
        assert sorted(tar.getnames()) == ["head_skills/look_at/deploy.yaml", "head_skills/look_at/policy.onnx"]


def test_engine_json_carries_the_unreal_contract(tmp_path):
    """deploy.json is what the Unreal scenario lab reads (C++ has no YAML): same joints, limits and validation gate."""
    import json

    from robotcontrol.training.validate_head_skill import write_engine_json

    sk = hs.SKILLS["nod_yes"]
    jm = JointMap.load()
    dep = {
        "joint_names": list(sk.powered),
        "num_obs": hs.obs_size(sk),
        "validation": {"passed": True, "policy_sha256": "abc"},
    }
    (tmp_path / "deploy.yaml").write_text(yaml.safe_dump(dep))
    out = json.loads(write_engine_json("nod_yes", tmp_path).read_text())
    assert out["validation"] == dep["validation"] and out["num_obs"] == 6
    i = jm.index_of(hs.HEAD_PITCH)
    assert out["effort"] == [jm.effort[i]]
    assert out["soft_limit_lo"][0] == pytest.approx(jm.limits_lo[i] * hs.SOFT_LIMIT_FACTOR)
    legs = {jm.names[k] for k in jm.groups["left_leg"] + jm.groups["right_leg"]}
    hold = set(out["hold_joints"])
    assert hs.HEAD_YAW in hold and hs.HEAD_PITCH not in hold and not hold & legs  # the other head joint is held
    assert len(out["hold_default"]) == len(out["hold_kp"]) == len(out["hold_joints"])
    assert out["glance_keys"] == [list(k) for k in hs.GLANCE_KEYS]


# ----------------------------------------------------------------------------- versions, Library, Unreal link
def _fake_session(folder, passed=True, onnx=b"onnx-v"):
    folder.mkdir(parents=True)
    (folder / "policy.onnx").write_bytes(onnx)
    dep = {
        "joint_names": list(hs.SKILLS["nod_yes"].powered),
        "metadata": {"trained_at": "2026-11-22T10:00:00", "iters": 10, "num_envs": 4, "horizon": 24},
        "validation": {
            "passed": passed,
            "policy_sha256": hs.sha256_file(folder / "policy.onnx"),
            "metrics": {"track_err_deg": 0.5},
        },
    }
    (folder / "deploy.yaml").write_text(yaml.safe_dump(dep))


@pytest.fixture
def models(monkeypatch, tmp_path):
    from robotcontrol.behaviours import skill_versions as sv

    monkeypatch.setattr(sv, "MODELS_DIR", tmp_path)
    monkeypatch.setattr(hs, "MODELS_DIR", tmp_path)
    monkeypatch.setattr(hs, "skill_dirs", lambda sid: [tmp_path / hs.SKILLS_SUBDIR / sid])
    return sv, tmp_path / hs.SKILLS_SUBDIR / "nod_yes"


def test_sessions_are_kept_and_only_a_passed_one_can_be_active(models):
    sv, act = models
    _fake_session(act, onnx=b"first")  # a skill trained before history existed
    assert sv.next_version("nod_yes") == 2 and sv.active_version("nod_yes") == 1  # migrated to v1
    stage = act.parent / "stage"
    _fake_session(stage, passed=False, onnx=b"second")
    sv.store("nod_yes", stage, 2)
    with pytest.raises(ValueError, match="did not pass"):
        sv.activate("nod_yes", 2)
    assert (act / "policy.onnx").read_bytes() == b"first"  # a failed retrain never replaces the working version
    vs = sv.list_versions("nod_yes")
    assert [(v["version"], v["passed"], v["active"]) for v in vs] == [(2, False, False), (1, True, True)]
    assert vs[1]["steps"] == 10 * 4 * 24 and vs[1]["onnx_bytes"] == len(b"first")


def test_rollback_and_playing_a_stored_session(models):
    sv, act = models
    _fake_session(act, onnx=b"first")
    sv.migrate("nod_yes")
    stage = act.parent / "stage"
    _fake_session(stage, onnx=b"second")
    sv.store("nod_yes", stage, 2)
    sv.activate("nod_yes", 2)
    assert sv.active_version("nod_yes") == 2 and hs.skill_status("nod_yes")["state"] == "ready"
    sv.activate("nod_yes", 1)  # rollback: the old file comes back and still matches its own gate
    assert (act / "policy.onnx").read_bytes() == b"first" and hs.skill_status("nod_yes")["state"] == "ready"
    assert hs.skill_status("nod_yes", 2)["state"] == "ready"  # a stored session plays on the virtual robot


def test_stored_session_is_virtual_only(svc, monkeypatch):
    from robotcontrol.behaviours import runner

    p = S.parse_params({"behaviour": "nod_yes", "version": 2})
    assert S.runner_args(p)[:2] == ["--skill-version", "2"]
    assert "active version" in svc.check(p)["error"] and "active version" in svc.run(p, confirm=True)["error"]
    with pytest.raises(SystemExit):
        runner.main(["nod_yes", "--target", "real", "--skill-version", "2"])
    with pytest.raises(ValueError):
        S.parse_params({"behaviour": "neck_turn", "version": 1})  # coded moves have no sessions


def test_copy_to_robot_is_its_own_job_and_moves_nothing(svc, monkeypatch):
    monkeypatch.setattr(hs, "skill_status", lambda sid, version=None: {"state": "ready", "reason": "", "dir": "x"})
    assert svc.push_skill({"behaviour": "nod_yes"}) == {"ok": True}
    job, cmd, _ = svc.started[-1]
    assert job == "push" and cmd[-2:] == ["robotcontrol.behaviours.skill_push", "nod_yes"]
    assert not any("runner" in c for c in cmd)


def test_web_asks_the_unreal_lab_to_play(svc, monkeypatch):
    monkeypatch.setattr(hs, "skill_status", lambda sid, version=None: {"state": "ready", "reason": "", "dir": "x"})
    res = svc.lab_play({"behaviour": "look_at", "size": 20})
    assert res["ok"] and res["seq"] == 1 and res["unreal_running"] is False
    polled = svc.lab_poll(0)
    assert polled["request"]["behaviour"] == "look_at" and polled["request"]["size"] == 20.0
    svc.lab_poll(1)
    lab = svc.status()["lab"]
    assert lab["ack_seq"] == 1 and lab["unreal_seen_s"] is not None and lab["unreal_seen_s"] < 5
    assert svc.lab_play({"behaviour": "neck_turn"})["ok"] is False  # the lab runs trained skills only
