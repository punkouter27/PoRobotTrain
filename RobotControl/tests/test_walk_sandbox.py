"""Level 2 walk sandbox: course, observation contract, hold mode, validation marker, gate, service endpoints."""

import json
import time

import numpy as np
import onnx
import pytest
import yaml
from onnx import TensorProto, helper

from robotcontrol.behaviours import service as S
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.training import validate_level2 as V
from robotcontrol.walk import policy as P
from robotcontrol.walk.course import COURSES, course
from robotcontrol.walk.sandbox import WalkSandbox
from robotcontrol.walk.sim import WalkSim

TERMS = [
    {"name": "base_ang_vel", "dim": 3, "scale": 1.0},
    {"name": "projected_gravity", "dim": 3, "scale": 1.0},
    {"name": "velocity_commands", "dim": 3, "scale": 1.0},
    {"name": "gait_phase", "dim": 2, "scale": 1.0},
    {"name": "joint_pos_rel", "dim": 24, "scale": 1.0},
    {"name": "joint_vel_rel", "dim": 24, "scale": 1.0},
    {"name": "last_action", "dim": 24, "scale": 1.0},
]


def make_candidate(path, jm):
    """A contract-valid policy that outputs zeros (cannot walk): obs (B,83) -> actions (B,24)."""
    path.mkdir(parents=True, exist_ok=True)
    w = helper.make_tensor("W", TensorProto.FLOAT, [83, 24], np.zeros(83 * 24, dtype=np.float32))
    g = helper.make_graph(
        [helper.make_node("MatMul", ["obs", "W"], ["actions"])],
        "zero",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", 83])],
        [helper.make_tensor_value_info("actions", TensorProto.FLOAT, ["batch", 24])],
        [w],
    )
    onnx.save(helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)]), str(path / "policy.onnx"))
    n = P.NUM_POLICY_JOINTS
    deploy = {
        "num_obs": 83,
        "num_actions": 24,
        "joint_names": jm.names[:n],
        "default_joint_pos": [float(v) for v in jm.defaults[:n]],
        "kp": [float(v) for v in jm.kp[:n]],
        "kd": [float(v) for v in jm.kd[:n]],
        "action_scale": [float(v) for v in jm.action_scale[:n]],
        "step_dt": 0.02,
        "physics_dt": 0.005,
        "obs_terms": TERMS,
        "gait_phase": {"period": 0.6, "order": ["sin", "cos"]},
        "command_ranges": {"lin_vel_x": [-0.5, 1.0], "lin_vel_y": [-0.5, 0.5], "ang_vel_z": [-1.0, 1.0]},
        "metadata": {"source": "pytest", "task": "zero-policy"},
    }
    (path / "deploy.yaml").write_text(yaml.safe_dump(deploy, sort_keys=False), encoding="utf-8")
    dep = P.Level2Deploy.load(path / "deploy.yaml")
    rng = np.random.default_rng(0)
    rows = []
    for k in range(8):
        q = rng.normal(size=4)
        s = P.ObsSignals(rng.normal(size=3), q / np.linalg.norm(q), rng.normal(size=3), 0.07 * k,
                         rng.normal(size=n), rng.normal(size=n), rng.normal(size=n))  # fmt: skip
        rows.append(s)
    np.savez(
        path / "golden.npz",
        obs=np.stack([P.build_obs(dep, s) for s in rows]),
        actions=np.zeros((len(rows), n), dtype=np.float32),
        base_ang_vel=np.stack([s.base_ang_vel for s in rows]),
        base_quat=np.stack([s.base_quat for s in rows]),
        command=np.stack([s.command for s in rows]),
        phase_time=np.array([s.phase_time for s in rows]),
        q=np.stack([s.q for s in rows]),
        dq=np.stack([s.dq for s in rows]),
        last_action=np.stack([s.last_action for s in rows]),
    )
    return path


def test_courses_geometry():
    for name in COURSES:
        c = course(name)
        assert c["checkpoints"][-1][:2] == [0.0, 0.0]  # a lap ends back at the start
        assert c["checkpoint_radius"] > 0
        for b in c["boxes"]:
            assert len(b["pos"]) == 3 and len(b["size"]) == 3 and len(b["quat"]) == 4
            assert np.isclose(np.linalg.norm(b["quat"]), 1.0, atol=1e-4)
    with pytest.raises(ValueError):
        course("moon")


def test_observation_terms():
    jm = JointMap.load()
    dep = P.Level2Deploy(
        joint_names=jm.names[:24], default=np.asarray(jm.defaults[:24]), kp=jm.kp[:24], kd=jm.kd[:24],
        action_scale=jm.action_scale[:24], obs_terms=TERMS,
    )  # fmt: skip
    s = P.ObsSignals(np.zeros(3), np.array([1.0, 0, 0, 0]), np.array([0.5, 0, 0]), 0.15,
                     np.asarray(jm.defaults[:24]), np.zeros(24), np.zeros(24))  # fmt: skip
    o = P.build_obs(dep, s)
    assert o.shape == (83,) and dep.num_obs == 83
    np.testing.assert_allclose(o[3:6], [0, 0, -1])  # upright: gravity straight down in the body frame
    np.testing.assert_allclose(o[6:9], [0.5, 0, 0])
    np.testing.assert_allclose(o[9:11], [1.0, 0.0], atol=1e-6)  # quarter period: sin 1, cos 0
    np.testing.assert_allclose(o[11:35], 0.0)
    # rolled 90 deg about x: gravity appears along body +y... with this sign convention -> check norm and axis
    roll = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0])
    g = P.quat_rotate_inverse(roll, np.array([0, 0, -1.0]))
    np.testing.assert_allclose(np.abs(g), [0, 1, 0], atol=1e-9)


def test_hold_mode_stands_in_its_harness():
    sim = WalkSim("loop", mode="hold")
    for _ in range(150):
        sim.step()
    assert sim.falls == 0 and not sim.is_fallen()
    snap = sim.snapshot()
    assert snap["mode"] == "hold" and len(snap["q"]) == 26 and snap["checkpoint"]["index"] == 0
    root, q = sim.full_qpos()
    assert root.shape == (7,) and q.shape == (26,)


def test_policy_status_requires_matching_validation(tmp_path):
    jm = JointMap.load()
    ok, reason = P.policy_status(tmp_path / "missing")
    assert not ok and "no validated" in reason
    cand = make_candidate(tmp_path / "cand", jm)
    ok, reason = P.policy_status(cand)
    assert not ok and "no passed validation" in reason
    d = yaml.safe_load((cand / "deploy.yaml").read_text())
    d["validation"] = {"passed": True, "policy_sha256": "0" * 64}
    (cand / "deploy.yaml").write_text(yaml.safe_dump(d))
    ok, reason = P.policy_status(cand)
    assert not ok and "changed after validation" in reason
    d["validation"]["policy_sha256"] = P.sha256(cand / "policy.onnx")
    (cand / "deploy.yaml").write_text(yaml.safe_dump(d))
    assert P.policy_status(cand) == (True, None)


def test_gate_rejects_a_policy_that_cannot_walk(tmp_path, monkeypatch):
    jm = JointMap.load()
    cand = make_candidate(tmp_path / "cand", jm)
    models = tmp_path / "models"
    monkeypatch.setattr(V, "MODEL_DIR", models)
    monkeypatch.setattr(V, "SCENARIOS", V.SCENARIOS[:2])
    monkeypatch.setattr(V, "course_lap", lambda policy: {"laps": 0})
    code = V.main(["--candidate", str(cand), "--out", str(tmp_path / "out"), "--promote"])
    report = (tmp_path / "out" / "report.md").read_text(encoding="utf-8")
    assert code == 1 and not models.exists()  # never promoted
    assert "| PASS | joint order" in report and "| PASS | observation builder vs trainer" in report
    assert "| PASS | onnxruntime actions vs trainer" in report
    assert "| FAIL | stand still" in report  # a zero policy cannot balance


def test_sandbox_hold_runs_in_real_time_and_watchdog(tmp_path):
    sb = WalkSandbox(mirror_factory=None, log=lambda s: None, model_dir=tmp_path / "none")
    st = sb.state()
    assert not st["running"] and not st["policy_available"] and st["geometry"]["checkpoints"]
    res = sb.start("loop", mode="policy")
    assert not res["ok"] and "no validated" in res["policy_reason"]
    assert sb.start("loop", mode="hold")["ok"]
    try:
        time.sleep(0.6)
        st = sb.state()
        assert st["running"] and st["mode"] == "hold" and 0.3 < st["t"] < 1.2
        assert sb.command(0.5, 0, 0)["note"].startswith("hold mode")
    finally:
        sb.stop()
    assert not sb.state()["running"]


@pytest.fixture
def svc():
    s = S.Service(mirror_enabled=False)
    yield s
    if s.walk is not None:
        s.walk.stop()


def test_service_walk_and_robustness_endpoints(svc, monkeypatch, tmp_path):
    monkeypatch.setattr(S, "ROBUSTNESS_REPORT", tmp_path / "none.json")
    assert svc.robustness_report() == {"available": False}
    assert svc.interpret({"text": "nod twenty degrees"})["id"] == "neck_nod"
    svc.job = "preview"
    assert svc.walk_post("start", {"mode": "hold"})["_code"] == 409
    assert svc.robustness_run(quick=True)["_code"] == 409
    svc.job = None
    assert svc.walk_post("start", {"course": "flat", "mode": "hold"})["ok"]
    assert svc.job == "walk" and svc.status()["job"] == "walk"
    assert svc.robustness_run(quick=True)["_code"] == 409  # walk holds the job slot
    assert svc.walk_state()["course"] == "flat"
    assert svc.stop()["ok"] and svc.job is None and not svc.walk_state()["running"]
    assert svc.walk_post("dance", {})["_code"] == 404


def test_service_reads_chunked_post_bodies(svc):
    """.NET PostAsJsonAsync streams JSON without Content-Length; the parameters must still arrive."""
    import http.client
    import threading
    from http.server import ThreadingHTTPServer

    srv = ThreadingHTTPServer(("127.0.0.1", 0), S.make_handler(svc))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        c = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=10)
        body = b'{"text": "nod the head twenty degrees"}'
        c.putrequest("POST", "/assistant/interpret")
        c.putheader("Transfer-Encoding", "chunked")
        c.putheader("Content-Type", "application/json")
        c.endheaders()
        for part in (body[:10], body[10:]):
            c.send(b"%x\r\n%s\r\n" % (len(part), part))
        c.send(b"0\r\n\r\n")
        r = json.loads(c.getresponse().read())
        assert r["id"] == "neck_nod" and r["amplitude"] == 20.0
    finally:
        srv.shutdown()


def test_deploy_accepts_mjlab_export_names(tmp_path):
    """scripts/level2/export_level2.py writes mjlab's term names and commands.standing_threshold."""
    jm = JointMap.load()
    cand = make_candidate(tmp_path / "cand", jm)
    d = yaml.safe_load((cand / "deploy.yaml").read_text())
    alias = {"velocity_commands": "command", "gait_phase": "phase", "joint_pos_rel": "joint_pos",
             "joint_vel_rel": "joint_vel", "last_action": "actions"}  # fmt: skip
    d["obs_terms"] = [dict(t, name=alias.get(t["name"], t["name"])) for t in d["obs_terms"]]
    del d["gait_phase"], d["command_ranges"]
    d["gait_period"] = 0.6
    d["commands"] = {"lin_vel_x": [-0.5, 1.0], "lin_vel_y": [-0.5, 0.5], "ang_vel_z": [-0.5, 0.5],
                     "standing_threshold": 0.1}  # fmt: skip
    (cand / "deploy.yaml").write_text(yaml.safe_dump(d))
    dep = P.Level2Deploy.load(cand / "deploy.yaml")
    assert [t["name"] for t in dep.obs_terms] == [t["name"] for t in TERMS]
    assert dep.cmd_limits["wz"] == [-0.5, 0.5] and dep.zero_phase_below == 0.1
    s = P.ObsSignals(np.zeros(3), np.array([1.0, 0, 0, 0]), np.array([0.05, 0, 0]), 0.15,
                     np.zeros(24), np.zeros(24), np.zeros(24))  # fmt: skip
    np.testing.assert_allclose(P.build_obs(dep, s)[9:11], 0.0)  # standing: phase zeroed
    s.command = np.array([0.5, 0, 0])
    np.testing.assert_allclose(P.build_obs(dep, s)[9:11], [1.0, 0.0], atol=1e-6)
