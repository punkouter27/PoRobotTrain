"""Operator console (docs/service_api.md sections 4-7): events, history, limits, pre-flight, motions, player,
calibration, network, docs, chat and the HTTP routes. Nothing here touches the robot or writes tracked files."""

import json
import math
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pytest

from robotcontrol.behaviours import motions as M
from robotcontrol.behaviours import ops
from robotcontrol.behaviours import service as S
from robotcontrol.behaviours.console import Console
from robotcontrol.behaviours.telemetry_log import EventLog, TelemetryHistory, gravity_in_body, soc_from_cell_v
from robotcontrol.configs.joint_map import JointMap

JM = JointMap.load()
N = JM.num_joints


class FakeBackend:
    """Stands in for SimBackend: joints jump to their targets (no MuJoCo)."""

    def __init__(self, powered, start_q_deg):
        self.powered = list(powered)
        self.q = np.radians(np.array(start_q_deg, dtype=float))
        self.kp = np.ones(len(self.powered))

    def command(self, targets):
        for j, v in targets.items():
            self.q[JM.index_of(j)] = v

    def advance(self, t):
        pass

    def read(self):
        return self.q.copy(), np.zeros(N), 0.0

    def telemetry(self):
        return {"imu_rpy_deg": [0.0, 0.0, 0.0]}

    def close(self):
        pass


@pytest.fixture
def svc(tmp_path, monkeypatch):
    s = S.Service(mirror_enabled=False)
    s.console = Console(s, data_dir=tmp_path / "user_data")
    s.console.player.backend_factory = FakeBackend
    started = []

    def fake_start(job, cmd, p, pty=False):
        started.append((job, cmd, pty))
        s.job = job

    monkeypatch.setattr(s, "_start", fake_start)
    s.started = started
    return s


def wait_idle(svc, timeout=10.0):
    t0 = time.time()
    while (svc.console.player.running or svc.job) and time.time() - t0 < timeout:
        time.sleep(0.02)
    assert not svc.console.player.running


# --------------------------------------------------------------------------------------- events + history (#2-#4)
def test_event_log_dedup_filter_and_file(tmp_path):
    log = EventLog(tmp_path / "ev.jsonl")
    assert log.record("warn", "temperature", "hot", dedup_key="k") is not None
    assert log.record("warn", "temperature", "hot again", dedup_key="k") is None  # deduplicated
    log.record("critical", "estop", "stop")
    log.record("info", "job", "done")
    assert [e["kind"] for e in log.list()] == ["job", "estop", "temperature"]  # newest first
    assert [e["kind"] for e in log.list(level="warn")] == ["estop", "temperature"]
    assert log.counts() == {"info": 1, "warn": 1, "critical": 1}
    assert len((tmp_path / "ev.jsonl").read_text().splitlines()) == 3
    with pytest.raises(ValueError):
        log.record("panic", "x", "y")


def test_telemetry_alarms_from_packets(svc):
    temps = [40] * N
    temps[25] = 70
    temps[24] = 58
    q = [0.0] * N
    q[24] = math.degrees(float(JM.limits_hi[24])) - 0.5  # head pitch at its limit
    svc.update_telemetry("real", {"temps_c": temps, "q_deg": q, "bus_v": 40.0}, source="monitor")
    kinds = {(e["kind"], e["level"]) for e in svc.console.events.list()}
    assert ("temperature", "critical") in kinds and ("temperature", "warn") in kinds
    assert ("limit", "warn") in kinds and ("power", "warn") in kinds  # 40 V / 13 cells < 3.5 V


def test_temperature_history_and_power_estimate():
    h = TelemetryHistory(JM.names, series_cells=13)
    t0 = time.time() - 120
    for k in range(120):
        h.add("real", {"temps_c": [30 + k // 10] * N, "bus_v": 54.0 - k * 0.02}, now=t0 + k)
    tm = h.temperatures("real")
    assert len(tm["t"]) == 120 and len(tm["series"]) == N and tm["hottest"]["temp_c"] == 41
    assert tm["rate_c_per_min"][0] == pytest.approx(6.0, abs=0.5)
    pw = h.power("real")
    assert pw["available"] and pw["latest_v"] == pytest.approx(51.62, abs=0.01)
    assert 0 < pw["soc_pct"] < 100 and pw["minutes_left"] is not None and pw["current_a"] is None
    assert h.power("nobody")["available"] is False


def test_soc_curve_and_gravity():
    assert soc_from_cell_v(4.2) == 100 and soc_from_cell_v(2.9) == 0
    assert 40 < soc_from_cell_v(3.75) < 55
    assert gravity_in_body(0, 0) == [-0.0, 0.0, 1.0]
    g = gravity_in_body(0, 90)
    assert g[0] == pytest.approx(-1.0)


def test_imu_check_drift_and_no_data():
    h = TelemetryHistory(JM.names)
    assert h.imu_check("real")["verdict"] == "no_data"
    t0 = time.time() - 30
    for k in range(60):
        h.add("real", {"imu_rpy_deg": [0.1, -0.2, k * 0.05]}, now=t0 + k * 0.5)  # 6 deg/min yaw drift
    im = h.imu_check("real")
    assert im["still"] and im["verdict"] == "warn" and im["yaw_drift_deg_per_min"] == pytest.approx(6.0, abs=0.1)
    assert im["gravity_body"][2] == pytest.approx(1.0, abs=1e-3)


# --------------------------------------------------------------------------------------- limits + pre-flight
def test_limits_only_tighten(svc):
    lim = svc.console.limits
    with pytest.raises(ValueError):
        lim.set({"amplitude_cap_deg": 45})  # above the built-in maximum
    with pytest.raises(ValueError):
        lim.set({"bogus": 1})
    lim.set({"amplitude_cap_deg": 15, "walk_speed_scale": 0.5, "joint_range_frac": 0.5})
    with pytest.raises(ValueError, match="session cap"):
        lim.check_params({"amplitude": 20.0, "freq": 0.2})
    assert lim.scale_walk(1.0, 0.4, -1.0) == (0.5, 0.2, -0.5)
    deg, clamped = lim.clamp_joint(JM, "head_yaw_joint", 100.0)
    assert clamped and deg == pytest.approx(57.5, abs=0.01)
    # move_joint applies the cap to the target it sends
    svc.move_joint("head_yaw_joint", 100.0, target="sim")
    assert float(svc.started[-1][1][5]) == pytest.approx(57.5, abs=0.01)
    assert lim.reset()["tightened"] == []


def test_run_needs_preflight(svc, monkeypatch):
    p = {"behaviour": "neck_turn", "amplitude": 10.0, "freq": 0.2, "duration": 20.0}
    svc.previewed, svc.checked, svc.checked_at = dict(p), dict(p), time.time()
    assert "pre-flight" in svc.run(p, confirm=True)["error"]
    assert "pre-flight" in svc.move_joint("head_yaw_joint", 5.0, target="real", confirm=True)["error"]
    assert svc.run(p, confirm=True, practice=True) == {"ok": True}  # practice never needs the checklist
    svc.job = None
    monkeypatch.setattr(svc.console.preflight, "auto_checks", lambda: [{"id": "x", "label": "a", "ok": True, "detail": ""}])
    pf = svc.console.preflight
    for item, _ in pf.MANUAL:
        st = pf.tick(item, True)
    assert st["complete"] and st["valid_for_s"] > 0
    assert svc.run(p, confirm=True) == {"ok": True}
    with pytest.raises(ValueError):
        pf.tick("nope", True)
    assert pf.reset()["complete"] is False


def test_preflight_auto_check_failure_blocks(svc, monkeypatch):
    monkeypatch.setattr(svc.console.preflight, "auto_checks", lambda: [{"id": "x", "label": "r", "ok": False, "detail": ""}])
    for item, _ in svc.console.preflight.MANUAL:
        svc.console.preflight.tick(item, True)
    assert "pre-flight" in svc.console.preflight_why_not()


# --------------------------------------------------------------------------------------- motions (#11-#14)
def clip(**kw):
    c = {"name": "test_clip", "keyframes": [{"t": 0.0, "q_deg": {"head_yaw_joint": 0.0}},
                                            {"t": 2.0, "q_deg": {"head_yaw_joint": 20.0}}]}  # fmt: skip
    c.update(kw)
    return c


def test_validate_clip_catches_problems():
    assert M.validate_clip(clip(), JM)["ok"]
    assert not M.validate_clip(clip(name="Bad Name"), JM)["ok"]
    fast = clip(keyframes=[{"t": 0, "q_deg": {"head_yaw_joint": 0}}, {"t": 0.1, "q_deg": {"head_yaw_joint": 30}}])
    v = M.validate_clip(fast, JM)
    assert not v["ok"] and "deg/s" in v["issues"][0]["message"]
    far = clip(keyframes=[{"t": 0, "q_deg": {"head_pitch_joint": 35.0}}])  # outside 90 % of +-36
    assert "soft range" in M.validate_clip(far, JM)["issues"][0]["message"]
    order = clip(keyframes=[{"t": 1, "q_deg": {"head_yaw_joint": 0}}, {"t": 0.5, "q_deg": {"head_yaw_joint": 1}}])
    assert not M.validate_clip(order, JM)["ok"]
    assert not M.validate_clip(clip(keyframes=[{"t": 0, "q_deg": {"tail_joint": 1}}]), JM)["ok"]
    v = M.validate_clip(clip(keyframes=[{"t": 0, "q_deg": {"left_knee_joint": 20.0}}]), JM)
    assert v["ok"] and not v["real_eligible"] and "leg" in v["real_reasons"][0]


def test_sample_lead_in_and_carry_forward():
    c = {"name": "x", "keyframes": [{"t": 0.0, "q_deg": {"head_yaw_joint": 20.0, "head_pitch_joint": 5.0}},
                                    {"t": 1.0, "q_deg": {"head_yaw_joint": -20.0}}]}  # fmt: skip
    start = {"head_yaw_joint": 0.0, "head_pitch_joint": 0.0}
    lead = M.lead_in_s(c, start)
    assert lead >= 0.8
    assert M.sample(c, 0.0, start, lead) == pytest.approx(start)
    assert M.sample(c, lead, start, lead)["head_yaw_joint"] == pytest.approx(20.0)
    end = M.sample(c, lead + 5.0, start, lead)
    assert end == pytest.approx({"head_yaw_joint": -20.0, "head_pitch_joint": 5.0})  # pitch carried forward


def test_library_save_protects_builtins_and_store_roundtrip(svc):
    lib = svc.console.library
    names = {m["name"]: m for m in lib.list()}
    assert {"look_around", "nod_yes", "wave_hello"} <= set(names) and names["wave_hello"]["valid"]
    assert lib.save(clip(name="wave_hello"))["_code"] == 409  # cannot shadow a built-in
    assert lib.save(clip())["ok"]
    assert lib.save(clip())["_code"] == 409 and lib.save(clip(), overwrite=True)["ok"]
    pkg = lib.export_package("test_clip")
    assert pkg["format"] == M.PACKAGE_FORMAT and pkg["validation"]["ok"]
    assert lib.delete("test_clip")["ok"] and lib.get("test_clip") is None
    assert lib.import_package(pkg)["ok"] and lib.get("test_clip")["source"] == "user"
    tampered = json.loads(json.dumps(pkg))
    tampered["clip"]["keyframes"][1]["q_deg"]["head_yaw_joint"] = 25.0
    assert "checksum" in lib.import_package(tampered, overwrite=True)["error"]
    assert lib.delete("look_around")["_code"] == 409
    bad = lib.save(clip(keyframes=[{"t": 0, "q_deg": {"head_pitch_joint": 90}}]))
    assert not bad["ok"] and not bad["validation"]["ok"]


def test_poses_resolve_home_and_validate(svc):
    poses = {p["name"]: p for p in svc.console.library.poses()}
    assert "home" in poses and len(poses["home"]["q_deg"]) == N - M.LEG_JOINTS
    for name in poses:
        assert M.validate_clip(svc.console.library.pose_clip(name), JM)["ok"], name


def test_recorder_turns_telemetry_into_keyframes(svc):
    rec = svc.console.recorder
    with pytest.raises(ValueError):
        rec.start("moon")
    rec.start("practice")
    base = [round(math.degrees(float(d)), 1) for d in JM.defaults]
    for k in range(30):
        q = list(base)
        q[25] = k * 1.0  # head yaw sweeps 0..29 deg
        rec.samples.append((k * 0.1, q))
    res = rec.stop("my_take")
    assert res["ok"] and res["validation"]["ok"]
    kfs = res["clip"]["keyframes"]
    assert list(kfs[0]["q_deg"]) == ["head_yaw_joint"] and kfs[-1]["q_deg"]["head_yaw_joint"] == 29.0
    assert not rec.active and rec.stop()["ok"] is False


def test_play_motion_on_practice_twin_and_refuse_real(svc):
    c = svc.console
    assert "not on the real robot" in c.play(clip(), "real", False, "motion")["error"]
    res = c.play(clip(), "practice", False, "motion")
    assert res["ok"] and svc.job == "motion"
    assert c.play(clip(), "practice", False, "motion")["_code"] == 409  # busy
    wait_idle(svc)
    assert svc.results["motion"]["ok"] and svc.job is None
    assert svc.practice_q_deg[JM.index_of("head_yaw_joint")] == pytest.approx(20.0, abs=0.1)
    assert svc.telemetry["practice"]["source"] == "motion"


def test_stop_and_estop_halt_the_player(svc):
    c = svc.console
    long_clip = clip(loop=True)
    assert c.play(long_clip, "sim", True, "motion")["ok"]
    time.sleep(0.1)
    assert svc.stop() == {"ok": True}
    wait_idle(svc)
    assert c.play(long_clip, "sim", True, "motion")["ok"]
    time.sleep(0.05)
    res = c.estop()
    assert "player" in res["stopped"]
    wait_idle(svc)
    assert any(e["kind"] == "estop" and e["level"] == "critical" for e in c.events.list())


def test_modes(svc):
    c = svc.console
    assert "not on the real robot" in c.set_mode({"mode": "damp", "target": "real", "joints": ["head_yaw_joint"]})["error"]
    with pytest.raises(ValueError, match="only the joints you name"):
        c.set_mode({"mode": "hold", "target": "practice"})
    with pytest.raises(ValueError):
        c.set_mode({"mode": "dance", "target": "practice"})
    res = c.set_mode({"mode": "damp", "target": "practice", "joints": ["right_elbow_joint"], "max_s": 0.2})
    assert res["ok"] and c.player.state["mode"] == "damp"
    wait_idle(svc)
    assert c.set_mode({"mode": "zero_torque", "target": "practice", "max_s": 0.1})["ok"]
    wait_idle(svc)


# --------------------------------------------------------------------------------------- calibration, network, docs
def test_zero_offsets_capture_and_save(svc, tmp_path):
    zo = svc.console.offsets
    q = [0.0] * N
    q[25], q[24] = 0.8, -2.5
    cap = zo.capture(q, "real", "zero", ["head_yaw_joint", "head_pitch_joint"])
    by = {r["joint"]: r for r in cap["rows"]}
    assert by["head_yaw_joint"]["verdict"] == "ok" and by["head_pitch_joint"]["verdict"] == "warn"
    assert zo.save(False)["ok"] is False
    saved = zo.save(True)
    assert saved["applied"] is False and saved["offsets_deg"]["head_pitch_joint"] == -2.5
    assert json.loads(zo.path.read_text())["offsets_deg"]["head_yaw_joint"] == 0.8
    with pytest.raises(ValueError, match="implausible"):
        zo.save(True, {"head_yaw_joint": 40})
    home = zo.capture([math.degrees(float(d)) for d in JM.defaults], "practice", "home")
    assert all(r["offset_deg"] == pytest.approx(0.0, abs=1e-6) for r in home["rows"])


def test_connection_profiles_and_probe(tmp_path):
    pr = ops.ConnectionProfiles(tmp_path / "profiles.json")
    assert {p["name"] for p in pr.load()["profiles"]} == {"sim", "urlab", "real"}
    pr.upsert({"name": "lab", "target": "real", "host": "10.0.0.5", "port": 22, "interface": "eth0"})
    assert pr.activate("lab")["active"] == "lab"
    with pytest.raises(ValueError):
        pr.upsert({"name": "x", "target": "real", "host": "bad host; rm -rf"})
    with pytest.raises(ValueError):
        pr.activate("missing")
    assert "lab" not in {p["name"] for p in pr.delete("lab")["profiles"]}
    with pytest.raises(ValueError):
        ops.probe_connection("$(whoami)", 22)
    res = ops.probe_connection("127.0.0.1", 9, icmp=False)  # discard port: closed on a dev box
    assert res["host"] == "127.0.0.1" and "tcp_ms" in res


def test_docs_index_and_traversal():
    idx = {d["name"]: d for d in ops.docs_index()}
    assert "service_api" in idx and idx["service_api"]["title"].startswith("Behaviour service API")
    assert ops.doc_text("service_api")["markdown"].startswith("# ")
    assert ops.doc_text("../AGENTS")["_code"] == 400
    assert ops.doc_text("nope_nope")["_code"] == 404


def test_versions_report_policies():
    v = ops.versions()
    names = [p["name"] for p in v["policies"]]
    assert "r1_level0_upperbody" in names
    p0 = next(p for p in v["policies"] if p["name"] == "r1_level0_upperbody")
    assert len(p0["policy_sha256"]) == 64 and p0["num_actions"] == 7


# --------------------------------------------------------------------------------------- chat (#20)
def test_chat_answers_status_and_remembers(svc):
    ch = svc.console.chat
    assert "No battery reading" in ch.chat({"text": "what's the battery?"})["answer"]
    svc.update_telemetry("real", {"temps_c": [40] * N, "bus_v": 50.0}, source="snapshot")
    assert "%" in ch.chat({"text": "how is the battery"})["answer"]
    assert "Hottest motor" in ch.chat({"text": "how hot are the motors?"})["answer"]
    r = ch.chat({"text": "nod the head ten degrees", "session": "s1"})
    assert r["kind"] == "behaviour" and r["id"] == "neck_nod"
    r2 = ch.chat({"text": "bigger", "session": "s1"})
    assert r2["kind"] == "behaviour" and r2["amplitude"] == 20.0
    assert ch.chat({"text": "again", "session": "s1"})["id"] == "neck_nod"
    assert "nod the head" in ch.chat({"text": "what did I ask?", "session": "s1"})["answer"]
    assert ch.chat({"text": "go to the arms out pose"})["pose"] == "arms_out"
    assert "Idle" in ch.chat({"text": "what is the robot doing?"})["answer"]
    assert ch.chat({"text": "is the robot ready?"})["answer"].startswith("The twin is ready")
    assert ch.history("s1")["turns"][0]["text"] == "nod the head ten degrees"
    assert ch.reset("s1")["ok"] and ch.history("s1")["turns"] == []


# --------------------------------------------------------------------------------------- HTTP
def test_console_http_routes(svc):
    srv = S.ThreadingHTTPServer(("127.0.0.1", 0), S.make_handler(svc))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def get(path):
        return json.load(urllib.request.urlopen(base + path, timeout=30))

    def post(path, body):
        req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST")
        try:
            return json.load(urllib.request.urlopen(req, timeout=30))
        except urllib.error.HTTPError as e:
            return dict(json.load(e), _http=e.code)

    try:
        h = get("/health")
        assert {r["section"] for r in h["rows"]} >= {"Services", "Data", "Robot", "Jobs"}
        assert get("/limits")["values"]["amplitude_cap_deg"] == 30.0
        assert post("/limits", {"values": {"amplitude_cap_deg": 10}})["tightened"] == ["amplitude_cap_deg"]
        refused = post("/preview", {"behaviour": "neck_turn", "amplitude": 20})
        assert refused["_http"] == 400 and "session cap" in refused["error"]
        assert get("/status")["console"]["limits"]["amplitude_cap_deg"] == 10
        assert len(get("/poses")["poses"]) >= 5
        assert any(m["name"] == "wave_hello" for m in get("/motions")["motions"])
        assert get("/motions/wave_hello")["validation"]["ok"]
        assert get("/motions/wave_hello/export")["format"] == M.PACKAGE_FORMAT
        assert post("/poses/preview", {"name": "look_left", "target": "real"})["_http"] == 400
        assert post("/poses/preview", {"name": "nope"})["_http"] == 404
        assert post("/poses/preview", {"name": "look_left", "target": "practice"})["ok"]
        wait_idle(svc)
        assert post("/estop", {})["ok"]
        assert get("/events?level=critical")["events"][0]["kind"] == "estop"
        assert get("/preflight")["complete"] is False
        assert post("/preflight", {"item": "on_stand", "checked": True})["items"][0]["ok"]
        assert get("/docs")["docs"] and get("/docs/service_api")["ok"]
        assert post("/assistant/chat", {"text": "help?"})["kind"] == "answer"
        assert get("/imu?source=practice")["ok"]
        assert post("/calibration/capture", {"source": "real"})["_http"] == 409  # no fresh snapshot
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + "/camera.png?source=real", timeout=5)
        assert e.value.code == 503
        assert get("/versions")["policies"]
        assert get("/network")["profiles"]
    finally:
        srv.shutdown()


def test_preflight_brief_in_status_is_cheap(svc):
    brief = svc.status()["console"]["preflight"]
    assert brief == {"complete": False, "open": 5, "reason": "5 item(s) to tick"}
    for item, _ in svc.console.preflight.MANUAL:
        svc.console.preflight.tick(item, True)
    svc.console._auto_cache = (time.time(), [{"id": "x", "label": "robot", "ok": True, "detail": ""}])
    assert svc.status()["console"]["preflight"]["complete"] is True
    svc.console._auto_cache = (time.time(), [{"id": "x", "label": "Robot reachable", "ok": False, "detail": ""}])
    assert svc.status()["console"]["preflight"] == {"complete": False, "open": 1, "reason": "Robot reachable"}
