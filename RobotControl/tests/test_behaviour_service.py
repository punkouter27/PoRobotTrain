"""Behaviour service: robot runs are gated on preview + fresh read-only check + confirmation."""

import json
import threading
import time
import urllib.request

import pytest

from robotcontrol.behaviours import service as S


@pytest.fixture
def svc(monkeypatch):
    s = S.Service(mirror_enabled=False)
    started = []

    def fake_start(job, cmd, p, pty=False):
        started.append((job, cmd, pty))
        s.job = job

    monkeypatch.setattr(s, "_start", fake_start)
    s.started = started
    return s


P = {"behaviour": "neck_turn", "amplitude": 10.0, "freq": 0.2, "duration": 20.0}


def test_params_validated():
    assert S.parse_params({}) == P
    with pytest.raises(ValueError):
        S.parse_params({"amplitude": 45})
    with pytest.raises(ValueError):
        S.parse_params({"behaviour": "dance"})


def test_run_refused_until_preview_and_check(svc):
    assert "confirmation" in svc.run(P, confirm=False)["error"]
    assert "preview" in svc.run(P, confirm=True)["error"]
    svc.previewed = dict(P)
    assert "check" in svc.run(P, confirm=True)["error"]
    svc.checked, svc.checked_at = dict(P), time.time() - S.CHECK_VALID_S - 1  # stale
    assert "check" in svc.run(P, confirm=True)["error"]
    svc.checked_at = time.time()
    other = dict(P, amplitude=20.0)  # different settings
    assert "preview" in svc.run(other, confirm=True)["error"]
    assert svc.run(P, confirm=True) == {"ok": True}
    job, cmd, pty = svc.started[-1]
    assert job == "run" and pty and "--go" in cmd[-1] and "--amplitude 10" in cmd[-1]
    assert "busy" not in svc.status() and svc.status()["job"] == "run"


def test_preview_targets_urlab_and_check_is_read_only(svc):
    svc.preview(P)
    assert svc.started[-1][1][-2:] == ["--target", "urlab"]
    svc.job = None
    svc.check(P)
    remote = svc.started[-1][1][-1]
    assert "--target real" in remote and "--go" not in remote


def test_http_api(svc):
    srv = S.ThreadingHTTPServer(("127.0.0.1", 0), S.make_handler(svc))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        st = json.load(urllib.request.urlopen(base + "/status"))
        assert st["can_run_robot"] is False
        req = urllib.request.Request(base + "/run", data=json.dumps(dict(P, confirm=True)).encode(), method="POST")
        assert "preview" in json.load(urllib.request.urlopen(req))["error"]
        bad = urllib.request.Request(base + "/preview", data=b'{"amplitude": 90}', method="POST")
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(bad)
    finally:
        srv.shutdown()


def test_failure_reason_is_short_and_plain():
    assert (
        S.failure_reason("ABORT at t=0.40s: right_knee_joint moved 5.1 deg (> 5.0 deg) - only head_yaw_joint may move")
        == "right_knee_joint moved 5.1 deg (> 5.0 deg)"
    )
    assert S.failure_reason("NOT STARTED: head is 14 deg off centre") == "head is 14 deg off centre"
    assert "Connection timed out" in S.failure_reason("ssh: connect to host 1.2.3.4 port 22: Connection timed out")
    assert S.failure_reason("t=  1.0s sweep head_yaw target +5.9 actual +7.1 deg") == ""


class _FakeProc:
    def __init__(self, lines, code):
        self.stdout, self._code = iter(lines), code

    def wait(self):
        return self._code


def test_results_reported_per_job(svc):
    svc.job = "preview"
    svc._pump("preview", _FakeProc(["done. other joints moved at most 1.2 deg\n"], 0), P)
    svc.job = "check"
    svc._pump("check", _FakeProc(["ssh: connect to host x port 22: Connection timed out\n"], 255), P)
    r = svc.status()["results"]
    assert r["preview"]["ok"] and r["preview"]["amplitude"] == 10.0 and r["preview"]["reason"] == ""
    assert not r["check"]["ok"] and "timed out" in r["check"]["reason"]
    assert svc.status()["why_not_run"].startswith("run the read-only robot check")


def test_failed_preview_clears_step1_and_names_the_error(svc):
    svc.previewed = P
    svc.job = "preview"
    lines = [
        "Traceback (most recent call last):\n",
        '  File "x.py", line 1\n',
        "zmq.error.Again: Resource temporarily unavailable\n",
    ]
    svc._pump("preview", _FakeProc(lines, 1), P)
    assert svc.previewed is None
    assert "Unreal twin" in svc.status()["results"]["preview"]["reason"]


def test_parse_head_lines():
    h = S.parse_head("@head t=4.20 phase=sweep yaw=5.10 pitch=-0.30 target=6.90 other=1.20")
    assert h == {"phase": "sweep", "t": 4.2, "yaw": 5.1, "pitch": -0.3, "target": 6.9, "other": 1.2}
    # the 1 Hz line every deployed runner prints (no pitch)
    h = S.parse_head("t= 18.0s sweep    head_yaw target   +6.9 actual   +2.0 deg   other joints max change 0.0 deg")
    assert h == {"t": 18.0, "phase": "sweep", "target": 6.9, "yaw": 2.0, "other": 0.0}
    assert S.parse_head("check OK") is None


def test_head_readout_in_status_and_not_in_log(svc):
    svc.job = "check"
    svc._pump(
        "check",
        _FakeProc(
            [
                "@head t=0.00 phase=centre yaw=2.00 pitch=-1.00 target=nan other=0.00\n",
                "t=  1.0s centre   head_yaw target   +0.0 actual   +2.1 deg   other joints max change 0.0 deg\n",
            ],
            0,
        ),
        P,
    )
    h = svc.status()["head"]["real"]
    assert h["yaw"] == 2.1 and h["pitch"] == -1.0 and h["source"] == "check" and "target" in h
    assert not any("@head" in line for line in svc.lines)


def test_different_behaviour_requires_own_preview_and_check(svc):
    hw = dict(P, behaviour="hand_wave")
    svc.previewed = dict(P)
    svc.checked, svc.checked_at = dict(P), time.time()
    assert "preview" in svc.run(hw, confirm=True)["error"]


def test_telemetry_parsed_and_monitor_lifecycle(svc, monkeypatch):
    pkt = {
        "t": 1.0,
        "phase": "sweep",
        "primary_label": "R.SHOULDER",
        "actual_deg": 5.2,
        "imu_rpy_deg": [0.1, -0.2, 0.0],
    }
    svc.job = "preview"
    svc._pump("preview", _FakeProc(["@telemetry " + json.dumps(pkt) + "\n", "done.\n"], 0), P)
    st = svc.status()
    assert st["telemetry"]["virtual"]["actual_deg"] == 5.2
    assert not any("@telemetry" in line for line in svc.lines)
    assert len(st["behaviours"]) == 4 and len(st["joint_names"]) == 26

    monkeypatch.setattr(S, "robot_stream", lambda *a, **k: iter([{"q": [0.0] * 26, "max_temp_c": 38, "bus_v": 48.0}]))
    assert svc.monitor() == {"ok": True}
    deadline = time.time() + 2.0
    while svc.status()["job"] == "monitor" and time.time() < deadline:
        time.sleep(0.02)
    st = svc.status()
    assert st["job"] is None
    assert st["telemetry"]["real"]["max_temp_c"] == 38
    assert st["results"]["monitor"]["ok"] is True


def test_web_ui_served_and_sim_preview_option(svc):
    srv = S.ThreadingHTTPServer(("127.0.0.1", 0), S.make_handler(svc))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        html = urllib.request.urlopen(base + "/").read().decode("utf-8")
        assert "PHYSICAL R1" in html and "26 DOF" in html
        req = urllib.request.Request(base + "/preview", data=json.dumps(dict(P, target="sim")).encode(), method="POST")
        assert json.load(urllib.request.urlopen(req)) == {"ok": True}
        assert svc.started[-1][1][-2:] == ["--target", "sim"]
    finally:
        srv.shutdown()


def test_preview_uses_last_real_pose_for_virtual_matching(svc):
    q_real = [round(i * 0.5, 2) for i in range(26)]
    pkt = {"t": 0.5, "phase": "centre", "primary_label": "HEAD YAW", "actual_deg": 1.0, "q_deg": q_real}
    svc.job = "check"
    svc._pump("check", _FakeProc(["@telemetry " + json.dumps(pkt) + "\n", "done.\n"], 0), P)
    assert svc.last_real_q_deg == q_real

    assert svc.preview(P) == {"ok": True}
    cmd = svc.started[-1][1]
    assert "--start-q-deg" in cmd
    arg = cmd[cmd.index("--start-q-deg") + 1]
    assert len(arg.split(",")) == 26
    assert arg.startswith("0.00,0.50,1.00")


def test_snapshot_reads_one_real_sample_over_http(svc, monkeypatch):
    samples = [{"t": 0.1, "q": [0.1] * 26, "max_temp_c": 34}, {"t": 0.2, "q": [0.2] * 26, "max_temp_c": 35}]
    monkeypatch.setattr(S, "robot_stream", lambda *a, **k: iter(samples))
    srv = S.ThreadingHTTPServer(("127.0.0.1", 0), S.make_handler(svc))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        req = urllib.request.Request(base + "/snapshot", data=b"{}", method="POST")
        assert json.load(urllib.request.urlopen(req)) == {"ok": True}
        deadline = time.time() + 2.0
        while svc.status()["job"] == "snapshot" and time.time() < deadline:
            time.sleep(0.02)
        st = svc.status()
        assert st["job"] is None
        assert st["mirror_samples"] == 1
        assert st["telemetry"]["real"]["source"] == "snapshot"
        assert st["telemetry"]["real"]["q_deg"][0] == 5.7
    finally:
        srv.shutdown()


def test_move_joint_safety_and_targets(svc):
    # Legs are stand-locked read-only on physical robot
    with pytest.raises(ValueError):
        svc.move_joint("left_knee_joint", 10.0, target="real", confirm=True)
    # Physical move requires confirmation
    refused = svc.move_joint("head_yaw_joint", 12.0, target="real", confirm=False)
    assert refused["ok"] is False and "confirmation" in refused["error"]
    # Virtual move runs without hardware confirmation
    assert svc.move_joint("head_yaw_joint", 12.0, target="sim", confirm=False) == {"ok": True}
    assert svc.started[-1][0] == "move_virtual"
    assert "robotcontrol.behaviours.move_joint" in svc.started[-1][1]
    svc.job = None
    # Confirmed physical move uses pty and sets job to move_real
    assert svc.move_joint("head_yaw_joint", 12.0, target="real", confirm=True) == {"ok": True}
    job, cmd, pty = svc.started[-1]
    assert job == "move_real" and pty is True and "move_joint" in cmd[-1]


def test_robot_view_png_endpoint(svc):
    srv = S.ThreadingHTTPServer(("127.0.0.1", 0), S.make_handler(svc))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        data = urllib.request.urlopen(base + "/robot_view.png?mode=real&cam=iso").read()
        assert data.startswith(b"\x89PNG\r\n\x1a\n") and len(data) > 5000
    finally:
        srv.shutdown()


def test_offline_practice_mode_runs_all_workflow_steps(svc):
    assert svc.status()["real_connected"] is False
    # Snapshot in practice mode immediately populates telemetry.practice without SSH
    assert svc.snapshot(practice=True) == {"ok": True}
    st = svc.status()
    assert st["telemetry"]["practice"]["source"] == "snapshot"
    assert len(st["telemetry"]["practice"]["q_deg"]) == 26

    # Check & Run in practice mode still enforce the Step 1 -> Step 2 -> confirm safety sequence
    assert svc.check(P, practice=True) == {"ok": True}
    assert svc.started[-1][0] == "check" and "--target" in svc.started[-1][1] and "sim" in svc.started[-1][1]
    svc.job = None
    svc.previewed = dict(P)
    svc.checked, svc.checked_at = dict(P), time.time()
    assert "confirmation" in svc.run(P, confirm=False, practice=True)["error"]
    assert svc.run(P, confirm=True, practice=True) == {"ok": True}
    assert svc.started[-1][0] == "run" and "sim" in svc.started[-1][1] and svc.started[-1][2] is False
    svc.job = None

    # Practice slider move works for upper body and still locks legs
    with pytest.raises(ValueError):
        svc.move_joint("left_knee_joint", 10.0, target="practice", confirm=True)
    assert svc.move_joint("head_yaw_joint", 8.0, target="practice", confirm=True) == {"ok": True}
    assert svc.started[-1][0] == "move_practice"
    svc.job = None

    # Practice monitor streams live simulated state and stops cleanly
    assert svc.monitor(practice=True) == {"ok": True}
    time.sleep(0.25)
    assert svc.stop() == {"ok": True}
    deadline = time.time() + 3.0
    while svc.status()["job"] is not None and time.time() < deadline:
        time.sleep(0.05)
    assert svc.status()["job"] is None and svc.status()["mirror_samples"] >= 1
