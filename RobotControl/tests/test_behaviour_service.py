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
    svc.checked, svc.checked_at = dict(P), time.time() - S.CHECK_VALID_S - 1      # stale
    assert "check" in svc.run(P, confirm=True)["error"]
    svc.checked_at = time.time()
    other = dict(P, amplitude=20.0)                                              # different settings
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
