"""Behaviour service: the Unreal panel's buttons call this local HTTP API (127.0.0.1 only).

    python -m robotcontrol.behaviours.service            # then press Play in Unreal

Endpoints (JSON):
  GET  /status                 state, last lines of output, which buttons are allowed and why not
  POST /preview   {params}     run the behaviour on the Unreal twin (behaviours.runner --target urlab)
  POST /check     {params}     READ-ONLY run on the robot (nothing published): robot reachable, head centred
  POST /run       {params, "confirm": true}
                               powered run on the robot + live mirror of the measured angles in Unreal
  POST /stop                   interrupt whatever is running (robot: Ctrl-C -> head released, then exit)
params: {"behaviour": "neck_turn", "amplitude": deg, "freq": Hz, "duration": s}

Robot gating (all must hold, else /run is refused with the reason):
  * a preview with exactly these params finished cleanly (exit 0) in this service session
  * a read-only /check with these params passed less than CHECK_VALID_S ago
  * "confirm": true (the panel asks: robot on stand, head forward, remote in hand)
  * nothing else is running
STOP is a convenience: it goes over the network. The remote (L2+B for 5 s) is the emergency stop.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

from .. import PROJECT_ROOT
from ..configs.joint_map import JointMap
from .mirror import ROBOT, ROBOT_DIR, UnrealMirror, robot_stream, run_mirror

CHECK_VALID_S = 300.0
LIMITS = {"amplitude": (1.0, 30.0), "freq": (0.05, 0.5), "duration": (2.0, 60.0)}
BEHAVIOURS = ("neck_turn",)


def parse_params(body: dict) -> dict:
    p = {
        "behaviour": str(body.get("behaviour", "neck_turn")),
        "amplitude": float(body.get("amplitude", 10.0)),
        "freq": float(body.get("freq", 0.2)),
        "duration": float(body.get("duration", 20.0)),
    }
    if p["behaviour"] not in BEHAVIOURS:
        raise ValueError(f"unknown behaviour {p['behaviour']!r}")
    for k, (lo, hi) in LIMITS.items():
        if not lo <= p[k] <= hi:
            raise ValueError(f"{k} must be in [{lo}, {hi}], got {p[k]}")
    return p


def failure_reason(line: str) -> str:
    """Short reason for the panel from a runner output line, or "" if the line is not a failure."""
    s = line.strip()
    if s.startswith("ABORT"):
        s = s.split(":", 1)[1].strip() if ":" in s else s
        return s.split(" - ")[0][:90]
    if s.startswith("NOT STARTED"):
        return s.split(":", 1)[-1].strip()[:90]
    if "Resource temporarily unavailable" in s or "zmq.error.Again" in s:
        return "lost the connection to the Unreal twin (is it still running?)"
    for key in ("REFUSE", "Error:", "error:", "Connection refused", "timed out", "No route to host"):
        if key in s:
            return s[:90]
    return ""


_TICK = re.compile(
    r"t=\s*([\d.]+)s\s+(\w+)\s+head_yaw target\s+([+-]?[\d.]+) actual\s+([+-]?[\d.]+) deg"
    r"\s+other joints max change ([\d.]+)"
)


def parse_head(line: str) -> Optional[dict]:
    """Head readout from a runner line: the 5 Hz '@head k=v ...' line (yaw, pitch, target, other) or the 1 Hz
    progress line every deployed runner prints (yaw, target, other; no pitch). None if the line has neither."""
    s = line.strip()
    if s.startswith("@head "):
        d = dict(kv.split("=", 1) for kv in s[6:].split() if "=" in kv)
        out = {"phase": d.get("phase", "")}
        for k in ("t", "yaw", "pitch", "target", "other"):
            try:
                v = float(d[k])
                out[k] = None if math.isnan(v) else v
            except (KeyError, ValueError):
                pass
        return out
    m = _TICK.search(s)
    if m:
        return {
            "t": float(m.group(1)),
            "phase": m.group(2),
            "target": float(m.group(3)),
            "yaw": float(m.group(4)),
            "other": float(m.group(5)),
        }
    return None


def runner_args(p: dict) -> list:
    return [
        p["behaviour"],
        "--amplitude",
        f"{p['amplitude']:g}",
        "--freq",
        f"{p['freq']:g}",
        "--duration",
        f"{p['duration']:g}",
    ]


class Service:
    def __init__(self, address: str = "tcp://localhost", mirror_enabled: bool = True):
        self.lock = threading.Lock()
        self.lines = collections.deque(maxlen=200)
        self.job: Optional[str] = None  # "preview" | "check" | "run"
        self.proc: Optional[subprocess.Popen] = None
        self.previewed: Optional[dict] = None
        self.checked: Optional[dict] = None
        self.checked_at = 0.0
        self.last_result = ""
        # per job: {"ok": bool, "amplitude": deg, "reason": short plain-language reason, "at": epoch s}
        self.results: dict = {}
        # latest head readout per robot: {"virtual"|"real": {yaw, pitch, target, other, phase, t, source, at}}
        self.head: dict = {}
        self.address = address
        self.mirror_enabled = mirror_enabled
        self.mirror_stop = threading.Event()
        self.mirror_samples = 0
        self.jm = JointMap.load()

    # ------------------------------------------------------------------ helpers
    def log(self, s: str) -> None:
        self.lines.append(time.strftime("%H:%M:%S ") + s.rstrip())

    def update_head(self, who: str, h: dict, source: str) -> None:
        cur = dict(self.head.get(who, {}))
        if cur.get("source") != source:
            cur = {}  # a new job: drop the previous job's target / phase
        cur.update({k: v for k, v in h.items() if v is not None})
        cur["source"], cur["at"] = source, time.time()
        self.head[who] = cur

    def why_not_run(self, p: Optional[dict]) -> Optional[str]:
        if self.job:
            return f"{self.job} is running"
        if p is None:
            return "no parameters"
        if self.previewed != p:
            return "preview these settings in the twin first"
        if self.checked != p or time.time() - self.checked_at > CHECK_VALID_S:
            return "run the read-only robot check first (valid 5 min)"
        return None

    def status(self) -> dict:
        with self.lock:
            return {
                "job": self.job,
                "last_result": self.last_result,
                "previewed": self.previewed,
                "checked": self.checked,
                "check_age_s": round(time.time() - self.checked_at, 1) if self.checked else None,
                "can_run_robot": self.why_not_run(self.previewed) is None,
                "why_not_run": self.why_not_run(self.previewed),
                "mirror_samples": self.mirror_samples,
                "results": dict(self.results),
                "head": {k: dict(v, age_s=round(time.time() - v["at"], 1)) for k, v in list(self.head.items())},
                "lines": list(self.lines)[-12:],
            }

    def _start(self, job: str, cmd: list, p: dict, pty: bool = False) -> None:
        self.job = job
        self.last_result = ""
        self.log(f"{job}: {' '.join(cmd[-12:])}")
        kw = {}
        if os.name == "nt" and not pty:
            kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        self.proc = subprocess.Popen(
            cmd,
            cwd=str(PROJECT_ROOT),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            **kw,
        )
        threading.Thread(target=self._pump, args=(job, self.proc, p), daemon=True).start()

    def _pump(self, job: str, proc: subprocess.Popen, p: dict) -> None:
        reason = ""
        who = "virtual" if job == "preview" else "real"
        for line in proc.stdout:
            if "post-quantum" in line or "store now" in line or "pq.html" in line or not line.strip():
                continue
            h = parse_head(line)
            if h is not None:
                self.update_head(who, h, source=job)
                if line.lstrip().startswith("@head"):
                    continue  # 5 Hz readout: panel only, keep the log readable
            self.log(line)
            if not line.startswith((" ", "	")):  # skip indented traceback frames
                reason = failure_reason(line) or reason
        code = proc.wait()
        with self.lock:
            ok = code == 0
            self.results[job] = {
                "ok": ok,
                "amplitude": p["amplitude"],
                "at": time.time(),
                "reason": "" if ok else (reason or f"exit code {code}"),
            }
            if job == "preview":
                self.previewed = p if ok else None  # a failed virtual run clears step 1
            if job == "check" and ok:
                self.checked, self.checked_at = p, time.time()
            self.last_result = f"{job} {'OK' if ok else f'FAILED (exit {code})'}"
            self.log(self.last_result)
            self.job, self.proc = None, None
        if job == "run":
            self.checked = None  # a fresh check is required before the next robot run
            time.sleep(1.0)
            self.mirror_stop.set()

    # ------------------------------------------------------------------ actions
    def preview(self, p: dict) -> dict:
        with self.lock:
            if self.job:
                return {"ok": False, "error": f"{self.job} is running"}
            self._start(
                "preview",
                [sys.executable, "-u", "-m", "robotcontrol.behaviours.runner"] + runner_args(p) + ["--target", "urlab"],
                p,
            )
        return {"ok": True}

    def check(self, p: dict) -> dict:
        with self.lock:
            if self.job:
                return {"ok": False, "error": f"{self.job} is running"}
            remote = (
                f"cd {ROBOT_DIR} && PYTHONUNBUFFERED=1 timeout 60 ./run.sh robotcontrol.behaviours.runner "
                + " ".join(runner_args(p))
                + " --target real"
            )
            self._start("check", ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", ROBOT, remote], p)
        return {"ok": True}

    def run(self, p: dict, confirm: bool) -> dict:
        with self.lock:
            if not confirm:
                return {"ok": False, "error": "confirmation required"}
            why = self.why_not_run(p)
            if why:
                return {"ok": False, "error": why}
            remote = (
                f"cd {ROBOT_DIR} && PYTHONUNBUFFERED=1 ./run.sh robotcontrol.behaviours.runner "
                + " ".join(runner_args(p))
                + " --target real --go --yes"
            )
            self._start(
                "run", ["ssh", "-tt", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", ROBOT, remote], p, pty=True
            )
        if self.mirror_enabled:
            threading.Thread(target=self._mirror, daemon=True).start()
        return {"ok": True}

    def _mirror(self) -> None:
        self.mirror_stop.clear()
        self.mirror_samples = 0
        procs: list = []
        try:
            m = UnrealMirror(self.jm, self.address)
        except Exception as e:
            self.log(f"mirror unavailable: {e}")
            return
        self.log("mirror: showing the robot's measured angles in Unreal")

        i_yaw, i_pitch = self.jm.index_of("head_yaw_joint"), self.jm.index_of("head_pitch_joint")

        def count(q):
            self.mirror_samples += 1
            self.update_head("real", {"yaw": math.degrees(q[i_yaw]), "pitch": math.degrees(q[i_pitch])}, source="run")

        try:
            run_mirror(robot_stream(30.0, procs), m, self.mirror_stop, count)
        except Exception as e:
            self.log(f"mirror stopped: {e}")
        finally:
            for pr in procs:
                pr.terminate()
            m.close()
            self.log(f"mirror: ended after {self.mirror_samples} samples")

    def stop(self) -> dict:
        with self.lock:
            job, proc = self.job, self.proc
        if not proc:
            return {"ok": True, "note": "nothing running"}
        self.log(f"STOP requested ({job})")
        if job == "run":
            try:  # Ctrl-C through the remote pty -> runner releases the head
                proc.stdin.write("\x03")
                proc.stdin.flush()
            except Exception:
                pass
            subprocess.run(
                [
                    "ssh",
                    "-o",
                    "BatchMode=yes",
                    "-o",
                    "ConnectTimeout=5",
                    ROBOT,
                    "pkill -INT -f robotcontrol.behaviours.runner",
                ],
                timeout=15,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        elif os.name == "nt":
            proc.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            proc.send_signal(signal.SIGINT)
        return {"ok": True}


def make_handler(svc: Service):
    class H(BaseHTTPRequestHandler):
        def _send(self, code: int, obj: dict) -> None:
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/status"):
                self._send(200, svc.status())
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                if self.path == "/stop":
                    return self._send(200, svc.stop())
                p = parse_params(body)
                if self.path == "/preview":
                    return self._send(200, svc.preview(p))
                if self.path == "/check":
                    return self._send(200, svc.check(p))
                if self.path == "/run":
                    return self._send(200, svc.run(p, bool(body.get("confirm"))))
                self._send(404, {"ok": False, "error": "not found"})
            except (ValueError, json.JSONDecodeError) as e:
                self._send(400, {"ok": False, "error": str(e)})

    return H


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Local behaviour service for the Unreal panel")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--address", default="tcp://localhost", help="URLab server")
    ap.add_argument("--no-mirror", action="store_true")
    args = ap.parse_args(argv)
    svc = Service(args.address, mirror_enabled=not args.no_mirror)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(svc))
    print(f"behaviour service on http://127.0.0.1:{args.port}  (Ctrl-C to quit)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        svc.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
