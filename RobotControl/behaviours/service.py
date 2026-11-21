"""Behaviour service: the Unreal panel's buttons call this local HTTP API (127.0.0.1 only).

    python -m robotcontrol.behaviours.service            # then press Play in Unreal

Endpoints (JSON):
  GET  /status                 state, last lines of output, which buttons are allowed and why not
  POST /preview   {params}     run the behaviour on the Unreal twin (behaviours.runner --target urlab)
  POST /check     {params}     READ-ONLY run on the robot (nothing published): robot reachable, head centred
  POST /run       {params, "confirm": true}
                               powered run on the robot + live mirror of the measured angles in Unreal
  POST /stop                   interrupt whatever is running (robot: Ctrl-C -> head released, then exit)
  POST /assistant/interpret    plain words -> suggestion (voice), never starts anything
  GET  /robustness             latest robustness sweep report (logs/robustness/latest.json)
  POST /robustness/run         run the robustness sweep as job "robustness"
  GET  /walk/state, POST /walk/start|command|reset|stop   Level 2 walk sandbox (simulation only)
  Operator console (health, events, history, power, IMU, camera, versions, e-stop, limits, modes, poses, motions,
  store, calibration, network, pre-flight, docs, chat): behaviours/console.py
  Contract for the last four groups: docs/service_api.md
params: {"behaviour": "neck_turn", "amplitude": deg, "freq": Hz, "duration": s}

Robot gating (all must hold, else /run is refused with the reason):
  * a preview with exactly these params finished cleanly (exit 0) in this service session
  * a read-only /check with these params passed less than CHECK_VALID_S ago
  * "confirm": true (the panel asks: robot on stand, head forward, remote in hand)
  * the pre-flight checklist is complete (GET/POST /preflight; manual ticks valid 30 min)
  * the params are inside the session limits (POST /limits can only tighten them)
  * nothing else is running
STOP is a convenience: it goes over the network. The remote (L2+B for 5 s) is the emergency stop.
"""

from __future__ import annotations

import argparse
import base64
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
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

from .. import LOGS_DIR, PROJECT_ROOT
from ..configs.joint_map import JointMap
from ..configs.sim_calibration import SimCalibration
from .mirror import ROBOT, ROBOT_DIR, UnrealMirror, robot_stream
from .console import Console
from .move_joint import validate_move_request
from .robot_view import StandRenderer
from .stand_behaviours import BEHAVIOUR_SPECS
from .voice_intent import interpret as interpret_intent

CHECK_VALID_S = 300.0
ROBUSTNESS_REPORT = PROJECT_ROOT / "logs/robustness/latest.json"
PROGRESS_RE = re.compile(r"^@progress (\d+)/(\d+)")
LIMITS = {"amplitude": (1.0, 30.0), "freq": (0.05, 0.5), "duration": (2.0, 60.0)}
BEHAVIOURS = tuple(BEHAVIOUR_SPECS.keys())


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


def parse_telemetry(line: str) -> Optional[dict]:
    """Full telemetry snapshot from a runner '@telemetry {...}' JSON line."""
    s = line.strip()
    if not s.startswith("@telemetry "):
        return None
    try:
        obj = json.loads(s[11:])
        return obj if isinstance(obj, dict) else None
    except ValueError:
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
        self.job: Optional[str] = None  # "preview" | "check" | "run" | "monitor" | "snapshot" | "robustness" | "walk"
        self.progress: Optional[dict] = None  # {"job", "done", "total"} from a job's "@progress k/n" lines
        self.walk = None  # WalkSandbox, created on first use (loads MuJoCo + the Level 2 policy)
        self.proc: Optional[subprocess.Popen] = None
        self.previewed: Optional[dict] = None
        self.checked: Optional[dict] = None
        self.checked_at = 0.0
        self.last_result = ""
        # per job: {"ok": bool, "amplitude": deg, "reason": short plain-language reason, "at": epoch s}
        self.results: dict = {}
        # latest head readout per robot: {"virtual"|"real": {yaw, pitch, target, other, phase, t, source, at}}
        self.head: dict = {}
        # latest full telemetry per robot: {"virtual"|"real"|"practice": {imu_rpy_deg, q_deg, ...}}
        self.telemetry: dict = {}
        self.last_real_q_deg: Optional[list] = None
        self.real_connected: bool = False
        self.address = address
        self.mirror_enabled = mirror_enabled
        self.mirror_stop = threading.Event()
        self.mirror_procs: list = []
        self.mirror_samples = 0
        self.jm = JointMap.load()
        self.practice_q_deg: list[float] = [round(math.degrees(float(d)), 1) for d in self.jm.defaults]
        self.calibration = SimCalibration.load(jm=self.jm)
        self.stand_renderer = StandRenderer(self.jm)
        self.console = Console(self)

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

    def update_telemetry(self, who: str, pkt: dict, source: str) -> None:
        cur = dict(self.telemetry.get(who, {}))
        if cur.get("source") != source:
            cur = {}
        cur.update({k: v for k, v in pkt.items() if v is not None})
        cur["source"], cur["at"] = source, time.time()
        self.telemetry[who] = cur
        self.console.on_telemetry(who, pkt)
        if who == "real" and isinstance(cur.get("q_deg"), list) and len(cur["q_deg"]) == self.jm.num_joints:
            self.last_real_q_deg = [float(v) for v in cur["q_deg"]]
            self.real_connected = True
            if cur.get("bus_v") is not None:
                self.calibration.bus_voltage_v = float(cur["bus_v"])
        elif who == "practice" and isinstance(cur.get("q_deg"), list) and len(cur["q_deg"]) == self.jm.num_joints:
            self.practice_q_deg = [float(v) for v in cur["q_deg"]]

    def _sync_unreal_to_real(self) -> None:
        if not self.mirror_enabled or not self.last_real_q_deg:
            return
        try:
            m = UnrealMirror(self.jm, self.address)
            try:
                m.push([math.radians(v) for v in self.last_real_q_deg])
            finally:
                m.close()
            self.log("twin synced to physical robot pose")
        except Exception as e:
            self.log(f"twin pose sync skipped: {e}")

    def why_not_run(self, p: Optional[dict], practice: bool = False, cheap: bool = False) -> Optional[str]:
        """cheap: for /status (polled often, under the lock) the pre-flight part uses cached checks only."""
        if self.job:
            return f"{self.job} is running"
        if p is None:
            return "no parameters"
        if self.previewed != p:
            return "preview these settings in the twin first"
        if self.checked != p or time.time() - self.checked_at > CHECK_VALID_S:
            return "run the read-only robot check first (valid 5 min)"
        if not practice:
            if cheap:
                b = self.console.preflight_brief()
                return None if b["complete"] else f"complete the pre-flight checklist ({b['reason']})"
            return self.console.preflight_why_not()
        return None

    def status(self) -> dict:
        with self.lock:
            now = time.time()
            return {
                "job": self.job,
                "progress": dict(self.progress) if self.progress else None,
                "last_result": self.last_result,
                "previewed": self.previewed,
                "checked": self.checked,
                "check_age_s": round(now - self.checked_at, 1) if self.checked else None,
                "can_run_robot": self.why_not_run(self.previewed, cheap=True) is None,
                "why_not_run": self.why_not_run(self.previewed, cheap=True),
                "real_connected": self.real_connected,
                "mirror_samples": self.mirror_samples,
                "results": dict(self.results),
                "head": {k: dict(v, age_s=round(now - v["at"], 1)) for k, v in list(self.head.items())},
                "telemetry": {k: dict(v, age_s=round(now - v["at"], 1)) for k, v in list(self.telemetry.items())},
                "behaviours": [
                    {
                        "id": s.id,
                        "label": s.label,
                        "desc": s.desc,
                        "primary_label": s.primary_label,
                        "max_deg": s.max_deg,
                    }
                    for s in BEHAVIOUR_SPECS.values()
                ],
                "joint_names": [n.replace("_joint", "") for n in self.jm.names],
                "joint_limits_deg": [
                    [round(math.degrees(float(lo)), 1), round(math.degrees(float(hi)), 1)]
                    for lo, hi in zip(self.jm.limits_lo, self.jm.limits_hi)
                ],
                "joint_defaults_deg": [round(math.degrees(float(d)), 1) for d in self.jm.defaults],
                "calibration": self.calibration.summary(),
                "console": {
                    "limits": dict(self.console.limits.values),
                    "limits_tightened": self.console.limits.describe()["tightened"],
                    "event_counts": self.console.events.counts(now - 600),
                    "preflight": self.console.preflight_brief(),
                    "recording": self.console.recorder.status(),
                    "player": {k: self.console.player.state.get(k) for k in ("running", "kind", "name", "t")},
                },
                "lines": list(self.lines)[-12:],
            }

    def _start(self, job: str, cmd: list, p: dict, pty: bool = False) -> None:
        self.job = job
        self._job_is_practice = bool(p.get("_practice"))
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
        who = "practice" if p.get("_practice") else ("virtual" if job in ("preview", "move_virtual") else "real")
        for line in proc.stdout:
            pm = PROGRESS_RE.match(line)
            if pm:
                self.progress = {"job": job, "done": int(pm.group(1)), "total": int(pm.group(2))}
                continue
            if "post-quantum" in line or "store now" in line or "pq.html" in line or not line.strip():
                continue
            tm = parse_telemetry(line)
            if tm is not None:
                self.update_telemetry(who, tm, source=job)
                if who == "practice":
                    self.mirror_samples += 1
                continue  # 5 Hz telemetry: panel only, keep the log readable
            h = parse_head(line)
            if h is not None:
                self.update_head(who, h, source=job)
                if who == "real":
                    self.real_connected = True
                if line.lstrip().startswith("@head"):
                    continue  # 5 Hz readout: panel only, keep the log readable
            self.log(line)
            if not line.startswith((" ", "	")):  # skip indented traceback frames
                reason = failure_reason(line) or reason
        code = proc.wait()
        with self.lock:
            ok = code == 0
            if not ok and who == "real":
                self.real_connected = False
            self.results[job] = {
                "ok": ok,
                "amplitude": p["amplitude"],
                "at": time.time(),
                "reason": "" if ok else (reason or f"exit code {code}"),
            }
            if job == "preview":
                self.previewed = p if ok else None  # a failed virtual run clears step 1
            if job == "check" and ok:
                clean_p = {k: v for k, v in p.items() if not k.startswith("_")}
                self.checked, self.checked_at = clean_p, time.time()
            self.last_result = f"{job} {'OK' if ok else f'FAILED (exit {code})'}"
            self.log(self.last_result)
            self.job, self.proc = None, None
            self.progress = None
        self.console.on_job_done(job, ok, "" if ok else (reason or f"exit code {code}"), bool(p.get("_practice")))
        if job in ("check", "move_real") and ok and not p.get("_practice"):
            self._sync_unreal_to_real()
        if job == "run":
            self.checked = None  # a fresh check is required before the next robot run
            time.sleep(1.0)
            self.mirror_stop.set()

    # ------------------------------------------------------------------ actions
    def preview(self, p: dict, target: str = "urlab") -> dict:
        if target not in ("urlab", "sim"):
            raise ValueError(f"preview target must be 'urlab' or 'sim', got {target!r}")
        with self.lock:
            if self.job:
                return {"ok": False, "error": f"{self.job} is running"}
            cmd = [sys.executable, "-u", "-m", "robotcontrol.behaviours.runner"] + runner_args(p) + ["--target", target]
            if self.last_real_q_deg and len(self.last_real_q_deg) == self.jm.num_joints:
                cmd += ["--start-q-deg", ",".join(f"{v:.2f}" for v in self.last_real_q_deg)]
            self._start("preview", cmd, p)
        return {"ok": True}

    def check(self, p: dict, practice: bool = False) -> dict:
        with self.lock:
            if self.job:
                return {"ok": False, "error": f"{self.job} is running"}
            if practice:
                meta = dict(p, _practice=True)
                cmd = (
                    [sys.executable, "-u", "-m", "robotcontrol.behaviours.runner"]
                    + runner_args(p)
                    + [
                        "--target",
                        "sim",
                        "--duration",
                        "2",
                        "--start-q-deg",
                        ",".join(f"{v:.2f}" for v in self.practice_q_deg),
                    ]
                )
                self.log("practice check: simulating pre-run check on virtual stand twin")
                self._start("check", cmd, meta)
                return {"ok": True}
            remote = (
                f"cd {ROBOT_DIR} && PYTHONUNBUFFERED=1 timeout 60 ./run.sh robotcontrol.behaviours.runner "
                + " ".join(runner_args(p))
                + " --target real"
            )
            self._start("check", ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", ROBOT, remote], p)
        return {"ok": True}

    def run(self, p: dict, confirm: bool, practice: bool = False) -> dict:
        with self.lock:
            if not confirm:
                return {"ok": False, "error": "confirmation required"}
            why = self.why_not_run(p, practice)
            if why:
                return {"ok": False, "error": why}
            if practice:
                self.mirror_samples = 0
                meta = dict(p, _practice=True)
                cmd = (
                    [sys.executable, "-u", "-m", "robotcontrol.behaviours.runner"]
                    + runner_args(p)
                    + ["--target", "sim", "--start-q-deg", ",".join(f"{v:.2f}" for v in self.practice_q_deg)]
                )
                self.log(f"practice run: executing {p['behaviour']} on virtual stand twin")
                self._start("run", cmd, meta)
                return {"ok": True}
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

    def move_joint(self, joint: str, target_deg: float, target: str = "real", confirm: bool = False) -> dict:
        if target not in ("real", "urlab", "sim", "practice"):
            raise ValueError(f"target must be 'real', 'practice', 'urlab', or 'sim', got {target!r}")
        _, clamped_rad = validate_move_request(
            self.jm, joint, float(target_deg), "real" if target in ("real", "practice") else target
        )
        capped, _ = self.console.limits.clamp_joint(self.jm, joint, math.degrees(clamped_rad))  # session limits
        clamped_deg = round(capped, 2)
        meta = {"behaviour": joint, "amplitude": clamped_deg, "target": target, "_practice": target == "practice"}
        with self.lock:
            if target == "real" and not confirm:
                return {"ok": False, "error": "confirmation required to move physical joint"}
            if target == "real" and (why := self.console.preflight_why_not()):
                return {"ok": False, "error": why}
            if self.job:
                return {"ok": False, "error": f"{self.job} is running"}
            if target == "real":
                b64 = base64.b64encode(Path(__file__).with_name("move_joint.py").read_bytes()).decode("ascii")
                py_cmd = (
                    f"import base64,sys,robotcontrol.behaviours;"
                    f'sys.argv=["robotcontrol.behaviours.move_joint","{joint}","{clamped_deg:.2f}","--target","real","--go","--yes"];'
                    f'ns={{"__name__":"__main__","__package__":"robotcontrol.behaviours"}};'
                    f'exec(compile(base64.b64decode("{b64}").decode("utf-8"),"move_joint.py","exec"),ns)'
                )
                remote = (
                    f"cd {ROBOT_DIR} && ([ -f env.sh ] && source env.sh; "
                    f'LD_LIBRARY_PATH="${{CYCLONEDDS_HOME:-}}/lib:${{LD_LIBRARY_PATH:-}}" '
                    f"PYTHONUNBUFFERED=1 ./.venv/bin/python -u -c '{py_cmd}')"
                )
                self._start(
                    "move_real",
                    ["ssh", "-tt", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", ROBOT, remote],
                    meta,
                    pty=True,
                )
            else:
                sim_tgt = "sim" if target == "practice" else target
                cmd = [
                    sys.executable,
                    "-u",
                    "-m",
                    "robotcontrol.behaviours.move_joint",
                    joint,
                    f"{clamped_deg:.2f}",
                    "--target",
                    sim_tgt,
                ]
                if target == "practice":
                    cmd += ["--start-q-deg", ",".join(f"{v:.2f}" for v in self.practice_q_deg)]
                    job_name = "move_practice"
                else:
                    virt_q = (self.telemetry.get("virtual") or {}).get("q_deg") or self.last_real_q_deg
                    if isinstance(virt_q, list) and len(virt_q) == self.jm.num_joints:
                        cmd += ["--start-q-deg", ",".join(f"{float(v):.2f}" for v in virt_q)]
                    job_name = "move_virtual"
                self._start(job_name, cmd, meta)
        return {"ok": True}

    def render_stand_png(
        self, mode: str = "real", cam: str = "iso", joint_idx: Optional[int] = None, deg: Optional[float] = None
    ) -> bytes:
        with self.lock:
            tel = self.telemetry.get("virtual" if mode == "virtual" else "real") or {}
            if mode == "real" and not tel.get("q_deg"):
                tel = self.telemetry.get("practice") or tel
            q_deg = list(tel.get("q_deg") or [])
            if len(q_deg) != self.jm.num_joints:
                if mode == "real" and self.last_real_q_deg and len(self.last_real_q_deg) == self.jm.num_joints:
                    q_deg = list(self.last_real_q_deg)
                else:
                    q_deg = [round(math.degrees(float(d)), 1) for d in self.jm.defaults]
                    if mode == "real":
                        rh = self.head.get("real") or {}
                        if rh.get("pitch") is not None:
                            q_deg[self.jm.index_of("head_pitch_joint")] = float(rh["pitch"])
                        if rh.get("yaw") is not None:
                            q_deg[self.jm.index_of("head_yaw_joint")] = float(rh["yaw"])
            if joint_idx is not None and 0 <= joint_idx < self.jm.num_joints and deg is not None:
                q_deg[joint_idx] = float(deg)
        return self.stand_renderer.render_png(q_deg, cam_name=cam)

    def _consume_stream(
        self, source_name: str, stop_job_on_exit: bool = False, max_samples: Optional[int] = None
    ) -> None:
        self.mirror_stop.clear()
        self.mirror_samples = 0
        procs: list = []
        self.mirror_procs = procs
        mirror: Optional[UnrealMirror] = None
        if self.mirror_enabled and not max_samples:
            try:
                mirror = UnrealMirror(self.jm, self.address)
            except Exception as e:
                self.log(f"mirror unavailable: {e}")
                if not stop_job_on_exit:
                    return
        if mirror is not None:
            self.log("mirror: showing the robot's measured angles in Unreal")

        i_yaw, i_pitch = self.jm.index_of("head_yaw_joint"), self.jm.index_of("head_pitch_joint")
        reason = ""
        try:
            dur = 2.0 if max_samples else 0.0
            for pkt in robot_stream(
                10.0 if max_samples else (20.0 if stop_job_on_exit else 30.0),
                procs,
                include_telemetry=True,
                duration=dur,
            ):
                if self.mirror_stop.is_set():
                    break
                q = pkt["q"]
                if mirror is not None:
                    mirror.push(q)
                self.mirror_samples += 1
                self.update_head(
                    "real", {"yaw": math.degrees(q[i_yaw]), "pitch": math.degrees(q[i_pitch])}, source=source_name
                )
                tpkt = dict(pkt)
                tpkt["q_deg"] = [round(math.degrees(float(v)), 1) for v in q]
                tpkt.pop("q", None)
                self.update_telemetry("real", tpkt, source=source_name)
                if max_samples and self.mirror_samples >= max_samples:
                    break
            if max_samples and self.mirror_samples > 0:
                threading.Thread(target=self._sync_unreal_to_real, daemon=True).start()
            if stop_job_on_exit and self.mirror_samples == 0 and not self.mirror_stop.is_set():
                reason = "physical robot 192.168.123.164 unreachable or no lowstate yet"
                with self.lock:
                    self.real_connected = False
        except Exception as e:
            reason = str(e)[:90]
            self.log(f"{source_name} stopped: {e}")
        finally:
            for pr in procs:
                try:
                    pr.terminate()
                except Exception:
                    pass
            self.mirror_procs = []
            if mirror is not None:
                mirror.close()
            self.log(f"{source_name}: ended after {self.mirror_samples} samples")
            if stop_job_on_exit:
                with self.lock:
                    ok = self.mirror_samples > 0 or self.mirror_stop.is_set()
                    self.results[source_name] = {
                        "ok": ok,
                        "amplitude": 0.0,
                        "at": time.time(),
                        "reason": "" if ok else reason,
                    }
                    self.last_result = f"{source_name} OK" if ok else f"{source_name} FAILED ({reason})"
                    if self.job == source_name:
                        self.job = None

    def _mirror(self) -> None:
        self._consume_stream("run", stop_job_on_exit=False)

    def _practice_stream(self) -> None:
        self.mirror_stop.clear()
        self.mirror_samples = 0
        from .backends import SimBackend

        i_yaw, i_pitch = self.jm.index_of("head_yaw_joint"), self.jm.index_of("head_pitch_joint")
        start_q = {n: math.radians(v) for n, v in zip(self.jm.names, self.practice_q_deg)}
        backend = SimBackend(self.jm, (), start_q=start_q, calibration=self.calibration, realistic=True, settle_s=0.3)
        t0 = time.monotonic()
        try:
            while not self.mirror_stop.is_set() and (time.monotonic() - t0) < 20.0:
                backend.advance(time.monotonic() - t0)
                q, _, _ = backend.read()
                self.mirror_samples += 1
                self.update_head(
                    "practice", {"yaw": math.degrees(q[i_yaw]), "pitch": math.degrees(q[i_pitch])}, source="monitor"
                )
                tpkt = {
                    "t": round(time.monotonic() - t0, 2),
                    "q_deg": [round(math.degrees(float(v)), 1) for v in q],
                    "temps_c": [34] * self.jm.num_joints,
                    "max_temp_c": 34,
                    "bus_v": self.calibration.bus_voltage_v,
                    "mode_machine": 1,
                    "active_motors": 0,
                }
                tpkt.update(backend.telemetry())
                self.update_telemetry("practice", tpkt, source="monitor")
                time.sleep(0.1)
        finally:
            backend.close()
            self.log(f"practice monitor: ended after {self.mirror_samples} samples")
            with self.lock:
                self.results["monitor"] = {"ok": True, "amplitude": 0.0, "at": time.time(), "reason": ""}
                self.last_result = "monitor OK"
                if self.job == "monitor":
                    self.job = None

    def monitor(self, practice: bool = False) -> dict:
        with self.lock:
            if self.job:
                return {"ok": False, "error": f"{self.job} is running"}
            self.job = "monitor"
            self.last_result = ""
            if practice:
                self.log("practice monitor: read-only live stream on virtual stand twin")
                threading.Thread(target=self._practice_stream, daemon=True).start()
                return {"ok": True}
            self.log("monitor: read-only live telemetry + Unreal twin mirror")
        threading.Thread(target=self._consume_stream, args=("monitor", True), daemon=True).start()
        return {"ok": True}

    def snapshot(self, practice: bool = False) -> dict:
        if practice:
            with self.lock:
                if self.job:
                    return {"ok": False, "error": f"{self.job} is running"}
                i_yaw, i_pitch = self.jm.index_of("head_yaw_joint"), self.jm.index_of("head_pitch_joint")
                q_deg = list(self.practice_q_deg)
                self.update_head("practice", {"yaw": q_deg[i_yaw], "pitch": q_deg[i_pitch]}, source="snapshot")
                self.update_telemetry(
                    "practice",
                    {
                        "t": 0.0,
                        "q_deg": q_deg,
                        "imu_rpy_deg": [0.0, 0.0, 0.0],
                        "temps_c": [34] * self.jm.num_joints,
                        "max_temp_c": 34,
                        "bus_v": self.calibration.bus_voltage_v,
                        "mode_machine": 1,
                        "active_motors": 0,
                    },
                    source="snapshot",
                )
                self.results["snapshot"] = {"ok": True, "amplitude": 0.0, "at": time.time(), "reason": ""}
                self.last_result = "snapshot OK (practice)"
                self.log("practice snapshot: read virtual stand twin pose (0% motor power)")
            return {"ok": True}
        with self.lock:
            if self.job:
                return {"ok": False, "error": f"{self.job} is running"}
            self.job = "snapshot"
            self.last_result = ""
            self.log("snapshot: reading current physical robot joint angles (0% motor power)")
        threading.Thread(target=self._consume_stream, args=("snapshot", True, 1), daemon=True).start()
        return {"ok": True}

    # ------------------------------------------------------------------ voice + robustness (docs/service_api.md)
    def interpret(self, body: dict) -> dict:
        return interpret_intent(
            str(body.get("text", "")),
            arm_unlocked=bool(body.get("arm_unlocked", False)),
            walk_active=bool(body.get("walk_active", False)),
            jm=self.jm,
        )

    def robustness_report(self) -> dict:
        try:
            rep = json.loads(ROBUSTNESS_REPORT.read_text(encoding="utf-8"))
            rep["available"] = True
            return rep
        except FileNotFoundError:
            return {"available": False}
        except (OSError, json.JSONDecodeError) as e:
            return {"available": False, "error": f"unreadable report: {e}"}

    def robustness_run(self, quick: bool) -> dict:
        with self.lock:
            if self.job:
                return {"ok": False, "error": f"{self.job} is running", "_code": 409}
            cmd = [sys.executable, "-u", "-m", "robotcontrol.tools.robustness_sweep"] + (["--quick"] if quick else [])
            self.progress = {"job": "robustness", "done": 0, "total": 0}
            self._start("robustness", cmd, {"amplitude": 0.0})
        return {"ok": True}

    # ------------------------------------------------------------------ Level 2 walk sandbox (simulation only)
    def _walk(self):
        if self.walk is None:
            from ..walk.sandbox import WalkSandbox

            factory = (lambda: UnrealMirror(self.jm, self.address)) if self.mirror_enabled else None
            self.walk = WalkSandbox(mirror_factory=factory, log=self.log)
        return self.walk

    def walk_state(self) -> dict:
        return self._walk().state()

    def walk_stop(self) -> dict:
        res = self._walk().stop()
        with self.lock:
            if self.job == "walk":
                self.job = None
                self.last_result = "walk stopped"
        return res

    def walk_post(self, action: str, body: dict) -> dict:
        w = self._walk()
        if action == "start":
            with self.lock:
                if self.job and self.job != "walk":
                    return {"ok": False, "error": f"{self.job} is running", "_code": 409}
                self.job = "walk"  # reserve URLab + the job slot while the scene loads
            res = w.start(str(body.get("course", "loop")), str(body.get("mode", "policy")))
            if not res.get("ok"):
                with self.lock:
                    self.job = None
            else:
                self.last_result = ""
            return res
        if action == "command":
            vx, vy, wz = self.console.limits.scale_walk(
                float(body.get("vx", 0.0)), float(body.get("vy", 0.0)), float(body.get("wz", 0.0))
            )
            return w.command(vx, vy, wz)
        if action == "reset":
            return w.reset()
        if action == "stop":
            return self.walk_stop()
        return {"ok": False, "error": f"unknown walk action {action!r}", "_code": 404}

    def stop(self) -> dict:
        with self.lock:
            job, proc = self.job, self.proc
        if job == "walk" and self.walk is not None:
            self.log("STOP requested (walk)")
            return self.walk_stop()
        if job in ("motion", "pose", "mode"):
            self.log(f"STOP requested ({job})")
            self.console.player.stop()
            return {"ok": True}
        if job in ("monitor", "snapshot"):
            self.log(f"STOP requested ({job})")
            self.mirror_stop.set()
            for pr in list(self.mirror_procs):
                try:
                    pr.terminate()
                except Exception:
                    pass
            return {"ok": True}
        if not proc:
            return {"ok": True, "note": "nothing running"}
        self.log(f"STOP requested ({job})")
        if job in ("run", "move_real") and not getattr(self, "_job_is_practice", False):
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
                    "pkill -INT -f robotcontrol.behaviours.",
                ],
                timeout=15,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        elif job == "check" and not getattr(self, "_job_is_practice", False):
            proc.terminate()
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

        def _read_body(self) -> bytes:
            """Request body with Content-Length or chunked transfer encoding (a client that streams JSON, like
            .NET PostAsJsonAsync, sends no Content-Length: reading 0 bytes would silently drop every parameter)."""
            if "chunked" in (self.headers.get("Transfer-Encoding") or "").lower():
                out = b""
                while True:
                    size = int(self.rfile.readline().split(b";")[0].strip() or b"0", 16)
                    if size == 0:
                        self.rfile.readline()  # trailing CRLF after the last chunk
                        return out
                    out += self.rfile.read(size)
                    self.rfile.readline()  # CRLF after each chunk
            return self.rfile.read(int(self.headers.get("Content-Length") or 0))

        def _reply(self, res: dict) -> None:
            """New endpoints: HTTP status from the result (409 busy, 400 refused, 200 ok)."""
            code = res.pop("_code", None) or (200 if res.get("ok") else 400)
            self._send(code, res)

        def _console(self, res) -> bool:
            if res is None:
                return False
            if len(res) == 3:
                code, data, ctype = res
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self._send(*res)
            return True

        def do_GET(self):
            try:
                if self._console(svc.console.handle_get(self.path)):
                    return
            except (ValueError, KeyError) as e:
                return self._send(400, {"ok": False, "error": str(e)})
            if self.path in ("/", "/index.html"):
                html_path = os.path.join(os.path.dirname(__file__), "web_ui.html")
                with open(html_path, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif self.path.startswith("/status"):
                self._send(200, svc.status())
            elif self.path.startswith("/robustness"):
                self._send(200, svc.robustness_report())
            elif self.path.startswith("/walk/state"):
                self._send(200, svc.walk_state())
            elif self.path.startswith("/robot_view.png"):
                qs = parse_qs(urlparse(self.path).query)
                mode = (qs.get("mode") or ["real"])[0]
                cam = (qs.get("cam") or ["iso"])[0]
                j_raw = (qs.get("j") or [None])[0]
                d_raw = (qs.get("deg") or [None])[0]
                j_idx = int(j_raw) if j_raw is not None and j_raw.isdigit() else None
                d_val = float(d_raw) if d_raw is not None else None
                png = svc.render_stand_png(mode=mode, cam=cam, joint_idx=j_idx, deg=d_val)
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(png)))
                self.end_headers()
                self.wfile.write(png)
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            try:
                body = json.loads(self._read_body() or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("body must be a JSON object")
                if self._console(svc.console.handle_post(self.path, body)):
                    return
                if self.path == "/stop":
                    return self._send(200, svc.stop())
                if self.path == "/assistant/interpret":
                    return self._send(200, svc.interpret(body))
                if self.path == "/robustness/run":
                    return self._reply(svc.robustness_run(quick=bool(body.get("quick", False))))
                if self.path.startswith("/walk/"):
                    return self._reply(svc.walk_post(self.path[len("/walk/") :], body))
                if self.path == "/monitor":
                    return self._send(200, svc.monitor(practice=bool(body.get("practice", False))))
                if self.path == "/snapshot":
                    return self._send(200, svc.snapshot(practice=bool(body.get("practice", False))))
                if self.path == "/move_joint":
                    return self._send(
                        200,
                        svc.move_joint(
                            joint=str(body.get("joint", "")),
                            target_deg=float(body["target_deg"]),
                            target=str(body.get("target", "real")),
                            confirm=bool(body.get("confirm", False)),
                        ),
                    )
                p = parse_params(body)
                svc.console.limits.check_params(p)
                if self.path == "/preview":
                    return self._send(200, svc.preview(p, target=str(body.get("target", "urlab"))))
                if self.path == "/check":
                    return self._send(200, svc.check(p, practice=bool(body.get("practice", False))))
                if self.path == "/run":
                    return self._send(
                        200, svc.run(p, bool(body.get("confirm")), practice=bool(body.get("practice", False)))
                    )
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
    svc.console.events.path = LOGS_DIR / "events" / "events.jsonl"
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(svc))
    print(f"behaviour service on http://127.0.0.1:{args.port}  (Ctrl-C to quit)", flush=True)
    svc.snapshot()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        svc.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
