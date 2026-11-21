"""Operator console for the behaviour service: the 20 Explore-style features (docs/service_api.md sections 4-7).

``Console`` hangs off ``Service`` (``svc.console``) and answers the new endpoints through ``handle_get`` /
``handle_post``. It sees every telemetry packet (``on_telemetry``) for the event log, history and teach mode.

Safety contract:
  * read-only features (health, events, history, power, IMU, camera, versions, network, docs, chat) never command;
  * motion playback, poses and modes run on the simulated stand twin (optionally mirrored onto Unreal); a request
    with target "real" is refused, because these power several joints at once;
  * the pre-flight checklist and session limits only add restrictions to the existing robot gates;
  * E-STOP stops every job, the walk sandbox and the simulation player (the remote's L2+B stays the real e-stop).
"""

from __future__ import annotations

import json
import threading
import math
import time
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

from .. import CONFIGS_DIR, LOGS_DIR, MODELS_DIR, PACKAGE_DIR
from . import ops
from .chat import ChatAssistant
from .mirror import ROBOT
from .motions import MODES, MotionLibrary, Recorder, SimPlayer, validate_clip
from .telemetry_log import TEMP_WARN_C, EventLog, TelemetryHistory, scan_telemetry

USER_DATA_DIR = PACKAGE_DIR / "user_data"  # git-ignored: the operator's motions, profiles and offsets
PLAY_TARGETS = ("sim", "urlab", "practice")
REAL_REFUSAL = (
    "not on the real robot: motions, poses and modes power several joints at once, so they run on the twin only "
    "(use a single-joint slider or a gated behaviour for the robot)"
)


def _q(path: str) -> tuple[str, dict]:
    u = urlparse(path)
    return u.path.rstrip("/") or "/", {k: v[0] for k, v in parse_qs(u.query).items()}


class Console:
    def __init__(self, svc, data_dir: Optional[Path] = None, events_path: Optional[Path] = None):
        self.svc = svc
        self.jm = svc.jm
        self.started_at = time.time()
        data = data_dir or USER_DATA_DIR
        try:
            cells = int(json.loads((CONFIGS_DIR / "battery.json").read_text(encoding="utf-8"))["series_cells"])
        except (OSError, ValueError, KeyError):
            cells = 13
        self.series_cells = cells
        self.events = EventLog(events_path)
        self.history = TelemetryHistory(self.jm.names, series_cells=cells)
        self.limits = ops.SessionLimits()
        self.library = MotionLibrary(self.jm, PACKAGE_DIR / "motions", data / "motions", CONFIGS_DIR / "poses.json")
        self.recorder = Recorder(self.jm)
        self.player = SimPlayer(self.jm, svc.calibration, self._publish, svc.log, mirror_factory=self._mirror_factory)
        self.offsets = ops.ZeroOffsets(self.jm, data / "joint_zero_offsets.json",
                                       getattr(svc.calibration, "zero_offset_rad", None))  # fmt: skip
        self.profiles = ops.ConnectionProfiles(data / "connection_profiles.json")
        self.preflight = ops.Preflight(self._preflight_auto)
        self._auto_cache: tuple[float, list] = (0.0, [])
        self._auto_refreshing = False
        self.chat = ChatAssistant(self._chat_ctx, lambda text, body: svc.interpret(dict(body, text=text)),
                                  self.library.poses)  # fmt: skip
        self.limits_deg = [
            (math.degrees(float(lo)), math.degrees(float(hi))) for lo, hi in zip(self.jm.limits_lo, self.jm.limits_hi)
        ]

    # ------------------------------------------------------------------ hooks from Service
    def on_telemetry(self, who: str, pkt: dict) -> None:
        self.history.add(who, pkt)
        self.recorder.on_telemetry(who, pkt)
        scan_telemetry(self.events, who, pkt, self.jm.names, self.limits_deg, self.series_cells)

    def on_job_done(self, job: str, ok: bool, reason: str, practice: bool) -> None:
        src = "practice" if practice else ("virtual" if job in ("preview", "move_virtual") else "real")
        if ok:
            self.events.record("info", "job", f"{job} finished", src)
        elif reason.startswith(("tracking", "torque", "other joint", "stale")) or "ABORT" in reason or "abort" in reason:
            self.events.record("critical", "trip", f"{job} safety trip: {reason}", src)
        else:
            self.events.record("warn", "fault", f"{job} failed: {reason}", src)

    def _publish(self, who: str, pkt: dict, job: str) -> None:
        self.svc.update_telemetry(who, pkt, source=job)

    def _mirror_factory(self):
        if not self.svc.mirror_enabled:
            raise RuntimeError("mirror disabled (--no-mirror)")
        from .mirror import UnrealMirror

        return UnrealMirror(self.jm, self.svc.address)

    # ------------------------------------------------------------------ helpers
    def _latest(self, who: Optional[str] = None) -> tuple[Optional[str], dict]:
        tel = self.svc.telemetry
        order = [who] if who else ["real", "practice", "virtual"]
        for w in order:
            t = tel.get(w)
            if t:
                return w, t
        return (who or None), {}

    def _source(self, qs: dict) -> str:
        s = qs.get("source")
        if s in ("real", "practice", "virtual"):
            return s
        srcs = self.history.sources()
        for s in ("real", "practice", "virtual"):
            if s in srcs:
                return s
        return "real"

    def _start_q_deg(self, who: str) -> list:
        if who == "practice":
            return list(self.svc.practice_q_deg)
        q = (self.svc.telemetry.get("virtual") or {}).get("q_deg")
        if isinstance(q, list) and len(q) == self.jm.num_joints:
            return [float(v) for v in q]
        if self.svc.last_real_q_deg:
            return list(self.svc.last_real_q_deg)
        return [round(math.degrees(float(d)), 1) for d in self.jm.defaults]

    def _claim(self, job: str) -> Optional[dict]:
        with self.svc.lock:
            if self.svc.job:
                return {"ok": False, "error": f"{self.svc.job} is running", "_code": 409}
            if self.player.running:
                return {"ok": False, "error": "the simulation player is busy", "_code": 409}
            self.svc.job = job
            self.svc.last_result = ""
        return None

    def _done(self, job: str):
        def cb(ok: bool, reason: str) -> None:
            with self.svc.lock:
                self.svc.results[job] = {"ok": ok, "amplitude": 0.0, "at": time.time(), "reason": reason}
                self.svc.last_result = f"{job} {'OK' if ok else 'FAILED'}" + (f" ({reason})" if reason else "")
                if self.svc.job == job:
                    self.svc.job = None
            self.events.record("info" if ok else "warn", "job", f"{job} {'finished' if ok else 'failed: ' + reason}")

        return cb

    # ------------------------------------------------------------------ health (#1), versions (#6)
    def health(self) -> dict:
        svc = self.svc
        urlab_host, urlab_port = ops._host_port(svc.address, 5555)
        robot_host = ROBOT.split("@", 1)[-1]
        _, real = self._latest("real")
        live = bool(real) and time.time() - real.get("at", 0) < 5.0
        lib = self.library.list()
        pols = ops.policy_versions()
        probes = [
            ("Services", "Behaviour service", False,
             lambda: (True, f"up {int(time.time() - self.started_at)} s, v{ops.SERVICE_VERSION}")),
            ("Services", "Unreal twin (URLab)", True, lambda: ops.tcp_probe(urlab_host, urlab_port, 0.3)[::2]),
            ("Services", f"Robot SSH ({robot_host}:22)", True, lambda: ops.tcp_probe(robot_host, 22, 0.5)[::2]),
            ("Services", "Robot state stream", True,
             lambda: (live, "streaming" if live else "no live robot state (Monitor or Snapshot reads it)")),
            ("Data", "Joint map", False, lambda: (self.jm.num_joints == 26, f"{self.jm.num_joints} joints")),
            ("Data", "Pose library", False, lambda: (len(self.library.poses()) > 0, f"{len(self.library.poses())} poses")),
            ("Data", "Motion library", False,
             lambda: (all(m["valid"] for m in lib), f"{len(lib)} motions, {sum(not m['valid'] for m in lib)} invalid")),
            ("Data", "Robustness report", True,
             lambda: ((LOGS_DIR / "robustness/latest.json").is_file(), "logs/robustness/latest.json")),
            ("Data", "Level 2 walk policy", True,
             lambda: ((MODELS_DIR / "r1_level2_velocity/policy.onnx").is_file(), "sandbox only, needs validate_level2")),
            ("Data", "Docs", False, lambda: (len(ops.docs_index()) > 0, f"{len(ops.docs_index())} documents")),
            ("Robot", "Critical events (10 min)", False,
             lambda: ((n := self.events.counts(time.time() - 600)["critical"]) == 0, f"{n} critical")),
            ("Robot", "Motor temperatures", True, lambda: self._temp_probe()),
            ("Robot", "Battery", True, lambda: self._power_probe()),
            ("Jobs", "Active job", False, lambda: (True, svc.job or "idle")),
            ("Jobs", "Last job result", True,
             lambda: (not svc.last_result or "FAILED" not in svc.last_result, svc.last_result or "none yet")),
        ]  # fmt: skip
        for p in pols:
            probes.append(("Data", f"Policy {p['name']}", p["name"] != "r1_level0_upperbody",
                           lambda p=p: (p["ok"], f"{p['gate']} · sha {p['policy_sha256'][:12]}")))  # fmt: skip
        rows = ops.run_probes(probes)
        failing = [r for r in rows if not r["ok"] and not r["optional"]]
        return {"ok": True, "overall": "ok" if not failing else "degraded", "failing": len(failing), "rows": rows,
                "checked_at": time.time()}  # fmt: skip

    def _temp_probe(self) -> tuple[bool, str]:
        who, t = self._latest()
        mt = t.get("max_temp_c")
        if mt is None:
            return False, "no temperature reading yet"
        return mt < TEMP_WARN_C, f"{who}: max {mt} C ({t.get('hot_joint') or '?'})"

    def _power_probe(self) -> tuple[bool, str]:
        p = self.history.power(self._source({}))
        if not p.get("available"):
            return False, "no bus voltage yet"
        return not p["low"], f"{p['latest_v']} V, about {p['soc_pct']:.0f} % (estimate)"

    # ------------------------------------------------------------------ pre-flight (#18)
    def _preflight_auto(self) -> list[dict]:
        now = time.time()
        if now - self._auto_cache[0] < 3.0:
            return self._auto_cache[1]
        _, real = self._latest("real")
        age = now - real["at"] if real.get("at") else None
        robot_host = ROBOT.split("@", 1)[-1]
        rows = [
            ("l0_validated", "Level 0 validation report passed", lambda: ops.level0_validation_passed()),
            ("policies", "Policies validated and trained on this joint map",
             lambda: (all(p["ok"] and p["joint_map_matches"] is not False for p in ops.policy_versions()
                          if p["name"] == "r1_level0_upperbody"), "r1_level0_upperbody")),
            ("robot_reachable", f"Robot reachable ({robot_host}:22)", lambda: ops.tcp_probe(robot_host, 22, 0.5)[::2]),
            ("fresh_state", "Robot state read in the last 60 s (Snapshot)",
             lambda: (age is not None and age < 60.0, f"{age:.0f} s ago" if age is not None else "take a Snapshot")),
            ("temps", f"Motor temperatures below {TEMP_WARN_C} C",
             lambda: ((real.get("max_temp_c") is not None and real["max_temp_c"] < TEMP_WARN_C),
                      f"max {real.get('max_temp_c')} C" if real.get("max_temp_c") is not None else "no reading")),
            ("no_critical", "No critical events in the last 10 min",
             lambda: ((n := self.events.counts(now - 600)["critical"]) == 0, f"{n} critical")),
        ]  # fmt: skip
        res = ops.run_probes([("preflight", label, False, fn) for _, label, fn in rows])
        out = [{"id": k, "label": label, "ok": r["ok"], "detail": r["detail"]} for (k, label, _), r in zip(rows, res)]
        self._auto_cache = (now, out)
        return out

    def preflight_why_not(self) -> Optional[str]:
        """Cheap: manual ticks first (no probes while they are open), cached automatic checks after."""
        st_ticks = self.preflight.ticks
        now = time.time()
        open_manual = [k for k, _ in self.preflight.MANUAL if not (k in st_ticks and now - st_ticks[k] <= ops.PREFLIGHT_VALID_S)]
        if open_manual:
            return f"complete the pre-flight checklist ({len(open_manual)} item(s) open)"
        return self.preflight.why_not()

    # ------------------------------------------------------------------ e-stop (#8)
    def estop(self) -> dict:
        stopped = []
        if self.svc.job or self.svc.proc:
            stopped.append(self.svc.job or "job")
            self.svc.stop()
        if self.player.running:
            stopped.append("player")
            self.player.stop()
        if self.svc.walk is not None and (self.svc.walk.state() or {}).get("running"):
            stopped.append("walk")
            self.svc.walk_stop()
        if self.recorder.active:
            self.recorder.stop()
            stopped.append("recording")
        self.events.record("critical", "estop", "E-STOP pressed" + (f": stopped {', '.join(stopped)}" if stopped else ""))
        self.svc.log("E-STOP: " + (", ".join(stopped) if stopped else "nothing was running"))
        return {"ok": True, "stopped": stopped,
                "note": "software stop sent; on the real robot the remote (L2+B for 5 s) is the emergency stop"}  # fmt: skip

    # ------------------------------------------------------------------ modes (#7), poses (#11), motions (#12)
    def set_mode(self, body: dict) -> dict:
        mode, target = str(body.get("mode", "")), str(body.get("target", "practice"))
        if mode not in MODES + ("off",):
            raise ValueError(f"mode must be one of {MODES + ('off',)}")
        if target == "real":
            return {"ok": False, "error": REAL_REFUSAL, "_code": 400}
        if target not in PLAY_TARGETS:
            raise ValueError(f"target must be one of {PLAY_TARGETS}")
        if mode == "off":
            if self.svc.job == "mode":
                self.player.stop()
            return {"ok": True, "mode": None}
        joints = [str(j) for j in body.get("joints") or []]
        for j in joints:
            if j not in self.jm.names:
                raise ValueError(f"unknown joint {j!r}")
        if mode in ("damp", "hold") and not joints:
            raise ValueError(f"{mode} needs the joints to apply it to (only the joints you name get stiffness)")
        busy = self._claim("mode")
        if busy:
            return busy
        who = "practice" if target == "practice" else "virtual"
        self.player.start("mode", self._start_q_deg(who), who, "mode", mode=mode, joints=joints,
                          mirror=target == "urlab", max_s=float(body.get("max_s", 120.0)), on_done=self._done("mode"))  # fmt: skip
        self.events.record("info", "mode", f"{mode} on {target}: {', '.join(joints) or 'all joints limp'}")
        return {"ok": True, "mode": mode, "target": target, "joints": joints}

    def play(self, clip: dict, target: str, loop: bool, job: str) -> dict:
        if target == "real":
            return {"ok": False, "error": REAL_REFUSAL, "_code": 400}
        if target not in PLAY_TARGETS:
            raise ValueError(f"target must be one of {PLAY_TARGETS}")
        v = validate_clip(clip, self.jm, self.limits.values["joint_range_frac"], 1.0)
        if not v["ok"]:
            return {"ok": False, "error": "motion failed validation", "validation": v}
        busy = self._claim(job)
        if busy:
            return busy
        who = "practice" if target == "practice" else "virtual"
        self.player.start("clip", self._start_q_deg(who), who, job, clip=clip, mirror=target == "urlab", loop=loop,
                          time_scale=self.limits.values["motion_speed_scale"], on_done=self._done(job))  # fmt: skip
        self.svc.log(f"{job}: {clip.get('name')} on {target}{' (loop)' if loop else ''}")
        return {"ok": True, "name": clip.get("name"), "target": target, "duration_s": v["duration_s"], "loop": loop}

    def player_state(self) -> dict:
        st = dict(self.player.state)
        st["recording"] = self.recorder.status()
        return {"ok": True, **st}

    # ------------------------------------------------------------------ chat context (#20)
    def _chat_ctx(self) -> dict:
        src = self._source({})
        return {
            "status": {"job": self.svc.job, "last_result": self.svc.last_result,
                       "real_connected": self.svc.real_connected},  # fmt: skip
            "power": self.history.power(src),
            "temps": self.history.temperatures(src),
            "events": self.events.list(limit=10),
            "preflight": self.preflight.state() if not self.preflight_why_not_manual() else
                         {"complete": False, "missing": self._open_manual_labels()},  # fmt: skip
            "limits": self.limits.describe(),
            "versions": {"service": ops.SERVICE_VERSION, "repo": ops.repo_version(), "policies": ops.policy_versions()},
            "motions": self.library.list(),
            "imu": self.history.imu_check(src),
            "walk": {"running": bool(self.svc.walk and (self.svc.walk.state() or {}).get("running"))},
        }

    def preflight_brief(self) -> dict:
        """Cheap summary for /status (polled several times a second): manual ticks first, then the cached probes."""
        open_manual = self._open_manual_labels()
        if open_manual:
            return {"complete": False, "open": len(open_manual), "reason": f"{len(open_manual)} item(s) to tick"}
        at, autos = self._auto_cache
        if time.time() - at > 5.0 and not self._auto_refreshing:  # never probe the network on the /status path
            self._auto_refreshing = True

            def refresh() -> None:
                try:
                    self._preflight_auto()
                finally:
                    self._auto_refreshing = False

            threading.Thread(target=refresh, daemon=True).start()
        if not autos:
            return {"complete": False, "open": 0, "reason": "checking the automatic items…"}
        bad = [a["label"] for a in autos if not a["ok"]]
        return {"complete": not bad, "open": len(bad), "reason": "; ".join(bad[:2]) or None}

    def preflight_why_not_manual(self) -> bool:
        return bool(self._open_manual_labels())

    def _open_manual_labels(self) -> list:
        now = time.time()
        t = self.preflight.ticks
        return [lbl for k, lbl in self.preflight.MANUAL if not (k in t and now - t[k] <= ops.PREFLIGHT_VALID_S)]

    # ------------------------------------------------------------------ camera (#5)
    def camera_png(self, qs: dict) -> tuple[int, bytes, str]:
        src = qs.get("source", "twin")
        if src == "real":
            msg = json.dumps({"ok": False, "error": "no robot camera stream: the R1 head camera needs a video streamer "
                              "on the Jetson, which this repo does not ship yet (planned with Level 3)"}).encode()  # fmt: skip
            return 503, msg, "application/json"
        mode = qs.get("mode", "virtual")
        png = self.svc.render_stand_png(mode="virtual" if mode == "virtual" else "real", cam="head")
        return 200, png, "image/png"

    # ------------------------------------------------------------------ routing
    def handle_get(self, raw_path: str):
        """(code, obj) for a JSON reply, (code, bytes, content_type) for binary, or None if not ours."""
        path, qs = _q(raw_path)
        if path == "/health":
            return 200, self.health()
        if path == "/versions":
            return 200, {"ok": True, **ops.versions()}
        if path == "/events":
            evs = self.events.list(qs.get("level"), qs.get("kind"), float(qs.get("since", 0) or 0),
                                   int(qs.get("limit", 200) or 200))  # fmt: skip
            return 200, {"ok": True, "events": evs, "counts": self.events.counts()}
        if path == "/history/temps":
            return 200, {"ok": True, **self.history.temperatures(self._source(qs), float(qs.get("window", 600)))}
        if path == "/power":
            return 200, {"ok": True, **self.history.power(self._source(qs))}
        if path == "/imu":
            return 200, {"ok": True, **self.history.imu_check(self._source(qs), float(qs.get("window", 60)))}
        if path == "/camera.png":
            return self.camera_png(qs)
        if path == "/limits":
            return 200, self.limits.describe()
        if path == "/poses":
            return 200, {"ok": True, "poses": self.library.poses()}
        if path == "/motions":
            lv = self.limits.values
            return 200, {"ok": True, "motions": self.library.list(lv["joint_range_frac"])}
        if path == "/motions/record":
            return 200, {"ok": True, **self.recorder.status()}
        if path.startswith("/motions/") and path.endswith("/export"):
            res = self.library.export_package(path[len("/motions/") : -len("/export")])
            return res.pop("_code", 200), res
        if path.startswith("/motions/"):
            clip = self.library.get(path[len("/motions/") :])
            if clip is None:
                return 404, {"ok": False, "error": "no such motion"}
            return 200, {"ok": True, "clip": clip, "validation": validate_clip(clip, self.jm)}
        if path == "/player":
            return 200, self.player_state()
        if path == "/calibration/offsets":
            return 200, {"ok": True, "saved": self.offsets.saved(), "pending": self.offsets.pending}
        if path == "/network":
            doc = self.profiles.load()
            return 200, {"ok": True, "interfaces": ops.local_interfaces(), **doc}
        if path == "/preflight":
            return 200, self.preflight.state()
        if path == "/docs":
            return 200, {"ok": True, "docs": ops.docs_index()}
        if path.startswith("/docs/"):
            res = ops.doc_text(path[len("/docs/") :])
            return res.pop("_code", 200), res
        if path == "/assistant/chat/history":
            return 200, self.chat.history(qs.get("session", "default"))
        return None

    def handle_post(self, raw_path: str, body: dict):
        path, _ = _q(raw_path)
        if path == "/estop":
            return 200, self.estop()
        if path == "/limits":
            res = self.limits.set(body.get("values") or {})
            self.events.record("info", "limits", "session limits: " + (", ".join(res["tightened"]) or "defaults"))
            return 200, res
        if path == "/limits/reset":
            return 200, self.limits.reset()
        if path == "/mode":
            return self._r(self.set_mode(body))
        if path == "/poses/preview":
            try:
                clip = self.library.pose_clip(str(body.get("name", "")))
            except KeyError as e:
                return 404, {"ok": False, "error": str(e).strip("'\"")}
            return self._r(self.play(clip, str(body.get("target", "practice")), False, "pose"))
        if path == "/motions/check":
            clip = body.get("clip") or self.library.get(str(body.get("name", ""))) or {}
            lv = self.limits.values
            return 200, {"ok": True, "validation": validate_clip(clip, self.jm, lv["joint_range_frac"], 1.0)}
        if path == "/motions/save":
            return self._r(self.library.save(body.get("clip") or {}, bool(body.get("overwrite", False))))
        if path == "/motions/delete":
            return self._r(self.library.delete(str(body.get("name", ""))))
        if path == "/motions/import":
            return self._r(self.library.import_package(body.get("package") or {}, bool(body.get("overwrite", False))))
        if path == "/motions/play":
            clip = body.get("clip") or self.library.get(str(body.get("name", "")))
            if not clip:
                return 404, {"ok": False, "error": "no such motion"}
            return self._r(self.play(clip, str(body.get("target", "practice")), bool(body.get("loop", False)), "motion"))
        if path == "/motions/record/start":
            return 200, self.recorder.start(str(body.get("source", "practice")))
        if path == "/motions/record/stop":
            return self._r(self.recorder.stop(str(body.get("name") or "recorded_motion")))
        if path == "/player/stop":
            self.player.stop()
            return 200, {"ok": True}
        if path == "/calibration/capture":
            src = str(body.get("source", "real"))
            if src not in ("real", "practice", "virtual"):
                raise ValueError("source must be real, practice or virtual")
            tel = self.svc.telemetry.get(src) or {}
            q = tel.get("q_deg")
            if not (isinstance(q, list) and len(q) == self.jm.num_joints) or time.time() - tel.get("at", 0) > 30.0:
                return 409, {"ok": False, "error": f"no {src} joint angles from the last 30 s: take a Snapshot first"}
            res = self.offsets.capture(q, src, str(body.get("reference", "zero")), body.get("joints") or None)
            self.events.record("info", "calibration", f"zero offsets captured from {src}")
            return 200, res
        if path == "/calibration/save":
            res = self.offsets.save(bool(body.get("confirm", False)), body.get("offsets"))
            if res.get("ok"):
                self.events.record("info", "calibration", f"zero offsets stored ({len(res['offsets_deg'])} joints)")
            return self._r(res)
        if path == "/network/test":
            return 200, ops.probe_connection(str(body.get("host", "")), int(body.get("port", 22)), bool(body.get("icmp", True)))
        if path == "/network/profiles":
            return 200, self.profiles.upsert(body.get("profile") or body)
        if path == "/network/profiles/delete":
            return 200, self.profiles.delete(str(body.get("name", "")))
        if path == "/network/active":
            return 200, self.profiles.activate(str(body.get("name", "")))
        if path == "/preflight":
            res = self.preflight.tick(str(body.get("item", "")), bool(body.get("checked", True)))
            if res["complete"]:
                self.events.record("info", "preflight", "pre-flight checklist complete", dedup_key="pf", dedup_s=300)
            return 200, res
        if path == "/preflight/reset":
            return 200, self.preflight.reset()
        if path == "/assistant/chat":
            return 200, self.chat.chat(body)
        if path == "/assistant/chat/reset":
            return 200, self.chat.reset(str(body.get("session") or "default"))
        return None

    @staticmethod
    def _r(res: dict):
        code = res.pop("_code", None) or (200 if res.get("ok") else 400)
        return code, res
