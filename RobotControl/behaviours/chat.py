"""Chat assistant with memory (docs/service_api.md section 7).

A superset of ``/assistant/interpret``: questions about the robot ("how hot is it?", "is the robot connected?",
"what's the battery?") are answered from the live service state; follow-ups ("again", "bigger", "what did you just
do?") use the conversation memory; everything else goes to the deterministic intent matcher, plus named poses.
It stays suggest-only: a reply never starts, checks or runs anything on the robot.
Local and deterministic: no text leaves this machine.
"""

from __future__ import annotations

import collections
import re
import threading
import time
from typing import Callable, Optional

QUESTION_START = ("what", "how", "is", "are", "which", "why", "when", "do", "does", "can", "any", "who", "where",
                  "tell", "show", "list", "status", "report")  # fmt: skip
MAX_SESSIONS = 50
MAX_TURNS = 40


def _norm(text: str) -> str:
    return " " + re.sub(r"[^a-z0-9.\-? ]+", " ", text.lower()).strip() + " "


def _any(t: str, words) -> bool:
    return any(f" {w} " in t or f" {w}?" in t or f" {w}s " in t for w in words)


class ChatAssistant:
    def __init__(self, ctx: Callable[[], dict], interpret: Callable[[str, dict], dict], poses: Callable[[], list]):
        """ctx() returns a snapshot: status, power, temps, events, preflight, limits, versions, motions, walk."""
        self.ctx = ctx
        self.interpret = interpret
        self.poses = poses
        self.sessions: "collections.OrderedDict[str, dict]" = collections.OrderedDict()
        self._lock = threading.Lock()

    # ---------------------------------------------------------------- memory
    def _session(self, sid: str) -> dict:
        with self._lock:
            s = self.sessions.get(sid)
            if s is None:
                s = {"turns": collections.deque(maxlen=MAX_TURNS), "last_intent": None}
                self.sessions[sid] = s
                while len(self.sessions) > MAX_SESSIONS:
                    self.sessions.popitem(last=False)
            self.sessions.move_to_end(sid)
            return s

    def history(self, sid: str) -> dict:
        s = self._session(sid)
        return {"ok": True, "session": sid, "turns": list(s["turns"])}

    def reset(self, sid: str) -> dict:
        with self._lock:
            self.sessions.pop(sid, None)
        return {"ok": True, "session": sid}

    # ---------------------------------------------------------------- reply
    def chat(self, body: dict) -> dict:
        text = str(body.get("text", "")).strip()[:500]
        sid = re.sub(r"[^A-Za-z0-9_\-]", "", str(body.get("session") or "default"))[:40] or "default"
        s = self._session(sid)
        s["turns"].append({"role": "user", "text": text, "at": round(time.time(), 2)})
        res = self._reply(text, body, s)
        res.setdefault("ok", True)
        res["session"] = sid
        if res.get("kind") in ("behaviour", "joint", "walk", "pose") and not res.get("blocked"):
            s["last_intent"] = {k: res.get(k) for k in ("kind", "id", "label", "amplitude", "joint", "deg", "walk",
                                                         "pose")}  # fmt: skip
        s["turns"].append({"role": "assistant", "text": res.get("say") or res.get("note") or "", "kind": res["kind"],
                           "at": round(time.time(), 2)})  # fmt: skip
        res["turns"] = len(s["turns"])
        return res

    def _answer(self, text: str, say: Optional[str] = None) -> dict:
        return {"kind": "answer", "answer": text, "note": text, "say": say or text}

    def _reply(self, text: str, body: dict, s: dict) -> dict:
        t = _norm(text)
        if not text:
            return self._answer("Ask me about the robot, or describe a motion.")
        # follow-ups that need memory
        last = s["last_intent"]
        if _any(t, ("again", "repeat", "same")) and last:
            return dict(last, note=f"Again: {last.get('label') or last.get('pose') or last['kind']}.",
                        say="Same again, selected.")  # fmt: skip
        if last and last.get("kind") == "behaviour" and _any(t, ("bigger", "more", "larger", "further", "higher")):
            amp = min(30.0, float(last.get("amplitude") or 10.0) + 10.0)
            return dict(last, amplitude=amp, note=f"{last['label']} at {amp:.0f} deg.", say=f"{last['label']}, {amp:.0f} degrees.")
        if last and last.get("kind") == "behaviour" and _any(t, ("smaller", "less", "gentler", "lower")):
            amp = max(10.0, float(last.get("amplitude") or 20.0) - 10.0)
            return dict(last, amplitude=amp, note=f"{last['label']} at {amp:.0f} deg.", say=f"{last['label']}, {amp:.0f} degrees.")
        if re.search(r"what (did|have) (i|you)|what was|recap|history", t):
            past = [x["text"] for x in list(s["turns"])[:-1] if x["role"] == "user"][-3:]
            if not past:
                return self._answer("This is the start of our conversation.")
            return self._answer("Recent requests: " + "; ".join(f'"{p}"' for p in past) + ".",
                                f"You asked {len(past)} things recently, most recently {past[-1]}.")  # fmt: skip

        # named poses ("go to arms out", "look left pose")
        for p in self.poses():
            words = [p["name"].replace("_", " "), p["label"].lower()] + [w.lower() for w in p.get("words", [])]
            if any(f" {w} " in t for w in words) and (" pose " in t or " go to " in t or " strike " in t or "pose" in p["name"]
                                                      or _any(t, ("show", "do", "make"))):  # fmt: skip
                return {"kind": "pose", "pose": p["name"], "label": p["label"],
                        "note": f"Pose {p['label']}: preview it on the virtual twin (simulation only).",
                        "say": p.get("say") or f"{p['label']} pose, ready to preview on the twin."}  # fmt: skip

        words = t.split()
        if t.strip().endswith("?") or (words and words[0] in QUESTION_START):
            ans = self._status_answer(t)
            if ans is not None:
                return ans
        ans = self._status_answer(t) if _any(t, ("battery", "temperature", "temp", "preflight", "pre-flight", "faults",
                                                 "alarms", "limits", "version", "firmware", "imu")) else None  # fmt: skip
        if ans is not None:
            return ans
        intent = self.interpret(text, body)
        if intent.get("kind") == "unknown" and (t.strip().endswith("?") or _any(t, ("help",))):
            return self._answer(
                "I can answer: battery, temperatures, faults, what's running, is the robot connected, pre-flight, "
                "limits, versions, poses, motions, IMU. Or describe a motion: nod the head twenty degrees.",
                "Ask about battery, temperatures, faults or connection, or describe a motion.",
            )
        return intent

    def _status_answer(self, t: str) -> Optional[dict]:
        c = self.ctx()
        st = c.get("status") or {}
        if _any(t, ("battery", "power", "voltage", "charge")):
            p = c.get("power") or {}
            if not p.get("available"):
                return self._answer("No battery reading yet: take a snapshot or start Monitor (read-only).")
            left = f", about {p['minutes_left']:.0f} min left" if p.get("minutes_left") else ""
            return self._answer(f"Bus {p['latest_v']:.1f} V, about {p['soc_pct']:.0f} % (estimate){left}.",
                                f"Battery about {p['soc_pct']:.0f} percent.")  # fmt: skip
        if _any(t, ("temperature", "temp", "hot", "heat", "warm")):
            tm = c.get("temps") or {}
            h = tm.get("hottest")
            if not h:
                return self._answer("No temperature readings yet: take a snapshot or start Monitor.")
            word = "fine" if h["temp_c"] < 55 else ("warm, keep an eye on it" if h["temp_c"] < 65 else "too hot, stop")
            return self._answer(f"Hottest motor: {h['joint']} at {h['temp_c']} C ({word}).",
                                f"The hottest motor is {h['joint'].replace('_', ' ')} at {h['temp_c']} degrees, {word}.")  # fmt: skip
        if _any(t, ("fault", "error", "alarm", "event", "wrong", "problem", "warning")):
            evs = [e for e in (c.get("events") or []) if e["level"] != "info"][:3]
            if not evs:
                return self._answer("No warnings or faults recorded this session.")
            return self._answer("Latest: " + "; ".join(f"{e['level']}: {e['message']}" for e in evs) + ".",
                                f"{len(evs)} recent warnings. The latest: {evs[0]['message']}.")  # fmt: skip
        if _any(t, ("ready",)) and not _any(t, ("preflight", "pre-flight", "checklist")):
            pf = c.get("preflight") or {}
            if not st.get("real_connected"):
                return self._answer("The twin is ready. The real robot is not connected, so only the twin can move.",
                                    "The twin is ready. The real robot is not connected.")  # fmt: skip
            if pf.get("complete"):
                return self._answer("Real robot connected and the pre-flight checklist is complete: preview, check, then run.")
            n = len(pf.get("missing") or [])
            return self._answer(f"Real robot connected, but {n} pre-flight item(s) are open: see Setup.",
                                f"Not yet: {n} pre-flight items are open.")  # fmt: skip
        if _any(t, ("preflight", "pre-flight", "checklist", "ready")):
            pf = c.get("preflight") or {}
            if pf.get("complete"):
                return self._answer("Pre-flight checklist complete.")
            miss = pf.get("missing") or []
            return self._answer(f"Pre-flight open: {', '.join(miss[:4])}{'…' if len(miss) > 4 else ''}.",
                                f"{len(miss)} pre-flight items are still open.")  # fmt: skip
        if _any(t, ("walk", "walking")) and _any(t, ("is", "are", "status")):
            w = c.get("walk") or {}
            return self._answer("The walk sandbox is running." if w.get("running") else "The walk sandbox is stopped.")
        if _any(t, ("running", "doing", "busy", "job", "status", "happening")):
            job = st.get("job")
            lr = st.get("last_result") or "nothing yet"
            return self._answer(f"Running: {job}." if job else f"Idle. Last result: {lr}.")
        if _any(t, ("connected", "connection", "reachable", "online", "robot")) and not _any(t, ("move", "run")):
            rc = st.get("real_connected")
            return self._answer("The physical robot is streaming state." if rc else
                                "The physical robot is not connected right now (practice twin available).")  # fmt: skip
        if _any(t, ("limit", "cap", "speed")):
            lv = (c.get("limits") or {}).get("values") or {}
            tight = (c.get("limits") or {}).get("tightened") or []
            return self._answer(f"Sweep cap {lv.get('amplitude_cap_deg', 30):g} deg, walk scale "
                                f"{lv.get('walk_speed_scale', 1):g}x, motion speed {lv.get('motion_speed_scale', 1):g}x"
                                f"{' (tightened: ' + ', '.join(tight) + ')' if tight else ' (defaults)'}.")  # fmt: skip
        if _any(t, ("version", "firmware", "policy", "sha")):  # before "motion": "policy" questions
            v = c.get("versions") or {}
            pol = ", ".join(f"{p['name']} {p['gate']}" for p in v.get("policies", [])) or "no policies"
            return self._answer(f"Service {v.get('service')}, commit {(v.get('repo') or {}).get('commit')}. {pol}.",
                                "Versions are on the Health page.")  # fmt: skip
        if _any(t, ("pose",)):
            names = [p["label"] for p in self.poses()]
            return self._answer("Poses: " + ", ".join(names) + ".")
        if _any(t, ("motion", "recording", "clip", "store")):
            names = [m["title"] for m in (c.get("motions") or [])]
            return self._answer("Motions: " + (", ".join(names) if names else "none yet") + ".")
        if _any(t, ("imu", "tilt", "level", "gyro", "drift")):
            im = c.get("imu") or {}
            if not im.get("available"):
                return self._answer("No IMU samples yet: start Monitor or the practice stream.")
            return self._answer(f"IMU {im['verdict']}: tilt {im['tilt_deg']:.1f} deg, yaw drift "
                                f"{im.get('yaw_drift_deg_per_min')} deg/min.")  # fmt: skip
        return None
