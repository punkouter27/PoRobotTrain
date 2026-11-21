"""Poses, recorded motions (teach and replay), the local action store and the simulation player.

    clip  = {"name", "title", "description", "author", "tags", "loop",
             "keyframes": [{"t": s, "q_deg": {joint: deg}, "ease": "smooth"|"linear"}]}

Joints a keyframe leaves out keep their previous keyframe's angle; joints no keyframe mentions are not powered.
Playback starts with a lead-in from the current pose to the first keyframe, so a clip never jumps.

Playback, poses and modes run on the headless MuJoCo stand twin in this process (the same calibrated
``SimBackend`` as the practice twin) and, for target ``urlab``, are mirrored onto the Unreal twin in puppet mode.
There is no real-robot path here: a multi-joint clip on the stand-mounted R1 would power joints the operator did
not ask for, so the service refuses target ``real``. Contract: docs/service_api.md section 5.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import re
import threading
import time
from pathlib import Path
from typing import Callable, Optional, Sequence

from ..configs.joint_map import JointMap

NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_]{0,39}$")
MAX_KEYFRAMES = 200
MAX_DURATION_S = 120.0
SOFT_RANGE = 0.9  # deploy.yaml soft_limit_factor: targets stay inside 90 % of the joint range
MAX_VEL_DEG_S = {"head": 90.0, "arm": 120.0, "waist": 45.0, "leg": 90.0}
LEG_JOINTS = 12
PACKAGE_FORMAT = "porobottrain.motion/1"
EASES = ("smooth", "linear")
MODES = ("damp", "zero_torque", "hold")


def joint_group(name: str) -> str:
    if name.startswith("head_"):
        return "head"
    if name.startswith("waist_"):
        return "waist"
    if any(
        name.startswith(p) for p in ("left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle")
    ):
        return "leg"
    return "arm"


def soft_range_deg(jm: JointMap, joint: str, frac: float = SOFT_RANGE) -> tuple[float, float]:
    i = jm.index_of(joint)
    lo, hi = math.degrees(float(jm.limits_lo[i])), math.degrees(float(jm.limits_hi[i]))
    mid, half = (lo + hi) / 2.0, (hi - lo) / 2.0 * frac
    return mid - half, mid + half


def _smooth(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def clip_joints(clip: dict) -> list[str]:
    seen: list[str] = []
    for kf in clip.get("keyframes", []):
        for j in kf.get("q_deg", {}):
            if j not in seen:
                seen.append(j)
    return seen


def clip_duration(clip: dict) -> float:
    kfs = clip.get("keyframes") or []
    return float(kfs[-1]["t"]) if kfs else 0.0


def resolved_keyframes(clip: dict, start_deg: dict) -> list[tuple[float, dict, str]]:
    """Keyframes with every clip joint filled in (carried forward from the previous keyframe or the start pose)."""
    cur = {j: float(start_deg.get(j, 0.0)) for j in clip_joints(clip)}
    out = []
    for kf in clip["keyframes"]:
        cur = dict(cur)
        cur.update({j: float(v) for j, v in kf["q_deg"].items()})
        out.append((float(kf["t"]), cur, kf.get("ease", "smooth")))
    return out


def lead_in_s(clip: dict, start_deg: dict) -> float:
    """Time to reach the first keyframe from the start pose at no more than half the joint speed limits."""
    kfs = resolved_keyframes(clip, start_deg)
    if not kfs:
        return 0.0
    need = 0.0
    for j, v in kfs[0][1].items():
        need = max(need, abs(v - float(start_deg.get(j, v))) / (0.5 * MAX_VEL_DEG_S[joint_group(j)]))
    return round(max(0.0, max(0.8, need) - float(kfs[0][0])), 3)


def sample(clip: dict, t: float, start_deg: dict, lead: Optional[float] = None) -> dict:
    """Joint targets (deg) at playback time t (s from playback start, lead-in included)."""
    kfs = resolved_keyframes(clip, start_deg)
    if not kfs:
        return {}
    lead = lead_in_s(clip, start_deg) if lead is None else lead
    pts = [(-lead, {j: float(start_deg.get(j, kfs[0][1][j])) for j in kfs[0][1]}, "smooth")] + kfs
    tc = t - lead
    if tc <= pts[0][0]:
        return dict(pts[0][1])
    for (t0, q0, _), (t1, q1, ease) in zip(pts, pts[1:]):
        if tc <= t1:
            span = max(t1 - t0, 1e-6)
            u = (tc - t0) / span
            u = _smooth(u) if ease == "smooth" else min(1.0, max(0.0, u))
            return {j: q0[j] + (q1[j] - q0[j]) * u for j in q1}
    return dict(pts[-1][1])


def validate_clip(clip: dict, jm: JointMap, range_frac: float = SOFT_RANGE, speed_scale: float = 1.0) -> dict:
    """Static checks before any playback: names, timing, soft joint limits, joint speeds. Never moves anything."""
    issues: list[dict] = []

    def bad(msg: str, **kw) -> None:
        issues.append(dict({"level": "error", "message": msg}, **kw))

    name = str(clip.get("name", ""))
    if not NAME_RE.match(name):
        bad("name must be 1-40 characters: lowercase letters, digits, underscores")
    kfs = clip.get("keyframes")
    if not isinstance(kfs, list) or not kfs:
        bad("a motion needs at least one keyframe")
        return {"ok": False, "issues": issues, "duration_s": 0.0, "joints": [], "real_eligible": False}
    if len(kfs) > MAX_KEYFRAMES:
        bad(f"too many keyframes ({len(kfs)} > {MAX_KEYFRAMES})")
    last_t = -1.0
    for k, kf in enumerate(kfs):
        if not isinstance(kf, dict) or not isinstance(kf.get("q_deg"), dict):
            bad("keyframe needs t and q_deg", keyframe=k)
            continue
        try:
            t = float(kf.get("t"))
        except (TypeError, ValueError):
            bad("keyframe time is not a number", keyframe=k)
            continue
        if not math.isfinite(t) or t < 0 or t <= last_t:
            bad("keyframe times must start at 0 or later and increase", keyframe=k)
        last_t = t
        if kf.get("ease", "smooth") not in EASES:
            bad(f"ease must be one of {EASES}", keyframe=k)
        for j, v in kf["q_deg"].items():
            if j not in jm.names:
                bad(f"unknown joint {j!r}", keyframe=k)
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                bad(f"{j}: angle is not a number", keyframe=k, joint=j)
                continue
            lo, hi = soft_range_deg(jm, j, range_frac)
            if not (math.isfinite(v) and lo - 1e-6 <= v <= hi + 1e-6):
                bad(f"{j} {v:.1f} deg is outside the soft range [{lo:.1f}, {hi:.1f}]", keyframe=k, joint=j)
    if issues:
        return {"ok": False, "issues": issues, "duration_s": clip_duration(clip), "joints": clip_joints(clip),
                "real_eligible": False}  # fmt: skip
    dur = clip_duration(clip)
    if dur > MAX_DURATION_S:
        bad(f"motion is {dur:.0f} s long (limit {MAX_DURATION_S:.0f} s)")
    # speed between keyframes (each joint's own previous value carried forward)
    rk = resolved_keyframes(clip, {j: v for j, v in kfs[0]["q_deg"].items()})
    max_vel: dict = {}
    for (t0, q0, _), (t1, q1, ease) in zip(rk, rk[1:]):
        dt = max(t1 - t0, 1e-6)
        peak = 1.5 if ease == "smooth" else 1.0  # smoothstep peaks at 1.5x the mean speed
        for j in q1:
            vel = abs(q1[j] - q0.get(j, q1[j])) / dt * peak
            max_vel[j] = max(max_vel.get(j, 0.0), vel)
            cap = MAX_VEL_DEG_S[joint_group(j)] * speed_scale
            if vel > cap + 1e-6:
                bad(f"{j} moves at {vel:.0f} deg/s between {t0:g} s and {t1:g} s (limit {cap:.0f} deg/s)", joint=j)
    joints = clip_joints(clip)
    real_why = []
    if any(jm.index_of(j) < LEG_JOINTS for j in joints):
        real_why.append("uses leg joints (stand-locked on the real robot)")
    real_why.append("multi-joint playback on the real robot is not enabled (simulation and Unreal only)")
    return {
        "ok": not issues,
        "issues": issues,
        "duration_s": round(dur, 3),
        "joints": joints,
        "max_vel_deg_s": {j: round(v, 1) for j, v in max_vel.items()},
        "real_eligible": False,
        "real_reasons": real_why,
    }


def canonical_sha(clip: dict) -> str:
    body = {k: clip[k] for k in ("name", "keyframes", "loop") if k in clip}
    return hashlib.sha256(json.dumps(body, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def normalise_clip(raw: dict) -> dict:
    """Keep the known fields, coerce types, sort nothing (validation reports out-of-order keyframes)."""
    clip = {
        "name": str(raw.get("name", "")).strip(),
        "title": str(raw.get("title") or raw.get("name") or "").strip()[:80],
        "description": str(raw.get("description", "")).strip()[:500],
        "author": str(raw.get("author", "")).strip()[:80],
        "tags": [str(t)[:24] for t in (raw.get("tags") or [])][:10],
        "loop": bool(raw.get("loop", False)),
        "keyframes": [],
    }
    for kf in raw.get("keyframes") or []:
        if not isinstance(kf, dict):
            clip["keyframes"].append(kf)
            continue
        q = kf.get("q_deg") if isinstance(kf.get("q_deg"), dict) else kf.get("q_deg")
        item = {"t": kf.get("t"), "q_deg": q}
        if kf.get("ease"):
            item["ease"] = kf["ease"]
        clip["keyframes"].append(item)
    return clip


class MotionLibrary:
    """Built-in clips (tracked, read-only) + the operator's clips (user dir), poses from configs/poses.json."""

    def __init__(self, jm: JointMap, builtin_dir: Path, user_dir: Path, poses_path: Path):
        self.jm = jm
        self.builtin_dir = builtin_dir
        self.user_dir = user_dir
        self.poses_path = poses_path
        self._lock = threading.Lock()

    # ---------------------------------------------------------------- poses (#11)
    def poses(self) -> list[dict]:
        try:
            raw = json.loads(self.poses_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            raw = {"poses": []}
        out = []
        for p in raw.get("poses", []):
            q = dict(p.get("q_deg") or {})
            if p.get("name") == "home" and not q:
                q = {
                    n: round(math.degrees(float(d)), 1)
                    for n, d in zip(self.jm.names, self.jm.defaults)
                    if self.jm.index_of(n) >= LEG_JOINTS
                }
            out.append({"name": p["name"], "label": p.get("label", p["name"]), "say": p.get("say", ""),
                        "words": p.get("words", []), "q_deg": q})  # fmt: skip
        return out

    def pose_clip(self, name: str, hold_s: float = 0.6) -> dict:
        pose = next((p for p in self.poses() if p["name"] == name), None)
        if pose is None:
            raise KeyError(f"unknown pose {name!r}")
        return {
            "name": f"pose_{name}"[:40],
            "title": pose["label"],
            "loop": False,
            "keyframes": [{"t": 0.0, "q_deg": pose["q_deg"]}, {"t": hold_s, "q_deg": pose["q_deg"]}],
        }

    # ---------------------------------------------------------------- clips (#12, #13)
    def _files(self) -> list[tuple[Path, str]]:
        out = []
        for d, src in ((self.builtin_dir, "builtin"), (self.user_dir, "user")):
            if d.is_dir():
                out += [(p, src) for p in sorted(d.glob("*.json"))]
        return out

    def _load(self, path: Path) -> Optional[dict]:
        try:
            return normalise_clip(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, json.JSONDecodeError, AttributeError):
            return None

    def list(self, range_frac: float = SOFT_RANGE, speed_scale: float = 1.0) -> list[dict]:
        out = {}
        for path, src in self._files():
            clip = self._load(path)
            if clip is None or not clip["name"]:
                continue
            v = validate_clip(clip, self.jm, range_frac, speed_scale)
            out[clip["name"]] = {
                "name": clip["name"],
                "title": clip["title"] or clip["name"],
                "description": clip["description"],
                "author": clip["author"],
                "tags": clip["tags"],
                "loop": clip["loop"],
                "source": src,
                "keyframes": len(clip["keyframes"]),
                "duration_s": v["duration_s"],
                "joints": v["joints"],
                "valid": v["ok"],
                "issues": [i["message"] for i in v["issues"]][:5],
                "sha256": canonical_sha(clip),
            }
        return sorted(out.values(), key=lambda c: (c["source"] != "builtin", c["name"]))

    def get(self, name: str) -> Optional[dict]:
        if not NAME_RE.match(name or ""):
            return None
        for d, src in ((self.user_dir, "user"), (self.builtin_dir, "builtin")):
            p = d / f"{name}.json"
            if p.is_file():
                clip = self._load(p)
                if clip is not None:
                    clip["source"] = src
                    return clip
        return None

    def save(self, raw: dict, overwrite: bool = False) -> dict:
        clip = normalise_clip(raw)
        v = validate_clip(clip, self.jm)
        if not v["ok"]:
            return {"ok": False, "error": "motion failed validation", "validation": v}
        name = clip["name"]
        with self._lock:
            if (self.builtin_dir / f"{name}.json").is_file():
                return {"ok": False, "error": f"{name!r} is a built-in motion: save it under a new name", "_code": 409}
            path = self.user_dir / f"{name}.json"
            if path.exists() and not overwrite:
                return {"ok": False, "error": f"a motion named {name!r} exists (send overwrite: true)", "_code": 409}
            self.user_dir.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(clip, indent=2) + "\n", encoding="utf-8")
        return {"ok": True, "name": name, "validation": v, "sha256": canonical_sha(clip)}

    def delete(self, name: str) -> dict:
        if not NAME_RE.match(name or ""):
            return {"ok": False, "error": "bad name"}
        if (self.builtin_dir / f"{name}.json").is_file():
            return {"ok": False, "error": "built-in motions cannot be deleted", "_code": 409}
        p = self.user_dir / f"{name}.json"
        if not p.is_file():
            return {"ok": False, "error": f"no motion {name!r}", "_code": 404}
        p.unlink()
        return {"ok": True}

    # ---------------------------------------------------------------- local action store (#14)
    def export_package(self, name: str) -> dict:
        clip = self.get(name)
        if clip is None:
            return {"ok": False, "error": f"no motion {name!r}", "_code": 404}
        clip.pop("source", None)
        v = validate_clip(clip, self.jm)
        return {
            "ok": True,
            "format": PACKAGE_FORMAT,
            "exported_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "sha256": canonical_sha(clip),
            "validation": {"ok": v["ok"], "duration_s": v["duration_s"], "joints": v["joints"]},
            "clip": clip,
        }

    def import_package(self, pkg: dict, overwrite: bool = False) -> dict:
        if pkg.get("format") != PACKAGE_FORMAT:
            return {"ok": False, "error": f"not a {PACKAGE_FORMAT} package"}
        clip = normalise_clip(pkg.get("clip") or {})
        if pkg.get("sha256") and pkg["sha256"] != canonical_sha(clip):
            return {"ok": False, "error": "package checksum does not match its motion (edited or corrupted)"}
        return self.save(clip, overwrite=overwrite)


class Recorder:
    """Teach mode: records a telemetry source's joint angles (read-only) and turns them into keyframes."""

    def __init__(self, jm: JointMap):
        self.jm = jm
        self.source: Optional[str] = None
        self.samples: list[tuple[float, list]] = []
        self.started_at = 0.0
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        return self.source is not None

    def start(self, source: str) -> dict:
        if source not in ("real", "practice", "virtual"):
            raise ValueError("record source must be 'real', 'practice' or 'virtual'")
        with self._lock:
            self.source, self.samples, self.started_at = source, [], time.time()
        return {"ok": True, "source": source}

    def on_telemetry(self, who: str, pkt: dict) -> None:
        if who != self.source:
            return
        q = pkt.get("q_deg")
        if isinstance(q, list) and len(q) == self.jm.num_joints:
            with self._lock:
                t = time.time() - self.started_at
                if t <= MAX_DURATION_S and (not self.samples or t - self.samples[-1][0] >= 0.05):
                    self.samples.append((t, [float(v) for v in q]))

    def status(self) -> dict:
        with self._lock:
            return {
                "active": self.active,
                "source": self.source,
                "samples": len(self.samples),
                "elapsed_s": round(time.time() - self.started_at, 1) if self.active else 0.0,
            }

    def stop(self, name: str = "recorded_motion", keyframe_s: float = 0.5, min_range_deg: float = 1.0) -> dict:
        with self._lock:
            samples, src = list(self.samples), self.source
            self.source, self.samples = None, []
        if len(samples) < 2:
            return {"ok": False, "error": "nothing recorded: the source sent fewer than 2 joint samples"}
        moved = [
            j
            for j in range(self.jm.num_joints)
            if max(s[1][j] for s in samples) - min(s[1][j] for s in samples) >= min_range_deg
        ]
        if not moved:
            return {"ok": False, "error": f"no joint moved more than {min_range_deg:g} deg while recording"}
        t0 = samples[0][0]
        picked, next_t = [], 0.0
        for t, q in samples:
            if t - t0 >= next_t - 1e-9:
                picked.append((t - t0, q))
                next_t = (t - t0) + keyframe_s
        if picked[-1][0] < samples[-1][0] - t0 - 1e-6:
            picked.append((samples[-1][0] - t0, samples[-1][1]))
        picked = picked[:MAX_KEYFRAMES]
        kfs = []
        for t, q in picked:
            qd = {}
            for j in moved:
                lo, hi = soft_range_deg(self.jm, self.jm.names[j])
                qd[self.jm.names[j]] = round(min(hi, max(lo, q[j])), 1)
            kfs.append({"t": round(t, 2), "q_deg": qd, "ease": "linear"})
        clip = {"name": name, "title": f"Recorded from {src}", "description": "", "author": "", "tags": ["recorded"],
                "loop": False, "keyframes": kfs}  # fmt: skip
        return {"ok": True, "clip": clip, "validation": validate_clip(clip, self.jm)}


class SimPlayer:
    """Runs a clip, a pose or a mode on the headless MuJoCo stand twin in real time (50 Hz control)."""

    def __init__(
        self,
        jm: JointMap,
        calibration,
        publish: Callable[[str, dict, str], None],
        log: Callable[[str], None],
        mirror_factory: Optional[Callable[[], object]] = None,
    ):
        self.jm = jm
        self.calibration = calibration
        self.publish = publish
        self.log = log
        self.mirror_factory = mirror_factory
        self.stop_ev = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.state: dict = {"running": False}
        self.backend_factory = None  # tests inject a fake backend

    @property
    def running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def _make_backend(self, powered: Sequence[str], start_q_deg: Sequence[float]):
        if self.backend_factory is not None:
            return self.backend_factory(powered, start_q_deg)
        from .backends import SimBackend

        start_q = {n: math.radians(v) for n, v in zip(self.jm.names, start_q_deg)}
        return SimBackend(self.jm, powered, start_q=start_q, calibration=self.calibration, realistic=True, settle_s=0.3)

    def start(
        self,
        kind: str,
        start_q_deg: Sequence[float],
        who: str,
        job: str,
        clip: Optional[dict] = None,
        mode: Optional[str] = None,
        joints: Sequence[str] = (),
        mirror: bool = False,
        loop: bool = False,
        time_scale: float = 1.0,
        max_s: float = 600.0,
        on_done: Optional[Callable[[bool, str], None]] = None,
    ) -> None:
        if self.running:
            raise RuntimeError("the simulation player is busy")
        self.stop_ev.clear()
        args = dict(kind=kind, start=list(start_q_deg), who=who, job=job, clip=copy.deepcopy(clip), mode=mode,
                    joints=list(joints), mirror=mirror, loop=loop, time_scale=time_scale, max_s=max_s, on_done=on_done)  # fmt: skip
        self.thread = threading.Thread(target=self._run, kwargs=args, daemon=True, name="sim-player")
        self.thread.start()

    def stop(self) -> None:
        self.stop_ev.set()

    def join(self, timeout: float = 5.0) -> None:
        if self.thread is not None:
            self.thread.join(timeout)

    def _run(self, kind, start, who, job, clip, mode, joints, mirror, loop, time_scale, max_s, on_done) -> None:
        names = self.jm.names
        start_deg = dict(zip(names, start))
        if kind == "clip":
            powered = clip_joints(clip)
            lead = lead_in_s(clip, start_deg)
            total = (lead + clip_duration(clip)) / max(time_scale, 1e-3)
        else:
            powered = [] if mode == "zero_torque" else list(joints)
            lead, total = 0.0, max_s
        ok, reason, mir = True, "", None
        self.state = {"running": True, "kind": kind, "job": job, "target_who": who,
                      "name": (clip or {}).get("name") or mode, "t": 0.0, "duration_s": None if kind == "mode" else round(total, 2),
                      "loop": loop, "mode": mode, "joints": powered}  # fmt: skip
        backend = None
        try:
            backend = self._make_backend(powered, start)
            if kind == "mode" and mode == "damp":
                backend.kp = backend.kp * 0.0  # velocity damping only: the joint is back-drivable
            if mirror and self.mirror_factory is not None:
                try:
                    mir = self.mirror_factory()
                except Exception as e:  # Unreal not running: keep simulating, say so
                    self.log(f"{job}: Unreal twin not reachable, simulating headless ({e})")
            t0, step, k = time.monotonic(), 0.02, 0
            hold = {n: math.radians(start_deg[n]) for n in powered}
            while not self.stop_ev.is_set():
                t = k * step
                if kind == "clip":
                    tc = t * time_scale
                    if tc > lead + clip_duration(clip):
                        if not loop:
                            break
                        k, t0 = 0, time.monotonic()
                        start_deg = dict(zip(names, [math.degrees(float(v)) for v in backend.read()[0]]))
                        lead = lead_in_s(clip, start_deg)
                        continue
                    tgt = sample(clip, tc, start_deg, lead)
                    backend.command({j: math.radians(v) for j, v in tgt.items()})
                elif powered:
                    backend.command(hold)
                elif t > max_s:
                    break
                if kind == "mode" and t > max_s:
                    break
                backend.advance(t + step)
                q, _, _ = backend.read()
                q_deg = [round(math.degrees(float(v)), 1) for v in q]
                if mir is not None:
                    try:
                        mir.push(q)
                    except Exception as e:
                        self.log(f"{job}: Unreal mirror stopped ({e})")
                        mir = None
                if k % 5 == 0:
                    pkt = {"t": round(t, 2), "q_deg": q_deg, "phase": kind, "behaviour": self.state["name"],
                           "temps_c": [34] * self.jm.num_joints, "max_temp_c": 34,
                           "bus_v": getattr(self.calibration, "bus_voltage_v", None), "mode_machine": 1,
                           "active_motors": len(powered)}  # fmt: skip
                    pkt.update(backend.telemetry())
                    self.publish(who, pkt, job)
                self.state["t"] = round(t, 2)
                k += 1
                wait = t0 + k * step - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
            if backend is not None:  # final pose, so the twin shows exactly where playback ended
                q, _, _ = backend.read()
                pkt = {"t": round(k * 0.02, 2), "q_deg": [round(math.degrees(float(v)), 1) for v in q], "phase": "done",
                       "behaviour": self.state["name"], "active_motors": 0}  # fmt: skip
                self.publish(who, pkt, job)
        except Exception as e:
            ok, reason = False, str(e)[:120]
            self.log(f"{job}: {e}")
        finally:
            if mir is not None:
                try:
                    mir.close()
                except Exception:
                    pass
            if backend is not None:
                try:
                    backend.close()
                except Exception:
                    pass
            self.state = dict(self.state, running=False, stopped=self.stop_ev.is_set(), ok=ok, reason=reason)
            if on_done is not None:
                on_done(ok, reason)
