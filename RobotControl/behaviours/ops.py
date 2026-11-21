"""Operator console: health checks, versions, connection profiles, pre-flight checklist, session limits,
joint zero-offset calibration and the docs index. Contract: docs/service_api.md sections 4-7.

Nothing here commands a motor. The pre-flight checklist and the session limits only ADD restrictions to the
existing robot gates; zero offsets are stored for review and are not applied to any controller.
"""

from __future__ import annotations

import hashlib
import json
import math
import platform
import re
import socket
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from importlib import metadata
from pathlib import Path
from typing import Callable, Optional

from .. import LOGS_DIR, MODELS_DIR, PROJECT_ROOT
from ..configs.joint_map import JointMap
from .motions import LEG_JOINTS, soft_range_deg

DOCS_DIR = PROJECT_ROOT / "docs"
DOC_NAME_RE = re.compile(r"^[a-z0-9_]{1,60}$")
PREFLIGHT_VALID_S = 1800.0
SERVICE_VERSION = "2026.11.21"


def _host_port(address: str, default_port: int) -> tuple[str, int]:
    """'tcp://localhost' / 'tcp://1.2.3.4:5555' / 'user@host' -> (host, port)."""
    a = address.split("://", 1)[-1].split("@", 1)[-1]
    if ":" in a:
        h, p = a.rsplit(":", 1)
        try:
            return h or "127.0.0.1", int(p)
        except ValueError:
            return h, default_port
    return a or "127.0.0.1", default_port


def tcp_probe(host: str, port: int, timeout: float = 0.5) -> tuple[bool, float, str]:
    t0 = time.perf_counter()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, (time.perf_counter() - t0) * 1000.0, f"{host}:{port} accepted"
    except OSError as e:
        return False, (time.perf_counter() - t0) * 1000.0, f"{host}:{port} {type(e).__name__}"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


_SHA_CACHE: dict = {}


def cached_sha(path: Path) -> str:
    st = path.stat()
    key = (str(path), st.st_mtime_ns, st.st_size)
    if key not in _SHA_CACHE:
        _SHA_CACHE[key] = sha256_file(path)
    return _SHA_CACHE[key]


def _load_yaml(path: Path) -> dict:
    import yaml

    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return {}


def level0_validation_passed(report: Path = LOGS_DIR / "level0_validation" / "report.md") -> tuple[bool, str]:
    try:
        text = report.read_text(encoding="utf-8")
    except OSError:
        return False, "no validation report: run RobotControl/training/validate_level0.py"
    return ("ALL CHECKS PASSED" in text), ("report says ALL CHECKS PASSED" if "ALL CHECKS PASSED" in text
                                            else "report does not say ALL CHECKS PASSED")  # fmt: skip


# ------------------------------------------------------------------------------------------------ versions (#6)
def policy_versions(models_dir: Path = MODELS_DIR, joint_map_path: Optional[Path] = None) -> list[dict]:
    jm_path = joint_map_path or (PROJECT_ROOT / "RobotControl/configs/r1_joint_map.yaml")
    jm_sha = sha256_file(jm_path)[:16] if jm_path.is_file() else None
    out = []
    for d in sorted(p for p in models_dir.iterdir() if p.is_dir()) if models_dir.is_dir() else []:
        onnx, dep = d / "policy.onnx", d / "deploy.yaml"
        if not onnx.is_file():
            continue
        y = _load_yaml(dep) if dep.is_file() else {}
        meta = y.get("metadata") or {}
        val = y.get("validation") or {}
        actual = cached_sha(onnx)
        expected = val.get("policy_sha256")
        if expected:
            gate = "validated" if expected == actual else "MISMATCH: policy.onnx changed after validation"
        elif d.name == "r1_level0_upperbody":
            ok, why = level0_validation_passed()
            gate = "validated (Level 0 gate report)" if ok else f"not validated: {why}"
        else:
            gate = "no validation hash in deploy.yaml"
        trained_map = meta.get("joint_map_sha256")
        out.append(
            {
                "name": d.name,
                "policy_sha256": actual,
                "expected_sha256": expected,
                "gate": gate,
                "ok": gate.startswith("validated"),
                "trained_at": meta.get("trained_at"),
                "num_obs": y.get("num_obs"),
                "num_actions": y.get("num_actions"),
                "joint_map_sha256": trained_map,
                "joint_map_matches": None if not trained_map or not jm_sha else str(trained_map) == jm_sha[: len(str(trained_map))],
            }
        )  # fmt: skip
    return out


def _pkg_version(name: str) -> str:
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return "not installed"


_GIT: dict = {}


def repo_version() -> dict:
    if _GIT.get("at", 0) > time.time() - 30:
        return _GIT["v"]
    v = {"commit": None, "dirty_files": None}
    try:
        v["commit"] = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT, capture_output=True,
                                     text=True, timeout=3).stdout.strip() or None  # fmt: skip
        st = subprocess.run(["git", "status", "--porcelain"], cwd=PROJECT_ROOT, capture_output=True, text=True,
                            timeout=5).stdout  # fmt: skip
        v["dirty_files"] = len([ln for ln in st.splitlines() if ln.strip()])
    except (OSError, subprocess.SubprocessError):
        pass
    _GIT.update(at=time.time(), v=v)
    return v


def versions(robot_info: Optional[dict] = None) -> dict:
    pols = policy_versions()
    return {
        "service": SERVICE_VERSION,
        "repo": repo_version(),
        "python": platform.python_version(),
        "packages": {n: _pkg_version(n) for n in ("mujoco", "onnxruntime", "numpy", "unitree_sdk2py")},
        "robot": robot_info
        or {"firmware": None, "note": "the low-level state stream does not report firmware; read it on the robot"},
        "policies": pols,
        "warnings": [f"{p['name']}: {p['gate']}" for p in pols if not p["ok"]]
        + [f"{p['name']}: trained on a different joint map" for p in pols if p["joint_map_matches"] is False],
    }


# ------------------------------------------------------------------------------------------------ session limits (#10)
LIMIT_SPECS = {
    # key: (hard maximum = default, minimum, unit, label)
    "amplitude_cap_deg": (30.0, 1.0, "deg", "Behaviour sweep size cap"),
    "freq_cap_hz": (0.5, 0.05, "Hz", "Behaviour frequency cap"),
    "joint_range_frac": (0.9, 0.2, "x range", "Joint slider / motion range (fraction of the joint range)"),
    "walk_speed_scale": (1.0, 0.0, "x", "Walk sandbox velocity scale"),
    "motion_speed_scale": (1.0, 0.1, "x", "Motion playback speed scale"),
}


class SessionLimits:
    """Caps for this service session. They can only tighten the built-in limits, never loosen them."""

    def __init__(self):
        self.values = {k: spec[0] for k, spec in LIMIT_SPECS.items()}
        self.changed_at: Optional[float] = None

    def set(self, updates: dict) -> dict:
        new = dict(self.values)
        for k, v in updates.items():
            if k not in LIMIT_SPECS:
                raise ValueError(f"unknown limit {k!r}")
            hi, lo, unit, _ = LIMIT_SPECS[k]
            v = float(v)
            if not (math.isfinite(v) and lo <= v <= hi):
                raise ValueError(f"{k} must be in [{lo:g}, {hi:g}] {unit} (limits can only tighten the defaults)")
            new[k] = v
        self.values, self.changed_at = new, time.time()
        return self.describe()

    def reset(self) -> dict:
        self.values = {k: spec[0] for k, spec in LIMIT_SPECS.items()}
        self.changed_at = time.time()
        return self.describe()

    def describe(self) -> dict:
        return {
            "ok": True,
            "values": dict(self.values),
            "specs": {k: {"max": s[0], "min": s[1], "unit": s[2], "label": s[3]} for k, s in LIMIT_SPECS.items()},
            "tightened": [k for k, v in self.values.items() if v < LIMIT_SPECS[k][0]],
            "changed_at": self.changed_at,
        }

    def check_params(self, p: dict) -> None:
        if p["amplitude"] > self.values["amplitude_cap_deg"] + 1e-9:
            raise ValueError(f"amplitude {p['amplitude']:g} deg is above the session cap "
                             f"{self.values['amplitude_cap_deg']:g} deg (Limits)")  # fmt: skip
        if p["freq"] > self.values["freq_cap_hz"] + 1e-9:
            raise ValueError(
                f"frequency {p['freq']:g} Hz is above the session cap {self.values['freq_cap_hz']:g} Hz (Limits)"
            )

    def clamp_joint(self, jm: JointMap, joint: str, deg: float) -> tuple[float, bool]:
        if joint not in jm.names:
            return deg, False
        lo, hi = soft_range_deg(jm, joint, self.values["joint_range_frac"])
        c = min(hi, max(lo, deg))
        return c, abs(c - deg) > 1e-9

    def scale_walk(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        s = self.values["walk_speed_scale"]
        return vx * s, vy * s, wz * s


# ------------------------------------------------------------------------------------------------ calibration (#15)
class ZeroOffsets:
    """Joint zero-offset capture: measured angle at a known reference pose minus the reference. Stored, not applied."""

    def __init__(self, jm: JointMap, path: Path, twin_offsets_rad=None):
        self.jm = jm
        self.path = path
        self.twin = [math.degrees(float(v)) for v in twin_offsets_rad] if twin_offsets_rad is not None else None
        self.pending: Optional[dict] = None

    def saved(self) -> Optional[dict]:
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None

    def reference_deg(self, reference: str) -> list[float]:
        if reference == "home":
            return [math.degrees(float(d)) for d in self.jm.defaults]
        if reference == "zero":
            return [0.0] * self.jm.num_joints
        raise ValueError("reference must be 'zero' (joints at their zero marks) or 'home' (the stand home pose)")

    def capture(self, q_deg: list, source: str, reference: str, joints: Optional[list] = None) -> dict:
        ref = self.reference_deg(reference)
        sel = joints or [n for i, n in enumerate(self.jm.names) if i >= LEG_JOINTS]
        rows = []
        for n in sel:
            if n not in self.jm.names:
                raise ValueError(f"unknown joint {n!r}")
            i = self.jm.index_of(n)
            off = float(q_deg[i]) - ref[i]
            verdict = "ok" if abs(off) <= 1.0 else ("warn" if abs(off) <= 3.0 else "check")
            rows.append({"joint": n, "measured_deg": round(float(q_deg[i]), 2), "reference_deg": round(ref[i], 2),
                         "offset_deg": round(off, 2), "verdict": verdict,
                         "twin_offset_deg": round(self.twin[i], 2) if self.twin else None})  # fmt: skip
        self.pending = {"source": source, "reference": reference, "captured_at": time.time(), "rows": rows}
        return {"ok": True, **self.pending}

    def save(self, confirm: bool, offsets: Optional[dict] = None) -> dict:
        if not confirm:
            return {"ok": False, "error": "confirmation required to store offsets"}
        if offsets is None:
            if not self.pending:
                return {"ok": False, "error": "nothing captured yet"}
            offsets = {r["joint"]: r["offset_deg"] for r in self.pending["rows"]}
        clean = {}
        for n, v in offsets.items():
            if n not in self.jm.names:
                raise ValueError(f"unknown joint {n!r}")
            v = float(v)
            if not (math.isfinite(v) and abs(v) <= 15.0):
                raise ValueError(f"{n}: offset {v:g} deg is implausible (over 15 deg): re-seat the joint and recapture")
            clean[n] = round(v, 2)
        doc = {
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "source": (self.pending or {}).get("source"),
            "reference": (self.pending or {}).get("reference"),
            "offsets_deg": clean,
            "applied": False,
            "note": "stored for review; not applied to any controller (applying them needs a validation session)",
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        return {"ok": True, **doc}


# ------------------------------------------------------------------------------------------------ network (#17)
DEFAULT_PROFILES = [
    {"name": "sim", "target": "sim", "host": "127.0.0.1", "port": 8765, "interface": "", "note": "headless MuJoCo"},
    {"name": "urlab", "target": "urlab", "host": "127.0.0.1", "port": 5555, "interface": "", "note": "Unreal twin"},
    {"name": "real", "target": "real", "host": "192.168.123.164", "port": 22, "interface": "",
     "note": "R1 Jetson over Ethernet (set the interface for --network)"},
]  # fmt: skip


def local_interfaces() -> list[dict]:
    out = []
    try:
        import psutil  # optional, gives friendly names + addresses

        for name, addrs in psutil.net_if_addrs().items():
            v4 = [a.address for a in addrs if a.family == socket.AF_INET]
            out.append({"name": name, "ipv4": v4})
        return out
    except ImportError:
        pass
    try:
        for _, name in socket.if_nameindex():
            out.append({"name": name, "ipv4": []})
    except (OSError, AttributeError):
        pass
    try:
        v4 = sorted({ai[4][0] for ai in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)})
        out.append({"name": "(host addresses)", "ipv4": v4})
    except OSError:
        pass
    return out


def icmp_ping(host: str, timeout_ms: int = 800) -> tuple[bool, Optional[float]]:
    if not re.match(r"^[A-Za-z0-9.\-:]{1,253}$", host):
        return False, None
    cmd = (["ping", "-n", "1", "-w", str(timeout_ms), host] if sys.platform == "win32"
           else ["ping", "-c", "1", "-W", str(max(1, timeout_ms // 1000)), host])  # fmt: skip
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 2)
    except (OSError, subprocess.SubprocessError):
        return False, None
    m = re.search(r"time[=<]\s*([\d.]+)\s*ms", r.stdout)
    return r.returncode == 0, (float(m.group(1)) if m else None)


class ConnectionProfiles:
    def __init__(self, path: Path):
        self.path = path

    def load(self) -> dict:
        try:
            doc = json.loads(self.path.read_text(encoding="utf-8"))
            if isinstance(doc.get("profiles"), list):
                return doc
        except (OSError, json.JSONDecodeError):
            pass
        return {"profiles": [dict(p) for p in DEFAULT_PROFILES], "active": "sim"}

    def _write(self, doc: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")

    def upsert(self, p: dict) -> dict:
        name = str(p.get("name", "")).strip()
        if not re.match(r"^[A-Za-z0-9_\-]{1,32}$", name):
            raise ValueError("profile name: 1-32 letters, digits, - or _")
        target = str(p.get("target", "real"))
        if target not in ("sim", "urlab", "real"):
            raise ValueError("target must be sim, urlab or real")
        host = str(p.get("host", "")).strip()
        if not re.match(r"^[A-Za-z0-9.\-:]{1,253}$", host):
            raise ValueError("host must be a hostname or IP address")
        port = int(p.get("port", 22))
        if not 1 <= port <= 65535:
            raise ValueError("port must be 1-65535")
        prof = {"name": name, "target": target, "host": host, "port": port,
                "interface": str(p.get("interface", ""))[:64], "note": str(p.get("note", ""))[:120]}  # fmt: skip
        doc = self.load()
        doc["profiles"] = [q for q in doc["profiles"] if q["name"] != name] + [prof]
        self._write(doc)
        return {"ok": True, **doc}

    def delete(self, name: str) -> dict:
        doc = self.load()
        doc["profiles"] = [q for q in doc["profiles"] if q["name"] != name]
        if doc.get("active") == name:
            doc["active"] = doc["profiles"][0]["name"] if doc["profiles"] else None
        self._write(doc)
        return {"ok": True, **doc}

    def activate(self, name: str) -> dict:
        doc = self.load()
        if not any(q["name"] == name for q in doc["profiles"]):
            raise ValueError(f"no profile {name!r}")
        doc["active"] = name
        self._write(doc)
        return {"ok": True, **doc}


def probe_connection(host: str, port: int, icmp: bool = True) -> dict:
    if not re.match(r"^[A-Za-z0-9.\-:]{1,253}$", host or ""):
        raise ValueError("host must be a hostname or IP address")
    ok, ms, detail = tcp_probe(host, int(port), timeout=1.0)
    res = {"ok": True, "host": host, "port": int(port), "tcp_ok": ok, "tcp_ms": round(ms, 1), "detail": detail}
    if icmp:
        pok, pms = icmp_ping(host)
        res.update({"ping_ok": pok, "ping_ms": pms})
    res["reachable"] = ok or bool(res.get("ping_ok"))
    return res


# ------------------------------------------------------------------------------------------------ docs (#19)
def docs_index(docs_dir: Path = DOCS_DIR) -> list[dict]:
    out = []
    for p in sorted(docs_dir.glob("*.md")) if docs_dir.is_dir() else []:
        title = p.stem.replace("_", " ")
        try:
            for line in p.read_text(encoding="utf-8").splitlines():
                if line.startswith("# "):
                    title = line[2:].strip()
                    break
        except OSError:
            continue
        out.append({"name": p.stem, "title": title, "bytes": p.stat().st_size})
    return out


def doc_text(name: str, docs_dir: Path = DOCS_DIR) -> dict:
    if not DOC_NAME_RE.match(name or ""):
        return {"ok": False, "error": "bad document name", "_code": 400}
    p = docs_dir / f"{name}.md"
    if not p.is_file():
        return {"ok": False, "error": f"no document {name!r}", "_code": 404}
    text = p.read_text(encoding="utf-8")
    title = next((ln[2:].strip() for ln in text.splitlines() if ln.startswith("# ")), name)
    return {"ok": True, "name": name, "title": title, "markdown": text}


# ------------------------------------------------------------------------------------------------ health (#1)
def run_probes(probes: list[tuple[str, str, bool, Callable[[], tuple[bool, str]]]]) -> list[dict]:
    """Run (section, name, optional, fn) probes in parallel; each fn returns (ok, detail)."""

    def one(p):
        section, name, optional, fn = p
        t0 = time.perf_counter()
        try:
            ok, detail = fn()
        except Exception as e:  # a probe must never break /health
            ok, detail = False, f"{type(e).__name__}: {e}"[:160]
        return {"section": section, "name": name, "ok": bool(ok), "optional": optional,
                "ms": round((time.perf_counter() - t0) * 1000.0, 1), "detail": detail}  # fmt: skip

    with ThreadPoolExecutor(max_workers=8) as ex:
        return list(ex.map(one, probes))


class Preflight:
    """Pre-flight checklist gating real-robot runs: automatic checks + operator ticks (valid 30 min)."""

    MANUAL = (
        ("on_stand", "Robot hangs on its stand, feet clear of the floor"),
        ("estop_tested", "Remote in hand; L2+B emergency stop tested this session"),
        ("area_clear", "Nobody within arm's reach of the robot"),
        ("head_forward", "Head faces forward, arms hang at the sides"),
        ("battery_checked", "Battery charged (check the Power panel)"),
    )

    def __init__(self, auto_checks: Callable[[], list[dict]]):
        self.auto_checks = auto_checks
        self.ticks: dict = {}
        self._lock = threading.Lock()

    def tick(self, item: str, checked: bool) -> dict:
        if item not in dict(self.MANUAL):
            raise ValueError(f"unknown checklist item {item!r}")
        with self._lock:
            if checked:
                self.ticks[item] = time.time()
            else:
                self.ticks.pop(item, None)
        return self.state()

    def reset(self) -> dict:
        with self._lock:
            self.ticks.clear()
        return self.state()

    def state(self) -> dict:
        now = time.time()
        with self._lock:
            ticks = dict(self.ticks)
        items = []
        for k, label in self.MANUAL:
            at = ticks.get(k)
            fresh = at is not None and now - at <= PREFLIGHT_VALID_S
            items.append({"id": k, "label": label, "kind": "manual", "ok": fresh,
                          "detail": (f"ticked {int(now - at)} s ago" if fresh else ("expired: tick again" if at else ""))})  # fmt: skip
        autos = self.auto_checks()
        items += [dict(a, kind="auto") for a in autos]
        complete = all(i["ok"] for i in items)
        oldest = min((ticks[k] for k, _ in self.MANUAL if k in ticks), default=None)
        return {
            "ok": True,
            "complete": complete,
            "items": items,
            "valid_for_s": round(PREFLIGHT_VALID_S - (now - oldest), 0) if complete and oldest else None,
            "missing": [i["label"] for i in items if not i["ok"]],
        }

    def why_not(self) -> Optional[str]:
        st = self.state()
        if st["complete"]:
            return None
        return f"complete the pre-flight checklist ({len(st['missing'])} item(s) open)"
