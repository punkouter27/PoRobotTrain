"""Operator console: fault/alarm event log, telemetry history (temperatures, power, IMU) and their checks.

Everything here is read-only bookkeeping on the telemetry the service already receives (``Service.update_telemetry``);
nothing commands a motor. Thresholds match the web twin's tints (55 C warning, 65 C critical).
Contract: docs/service_api.md section 4.
"""

from __future__ import annotations

import collections
import json
import math
import threading
import time
from pathlib import Path
from typing import Optional, Sequence

TEMP_WARN_C = 55
TEMP_CRIT_C = 65
LIMIT_MARGIN_DEG = 2.0
HISTORY_S = 600.0  # 10 min of 1 Hz samples per source
EVENT_DEDUP_S = 60.0
MIN_DRIFT_WINDOW_S = 20.0  # shorter windows turn sensor noise into a fake yaw drift

# Generic Li-ion open-circuit voltage per cell -> state of charge (%). The pack's series cell count comes from
# configs/battery.json; the estimate is labelled as such everywhere it is shown.
LIION_OCV = ((3.00, 0), (3.30, 5), (3.50, 10), (3.60, 20), (3.70, 40), (3.80, 55), (3.90, 70), (4.00, 80),
             (4.10, 92), (4.20, 100))  # fmt: skip
LEVELS = ("info", "warn", "critical")


def soc_from_cell_v(v_cell: float) -> float:
    if v_cell <= LIION_OCV[0][0]:
        return 0.0
    for (v0, s0), (v1, s1) in zip(LIION_OCV, LIION_OCV[1:]):
        if v_cell <= v1:
            return s0 + (s1 - s0) * (v_cell - v0) / (v1 - v0)
    return 100.0


def _slope(ts: Sequence[float], ys: Sequence[float]) -> Optional[float]:
    """Least-squares slope dy/dt, or None with fewer than 3 samples or no time spread."""
    n = len(ts)
    if n < 3:
        return None
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    if den < 1e-9:
        return None
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den


def _std(xs: Sequence[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def gravity_in_body(roll_deg: float, pitch_deg: float) -> list[float]:
    """Unit 'up' vector expressed in the body frame (z-up when level), from roll and pitch."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    return [round(-math.sin(p), 4), round(math.sin(r) * math.cos(p), 4), round(math.cos(r) * math.cos(p), 4)]


class EventLog:
    """Ring buffer of operator events, mirrored to a JSON-lines file when a path is given."""

    def __init__(self, path: Optional[Path] = None, maxlen: int = 1000):
        self.path = path
        self.events: collections.deque = collections.deque(maxlen=maxlen)
        self._last: dict = {}
        self._lock = threading.Lock()
        self._seq = 0

    def record(
        self,
        level: str,
        kind: str,
        message: str,
        source: Optional[str] = None,
        joint: Optional[str] = None,
        dedup_key: Optional[str] = None,
        dedup_s: float = EVENT_DEDUP_S,
    ) -> Optional[dict]:
        if level not in LEVELS:
            raise ValueError(f"level must be one of {LEVELS}")
        now = time.time()
        with self._lock:
            if dedup_key is not None:
                last = self._last.get(dedup_key)
                if last is not None and now - last < dedup_s:
                    return None
                self._last[dedup_key] = now
            self._seq += 1
            ev = {"id": self._seq, "at": round(now, 3), "level": level, "kind": kind, "message": message}
            if source:
                ev["source"] = source
            if joint:
                ev["joint"] = joint
            self.events.append(ev)
        if self.path is not None:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                with self.path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(ev) + "\n")
            except OSError:
                pass
        return ev

    def list(
        self, level: Optional[str] = None, kind: Optional[str] = None, since: float = 0.0, limit: int = 200
    ) -> list[dict]:
        min_rank = LEVELS.index(level) if level in LEVELS else 0
        with self._lock:
            out = [
                e
                for e in self.events
                if LEVELS.index(e["level"]) >= min_rank and (not kind or e["kind"] == kind) and e["at"] > since
            ]
        return out[-max(1, min(int(limit), 1000)) :][::-1]  # newest first

    def counts(self, since: float = 0.0) -> dict:
        with self._lock:
            c = {lv: 0 for lv in LEVELS}
            for e in self.events:
                if e["at"] > since:
                    c[e["level"]] += 1
        return c

    def clear(self) -> None:
        with self._lock:
            self.events.clear()
            self._last.clear()


class TelemetryHistory:
    """Per-source history: 1 Hz temperature / bus-voltage samples and full-rate IMU samples (bounded)."""

    def __init__(self, joint_names: Sequence[str], series_cells: int = 13, history_s: float = HISTORY_S):
        self.names = [n.replace("_joint", "") for n in joint_names]
        self.series_cells = max(1, int(series_cells))
        self.history_s = history_s
        self.slow: dict = {}  # who -> deque[(t, temps|None, bus_v|None)]
        self.imu: dict = {}  # who -> deque[(t, r, p, y)]
        self._last_slow: dict = {}
        self._lock = threading.Lock()

    def add(self, who: str, pkt: dict, now: Optional[float] = None) -> None:
        now = time.time() if now is None else now
        with self._lock:
            rpy = pkt.get("imu_rpy_deg")
            if isinstance(rpy, (list, tuple)) and len(rpy) == 3 and all(v is not None for v in rpy):
                dq = self.imu.setdefault(who, collections.deque(maxlen=1200))
                dq.append((now, float(rpy[0]), float(rpy[1]), float(rpy[2])))
            if now - self._last_slow.get(who, 0.0) < 1.0:
                return
            temps = pkt.get("temps_c")
            temps = [int(t) for t in temps] if isinstance(temps, list) and len(temps) == len(self.names) else None
            bus_v = pkt.get("bus_v")
            bus_v = float(bus_v) if isinstance(bus_v, (int, float)) and bus_v > 1.0 else None
            if temps is None and bus_v is None:
                return
            self._last_slow[who] = now
            dq = self.slow.setdefault(who, collections.deque(maxlen=int(self.history_s) + 5))
            dq.append((now, temps, bus_v))
            while dq and now - dq[0][0] > self.history_s:
                dq.popleft()

    def sources(self) -> list[str]:
        with self._lock:
            return sorted(set(self.slow) | set(self.imu))

    # ---------------------------------------------------------------- temperatures (#4)
    def temperatures(self, who: str, window_s: float = HISTORY_S) -> dict:
        now = time.time()
        with self._lock:
            rows = [(t, temps) for t, temps, _ in self.slow.get(who, ()) if temps is not None and now - t <= window_s]
        out = {
            "source": who,
            "warn_c": TEMP_WARN_C,
            "crit_c": TEMP_CRIT_C,
            "joint_names": self.names,
            "t": [round(t - now, 1) for t, _ in rows],
            "series": [[temps[j] for _, temps in rows] for j in range(len(self.names))] if rows else [],
        }
        if rows:
            last = rows[-1][1]
            peak = [max(temps[j] for _, temps in rows) for j in range(len(self.names))]
            rates = []
            for j in range(len(self.names)):
                s = _slope([t for t, _ in rows[-120:]], [temps[j] for _, temps in rows[-120:]])
                rates.append(round(s * 60.0, 2) if s is not None else None)
            hot = max(range(len(last)), key=lambda j: last[j])
            out.update(
                {
                    "latest": last,
                    "peak": peak,
                    "rate_c_per_min": rates,
                    "hottest": {"joint": self.names[hot], "temp_c": last[hot]},
                }
            )
        return out

    # ---------------------------------------------------------------- power (#3)
    def power(self, who: str) -> dict:
        now = time.time()
        with self._lock:
            rows = [(t, v) for t, _, v in self.slow.get(who, ()) if v is not None]
        out = {
            "source": who,
            "series_cells": self.series_cells,
            "assumption": f"state of charge estimated from bus voltage, generic Li-ion curve, {self.series_cells}S pack",
            "current_a": None,
            "current_note": "the low-level state stream does not report pack current",
            "t": [round(t - now, 1) for t, _ in rows],
            "bus_v": [v for _, v in rows],
        }
        if not rows:
            out.update({"available": False, "latest_v": None, "soc_pct": None, "minutes_left": None})
            return out
        v = rows[-1][1]
        soc = soc_from_cell_v(v / self.series_cells)
        recent = [(t, soc_from_cell_v(vv / self.series_cells)) for t, vv in rows if now - t <= 300.0]
        slope = _slope([t for t, _ in recent], [s for _, s in recent])  # % per s
        span = (recent[-1][0] - recent[0][0]) if len(recent) > 1 else 0.0
        minutes_left = None
        if slope is not None and slope < -1e-4 and span >= 60.0:
            minutes_left = round(soc / (-slope) / 60.0, 1)
        out.update(
            {
                "available": True,
                "latest_v": round(v, 2),
                "cell_v": round(v / self.series_cells, 3),
                "soc_pct": round(soc, 1),
                "trend_v_per_min": round(_slope([t for t, _ in rows], [vv for _, vv in rows]) * 60.0, 3)
                if len(rows) >= 3 and rows[-1][0] - rows[0][0] > 1.0
                else None,
                "minutes_left": minutes_left,
                "low": v / self.series_cells < 3.5,
            }
        )
        return out

    # ---------------------------------------------------------------- IMU check (#16)
    def imu_check(self, who: str, window_s: float = 60.0) -> dict:
        now = time.time()
        with self._lock:
            rows = [r for r in self.imu.get(who, ()) if now - r[0] <= window_s]
        if len(rows) < 3:
            return {
                "source": who,
                "available": False,
                "verdict": "no_data",
                "samples": len(rows),
                "notes": ["no IMU samples yet: start Monitor (read-only) or a practice stream"],
            }
        r, p, y = [x[1] for x in rows], [x[2] for x in rows], [x[3] for x in rows]
        ts = [x[0] for x in rows]
        # unwrap yaw so a +-180 crossing does not look like drift
        yu = [y[0]]
        for a, b in zip(y, y[1:]):
            d = (b - a + 180.0) % 360.0 - 180.0
            yu.append(yu[-1] + d)
        drift = _slope(ts, yu)
        roll_std, pitch_std = _std(r), _std(p)
        tilt = math.degrees(
            math.acos(max(-1.0, min(1.0, math.cos(math.radians(r[-1])) * math.cos(math.radians(p[-1])))))
        )
        still = roll_std < 0.5 and pitch_std < 0.5
        notes, verdict = [], "pass"
        if not still:
            notes.append("robot moved during the window: hold it still for a drift reading")
            verdict = "warn"
        span = ts[-1] - ts[0]
        drift_min = round(drift * 60.0, 2) if drift is not None and span >= MIN_DRIFT_WINDOW_S else None
        if drift_min is None:
            notes.append(f"hold still for {MIN_DRIFT_WINDOW_S:.0f} s for a yaw drift reading ({span:.0f} s so far)")
        elif still and abs(drift_min) > 1.0:
            notes.append(f"yaw drifts {drift_min:+.2f} deg/min while still (over 1 deg/min)")
            verdict = "warn"
        if tilt > 10.0:
            notes.append(f"body tilted {tilt:.1f} deg from level: on the stand it should be near level")
            verdict = "warn"
        return {
            "source": who,
            "available": True,
            "verdict": verdict,
            "samples": len(rows),
            "window_s": round(ts[-1] - ts[0], 1),
            "rpy_deg": [r[-1], p[-1], y[-1]],
            "gravity_body": gravity_in_body(r[-1], p[-1]),
            "tilt_deg": round(tilt, 2),
            "roll_std_deg": round(roll_std, 3),
            "pitch_std_deg": round(pitch_std, 3),
            "yaw_drift_deg_per_min": drift_min,
            "still": still,
            "notes": notes,
        }


def scan_telemetry(
    log: EventLog,
    who: str,
    pkt: dict,
    joint_names: Sequence[str],
    limits_deg: Sequence[Sequence[float]],
    series_cells: int,
) -> None:
    """Turn a telemetry packet into alarm events (temperatures, joints at a limit, low battery). Deduplicated."""
    names = [n.replace("_joint", "") for n in joint_names]
    temps = pkt.get("temps_c")
    if isinstance(temps, list) and len(temps) == len(names):
        for n, t in zip(names, temps):
            if t is None:
                continue
            if t >= TEMP_CRIT_C:
                log.record("critical", "temperature", f"{n} at {t} C (critical {TEMP_CRIT_C} C)", who, n, f"tc:{who}:{n}")
            elif t >= TEMP_WARN_C:
                log.record("warn", "temperature", f"{n} at {t} C (warning {TEMP_WARN_C} C)", who, n, f"tw:{who}:{n}")
    q = pkt.get("q_deg")
    if isinstance(q, list) and len(q) == len(names):
        for n, v, (lo, hi) in zip(names, q, limits_deg):
            if v is None:
                continue
            if v <= lo + LIMIT_MARGIN_DEG or v >= hi - LIMIT_MARGIN_DEG:
                log.record(
                    "warn", "limit", f"{n} at {v:.1f} deg, within {LIMIT_MARGIN_DEG:g} deg of its limit", who, n,
                    f"lim:{who}:{n}",
                )  # fmt: skip
    bus_v = pkt.get("bus_v")
    if isinstance(bus_v, (int, float)) and bus_v > 1.0 and bus_v / max(1, series_cells) < 3.5:
        log.record("warn", "power", f"bus voltage low: {bus_v:.1f} V", who, None, f"lowv:{who}", dedup_s=300.0)
