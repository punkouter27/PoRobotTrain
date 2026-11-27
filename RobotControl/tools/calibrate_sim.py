"""Calibrate the MuJoCo digital twin (`sim_calibration.yaml`) against real Unitree R1 logs.

Implements automated sim-to-real system identification across:
  - noise   : measure encoder sigma, velocity jitter, and bus voltage from idle real logs
  - fit     : fit zero-offset biases + per-joint (armature, damping, frictionloss) via coordinate search
  - verify  : run `sim_real_diff` parity check comparing calibrated SimBackend vs. a real CSV/NPZ log
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from ..behaviours.backends import SimBackend
from ..behaviours.neck_turn import SweepParams
from ..behaviours.stand_behaviours import BEHAVIOUR_SPECS
from ..configs.joint_map import JointMap
from ..configs.sim_calibration import DEFAULT_CALIBRATION_PATH, SimCalibration
from .sim_real_diff import compare, format_table, load_log


def load_behaviour_or_traj_log(path: str | Path, jm: JointMap) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """Load either a `TrajectoryLogger` (.npz/.csv with q_ prefix) or `behaviours.runner` CsvLog."""
    p = Path(path)
    if p.suffix == ".npz":
        return load_log(p, "q")
    with open(p, encoding="utf-8", newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        raise ValueError(f"empty log {p}")
    hdr = rows[0]
    if any(h.startswith("q_") for h in hdr):
        return load_log(p, "q")
    cols = [hdr.index(n) for n in jm.names]
    t_col = hdr.index("t")
    arr = np.array([[float(r[c]) for c in ([t_col] + cols)] for r in rows[1:]], dtype=np.float64)
    return arr[:, 0], arr[:, 1:], list(jm.names)


def log_meta(path: str | Path) -> dict:
    """The sidecar a behaviour log is written with (empty when there is none)."""
    p = Path(path).with_suffix(".json")
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


def resolve_sweep(path: str | Path, behaviour=None, amplitude=None, freq=None, duration=None):
    """Behaviour parameters for a replay: the log's own sidecar first, an explicit CLI value second.

    A replay with different parameters is not a parity check — it is two different motions. The sidecar is
    therefore authoritative; a CLI value that disagrees is honoured but reported as an override.
    """
    meta = log_meta(path)
    name = behaviour or meta.get("behaviour") or "neck_turn"
    amp = float(amplitude if amplitude is not None else meta.get("amplitude_deg", 10.0))
    hz = float(freq if freq is not None else meta.get("freq_hz", 0.2))
    dur = duration if duration is not None else meta.get("duration_s")
    dur = None if dur is None else float(dur)
    overrides = []
    if (
        amplitude is not None
        and "amplitude_deg" in meta
        and abs(float(amplitude) - float(meta["amplitude_deg"])) > 1e-9
    ):
        overrides.append(f"amplitude {amplitude} vs logged {meta['amplitude_deg']}")
    if freq is not None and "freq_hz" in meta and abs(float(freq) - float(meta["freq_hz"])) > 1e-9:
        overrides.append(f"freq {freq} vs logged {meta['freq_hz']}")
    if duration is not None and "duration_s" in meta and abs(float(duration) - float(meta["duration_s"])) > 1e-9:
        overrides.append(f"duration {duration} vs logged {meta['duration_s']}")
    return name, amp, hz, dur, overrides


def measure_idle_noise(t: np.ndarray, q: np.ndarray) -> Dict[str, float]:
    """Estimate encoder position noise and finite-diff velocity jitter from an idle holding segment."""
    if len(t) < 5:
        raise ValueError("need at least 5 samples to estimate noise")
    pos_sigma = float(np.median(np.std(np.diff(q, axis=0), axis=0)) / math.sqrt(2.0))
    dt = np.maximum(np.diff(t), 1e-4)
    dq = np.diff(q, axis=0) / dt[:, None]
    vel_sigma = float(np.median(np.std(dq, axis=0)))
    return {
        "pos_noise_rad": round(pos_sigma, 6),
        "vel_noise_rad_s": round(vel_sigma, 4),
    }


def simulate_behaviour_trajectory(
    jm: JointMap,
    cal: SimCalibration,
    behaviour: str,
    amplitude_deg: float = 10.0,
    freq_hz: float = 0.2,
    duration_s: Optional[float] = None,
    start_q: Optional[Dict[str, float]] = None,
    stop_s: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Run an offline headless rollout in calibrated `SimBackend` and return `(t, q[T, 26])`.

    ``duration_s`` is the behaviour's own sweeping time (fades excluded); ``stop_s`` truncates the rollout.
    The two are separate on purpose: a replay has to use the recorded sweep parameters or it compares a
    different motion.
    """
    spec = BEHAVIOUR_SPECS[behaviour]
    params = SweepParams(amplitude_deg=amplitude_deg, freq_hz=freq_hz, duration_s=duration_s or 20.0)
    beh = spec.cls(jm, params, check_tracking=False)
    backend = SimBackend(jm, beh.powered, start_q=start_q, calibration=cal, realistic=True, settle_s=0.5)
    try:
        dt = 1.0 / backend.rate_hz
        q0, dq0, age0 = backend.read()
        beh.start(dict(zip(jm.names, q0)))
        ts, qs = [], []
        t, k = 0.0, 0
        total = beh.duration_s if stop_s is None else min(beh.duration_s, stop_s)
        while t <= total + 1e-9:
            q, dq, age = backend.read()
            ts.append(t)
            qs.append(q.copy())
            targets = beh.targets(t, dict(zip(jm.names, q)), dict(zip(jm.names, dq)), age)
            if targets is None:
                break
            backend.command(targets)
            k += 1
            t = k * dt
            backend.advance(t)
        return np.asarray(ts, dtype=np.float64), np.asarray(qs, dtype=np.float64)
    finally:
        backend.close()


def fit_calibration(
    real_log: Path,
    behaviour: Optional[str] = None,
    amplitude_deg: Optional[float] = None,
    freq_hz: Optional[float] = None,
    cal_path: Path = DEFAULT_CALIBRATION_PATH,
    write: bool = False,
    duration_s: Optional[float] = None,
) -> dict:
    """Fit zero-offset biases + powered-joint (armature, damping, frictionloss) against a real log.

    Parameters come from the log's sidecar unless the caller overrides them; a replay with different
    parameters would fit against a motion the robot never made.
    """
    jm = JointMap.load()
    cal = SimCalibration.load(cal_path, jm=jm)
    behaviour, amplitude_deg, freq_hz, duration_s, overrides = resolve_sweep(
        real_log, behaviour, amplitude_deg, freq_hz, duration_s
    )
    for o in overrides:
        print(f"WARNING: overriding the logged sweep: {o}")
    t_real, q_real, _ = load_behaviour_or_traj_log(real_log, jm)
    start_q = dict(zip(jm.names, q_real[0].tolist()))
    dur = float(t_real[-1] - t_real[0])

    # Baseline parity before fitting
    t_b, q_b = simulate_behaviour_trajectory(
        jm, cal, behaviour, amplitude_deg, freq_hz, duration_s=duration_s, start_q=start_q
    )
    before = compare(t_real, q_real, t_b, q_b, align_start=True)

    spec = BEHAVIOUR_SPECS[behaviour]
    pidx = [jm.index_of(n) for n in spec.cls(jm, SweepParams()).powered]
    mean_err_rad = np.radians(before["mean_deg"])

    # Update zero-offset bias on non-sweeping bias
    for i in pidx:
        cal.zero_offset_rad[i] += float(mean_err_rad[i]) * 0.5

    def loss(c: SimCalibration) -> float:
        ts, qs = simulate_behaviour_trajectory(
            jm,
            c,
            behaviour,
            amplitude_deg,
            freq_hz,
            duration_s=duration_s,
            start_q=start_q,
            stop_s=min(dur, 4.0),
        )
        res = compare(t_real[t_real <= t_real[0] + 4.0], q_real[t_real <= t_real[0] + 4.0], ts, qs, align_start=True)
        return float(np.sqrt(np.mean(res["rms_deg"][pidx] ** 2)))

    best_loss = loss(cal)
    for attr in ("frictionloss", "damping", "armature"):
        arr = getattr(cal, attr)
        for scale in (0.7, 0.85, 1.15, 1.35):
            cand_cal = SimCalibration.load(cal_path, jm=jm)
            cand_cal.zero_offset_rad[:] = cal.zero_offset_rad
            cand_arr = getattr(cand_cal, attr)
            cand_arr[pidx] = np.clip(arr[pidx] * scale, 1e-4, 10.0)
            cand_loss = loss(cand_cal)
            if cand_loss < best_loss:
                best_loss = cand_loss
                arr[pidx] = cand_arr[pidx]

    t_a, q_a = simulate_behaviour_trajectory(
        jm, cal, behaviour, amplitude_deg, freq_hz, duration_s=duration_s, start_q=start_q
    )
    after = compare(t_real, q_real, t_a, q_a, align_start=True)

    rms_before = float(np.sqrt(np.mean(before["rms_deg"][pidx] ** 2)))
    rms_after = float(np.sqrt(np.mean(after["rms_deg"][pidx] ** 2)))
    max_after = float(after["max_deg"][pidx].max())

    if write:
        raw = yaml.safe_load(cal_path.read_text(encoding="utf-8")) or {}
        raw["last_calibrated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        raw["parity_rms_deg"] = round(rms_after, 3)
        raw["parity_max_deg"] = round(max_after, 3)
        jdict = raw.setdefault("joints", {})
        for i in pidx:
            jn = jm.names[i]
            entry = jdict.setdefault(jn, {})
            entry["armature"] = round(float(cal.armature[i]), 5)
            entry["damping"] = round(float(cal.damping[i]), 5)
            entry["frictionloss"] = round(float(cal.frictionloss[i]), 5)
            entry["zero_offset_deg"] = round(math.degrees(float(cal.zero_offset_rad[i])), 3)
        cal_path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")

    return {
        "rms_before_deg": rms_before,
        "rms_after_deg": rms_after,
        "max_after_deg": max_after,
        "table": format_table(jm.names, after, tol_deg=5.0),
    }


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Calibrate MuJoCo digital twin parameters against real R1 logs")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_noise = sub.add_parser("noise", help="Measure idle encoder & velocity noise from a real log")
    p_noise.add_argument("real_log", type=Path)

    p_fit = sub.add_parser("fit", help="Fit joint friction/damping/armature & zero-offset against a real log")
    p_fit.add_argument("real_log", type=Path)
    p_fit.add_argument("--behaviour", default=None, choices=sorted(BEHAVIOUR_SPECS))
    p_fit.add_argument("--amplitude", type=float, default=None, help="deg (default: the log's sidecar)")
    p_fit.add_argument("--freq", type=float, default=None, help="Hz (default: the log's sidecar)")
    p_fit.add_argument("--write", action="store_true", help="Save fitted parameters to sim_calibration.yaml")

    p_ver = sub.add_parser("verify", help="Verify calibrated SimBackend trajectory parity against a real log")
    p_ver.add_argument("real_log", type=Path)
    p_ver.add_argument("--behaviour", default=None, choices=sorted(BEHAVIOUR_SPECS))
    p_ver.add_argument("--amplitude", type=float, default=None, help="deg (default: the log's sidecar)")
    p_ver.add_argument("--freq", type=float, default=None, help="Hz (default: the log's sidecar)")
    p_ver.add_argument("--tol-deg", type=float, default=5.0)

    args = ap.parse_args(argv)
    jm = JointMap.load()
    if args.cmd == "noise":
        t, q, _ = load_behaviour_or_traj_log(args.real_log, jm)
        print(yaml.safe_dump(measure_idle_noise(t, q), sort_keys=False).strip())
        return 0
    if args.cmd == "fit":
        res = fit_calibration(args.real_log, args.behaviour, args.amplitude, args.freq, write=args.write)
        print(
            f"RMS before: {res['rms_before_deg']:.3f} deg -> after: {res['rms_after_deg']:.3f} deg "
            f"(max {res['max_after_deg']:.3f} deg)"
        )
        print(res["table"])
        return 0
    cal = SimCalibration.load(jm=jm)
    behaviour, amplitude_deg, freq_hz, duration_s, overrides = resolve_sweep(
        args.real_log, args.behaviour, args.amplitude, args.freq
    )
    for o in overrides:
        print(f"WARNING: overriding the logged sweep: {o}")
    t_real, q_real, _ = load_behaviour_or_traj_log(args.real_log, jm)
    start_q = dict(zip(jm.names, q_real[0].tolist()))
    ts, qs = simulate_behaviour_trajectory(
        jm, cal, behaviour, amplitude_deg, freq_hz, duration_s=duration_s, start_q=start_q
    )
    cmp = compare(t_real, q_real, ts, qs, align_start=True)
    print(format_table(jm.names, cmp, args.tol_deg))
    return 0 if cmp["max_deg"].max() < args.tol_deg else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
