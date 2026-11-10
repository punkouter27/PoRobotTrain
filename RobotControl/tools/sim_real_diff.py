"""Compare two trajectory logs (``TrajectoryLogger`` .npz or .csv): per-joint max / RMS
position error in degrees after resampling B onto A's time base.  Pass if max < tol
(default 5 deg).

    python -m robotcontrol.tools.sim_real_diff logs/sim.npz logs/real.npz [--tol-deg 5] [--key q]
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def load_log(path: str | Path, key: str = "q") -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """Return ``(t, values(T, N), joint_names)``."""
    path = Path(path)
    if path.suffix == ".npz":
        d = np.load(path, allow_pickle=False)
        t = np.asarray(d["t"], dtype=np.float64)
        v = np.asarray(d[key], dtype=np.float64)
        names = [str(n) for n in d["joint_names"]] if "joint_names" in d else [f"j{i}" for i in range(v.shape[1])]
        return t, v, names
    if path.suffix == ".csv":
        with open(path, "r", encoding="utf-8", newline="") as f:
            r = csv.reader(f)
            header = next(r)
            rows = [row for row in r]
        prefix = {"q": "q_", "dq": "dq_", "target_q": "target_"}.get(key, key + "_")
        cols = [i for i, h in enumerate(header) if h.startswith(prefix)]
        names = [header[i][len(prefix):] for i in cols]
        arr = np.array([[float(x) for x in row] for row in rows]) if rows else np.zeros((0, len(header)))
        return arr[:, 0], arr[:, cols], names
    raise ValueError(f"unsupported log format {path.suffix}")


def compare(t_a: np.ndarray, a: np.ndarray, t_b: np.ndarray, b: np.ndarray, align_start: bool = True) -> Dict[str, np.ndarray]:
    if a.shape[1] != b.shape[1]:
        raise ValueError(f"joint count mismatch {a.shape[1]} vs {b.shape[1]}")
    if align_start:
        t_a = t_a - t_a[0]
        t_b = t_b - t_b[0]
    t0, t1 = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1])
    mask = (t_a >= t0) & (t_a <= t1)
    if mask.sum() < 2:
        raise ValueError("logs do not overlap in time")
    ta = t_a[mask]
    aa = a[mask]
    bb = np.stack([np.interp(ta, t_b, b[:, j]) for j in range(b.shape[1])], axis=1)
    err = np.degrees(aa - bb)
    return {
        "t": ta,
        "err_deg": err,
        "max_deg": np.abs(err).max(axis=0),
        "rms_deg": np.sqrt(np.mean(err ** 2, axis=0)),
        "mean_deg": err.mean(axis=0),
        "overlap_s": np.float64(t1 - t0),
    }


def format_table(names: List[str], res: Dict[str, np.ndarray], tol_deg: float) -> str:
    lines = [f"{'joint':30s} {'max[deg]':>9s} {'rms[deg]':>9s} {'bias[deg]':>10s}  status"]
    for i, n in enumerate(names):
        ok = res["max_deg"][i] < tol_deg
        lines.append(f"{n:30s} {res['max_deg'][i]:9.3f} {res['rms_deg'][i]:9.3f} {res['mean_deg'][i]:10.3f}  {'ok' if ok else 'FAIL'}")
    lines.append(f"{'OVERALL':30s} {res['max_deg'].max():9.3f} {np.sqrt(np.mean(res['rms_deg'] ** 2)):9.3f} "
                 f"{'':10s}  overlap {float(res['overlap_s']):.2f}s")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log_a", type=Path)
    ap.add_argument("log_b", type=Path)
    ap.add_argument("--tol-deg", type=float, default=5.0)
    ap.add_argument("--key", default="q", help="array to compare: q | dq | target_q")
    ap.add_argument("--no-align", action="store_true", help="do not shift both logs to start at t=0")
    args = ap.parse_args(argv)
    t_a, a, names = load_log(args.log_a, args.key)
    t_b, b, _ = load_log(args.log_b, args.key)
    res = compare(t_a, a, t_b, b, align_start=not args.no_align)
    print(format_table(names, res, args.tol_deg))
    worst = int(np.argmax(res["max_deg"]))
    passed = res["max_deg"].max() < args.tol_deg
    print(f"{'PASS' if passed else 'FAIL'}: max error {res['max_deg'].max():.3f} deg on {names[worst]} (tol {args.tol_deg} deg)")
    return 0 if passed else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
