"""Sim-to-real gap check (docs/sim_features.md #4): compare two run logs from ``logs/`` joint by joint.

    python -m robotcontrol.tools.gap_check look_at_sim_20261122_185903.csv look_at_real_20261122_191054.csv

Run logs are the runner's CSV files ``logs/<skill>_<sim|urlab|real>_<stamp>.csv`` (header ``t, phase,
target_<joint>..., <26 joint names>``). B is resampled onto A's time base after both start at t = 0
(``sim_real_diff.compare``); the curves are downsampled to at most MAX_POINTS for the web app's ghost view.
Only plain file names inside ``logs/`` are accepted (no paths).
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from .. import LOGS_DIR
from ..configs.joint_map import JointMap
from .sim_real_diff import compare

MAX_POINTS = 400
MAX_RUNS = 100
RUN_RE = re.compile(r"^(?P<skill>[A-Za-z0-9_]+?)_(?P<target>sim|urlab|real)_(?P<stamp>\d{8}_\d{6})\.csv$")
NAME_RE = re.compile(r"^[A-Za-z0-9_.-]{1,120}\.csv$")


def safe_log_path(name: str, logs_dir: Path = LOGS_DIR) -> Path:
    """A plain file name inside logs/ (no separators, no '..'); raises ValueError otherwise."""
    name = str(name or "")
    if not NAME_RE.match(name) or ".." in name or "/" in name or "\\" in name:
        raise ValueError(f"{name!r} is not a log file name (plain names inside logs/ only)")
    path = logs_dir / name
    if not path.is_file():
        raise ValueError(f"no log {name!r} in logs/")
    return path


def list_runs(logs_dir: Path = LOGS_DIR, limit: int = MAX_RUNS) -> List[dict]:
    """Run logs, newest first: [{file, skill, target, stamp}]."""
    out = []
    if not logs_dir.is_dir():
        return out
    for p in logs_dir.glob("*.csv"):
        m = RUN_RE.match(p.name)
        if m:
            out.append({"file": p.name, "skill": m["skill"], "target": m["target"], "stamp": m["stamp"]})
    out.sort(key=lambda r: (r["stamp"], r["file"]), reverse=True)
    return out[:limit]


def load_run(path: Path, names: Optional[List[str]] = None) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """(t, q (T, J) rad, joint names) from a runner CSV: every joint-map joint column present in the header."""
    names = names or list(JointMap.load().names)
    with open(path, encoding="utf-8", newline="") as f:
        r = csv.reader(f)
        header = next(r)
        rows = [row for row in r if row]
    if "t" not in header:
        raise ValueError(f"{path.name}: no t column")
    ti = header.index("t")
    cols = [(n, header.index(n)) for n in names if n in header]
    if not cols:
        raise ValueError(f"{path.name}: no joint columns")
    t, q = [], []
    for row in rows:
        try:
            t.append(float(row[ti]))
            q.append([float(row[i]) for _, i in cols])
        except (ValueError, IndexError):
            continue  # a truncated last line of an interrupted run
    if len(t) < 2:
        raise ValueError(f"{path.name}: fewer than 2 samples")
    return np.asarray(t), np.asarray(q), [n for n, _ in cols]


def gap(a: str, b: str, tol_deg: float = 5.0, logs_dir: Path = LOGS_DIR) -> dict:
    pa, pb = safe_log_path(a, logs_dir), safe_log_path(b, logs_dir)
    ta, qa, na = load_run(pa)
    tb, qb, nb = load_run(pb)
    common = [n for n in na if n in nb]
    qa = qa[:, [na.index(n) for n in common]]
    qb = qb[:, [nb.index(n) for n in common]]
    # B on A's time base (both shifted to start at 0), over the overlap
    ta0, tb0 = ta - ta[0], tb - tb[0]
    res = compare(ta0, qa, tb0, qb, align_start=False)
    t = res["t"]
    mask = (ta0 >= t[0]) & (ta0 <= t[-1])
    a_deg = np.degrees(qa[mask])
    b_deg = np.degrees(np.stack([np.interp(t, tb0, qb[:, j]) for j in range(qb.shape[1])], axis=1))
    err = res["err_deg"]
    sel = np.unique(np.linspace(0, len(t) - 1, min(MAX_POINTS, len(t))).round().astype(int))

    def r2(x) -> list:
        return np.round(np.asarray(x, dtype=np.float64), 2).tolist()

    max_deg = res["max_deg"]
    return {
        "ok": True,
        "a": pa.name,
        "b": pb.name,
        "tol_deg": float(tol_deg),
        "joints": common,
        "t": np.round(t[sel], 3).tolist(),
        "a_deg": r2(a_deg[sel]),
        "b_deg": r2(b_deg[sel]),
        "err_deg": r2(err[sel]),
        "max_deg": r2(max_deg),
        "rms_deg": r2(res["rms_deg"]),
        "overlap_s": round(float(res["overlap_s"]), 3),
        "worst": common[int(np.argmax(max_deg))],
        "pass": bool(max_deg.max() < tol_deg),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Compare two run logs in logs/ joint by joint")
    ap.add_argument("a", nargs="?")
    ap.add_argument("b", nargs="?")
    ap.add_argument("--tol-deg", type=float, default=5.0)
    ap.add_argument("--list", action="store_true", help="list the run logs")
    args = ap.parse_args(argv)
    if args.list or not (args.a and args.b):
        for r in list_runs():
            print(f"{r['file']:50s} {r['skill']:16s} {r['target']}")
        return 0
    res = gap(args.a, args.b, args.tol_deg)
    for n, mx, rms in zip(res["joints"], res["max_deg"], res["rms_deg"]):
        print(f"{n:30s} max {mx:7.2f} deg  rms {rms:7.2f} deg")
    print(json.dumps({k: res[k] for k in ("overlap_s", "worst", "pass")}))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
