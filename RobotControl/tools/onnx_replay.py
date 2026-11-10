"""Replay recorded training observations through the exported ONNX and compare with the
recorded actions.  Pass if max |err| < tol (default 1e-4).

    python -m robotcontrol.tools.onnx_replay policy.onnx replay.npz [--tol 1e-4]

The ``.npz`` needs ``obs`` (T, N) and ``actions`` (T, A) (keys configurable).
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np


def replay(onnx_path: str | Path, npz_path: str | Path, tol: float = 1e-4, obs_key: str = "obs",
           act_key: str = "actions", use_cuda: bool = False, max_rows: Optional[int] = None) -> Dict[str, object]:
    from ..execution.policy_runner import OnnxPolicy

    data = np.load(npz_path)
    if obs_key not in data or act_key not in data:
        raise KeyError(f"{npz_path} needs keys {obs_key!r} and {act_key!r}; has {list(data.keys())}")
    obs = np.asarray(data[obs_key], dtype=np.float32)
    ref = np.asarray(data[act_key], dtype=np.float32)
    if obs.ndim == 1:
        obs = obs[None]
    if ref.ndim == 1:
        ref = ref[None]
    if max_rows:
        obs, ref = obs[:max_rows], ref[:max_rows]
    pol = OnnxPolicy(onnx_path, use_cuda=use_cuda)
    if obs.shape[1] != pol.num_obs:
        raise ValueError(f"obs dim {obs.shape[1]} != model input {pol.num_obs}")
    if ref.shape[1] != pol.num_actions:
        raise ValueError(f"actions dim {ref.shape[1]} != model output {pol.num_actions}")
    out = np.stack([pol.infer(o).copy() for o in obs])
    err = np.abs(out - ref)
    per_action = err.max(axis=0)
    worst = int(np.argmax(err.max(axis=1)))
    return {
        "rows": int(obs.shape[0]),
        "max_abs_err": float(err.max()),
        "mean_abs_err": float(err.mean()),
        "per_action_max": per_action,
        "worst_row": worst,
        "tol": float(tol),
        "passed": bool(err.max() < tol),
        "onnx_actions": out,
    }


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx", type=Path)
    ap.add_argument("npz", type=Path)
    ap.add_argument("--tol", type=float, default=1e-4)
    ap.add_argument("--obs-key", default="obs")
    ap.add_argument("--actions-key", default="actions")
    ap.add_argument("--cuda", action="store_true")
    ap.add_argument("--max-rows", type=int, default=None)
    args = ap.parse_args(argv)
    r = replay(args.onnx, args.npz, args.tol, args.obs_key, args.actions_key, args.cuda, args.max_rows)
    print(f"rows={r['rows']} max_abs_err={r['max_abs_err']:.3e} mean_abs_err={r['mean_abs_err']:.3e} "
          f"worst_row={r['worst_row']} tol={r['tol']:.1e}")
    print("per-action max err: " + " ".join(f"{x:.2e}" for x in r["per_action_max"]))
    print("PASS" if r["passed"] else "FAIL")
    return 0 if r["passed"] else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
