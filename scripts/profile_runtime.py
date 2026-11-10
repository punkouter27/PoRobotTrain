"""Step 5 runtime profile of the Python control loop (sim and/or urlab targets) + allocation audit.

Measures, per 50 Hz policy tick: state read, observation build, ONNX inference, safety + command send,
physics advance (local MuJoCo or URLab RPC). Reports p50 / p99 / max in ms against the 20 ms budget, and
counts heap allocations inside OnnxPolicy.infer with tracemalloc (target: zero).

Usage:
  .venv/Scripts/python.exe scripts/profile_runtime.py --target sim
  .venv/Scripts/python.exe scripts/profile_runtime.py --target urlab   # PIE must be running
"""
from __future__ import annotations

import argparse
import json
import pathlib
import time
import tracemalloc

import numpy as np

from robotcontrol.configs.deploy_config import DeployConfig
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.execution.policy_runner import OnnxPolicy, PolicyRunner, build_interface

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODEL = ROOT / "RobotControl/models/r1_level0_upperbody"
BUDGET_MS = 20.0


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def alloc_audit(policy: OnnxPolicy, num_obs: int, n: int = 2000) -> dict:
    obs = np.zeros(num_obs, dtype=np.float32)
    for _ in range(50):
        policy.infer(obs)  # warm caches
    tracemalloc.start()
    snap0 = tracemalloc.take_snapshot()
    for _ in range(n):
        policy.infer(obs)
    snap1 = tracemalloc.take_snapshot()
    tracemalloc.stop()
    stats = snap1.compare_to(snap0, "lineno")
    grown = sum(s.size_diff for s in stats if s.size_diff > 0)
    blocks = sum(s.count_diff for s in stats if s.count_diff > 0)
    return {"calls": n, "python_bytes_retained": int(grown), "python_blocks_retained": int(blocks),
            "per_call_bytes": grown / n}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", choices=["sim", "urlab"], default="sim")
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "logs/profile")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    jm = JointMap.load()
    deploy = DeployConfig.load(MODEL / "deploy.yaml")
    policy = OnnxPolicy(MODEL / "policy.onnx")
    itf = build_interface(args.target, jm, scene="stand")
    itf.connect()
    runner = PolicyRunner(itf, policy, deploy, jm, log_dir=None, realtime=False, status_hz=0)
    policy.warmup()

    t = {k: [] for k in ("state", "obs", "infer", "send", "advance", "total")}
    last_action = runner.last_action
    for k in range(args.steps):
        tick_t = k * deploy.step_dt
        t0 = time.perf_counter()
        state = itf.get_low_state()
        t1 = time.perf_counter()
        cmd = np.array([np.deg2rad(30) * np.sin(2 * np.pi * 0.25 * tick_t), 0.35])
        obs = runner.obs_builder.build(state, last_action, cmd, tick_t)
        t2 = time.perf_counter()
        action = policy.infer(obs)
        t3 = time.perf_counter()
        target = runner.hold_full.copy()
        target[runner.passive_idx] = state.q[runner.passive_idx]
        target[runner.act_idx] = np.clip(runner.default_act + runner.scale_act * action,
                                         runner.soft_lo[runner.act_idx], runner.soft_hi[runner.act_idx])
        safe_q, rep = runner.safety.apply(target, state, runner.kp_full, runner.kd_full, t=state.timestamp)
        itf.send_low_cmd(safe_q, rep.kp, rep.kd)
        t4 = time.perf_counter()
        itf.tick()
        t5 = time.perf_counter()
        last_action[:] = action
        for key, dt in zip(("state", "obs", "infer", "send", "advance", "total"),
                           (t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4, t5 - t0)):
            t[key].append(dt * 1000.0)
    itf.close()

    rows = {k: {"p50_ms": pct(v, 50), "p99_ms": pct(v, 99), "max_ms": float(np.max(v))} for k, v in t.items()}
    audit = alloc_audit(policy, deploy.num_obs)
    total_p99 = rows["total"]["p99_ms"]
    result = {"target": args.target, "steps": args.steps, "budget_ms": BUDGET_MS, "stages": rows,
              "within_budget_p99": total_p99 < BUDGET_MS, "headroom_p99_ms": BUDGET_MS - total_p99,
              "infer_alloc_audit": audit}
    (args.out / f"profile_{args.target}.json").write_text(json.dumps(result, indent=2))

    print(f"target={args.target} steps={args.steps} budget={BUDGET_MS} ms per 50 Hz tick")
    print(f"{'stage':10s} {'p50 ms':>8s} {'p99 ms':>8s} {'max ms':>8s}")
    for k, r in rows.items():
        print(f"{k:10s} {r['p50_ms']:8.3f} {r['p99_ms']:8.3f} {r['max_ms']:8.3f}")
    print(f"p99 total {total_p99:.2f} ms -> {'WITHIN' if total_p99 < BUDGET_MS else 'OVER'} budget "
          f"(headroom {BUDGET_MS - total_p99:.2f} ms)")
    print(f"OnnxPolicy.infer allocation audit: {audit['python_blocks_retained']} retained blocks / "
          f"{audit['python_bytes_retained']} bytes over {audit['calls']} calls")


if __name__ == "__main__":
    main()
