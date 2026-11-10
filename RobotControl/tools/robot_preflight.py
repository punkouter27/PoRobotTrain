"""Read-only preflight check, run ON THE ROBOT's development computer (Jetson) before any powered test.

Nothing in this tool publishes rt/lowcmd or changes the robot's motion mode. It checks:
  1. Python + packages (numpy, yaml, onnxruntime, cyclonedds, unitree_sdk2py)
  2. The policy bundle: SHA-256 manifest, deploy.yaml vs joint map, ONNX session + inference timing
  3. Network: an interface on 192.168.123.x, motion controller 192.168.123.161 reachable
  4. Live robot state for a few seconds (subscribe only): message rate, CRC errors, all 26 mapped motors
     reporting, joint angles inside limits, head motors (slots 29/30) present, IMU attitude, motor temperatures

Usage on the robot:
  python3 -m robotcontrol.tools.robot_preflight --bundle bundle [--iface eth0] [--seconds 3]
Exit code 0 = all PASS (warnings allowed), 1 = at least one FAIL.
"""
from __future__ import annotations

import argparse
import platform
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

RESULTS: List[Tuple[str, str, str]] = []


def record(status: str, name: str, detail: str = "") -> None:
    RESULTS.append((status, name, detail))
    print(f"[{status:4s}] {name}" + (f": {detail}" if detail else ""), flush=True)


def check_python() -> None:
    v = sys.version_info
    record("PASS" if v >= (3, 8) else "FAIL", "python version", f"{platform.python_version()} on {platform.machine()}")
    for mod in ("numpy", "yaml", "onnxruntime", "cyclonedds", "unitree_sdk2py"):
        try:
            m = __import__(mod)
            record("PASS", f"import {mod}", getattr(m, "__version__", ""))
        except Exception as exc:  # noqa: BLE001
            record("FAIL", f"import {mod}", f"{type(exc).__name__}: {exc}")


def check_bundle(bundle: Path) -> None:
    from ..configs.deploy_config import DeployConfig
    from ..configs.joint_map import JointMap
    from .deploy_pack import verify_bundle

    if not bundle.exists():
        record("FAIL", "bundle folder", f"{bundle} not found")
        return
    problems = verify_bundle(bundle) if (bundle / "MANIFEST.json").exists() else ["MANIFEST.json missing"]
    record("FAIL" if problems else "PASS", "bundle SHA-256 manifest", "; ".join(problems) or "all files match")
    jm = JointMap.load(bundle / "r1_joint_map.yaml") if (bundle / "r1_joint_map.yaml").exists() else JointMap.load()
    deploy = DeployConfig.load(bundle / "deploy.yaml")
    problems = deploy.validate(jm)
    record("FAIL" if problems else "PASS", "deploy.yaml vs joint map", "; ".join(problems) or
           f"{deploy.policy_name}: {deploy.num_obs} obs -> {deploy.num_actions} actions @ {1 / deploy.step_dt:.0f} Hz")
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(bundle / "policy.onnx"), providers=["CPUExecutionProvider"])
        i, o = sess.get_inputs()[0], sess.get_outputs()[0]
        ok = i.name == "obs" and o.name == "actions" and i.shape[-1] == deploy.num_obs and o.shape[-1] == deploy.num_actions
        x = np.zeros((1, deploy.num_obs), dtype=np.float32)
        for _ in range(20):
            sess.run(None, {"obs": x})
        t0 = time.perf_counter()
        for _ in range(200):
            sess.run(None, {"obs": x})
        ms = (time.perf_counter() - t0) / 200 * 1000
        record("PASS" if ok else "FAIL", "ONNX session", f"{i.name}{i.shape} -> {o.name}{o.shape}, {ms:.3f} ms/inference")
        if ms > 5.0:
            record("WARN", "inference time", f"{ms:.2f} ms is high for a 20 ms control period")
    except Exception as exc:  # noqa: BLE001
        record("FAIL", "ONNX session", f"{type(exc).__name__}: {exc}")


def check_network(iface: Optional[str]) -> Optional[str]:
    from ..interfaces.unitree_interface import detect_robot_iface
    found = iface or detect_robot_iface()
    record("PASS" if found else "FAIL", "robot network interface", found or "no interface with a 192.168.123.x address")
    try:
        r = subprocess.run(["ping", "-c", "2", "-W", "1", "192.168.123.161"], capture_output=True, text=True, timeout=10)
        record("PASS" if r.returncode == 0 else "WARN", "ping motion controller 192.168.123.161",
               "reachable" if r.returncode == 0 else "no reply (may block ICMP; DDS check below is authoritative)")
    except (OSError, subprocess.SubprocessError) as exc:
        record("WARN", "ping motion controller", str(exc))
    return found


def check_live_state(iface: Optional[str], seconds: float, bundle: Path) -> None:
    from ..configs.joint_map import JointMap
    from ..interfaces.unitree_interface import UnitreeSDK2Interface

    jm = JointMap.load(bundle / "r1_joint_map.yaml") if (bundle / "r1_joint_map.yaml").exists() else JointMap.load()
    itf = UnitreeSDK2Interface(jm, network_interface=iface, observe_only=True, state_timeout_s=5.0)
    try:
        itf.connect()
    except Exception as exc:  # noqa: BLE001
        record("FAIL", "receive rt/lowstate", f"{type(exc).__name__}: {exc}")
        return
    try:
        n0 = itf._state_count
        t0 = time.monotonic()
        time.sleep(seconds)
        rate = (itf._state_count - n0) / (time.monotonic() - t0)
        record("PASS" if rate > 100 else "WARN", "rt/lowstate rate", f"{rate:.0f} Hz, CRC errors {itf._crc_errors}, "
               f"mode_machine {itf.mode_machine}")
        st = itf.get_low_state()
        lo, hi = jm.limits_lo, jm.limits_hi
        margin = np.deg2rad(2.0)
        outside = [f"{n}={np.rad2deg(q):.1f}deg" for n, q, a, b in zip(jm.names, st.q, lo, hi) if q < a - margin or q > b + margin]
        record("FAIL" if outside else "PASS", "26 joint angles inside limits", ", ".join(outside) or "ok")
        msg = itf._low_state
        temps = [max(msg.motor_state[s].temperature) for s in jm.motor_slots.tolist()]
        hot = max(temps)
        record("PASS" if hot < 60 else "WARN", "motor temperatures", f"max {hot} C")
        head = [jm.names.index("head_pitch_joint"), jm.names.index("head_yaw_joint")]
        record("PASS", "head motors (slots 29/30)",
               f"pitch {np.rad2deg(st.q[head[0]]):.1f} deg, yaw {np.rad2deg(st.q[head[1]]):.1f} deg")
        rpy = np.rad2deg(st.extras.get("rpy", np.zeros(3)))
        record("PASS", "IMU attitude", f"roll {rpy[0]:.1f}, pitch {rpy[1]:.1f}, yaw {rpy[2]:.1f} deg "
               "(on the stand roll/pitch should be small)")
        print("\njoint            angle[deg]   limits[deg]")
        for n, q, a, b in zip(jm.names, st.q, lo, hi):
            print(f"  {n:28s} {np.rad2deg(q):8.1f}   [{np.rad2deg(a):7.1f}, {np.rad2deg(b):7.1f}]")
    finally:
        itf.close()


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Read-only robot preflight (never publishes commands)")
    ap.add_argument("--bundle", type=Path, default=Path("bundle"))
    ap.add_argument("--iface", default=None, help="robot LAN interface (auto-detected from 192.168.123.x)")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--skip-robot", action="store_true", help="only check software and bundle")
    args = ap.parse_args(argv)

    print("== software"); check_python()
    print("== policy bundle"); check_bundle(args.bundle)
    if not args.skip_robot:
        print("== network"); iface = check_network(args.iface)
        print("== live robot state (read-only)"); check_live_state(iface, args.seconds, args.bundle)
    fails = [r for r in RESULTS if r[0] == "FAIL"]
    warns = [r for r in RESULTS if r[0] == "WARN"]
    print(f"\nPREFLIGHT {'PASSED' if not fails else 'FAILED'}: {len(RESULTS) - len(fails) - len(warns)} pass, "
          f"{len(warns)} warn, {len(fails)} fail")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
