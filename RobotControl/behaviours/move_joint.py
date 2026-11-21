"""Single-joint smooth pose writer (read-write joint slider target) for sim, urlab, or stand-mounted real R1.

Powers ONLY the requested joint (plus zero-torque mode 0 on all other slots on real hardware).
On physical hardware, leg joints remain stand-locked read-only; upper-body and head joints move
with a smoothstep speed ramp, torque-band cap, tracking-error timeout, and other-joint drift abort.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .. import LOGS_DIR
from ..configs.joint_map import JointMap
from .neck_turn import _smoothstep
from .runner import CsvLog

HEAD_JOINTS = frozenset({"head_pitch_joint", "head_yaw_joint"})
LEG_JOINTS_COUNT = 12


def is_upper_body_joint(jm: JointMap, joint: str) -> bool:
    idx = jm.index_of(joint)
    return idx >= LEG_JOINTS_COUNT


def validate_move_request(jm: JointMap, joint: str, target_deg: float, target: str) -> tuple[int, float]:
    if joint not in jm.names:
        raise ValueError(f"unknown joint {joint!r}")
    idx = jm.index_of(joint)
    if target == "real" and not is_upper_body_joint(jm, joint):
        raise ValueError(f"{joint} is a leg joint (read-only while stand-mounted)")
    lo_deg = math.degrees(float(jm.limits_lo[idx]))
    hi_deg = math.degrees(float(jm.limits_hi[idx]))
    if not lo_deg - 1.0 <= float(target_deg) <= hi_deg + 1.0:
        raise ValueError(f"{joint} target {target_deg:.1f} deg outside [{lo_deg:.1f}, {hi_deg:.1f}]")
    clamped_rad = min(
        max(math.radians(float(target_deg)), float(jm.limits_lo[idx]) + 0.02), float(jm.limits_hi[idx]) - 0.02
    )
    return idx, clamped_rad


def _read_telemetry(backend, jm: JointMap) -> dict:
    tel_fn = getattr(backend, "telemetry", None)
    if callable(tel_fn):
        try:
            data = tel_fn() or {}
            if data:
                return data
        except Exception:
            pass
    msg = getattr(backend, "_msg", None)
    if msg is None:
        return {}
    try:
        slots = jm.motor_slots.tolist()
        temps = [int(max(msg.motor_state[s].temperature)) for s in slots]
        hot_idx = int(np.argmax(temps))
        vols = [float(msg.motor_state[s].vol) for s in slots if float(msg.motor_state[s].vol) > 0.5]
        modes = [int(msg.motor_state[s].mode) for s in slots]
        rpy = [round(math.degrees(float(x)), 1) for x in msg.imu_state.rpy]
        return {
            "imu_rpy_deg": rpy,
            "temps_c": temps,
            "max_temp_c": temps[hot_idx],
            "hot_joint": jm.names[hot_idx].replace("_joint", ""),
            "bus_v": round(float(np.median(vols)), 1) if vols else None,
            "mode_machine": int(msg.mode_machine),
            "active_motors": sum(1 for m in modes if m != 0),
        }
    except Exception:
        return {}


class MoveJoint:
    """Smooth step trajectory + safety envelope for moving one joint to ``target_rad``."""

    def __init__(
        self,
        jm: JointMap,
        joint: str,
        target_rad: float,
        speed_deg_s: float = 25.0,
        hold_s: float = 0.4,
        check_tracking: bool = True,
    ):
        self.jm = jm
        self.name = f"move_{joint.replace('_joint', '')}"
        self.joint = joint
        self.powered = (joint,)
        self.idx = jm.index_of(joint)
        self.kp = float(jm.kp[self.idx])
        self.effort = float(jm.effort[self.idx])
        self.q_lo = float(jm.limits_lo[self.idx]) + 0.02
        self.q_hi = float(jm.limits_hi[self.idx]) - 0.02
        self.target_rad = min(max(float(target_rad), self.q_lo), self.q_hi)
        self.speed_rad_s = math.radians(min(max(float(speed_deg_s), 5.0), 45.0))
        self.hold_s = max(float(hold_s), 0.3)
        self.check_tracking = check_tracking
        self.q0 = 0.0
        self.move_s = 0.6
        self._err_since: Optional[float] = None
        self.abort_reason: Optional[str] = None

    @property
    def duration_s(self) -> float:
        return self.move_s + self.hold_s

    def describe(self) -> str:
        return f"move {self.joint} from {math.degrees(self.q0):+.1f} to {math.degrees(self.target_rad):+.1f} deg ({self.move_s:.1f}s)"

    def phase(self, t: float) -> str:
        return "move" if t < self.move_s else ("hold" if t < self.duration_s else "done")

    def start(self, q: dict[str, float]) -> None:
        v = float(q[self.joint])
        if not math.isfinite(v):
            raise RuntimeError(f"{self.joint} angle is not finite")
        self.q0 = v
        self.move_s = max(abs(self.target_rad - self.q0) / self.speed_rad_s, 0.6)

    def targets(self, t: float, q: dict[str, float], dq: dict[str, float], age: float) -> Optional[dict[str, float]]:
        jv, jdq = float(q[self.joint]), float(dq[self.joint])
        if not (math.isfinite(jv) and math.isfinite(jdq)):
            self.abort_reason = f"non-finite {self.joint} state"
            return None
        if age > 0.15:
            self.abort_reason = f"stale state ({age * 1000:.0f} ms)"
            return None
        u = _smoothstep(t / self.move_s)
        nom = (1.0 - u) * self.q0 + u * self.target_rad
        if self.check_tracking and abs(nom - jv) > math.radians(15.0):
            self._err_since = t if self._err_since is None else self._err_since
            if t - self._err_since > 0.5:
                self.abort_reason = f"{self.joint} tracking error {math.degrees(nom - jv):+.1f} deg"
                return None
        else:
            self._err_since = None
        band = self.effort / max(self.kp, 1e-6)
        capped = min(max(nom, jv - band), jv + band)
        return {self.joint: min(max(capped, self.q_lo), self.q_hi)}


def execute_move(backend, beh: MoveJoint, jm: JointMap, log: CsvLog, stop: dict, max_other_deg: float = 5.0) -> int:
    names = jm.names
    dt = 1.0 / backend.rate_hz
    q, dq, _ = backend.read()
    beh.start(dict(zip(names, q)))
    q_start = q.copy()
    skip = set(getattr(backend, "unchecked", ()))
    others = [i for i, n in enumerate(names) if n not in beh.powered and n not in skip]
    primary_label = beh.joint.replace("_joint", "").upper()
    print(f"start: {beh.describe()}; powered: {beh.joint}")
    t0 = time.monotonic()
    t, k, max_other = 0.0, 0, 0.0
    emit_every = max(1, int(round(backend.rate_hz / 5.0)))
    try:
        while t < beh.duration_s and not stop["flag"]:
            q, dq, age = backend.read()
            targets = beh.targets(t, dict(zip(names, q)), dict(zip(names, dq)), age)
            if targets is None:
                print(f"ABORT at t={t:.2f}s: {beh.abort_reason}")
                return 2
            moved = np.abs(q[others] - q_start[others])
            max_other = max(max_other, float(moved.max())) if len(others) else 0.0
            if beh.check_tracking and max_other > math.radians(max_other_deg):
                j = others[int(np.argmax(moved))]
                print(
                    f"ABORT at t={t:.2f}s: {names[j]} moved {math.degrees(moved.max()):.1f} deg (> {max_other_deg} deg)"
                )
                return 3
            backend.command(targets)
            log.row(t, beh.phase(t), [targets[beh.joint]], q)
            if k % emit_every == 0:
                pkt = {
                    "t": round(t, 2),
                    "phase": beh.phase(t),
                    "behaviour": beh.name,
                    "primary_joint": beh.joint,
                    "primary_label": primary_label,
                    "actual_deg": round(math.degrees(float(q[beh.idx])), 2),
                    "target_deg": round(math.degrees(float(targets[beh.joint])), 2),
                    "other_max_deg": round(math.degrees(max_other), 2),
                    "q_deg": [round(math.degrees(float(v)), 1) for v in q],
                }
                pkt.update(_read_telemetry(backend, jm))
                print("@telemetry " + json.dumps(pkt, separators=(",", ":")), flush=True)
            k += 1
            t = k * dt
            backend.advance(t)
            if backend.realtime:
                sleep = t0 + t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
        q, _, _ = backend.read()
        pkt = {
            "t": round(t, 2),
            "phase": "done",
            "behaviour": beh.name,
            "primary_joint": beh.joint,
            "primary_label": primary_label,
            "actual_deg": round(math.degrees(float(q[beh.idx])), 2),
            "target_deg": round(math.degrees(beh.target_rad), 2),
            "other_max_deg": round(math.degrees(max_other), 2),
            "q_deg": [round(math.degrees(float(v)), 1) for v in q],
        }
        pkt.update(_read_telemetry(backend, jm))
        print("@telemetry " + json.dumps(pkt, separators=(",", ":")), flush=True)
        print(
            "stopped by Ctrl-C"
            if stop["flag"]
            else f"done. {beh.joint} at {math.degrees(float(q[beh.idx])):+.1f} deg (other max drift {math.degrees(max_other):.1f} deg)"
        )
        return 0
    finally:
        backend.release()
        log.close()
        print(f"log: {log.path}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Move one joint smoothly to target_deg")
    ap.add_argument("joint")
    ap.add_argument("target_deg", type=float)
    ap.add_argument("--target", choices=["sim", "urlab", "real"], default="sim")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--speed", type=float, default=25.0, help="deg/s (default 25)")
    ap.add_argument("--address", default="tcp://localhost")
    ap.add_argument("--start-q-deg", default=None)
    ap.add_argument("--max-other", type=float, default=5.0)
    ap.add_argument("--log-dir", default=str(LOGS_DIR))
    args = ap.parse_args(argv)

    jm = JointMap.load()
    _, target_rad = validate_move_request(jm, args.joint, args.target_deg, args.target)
    start_q_map = None
    if args.start_q_deg:
        vals = [float(x) for x in args.start_q_deg.split(",") if x.strip()]
        if len(vals) != jm.num_joints:
            ap.error(f"--start-q-deg requires {jm.num_joints} values")
        start_q_map = {n: math.radians(v) for n, v in zip(jm.names, vals)}
    read_only = args.target == "real" and not args.go
    beh = MoveJoint(jm, args.joint, target_rad, speed_deg_s=args.speed, check_tracking=not read_only)

    if args.target == "sim":
        from .backends import SimBackend

        backend = SimBackend(jm, beh.powered, start_q=start_q_map, settle_s=1.0)
    elif args.target == "urlab":
        from .backends import URLabBackend

        backend = URLabBackend(jm, beh.powered, address=args.address, start_q=start_q_map)
    else:
        if args.go and not args.yes:
            if input("type 'yes' to move physical joint: ").strip().lower() != "yes":
                return 1
        from .backends import RealBackend

        backend = RealBackend(jm, beh.powered, None, publish=args.go)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    log = CsvLog(Path(args.log_dir) / f"move_{args.joint}_{args.target}_{stamp}.csv", beh.powered, jm.names)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, lambda *_: stop.__setitem__("flag", True))
    try:
        return execute_move(backend, beh, jm, log, stop, max_other_deg=args.max_other)
    except RuntimeError as exc:
        print(f"NOT STARTED: {exc}")
        return 1
    finally:
        backend.close()


if __name__ == "__main__":
    sys.exit(main())
