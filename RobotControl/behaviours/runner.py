"""Run a behaviour on the MuJoCo twin, the Unreal twin, or the real robot - same behaviour code everywhere.

    python -m robotcontrol.behaviours.runner neck_turn --target sim
    python -m robotcontrol.behaviours.runner neck_turn --target urlab          # Unreal editor in Play
    ./run.sh robotcontrol.behaviours.runner neck_turn --target real            # robot, read-only
    ./run.sh robotcontrol.behaviours.runner neck_turn --target real --go       # robot, powered

Only the behaviour's ``powered`` joints receive torque (see backends.py for what the others do per target).
Common aborts on every target: the behaviour's own checks, plus ANY non-powered joint moving more than
--max-other deg (default 5) from where it started. A CSV log (all joint angles) is flushed every 0.2 s so
it survives a power cut.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np

from .. import LOGS_DIR
from ..configs.joint_map import JointMap
from .neck_turn import NeckTurn, SweepParams

BEHAVIOURS = {"neck_turn": NeckTurn}


class CsvLog:
    def __init__(self, path: Path, powered, joint_names):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "phase"] + [f"target_{n}" for n in powered] + list(joint_names))
        self._last_flush = time.monotonic()

    def row(self, t, phase, targets, q_all):
        self.w.writerow([f"{t:.4f}", phase] + [f"{v:.5f}" for v in targets] + [f"{v:.5f}" for v in q_all])
        if time.monotonic() - self._last_flush > 0.2:
            self.f.flush()
            self._last_flush = time.monotonic()

    def close(self):
        self.f.flush()
        self.f.close()


def run(backend, beh, jm: JointMap, log: CsvLog, stop: dict, max_other_deg: float = 5.0) -> int:
    names = jm.names
    dt = 1.0 / backend.rate_hz
    q, dq, _ = backend.read()
    beh.start(dict(zip(names, q)))
    q_start = q.copy()
    skip = set(getattr(backend, "unchecked", ()))
    others = [i for i, n in enumerate(names) if n not in beh.powered and n not in skip]
    i_yaw, i_pitch = names.index("head_yaw_joint"), names.index("head_pitch_joint")
    print(f"start: {beh.describe()}; powered joints: {', '.join(beh.powered)}")
    t0 = time.monotonic()
    t, k, next_print, max_other = 0.0, 0, 0.0, 0.0
    try:
        while t < beh.duration_s and not stop["flag"]:
            q, dq, age = backend.read()
            targets = beh.targets(t, dict(zip(names, q)), dict(zip(names, dq)), age)
            if targets is None:
                print(f"ABORT at t={t:.2f}s: {beh.abort_reason}")
                return 2
            moved = np.abs(q[others] - q_start[others])
            max_other = max(max_other, float(moved.max()))
            if beh.check_tracking and max_other > math.radians(max_other_deg):
                j = others[int(np.argmax(moved))]
                print(
                    f"ABORT at t={t:.2f}s: {names[j]} moved {math.degrees(moved.max()):.1f} deg "
                    f"(> {max_other_deg} deg) - only {', '.join(beh.powered)} may move"
                )
                return 3
            backend.command(targets)
            log.row(t, beh.phase(t), [targets[n] for n in beh.powered], q)
            if k % 10 == 0:  # 5 Hz head readout for the Unreal panel (behaviours/service.py parses '@head')
                tg = targets.get("head_yaw_joint", float("nan"))
                print(
                    f"@head t={t:.2f} phase={beh.phase(t)} yaw={math.degrees(q[i_yaw]):.2f} "
                    f"pitch={math.degrees(q[i_pitch]):.2f} target={math.degrees(tg):.2f} "
                    f"other={math.degrees(max_other):.2f}",
                    flush=True,
                )
            if t >= next_print:
                tg = " ".join(
                    f"{n.replace('_joint', '')} target {math.degrees(targets[n]):+6.1f} actual "
                    f"{math.degrees(q[jm.index_of(n)]):+6.1f} deg"
                    for n in beh.powered
                )
                print(f"t={t:5.1f}s {beh.phase(t):8s} {tg}   other joints max change {math.degrees(max_other):.1f} deg")
                next_print += 1.0
            k += 1
            t = k * dt
            backend.advance(t)
            if backend.realtime:
                sleep = t0 + t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
        print(
            "stopped by Ctrl-C"
            if stop["flag"]
            else f"done. other joints moved at most {math.degrees(max_other):.1f} deg"
        )
        return 0
    finally:
        backend.release()
        log.close()
        print(f"log: {log.path}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Run a behaviour on sim | urlab | real; only its joints are powered.")
    ap.add_argument("behaviour", nargs="?", default="neck_turn", choices=sorted(BEHAVIOURS))
    ap.add_argument("--target", choices=["sim", "urlab", "real"], default="sim")
    ap.add_argument("--go", action="store_true", help="real: actually publish (otherwise read-only)")
    ap.add_argument("--yes", action="store_true", help="real --go: skip the confirmation prompt")
    ap.add_argument("--iface", default=None, help="real: network interface (auto-detected)")
    ap.add_argument("--address", default="tcp://localhost", help="urlab: server address")
    ap.add_argument("--amplitude", type=float, default=10.0, help="deg, max 30 (default 10)")
    ap.add_argument("--freq", type=float, default=0.2, help="Hz (default 0.2 = one left-right cycle per 5 s)")
    ap.add_argument("--duration", type=float, default=20.0, help="sweeping seconds (default 20)")
    ap.add_argument(
        "--max-other",
        type=float,
        default=5.0,
        help="deg: abort if any non-powered joint moves more than this (default 5)",
    )
    ap.add_argument("--sim-friction", type=float, default=0.5, help="sim: Nm friction on limp joints")
    ap.add_argument("--sim-start-deg", type=float, default=0.0, help="sim: initial head yaw")
    ap.add_argument("--log-dir", default=str(LOGS_DIR))
    args = ap.parse_args(argv)
    if not 0 < args.freq <= 0.5:
        ap.error("--freq must be in (0, 0.5] Hz")

    jm = JointMap.load()
    read_only = args.target == "real" and not args.go
    beh = BEHAVIOURS[args.behaviour](
        jm,
        SweepParams(amplitude_deg=args.amplitude, freq_hz=args.freq, duration_s=args.duration),
        check_tracking=not read_only,
    )
    if args.target == "sim":
        from .backends import SimBackend

        backend = SimBackend(
            jm, beh.powered, friction_nm=args.sim_friction, start_q={"head_yaw_joint": math.radians(args.sim_start_deg)}
        )
    elif args.target == "urlab":
        from .backends import URLabBackend

        backend = URLabBackend(jm, beh.powered, address=args.address)
    else:
        if args.go and not args.yes:
            print(
                f"REAL ROBOT. Only {', '.join(beh.powered)} will be powered; every other motor stays off.\n"
                "Robot on its stand, head facing forward, remote in hand (L2+B 5 s = damping)."
            )
            if input("type 'yes' to start: ").strip().lower() != "yes":
                print("cancelled")
                return 1
        from .backends import RealBackend

        backend = RealBackend(jm, beh.powered, args.iface, publish=args.go)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    log = CsvLog(
        Path(args.log_dir) / f"{beh.name}_{args.target}{'_readonly' if read_only else ''}_{stamp}.csv",
        beh.powered,
        jm.names,
    )
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    if hasattr(signal, "SIGBREAK"):  # Windows: the behaviour service stops previews with CTRL_BREAK
        signal.signal(signal.SIGBREAK, lambda *_: stop.__setitem__("flag", True))
    try:
        return run(backend, beh, jm, log, stop, max_other_deg=args.max_other)
    except RuntimeError as exc:
        print(f"NOT STARTED: {exc}")
        return 1
    finally:
        backend.close()


if __name__ == "__main__":
    sys.exit(main())
