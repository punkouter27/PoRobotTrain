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
from .neck_turn import SweepParams
from .stand_behaviours import BEHAVIOUR_SPECS

BEHAVIOURS = {k: spec.cls for k, spec in BEHAVIOUR_SPECS.items()}


class CsvLog:
    def __init__(self, path: Path, powered, joint_names, meta: Optional[dict] = None):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "phase"] + [f"target_{n}" for n in powered] + list(joint_names))
        self._last_flush = time.monotonic()
        # The sidecar is what makes the log replayable: without the behaviour's own parameters a re-run in the
        # twin cannot reproduce the trajectory, and a sim-vs-real diff compares two different motions.
        if meta is not None:
            payload = dict(meta)
            payload.setdefault("created_at", time.strftime("%Y-%m-%dT%H:%M:%S"))
            payload.setdefault("powered", list(powered))
            payload.setdefault("joint_names", list(joint_names))
            path.with_suffix(".json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

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
    spec = BEHAVIOUR_SPECS.get(beh.name)
    primary = spec.primary_joint if spec else beh.powered[0]
    i_primary = names.index(primary)
    primary_label = spec.primary_label if spec else primary.replace("_joint", "").upper()
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
            if k % 10 == 0:  # 5 Hz telemetry + head readout for the Unreal panel (behaviours/service.py)
                tg_yaw = targets.get("head_yaw_joint", float("nan"))
                print(
                    f"@head t={t:.2f} phase={beh.phase(t)} yaw={math.degrees(q[i_yaw]):.2f} "
                    f"pitch={math.degrees(q[i_pitch]):.2f} target={math.degrees(tg_yaw):.2f} "
                    f"other={math.degrees(max_other):.2f}",
                    flush=True,
                )
                tg_prim = targets.get(primary, float("nan"))
                pkt = {
                    "t": round(t, 2),
                    "phase": beh.phase(t),
                    "behaviour": beh.name,
                    "primary_joint": primary,
                    "primary_label": primary_label,
                    "actual_deg": round(math.degrees(float(q[i_primary])), 2),
                    "target_deg": None if math.isnan(tg_prim) else round(math.degrees(float(tg_prim)), 2),
                    "other_max_deg": round(math.degrees(max_other), 2),
                    "q_deg": [round(math.degrees(float(v)), 1) for v in q],
                }
                pkt.update(backend.telemetry())
                print("@telemetry " + json.dumps(pkt, separators=(",", ":")), flush=True)
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
    ap.add_argument("--sim-friction", type=float, default=None, help="sim: override Nm friction floor on limp joints")
    ap.add_argument("--sim-start-deg", type=float, default=0.0, help="sim: initial head yaw")
    ap.add_argument(
        "--no-realism", action="store_true", help="sim: disable calibrated delay/BEMF/noise/compliant stand"
    )
    ap.add_argument("--start-q-deg", default=None, help="comma-separated 26 joint angles (deg) to match physical robot")
    ap.add_argument("--log-dir", default=str(LOGS_DIR))
    ap.add_argument("--fast", action="store_true", help="sim: run as fast as possible instead of real time")
    ap.add_argument(
        "--skill-version",
        type=int,
        default=None,
        help="trained skills, virtual robot only: play a stored training session instead of the active version",
    )
    args = ap.parse_args(argv)
    if not 0 < args.freq <= 0.5:
        ap.error("--freq must be in (0, 0.5] Hz")

    jm = JointMap.load()
    start_q_map = None
    if args.start_q_deg:
        vals = [float(x) for x in args.start_q_deg.split(",") if x.strip()]
        if len(vals) != jm.num_joints:
            ap.error(f"--start-q-deg requires {jm.num_joints} values, got {len(vals)}")
        start_q_map = {n: math.radians(v) for n, v in zip(jm.names, vals)}
    read_only = args.target == "real" and not args.go
    if args.skill_version is not None and args.target == "real":
        ap.error("--skill-version is for the virtual robot only; the real robot runs the active version")
    extra = {"version": args.skill_version} if args.skill_version is not None else {}
    beh = BEHAVIOURS[args.behaviour](
        jm,
        SweepParams(amplitude_deg=args.amplitude, freq_hz=args.freq, duration_s=args.duration),
        check_tracking=not read_only,
        **extra,
    )
    if args.target == "sim":
        from .backends import SimBackend

        sim_start = dict(start_q_map or {})
        if not args.start_q_deg and abs(args.sim_start_deg) > 1e-9:
            sim_start["head_yaw_joint"] = math.radians(args.sim_start_deg)
        backend = SimBackend(
            jm, beh.powered, friction_nm=args.sim_friction, start_q=sim_start, realistic=not args.no_realism
        )
        # Someone is watching (web app twin preview, practice check/run): play at the real robot's speed, so a 20 s
        # move takes 20 s on the virtual robot too. Automated checks call run() directly and still fast-forward.
        backend.realtime = not args.fast
    elif args.target == "urlab":
        from .backends import URLabBackend

        backend = URLabBackend(jm, beh.powered, address=args.address, start_q=start_q_map)
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
        meta={
            "behaviour": beh.name,
            "target": args.target,
            "read_only": read_only,
            "amplitude_deg": args.amplitude,
            "freq_hz": args.freq,
            "duration_s": args.duration,
            "start_q_deg": [round(math.degrees(v), 4) for v in (start_q_map or {}).values()] or None,
            "sim_start_deg": args.sim_start_deg if args.target == "sim" else None,
            "rate_hz": getattr(backend, "rate_hz", None),
        },
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
