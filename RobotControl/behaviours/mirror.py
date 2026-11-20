"""Live mirror: show measured robot joint angles on the Unreal twin (URLab puppet mode, no physics).

Sources:
  robot   ssh to the Jetson and run the READ-ONLY ``stream_state`` (subscribes to rt/lowstate only)
  replay  a behaviour run log (CSV written by behaviours.runner), for testing without the robot

    python -m robotcontrol.behaviours.mirror --source robot
    python -m robotcontrol.behaviours.mirror --source replay --csv logs/neck_turn_real_....csv
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import threading
import time
from typing import Callable, Iterator, List, Optional

import numpy as np

from ..configs.joint_map import JointMap

ROBOT = "unitree@192.168.123.164"
ROBOT_DIR = "~/porobot"


class UnrealMirror:
    """Pushes joint angles into the running Unreal twin with URLab puppet steps (n_steps=0)."""

    def __init__(self, jm: JointMap, address: str = "tcp://localhost"):
        import mujoco

        from ..interfaces.urlab_interface import _import_urlab

        urlab = _import_urlab()
        self.mj = mujoco
        self.c = urlab.URLabClient(address, step_mode="puppet")
        self.c.connect()
        m = self.c.model
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "" for j in range(m.njnt)]
        self.adr = []
        for n in jm.names:
            j = next((k for k, s in enumerate(names) if s == n or s.endswith("_" + n)), None)
            if j is None:
                raise RuntimeError(f"Unreal twin has no joint {n!r}")
            self.adr.append(int(m.jnt_qposadr[j]))
        self.adr = np.array(self.adr)

    def push(self, q) -> None:
        d = self.c.data
        d.qpos[self.adr] = np.asarray(q, dtype=np.float64)
        d.qvel[:] = 0.0
        self.mj.mj_forward(self.c.model, d)
        self.c.step(n_steps=0)

    def close(self) -> None:
        try:
            self.c.close()
        except Exception:
            pass


def robot_stream(hz: float = 30.0, proc_out: Optional[list] = None) -> Iterator[List[float]]:
    """Yields joint-angle lists from the robot's read-only state stream over ssh."""
    p = subprocess.Popen(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            ROBOT,
            f"cd {ROBOT_DIR} && exec ./run.sh robotcontrol.behaviours.stream_state --hz {hz}",
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    if proc_out is not None:
        proc_out.append(p)
    try:
        for line in p.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if "q" in msg:
                yield msg["q"]
    finally:
        p.terminate()


def replay_stream(csv_path: str, jm: JointMap, hz: float = 30.0) -> Iterator[List[float]]:
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    hdr, rows = rows[0], rows[1:]
    cols = [hdr.index(n) for n in jm.names]
    t_col = hdr.index("t")
    t0, last = time.monotonic(), -1.0
    for r in rows:
        t = float(r[t_col])
        if t - last < 1.0 / hz:
            continue
        last = t
        wait = t0 + t - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        yield [float(r[c]) for c in cols]


def run_mirror(
    source: Iterator[List[float]],
    mirror: UnrealMirror,
    stop: threading.Event,
    on_sample: Optional[Callable[[List[float]], None]] = None,
) -> int:
    n = 0
    for q in source:
        if stop.is_set():
            break
        mirror.push(q)
        n += 1
        if on_sample:
            on_sample(q)
    return n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Mirror measured joint angles onto the Unreal twin")
    ap.add_argument("--source", choices=["robot", "replay"], default="robot")
    ap.add_argument("--csv", help="replay: behaviour run log")
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--address", default="tcp://localhost")
    args = ap.parse_args(argv)
    jm = JointMap.load()
    mirror = UnrealMirror(jm, args.address)
    stop = threading.Event()
    src = robot_stream(args.hz) if args.source == "robot" else replay_stream(args.csv, jm, args.hz)
    try:
        n = run_mirror(src, mirror, stop)
        print(f"mirrored {n} samples")
    except KeyboardInterrupt:
        pass
    finally:
        mirror.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
