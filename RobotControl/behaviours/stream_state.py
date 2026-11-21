"""READ-ONLY joint-state and telemetry stream for the live Unreal mirror. Runs on the robot's Jetson.

Subscribes to rt/lowstate only - no publisher is ever created, so it cannot move the robot.
Prints one JSON line per sample to stdout: {"t": seconds, "q": [26 joint angles in joint-map order], ...}.

    ./run.sh robotcontrol.behaviours.stream_state --hz 30 [--telemetry]
"""
from __future__ import annotations

import argparse
import json
import sys
import time

from ..configs.joint_map import JointMap
from .backends import extract_lowstate_telemetry


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until stdin closes / Ctrl-C)")
    ap.add_argument("--iface", default=None)
    ap.add_argument("--telemetry", action="store_true", help="include IMU, motor temps, bus voltage, and mode")
    args = ap.parse_args(argv)

    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from ..interfaces.unitree_interface import detect_robot_iface, ensure_channel_factory

    jm = JointMap.load()
    slots = jm.motor_slots.tolist()
    ensure_channel_factory(0, args.iface or detect_robot_iface())
    latest = {"msg": None}
    sub = ChannelSubscriber(jm.lowstate_topic, LowState_)
    sub.Init(lambda m: latest.__setitem__("msg", m), 10)
    print(json.dumps({"names": jm.names}), flush=True)
    t0 = time.monotonic()
    dt = 1.0 / args.hz
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            m = latest["msg"]
            if m is not None:
                q = [round(float(m.motor_state[s].q), 5) for s in slots]
                pkt = {"t": round(time.monotonic() - t0, 3), "q": q}
                if args.telemetry:
                    pkt.update(extract_lowstate_telemetry(m, jm))
                print(json.dumps(pkt), flush=True)
            time.sleep(dt)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        sub.Close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
