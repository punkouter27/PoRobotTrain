"""Run an ONNX policy on the MuJoCo twin, the UE5/URLab twin or the real robot.

    python -m robotcontrol.main_runner --target sim --policy models/r1_level0_upperbody/policy.onnx \
        --scene stand --command-mode sweep

Command modes (Level 0: command = [head_yaw_target_rad, shoulder_pitch_target_rad]):

* ``sweep``     head yaw +-30 deg at 0.25 Hz; shoulder pitch between default and default-1.2 rad at 0.15 Hz
* ``keyboard``  arrows: left/right = head yaw +-5 deg, up/down = shoulder pitch -+0.1 rad, r = reset, q = quit
* ``zero``      constant command = [0, default]

Ctrl-C -> damping command -> exit.
"""
from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from . import LOGS_DIR, MODELS_DIR, PACKAGE_DIR, PROJECT_ROOT
from .configs.deploy_config import DeployConfig
from .configs.joint_map import JointMap
from .execution.policy_runner import PolicyRunner, build_interface, load_policy_bundle
from .execution.safety_guard import SafetyConfig, SafetyGuard
from .interfaces.base_interface import LowStateData

log = logging.getLogger("robotcontrol.main")


# ----------------------------------------------------------------------------- commands
class SweepCommand:
    """Slow sinusoids for the two Level 0 commands."""

    def __init__(self, deploy: DeployConfig, joint_map: JointMap, head_yaw_deg: float = 30.0, head_yaw_hz: float = 0.25,
                 shoulder_drop_rad: float = 1.2, shoulder_hz: float = 0.15):
        self.dim = deploy.command_dim
        self.head_yaw_amp = math.radians(head_yaw_deg)
        self.head_yaw_hz = head_yaw_hz
        self.shoulder_drop = shoulder_drop_rad
        self.shoulder_hz = shoulder_hz
        self.shoulder_default = _shoulder_default(deploy, joint_map)
        self.cmd = np.zeros(self.dim)

    def __call__(self, t: float, state: LowStateData) -> np.ndarray:
        if self.dim >= 1:
            self.cmd[0] = self.head_yaw_amp * math.sin(2.0 * math.pi * self.head_yaw_hz * t)
        if self.dim >= 2:
            # oscillate in [default - drop, default]
            self.cmd[1] = self.shoulder_default - 0.5 * self.shoulder_drop * (1.0 - math.cos(2.0 * math.pi * self.shoulder_hz * t))
        return self.cmd


class ZeroCommand:
    def __init__(self, deploy: DeployConfig, joint_map: JointMap):
        self.cmd = np.zeros(deploy.command_dim)
        if deploy.command_dim >= 2:
            self.cmd[1] = _shoulder_default(deploy, joint_map)

    def __call__(self, t: float, state: LowStateData) -> np.ndarray:
        return self.cmd


class KeyboardCommand:
    """Arrow-key teleop. Uses msvcrt on Windows, termios/select elsewhere; a background
    thread reads keys so the control loop never blocks."""

    def __init__(self, deploy: DeployConfig, joint_map: JointMap, stop_event: threading.Event,
                 yaw_step_deg: float = 5.0, pitch_step_rad: float = 0.1):
        self.dim = deploy.command_dim
        self.stop_event = stop_event
        self.yaw_step = math.radians(yaw_step_deg)
        self.pitch_step = pitch_step_rad
        self.shoulder_default = _shoulder_default(deploy, joint_map)
        self.cmd = np.zeros(self.dim)
        self.reset()
        jm = joint_map
        self.yaw_lim = float(jm.soft_limits()[1][jm.index_of("head_yaw_joint")]) if "head_yaw_joint" in jm.names else 1.5
        i_sp = jm.index_of("right_shoulder_pitch_joint") if "right_shoulder_pitch_joint" in jm.names else None
        lo, hi = jm.soft_limits()
        self.pitch_lo = float(lo[i_sp]) if i_sp is not None else -2.5
        self.pitch_hi = float(hi[i_sp]) if i_sp is not None else 1.5
        self._thread = threading.Thread(target=self._reader, name="keyboard", daemon=True)
        self._thread.start()

    def reset(self) -> None:
        self.cmd[:] = 0.0
        if self.dim >= 2:
            self.cmd[1] = self.shoulder_default

    def __call__(self, t: float, state: LowStateData) -> np.ndarray:
        return self.cmd

    def _handle(self, key: str) -> None:
        if key == "LEFT" and self.dim >= 1:
            self.cmd[0] = min(self.yaw_lim, self.cmd[0] + self.yaw_step)
        elif key == "RIGHT" and self.dim >= 1:
            self.cmd[0] = max(-self.yaw_lim, self.cmd[0] - self.yaw_step)
        elif key == "UP" and self.dim >= 2:
            self.cmd[1] = max(self.pitch_lo, self.cmd[1] - self.pitch_step)   # raise the arm (pitch negative = forward/up)
        elif key == "DOWN" and self.dim >= 2:
            self.cmd[1] = min(self.pitch_hi, self.cmd[1] + self.pitch_step)
        elif key in ("r", "R"):
            self.reset()
        elif key in ("q", "Q", "\x03"):
            self.stop_event.set()

    def _reader(self) -> None:
        if sys.platform == "win32":
            import msvcrt
            arrows = {"H": "UP", "P": "DOWN", "K": "LEFT", "M": "RIGHT"}
            while not self.stop_event.is_set():
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    if ch in ("\x00", "\xe0"):
                        ch2 = msvcrt.getwch()
                        self._handle(arrows.get(ch2, ""))
                    else:
                        self._handle(ch)
                else:
                    time.sleep(0.01)
        else:  # pragma: no cover - posix fallback
            import select
            import termios
            import tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while not self.stop_event.is_set():
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        ch = sys.stdin.read(1)
                        if ch == "\x1b":
                            seq = sys.stdin.read(2)
                            self._handle({"[A": "UP", "[B": "DOWN", "[D": "LEFT", "[C": "RIGHT"}.get(seq, ""))
                        else:
                            self._handle(ch)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _shoulder_default(deploy: DeployConfig, joint_map: JointMap) -> float:
    if "right_shoulder_pitch_joint" in deploy.joint_names:
        return float(deploy.default_joint_pos[deploy.joint_names.index("right_shoulder_pitch_joint")])
    if "right_shoulder_pitch_joint" in joint_map.names:
        return float(joint_map.defaults[joint_map.index_of("right_shoulder_pitch_joint")])
    return 0.0


def make_command_fn(mode: str, deploy: DeployConfig, joint_map: JointMap, stop_event: threading.Event,
                    amplitude: float = 1.0):
    if mode == "sweep":
        a = min(max(float(amplitude), 0.0), 1.0)
        return SweepCommand(deploy, joint_map, head_yaw_deg=30.0 * a, shoulder_drop_rad=1.2 * a)
    if mode == "keyboard":
        return KeyboardCommand(deploy, joint_map, stop_event)
    if mode == "zero":
        return ZeroCommand(deploy, joint_map)
    raise ValueError(f"unknown command mode {mode!r}")


# ----------------------------------------------------------------------------- paths
def resolve_policy_path(p: str | Path) -> Path:
    cands = [Path(p), PACKAGE_DIR / p, MODELS_DIR / p, PROJECT_ROOT / p]
    for c in cands:
        if c.is_file():
            return c.resolve()
        if c.is_dir() and (c / "policy.onnx").is_file():
            return (c / "policy.onnx").resolve()
    raise FileNotFoundError(f"policy {p!r} not found (tried {[str(c) for c in cands]})")


# ----------------------------------------------------------------------------- main
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Run an ONNX policy on sim | urlab | real")
    ap.add_argument("--target", choices=["sim", "urlab", "real"], default="sim")
    ap.add_argument("--policy", required=True, help="policy.onnx (deploy.yaml must sit next to it) or its directory")
    ap.add_argument("--deploy", default=None, help="explicit deploy.yaml")
    ap.add_argument("--joint-map", default=None)
    ap.add_argument("--scene", default="stand", help="sim: stand | ground | path/to/scene.xml")
    ap.add_argument("--render", action="store_true", help="sim: open the MuJoCo viewer")
    ap.add_argument("--address", default="tcp://localhost", help="urlab: server address")
    ap.add_argument("--step-mode", default="direct", help="urlab: direct | live | puppet")
    ap.add_argument("--iface", default=None, help="real: network interface (e.g. eth0)")
    ap.add_argument("--dry-run", action="store_true", help="real: log LowCmd instead of publishing")
    ap.add_argument("--observe", action="store_true",
                    help="real: read the live robot state and run the policy in shadow mode; NOTHING is published")
    ap.add_argument("--command-mode", choices=["sweep", "keyboard", "zero"], default="sweep")
    ap.add_argument("--duration", type=float, default=None, help="seconds (default: until Ctrl-C)")
    ap.add_argument("--amplitude", type=float, default=1.0,
                    help="sweep: scale 0..1 of the motion (1.0 = head yaw +-30 deg, shoulder lift 1.2 rad)")
    ap.add_argument("--no-realtime", action="store_true", help="sim: run as fast as possible")
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--log-dir", default=str(LOGS_DIR))
    ap.add_argument("--cuda", action="store_true")
    ap.add_argument("--max-tilt-deg", type=float, default=25.0)
    ap.add_argument("--no-tilt-check", action="store_true")
    ap.add_argument("--status-hz", type=float, default=2.0)
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--yes", action="store_true", help="real: skip the confirmation prompt")
    return ap


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")

    jm = JointMap.load(args.joint_map)
    onnx_path = resolve_policy_path(args.policy)
    policy, deploy = load_policy_bundle(onnx_path, args.deploy, use_cuda=args.cuda)
    log.info("policy %s: %d obs -> %d actions, dt=%.3f, providers=%s", deploy.policy_name, policy.num_obs,
             policy.num_actions, deploy.step_dt, policy.providers)

    if args.target == "real" and args.iface is None and not args.dry_run:
        from .interfaces.unitree_interface import detect_robot_iface
        args.iface = detect_robot_iface()
        log.info("auto-detected robot network interface: %s", args.iface)
        if args.iface is None:
            log.error("no interface with a 192.168.123.x address; pass --iface explicitly")
            return 3

    if args.target == "real" and not args.dry_run and not args.observe and not args.yes:
        print("WARNING: real robot. Ensure it is on the stand with no obstacles around.")
        input("Press Enter to continue...")

    itf = build_interface(args.target, jm, scene=args.scene, address=args.address, iface=args.iface,
                          dry_run=args.dry_run, render=args.render, policy_dt=deploy.step_dt,
                          **({"step_mode": args.step_mode} if args.target == "urlab" else {}),
                          **({"observe_only": True} if args.target == "real" and args.observe else {}))
    stop_event = threading.Event()
    safety = SafetyGuard(jm, SafetyConfig(max_tilt_deg=args.max_tilt_deg, check_tilt=not args.no_tilt_check))

    def _sigint(_s, _f):
        log.info("Ctrl-C -> damping -> exit")
        stop_event.set()

    signal.signal(signal.SIGINT, _sigint)

    with itf:
        cmd_fn = make_command_fn(args.command_mode, deploy, jm, stop_event, amplitude=args.amplitude)
        runner = PolicyRunner(itf, policy, deploy, jm, command_fn=cmd_fn, safety=safety,
                              log_dir=None if args.no_log else args.log_dir,
                              log_stem=f"{deploy.policy_name}_{args.target}_{time.strftime('%Y%m%d_%H%M%S')}",
                              realtime=not args.no_realtime, status_hz=args.status_hz)
        if args.target == "real" and not args.dry_run and not args.observe:
            log.info("moving to the policy default pose over 3 s")
            itf.move_to(runner.hold_full, duration_s=3.0, kp=runner.kp_full, kd=runner.kd_full, stop_event=stop_event)
        status = runner.run(duration_s=args.duration, stop_event=stop_event)
    log.info("done: %d steps, tripped=%s", status["steps"], status["tripped"])
    return 2 if status["tripped"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
