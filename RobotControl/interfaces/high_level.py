"""Thin wrapper around Unitree's high-level ``LocoClient`` for the R1
(``example/r1/high_level/r1_loco_client_example.py``).

    from robotcontrol.interfaces.high_level import R1LocoClient
    loco = R1LocoClient("eth0"); loco.connect(); loco.Damp()

CLI: ``python -m robotcontrol.interfaces.high_level --iface eth0 damp``
"""
from __future__ import annotations

import argparse
import logging
import time
from typing import Optional

from .unitree_interface import SDK_INSTALL_HINT, ensure_channel_factory

log = logging.getLogger(__name__)

COMMANDS = ("damp", "stance", "start", "zero_torque", "stop_move", "wave_hand", "wave_hand_turn",
            "shake_hand", "lie2standup", "standup2lie")


class R1LocoClient:
    """Passthroughs: Damp, Stance, Start, ZeroTorque, StopMove, Move, WaveHand, ShakeHand,
    Lie2StandUp, StandUp2Lie, SetFsmId, SetVelocity."""

    def __init__(self, network_interface: Optional[str] = None, timeout_s: float = 10.0,
                 domain_id: int = 0, dry_run: bool = False):
        self.network_interface = network_interface
        self.timeout_s = float(timeout_s)
        self.domain_id = int(domain_id)
        self.dry_run = bool(dry_run)
        self._client = None

    def connect(self) -> "R1LocoClient":
        if self.dry_run:
            log.info("R1LocoClient DRY RUN")
            return self
        try:
            from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient
        except ImportError as exc:
            raise ImportError(SDK_INSTALL_HINT) from exc
        ensure_channel_factory(self.domain_id, self.network_interface)
        self._client = LocoClient()
        self._client.SetTimeout(self.timeout_s)
        self._client.Init()
        return self

    def _call(self, name: str, *args):
        if self.dry_run:
            log.info("[dry-run] LocoClient.%s%s", name, args)
            return 0
        if self._client is None:
            raise RuntimeError("call connect() first")
        return getattr(self._client, name)(*args)

    # -- passthroughs ------------------------------------------------------
    def Damp(self):
        return self._call("Damp")

    def Stance(self):
        return self._call("Stance")

    def Start(self):
        return self._call("Start")

    def ZeroTorque(self):
        return self._call("ZeroTorque")

    def StopMove(self):
        return self._call("StopMove")

    def Move(self, vx: float, vy: float, vyaw: float, continuous: bool = False):
        return self._call("Move", float(vx), float(vy), float(vyaw), bool(continuous))

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        return self._call("SetVelocity", float(vx), float(vy), float(omega), float(duration))

    def WaveHand(self, turn: bool = False):
        return self._call("WaveHand", bool(turn))

    def ShakeHand(self, stage: int = -1):
        return self._call("ShakeHand", int(stage))

    def Lie2StandUp(self):
        return self._call("Lie2StandUp")

    def StandUp2Lie(self):
        return self._call("StandUp2Lie")

    def SetFsmId(self, fsm_id: int):
        return self._call("SetFsmId", int(fsm_id))

    # -- snake_case aliases --------------------------------------------------
    damp, stance, start, zero_torque, stop_move = Damp, Stance, Start, ZeroTorque, StopMove
    wave_hand, shake_hand, lie2standup, standup2lie = WaveHand, ShakeHand, Lie2StandUp, StandUp2Lie

    def run_command(self, name: str) -> None:
        name = name.lower()
        if name == "damp":
            self.Damp()
        elif name == "stance":
            self.Stance()
        elif name == "start":
            self.Start()
        elif name == "zero_torque":
            self.ZeroTorque()
        elif name == "stop_move":
            self.StopMove()
        elif name == "wave_hand":
            self.WaveHand(False)
        elif name == "wave_hand_turn":
            self.WaveHand(True)
        elif name == "shake_hand":
            self.ShakeHand()
            time.sleep(3)
            self.ShakeHand()
        elif name == "lie2standup":
            self.Stance()
            time.sleep(3)
            self.Lie2StandUp()
        elif name == "standup2lie":
            self.StandUp2Lie()
        else:
            raise ValueError(f"unknown command {name!r}; choose from {COMMANDS}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Unitree R1 high-level (LocoClient) commands")
    ap.add_argument("command", choices=COMMANDS)
    ap.add_argument("--iface", default=None, help="network interface (e.g. eth0)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--yes", action="store_true", help="skip the safety confirmation prompt")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if not args.dry_run and not args.yes:
        print("WARNING: Please ensure there are no obstacles around the robot.")
        input("Press Enter to continue...")
    loco = R1LocoClient(args.iface, dry_run=args.dry_run).connect()
    loco.run_command(args.command)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
