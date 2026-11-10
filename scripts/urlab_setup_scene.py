"""Build the Level 0 stand scene in the running UE editor through URLab's RPC bridge.

Creates /Game/Levels/R1_Stand, imports scene_stand.xml (floor + gantry stand + pelvis weld + R1),
spawns it at the origin, saves the level, and frames the viewport on the robot.

Usage (editor must be open with the R1Twin project):
  .venv/Scripts/python.exe scripts/urlab_setup_scene.py [--force]
"""
from __future__ import annotations

import argparse
import pathlib
import time

from urlab_client import URLabClient

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCENE_XML = ROOT / "UnrealProject/MJCF/scene_stand.xml"
LEVEL = "R1_Stand"
ACTOR_ID = "r1_stand"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true", help="recreate level and reimport the MJCF")
    args = ap.parse_args()

    c = URLabClient("tcp://localhost", mujoco_version_check=False, recv_timeout_ms=120_000)
    try:
        c.connect()
        print("connected: server mujoco", c.mujoco_version, "| level", c.scene.current_level())
        st = c.sim.status()
        print("PIE status:", st)
        try:
            c.sim.stop()  # editor ops fail while any PIE session is running (even one without a URLab manager)
            time.sleep(2.0)
        except Exception as e:
            print("sim.stop:", e)

        t = time.time()
        try:
            c.scene.create_level(LEVEL, force_overwrite=args.force)
            print(f"created level /Game/Levels/{LEVEL}")
        except Exception as e:  # already exists
            print(f"create_level: {e}; loading existing")
            c.scene.load_level(LEVEL)

        bp = c.scene.import_xml(str(SCENE_XML).replace("\\", "/"), force_reimport=args.force)
        print(f"imported {SCENE_XML.name} -> {bp.class_path} ({time.time() - t:.1f} s)")

        h = c.scene.spawn_actor(bp, ACTOR_ID, location=(0.0, 0.0, 0.0))
        print(f"spawned {ACTOR_ID}: {h.actor_path} (was_existing={h.was_existing})")
        print("manager present/created:", c.scene.ensure_manager())
        # MJCF lights are MuJoCo-only; UE needs its own lights or a Lit viewport renders black.
        for kind, aid, loc, rot, inten in [("directional", "sun", (0, 0, 5), (0.0, -50.0, 120.0), 6.0),
                                           ("point", "fill", (2.0, -2.0, 2.2), (0, 0, 0), 8000.0)]:
            try:
                c.scene.spawn_light(kind, actor_id=aid, location=loc, rotation_euler=rot, intensity=inten)
                print("light:", aid)
            except Exception as e:
                print("light", aid, "failed:", e)
        c.scene.save_level()
        print("level saved")
        try:
            c.viewport.set_camera((2.2, -1.9, 1.35), rotation_euler=(0.0, -10.0, -139.0))  # location: MuJoCo-frame metres; rotation: UE-frame degrees
        except Exception as e:
            print("viewport framing skipped:", e)
    finally:
        c.close()


if __name__ == "__main__":
    main()
