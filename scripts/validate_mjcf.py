"""Phase A validation: load each MJCF, check joint/actuator counts, mass, joint-map agreement, settle stability."""
from __future__ import annotations

import pathlib
import sys

import mujoco
import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODELS = ROOT / "UnrealProject/MJCF"
jm = yaml.safe_load((ROOT / "RobotControl/configs/r1_joint_map.yaml").read_text())
ok = True


def check(cond, msg):
    global ok
    print(("  PASS " if cond else "  FAIL ") + msg)
    ok &= bool(cond)


for name in ("r1_twin.xml", "r1_twin_stand.xml", "scene_ground.xml", "scene_stand.xml"):
    print(f"\n== {name}")
    m = mujoco.MjModel.from_xml_path(str(MODELS / name))
    d = mujoco.MjData(m)
    hinge = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    check(len(hinge) == 26, f"26 hinge joints (got {len(hinge)})")
    check(m.nu == 26, f"26 actuators (got {m.nu})")
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in hinge]
    check(names == [j["name"] for j in jm["joints"]], "joint order == joint map order")
    for i, j in zip(hinge, jm["joints"]):
        if not np.allclose(m.jnt_range[i], j["limits"], atol=1e-5):
            check(False, f"limits mismatch {j['name']} {m.jnt_range[i]} vs {j['limits']}")
    pelvis_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    mass = float(m.body_subtreemass[pelvis_id])
    check(29.5 < mass < 30.6, f"robot mass {mass:.3f} kg (28.93 body + 1.22 head)")
    check(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "head_rgbd") >= 0, "head camera present")
    check(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "imu") >= 0, "imu site present")

    mujoco.mj_resetDataKeyframe(m, d, 0)
    q0 = d.qpos.copy()
    if name == "r1_twin.xml":
        print("  info: robot-only file (no floor), settle test skipped")
        continue
    if "stand" in name:
        for _ in range(int(5.0 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        pelvis_drift = float(np.linalg.norm(d.qpos[:3] - q0[:3]))
        check(pelvis_drift < 0.005, f"stand weld holds pelvis (drift {pelvis_drift*1000:.2f} mm over 5 s)")
        check(d.ncon == 0, f"no contacts while hanging (ncon={d.ncon})")
        qv = float(np.abs(d.qvel[6:]).max())
        check(qv < 0.05, f"joints at rest after 5 s (max |qvel| {qv:.4f} rad/s)")
        legs = np.array(jm["groups"]["left_leg"] + jm["groups"]["right_leg"])
        held = np.array([i for i in range(26) if i not in legs])
        dev = np.rad2deg(np.abs(d.qpos[7:] - q0[7:]))
        check(dev[held].max() < 3.0, f"held joints (waist/arms/head) stay within 3 deg of home ({dev[held].max():.2f} deg)")
        print(f"  info: passive legs (kp 0, kd 3) creep {dev[legs].max():.2f} deg under gravity in 5 s (expected)")
    else:
        # Open-loop PD cannot balance a 30 kg humanoid (ankle stiffness << m*g*h); standing is a
        # Level 1+ policy task. Here we only verify contact quality during the first 0.5 s on the floor.
        mujoco.mj_forward(m, d)
        foot_z = min(d.site("left_foot").xpos[2], d.site("right_foot").xpos[2])
        maxpen = 0.0
        for _ in range(int(0.5 / m.opt.timestep)):
            mujoco.mj_step(m, d)
            if d.ncon:
                maxpen = max(maxpen, float(-d.contact.dist[: d.ncon].min()))
        check(d.ncon > 0, f"feet in contact with floor (ncon={d.ncon}, foot site z at t0 {foot_z*1000:.1f} mm)")
        check(maxpen < 0.005, f"max ground penetration in first 0.5 s {maxpen*1000:.2f} mm")
        print(f"  info: pelvis z after 0.5 s open-loop PD = {d.qpos[2]:.3f} m "
              "(falls later; expected without a balance policy)")
    print(f"  info: dt {m.opt.timestep}, nbody {m.nbody}, ngeom {m.ngeom}")

print("\nALL PASS" if ok else "\nFAILURES PRESENT")
sys.exit(0 if ok else 1)
