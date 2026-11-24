"""The MuJoCo scene the twin and the trainers use has the physics the Po Game Standard asks for (Profile C, c_physics):
gravity, joint limits from the joint map, PD position actuators, no adjacent-body collisions, foot friction."""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

from robotcontrol.configs.joint_map import JointMap

ROOT = pathlib.Path(__file__).resolve().parents[2]
SCENE = ROOT / "UnrealProject/MJCF/scene_stand.xml"


def _model():
    return mujoco.MjModel.from_xml_path(str(SCENE))


def _hinges(m):
    return [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]


def test_gravity_pulls_down():
    m = _model()
    np.testing.assert_allclose(m.opt.gravity, [0.0, 0.0, -9.81])
    # gravity shows up as a joint-space bias force on the hanging robot
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert d.qfrc_bias.any()  # gravity loads the joints


def test_every_hinge_is_limited_to_the_joint_map():
    m = _model()
    jm = JointMap.load()
    h = _hinges(m)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in h]
    assert names == list(jm.names)
    assert all(m.jnt_limited[i] for i in h)
    rng = np.array([m.jnt_range[i] for i in h])
    np.testing.assert_allclose(rng[:, 0], jm.limits_lo, atol=1e-4)
    np.testing.assert_allclose(rng[:, 1], jm.limits_hi, atol=1e-4)


def test_actuators_are_pd_position_servos():
    m = _model()
    assert m.nu == 26
    assert {int(g) for g in m.actuator_gaintype} == {int(mujoco.mjtGain.mjGAIN_FIXED)}
    assert {int(b) for b in m.actuator_biastype} == {
        int(mujoco.mjtBias.mjBIAS_AFFINE)
    }  # bias = -kp*q - kd*qdot: a PD servo
    assert np.all(m.actuator_gainprm[:, 0] == -m.actuator_biasprm[:, 1])  # same kp in gain and bias


def test_no_adjacent_body_collision_at_the_default_pose():
    m = _model()
    assert not m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT  # parent/child contacts filtered
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for c in d.contact[: d.ncon]:
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        assert m.body_parentid[b1] != b2 and m.body_parentid[b2] != b1, "adjacent links collide"


def test_feet_have_friction():
    m = _model()
    feet = [i for i in range(m.ngeom) if "foot" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "")]
    assert len(feet) == 14
    assert np.all(m.geom_friction[feet, 0] >= 0.5)
