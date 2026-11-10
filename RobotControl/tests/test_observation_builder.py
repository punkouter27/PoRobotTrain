import math

import numpy as np
import pytest

from robotcontrol.configs.deploy_config import DeployConfig, ObsTerm, level0_deploy_config
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.execution.policy_runner import ObservationBuilder
from robotcontrol.interfaces.base_interface import LowStateData


@pytest.fixture(scope="module")
def jm():
    return JointMap.load()


def test_level0_layout(jm):
    cfg = level0_deploy_config(jm)
    ob = ObservationBuilder(cfg, jm)
    assert ob.num_obs == 23
    sl = ob.term_slices
    assert sl["joint_pos_rel"] == slice(0, 7)
    assert sl["joint_vel"] == slice(7, 14)
    assert sl["last_action"] == slice(14, 21)
    assert sl["command"] == slice(21, 23)

    st = LowStateData.zeros(26, timestamp=0.0)
    st.q[:] = jm.defaults
    st.q[jm.index_of("head_yaw_joint")] = jm.defaults[jm.index_of("head_yaw_joint")] + 0.3
    st.dq[jm.index_of("right_elbow_joint")] = -1.5
    last_action = np.arange(7, dtype=np.float32) * 0.1
    cmd = np.array([0.5, -0.8])
    obs = ob.build(st, last_action, cmd, t=0.0)
    assert obs.shape == (23,) and obs.dtype == np.float32
    assert math.isclose(obs[6], 0.3, abs_tol=1e-6)        # head yaw is action index 6
    assert np.allclose(obs[0:6], 0.0)
    assert math.isclose(obs[7 + 3], -1.5, abs_tol=1e-6)   # right_elbow is action index 3
    assert np.allclose(obs[14:21], last_action)
    assert np.allclose(obs[21:23], cmd)


def test_level2_layout(jm):
    joints = [n for n in jm.names if not n.startswith("head")]
    idx = jm.indices_of(joints)
    cfg = DeployConfig(
        policy_name="walk", step_dt=0.02, joint_names=joints, motor_slots=jm.motor_slots[idx].tolist(),
        default_joint_pos=jm.defaults[idx], kp=jm.kp[idx], kd=jm.kd[idx], action_scale=jm.action_scale[idx],
        action_clip=None,
        observations=[ObsTerm("base_ang_vel", 3), ObsTerm("projected_gravity", 3), ObsTerm("command", 3),
                      ObsTerm("gait_phase", 2, {"period": 0.8}), ObsTerm("joint_pos_rel", 24),
                      ObsTerm("joint_vel_rel", 24), ObsTerm("last_action", 24)],
        commands={"lin_vel_x": 1, "lin_vel_y": 1, "ang_vel_z": 1}, num_obs=83, num_actions=24,
        passive_joints=["head_pitch_joint", "head_yaw_joint"],
    )
    assert cfg.validate(jm) == []
    ob = ObservationBuilder(cfg, jm)
    assert ob.num_obs == 83
    st = LowStateData.zeros(26)
    st.q[:] = jm.defaults
    st.gyro[:] = [0.1, 0.2, 0.3]
    obs = ob.build(st, np.zeros(24, np.float32), np.array([1.0, 0.0, 0.0]), t=0.2)
    assert np.allclose(obs[0:3], [0.1, 0.2, 0.3])
    assert np.allclose(obs[3:6], [0, 0, -1])
    assert np.allclose(obs[6:9], [1, 0, 0])
    assert np.allclose(obs[9:11], [math.sin(2 * math.pi * 0.25), math.cos(2 * math.pi * 0.25)], atol=1e-6)
    assert np.allclose(obs[11:], 0.0)


def test_size_mismatch_rejected(jm):
    cfg = level0_deploy_config(jm)
    cfg.observations[0].size = 6
    assert any("joint_pos_rel" in e for e in cfg.validate(jm))
    with pytest.raises(ValueError):
        ObservationBuilder(cfg, jm)
