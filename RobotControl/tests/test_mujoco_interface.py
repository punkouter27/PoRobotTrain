import math

import numpy as np
import pytest

from robotcontrol.configs.joint_map import JointMap
from robotcontrol.interfaces.mujoco_interface import MujocoInterface, resolve_scene

pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def jm():
    return JointMap.load()


def test_scene_files_exist():
    assert resolve_scene("stand").exists()
    assert resolve_scene("ground").exists()


def test_head_yaw_ramp_on_stand(jm):
    with MujocoInterface(jm, scene="stand") as itf:
        assert itf.substeps == 4 and math.isclose(itf.sim_dt, 0.005)
        s0 = itf.get_low_state()
        assert s0.q.shape == (26,) and s0.imu_quat.shape == (4,) and s0.gyro.shape == (3,) and s0.accel.shape == (3,)
        assert np.allclose(s0.q, jm.defaults, atol=1e-6)
        assert np.allclose(s0.imu_quat, [1, 0, 0, 0], atol=1e-6)

        i_yaw = jm.index_of("head_yaw_joint")
        legs = jm.group_indices("left_leg", "right_leg")
        q_goal = 0.5
        steps = int(2.0 / 0.02)
        for k in range(steps):
            target = jm.defaults.copy()
            target[i_yaw] = q_goal * min(1.0, (k + 1) / (steps * 0.75))  # 1.5 s ramp, 0.5 s settle
            itf.send_low_cmd(target, jm.kp, jm.kd)
            itf.tick()
        s = itf.get_low_state()
        assert math.isclose(s.timestamp, 2.0, abs_tol=1e-6)
        assert abs(s.q[i_yaw] - q_goal) < 0.05, s.q[i_yaw]
        leg_move_deg = np.degrees(np.abs(s.q[legs] - jm.defaults[legs]))
        assert leg_move_deg.max() < 2.0, leg_move_deg
        # IMU: pelvis welded to the stand -> upright, gravity ~ 9.81 on +z of the accelerometer
        assert abs(np.degrees(s.rpy()[:2])).max() < 2.0
        assert abs(s.accel[2] - 9.81) < 1.0


def test_gains_are_applied_per_command(jm):
    with MujocoInterface(jm, scene="stand") as itf:
        i = jm.index_of("head_yaw_joint")
        target = jm.defaults.copy()
        target[i] = 0.5
        kp = jm.kp.copy()
        kd = jm.kd.copy()
        kp[i] = 0.0  # passive head yaw: must not move
        for _ in range(50):
            itf.send_low_cmd(target, kp, kd)
            itf.tick()
        assert abs(itf.get_low_state().q[i]) < 0.02
        aid = itf._act_ids[i]
        assert itf.model.actuator_gainprm[aid, 0] == 0.0 and itf.model.actuator_biasprm[aid, 2] == -kd[i]


def test_set_passive_forces_zero_kp(jm):
    with MujocoInterface(jm, scene="stand") as itf:
        arm = jm.group_indices("left_arm")
        itf.set_passive(arm, kd=1.0)
        cmd = itf.send_low_cmd(jm.defaults, jm.kp, jm.kd)
        assert np.allclose(cmd.kp[arm], 0.0) and np.allclose(cmd.kd[arm], 1.0)
        assert np.allclose(cmd.kp[jm.group_indices("right_arm")], jm.kp[jm.group_indices("right_arm")])
        itf.set_passive([])
        cmd2 = itf.send_low_cmd(jm.defaults, jm.kp, jm.kd)
        assert np.allclose(cmd2.kp, jm.kp)
