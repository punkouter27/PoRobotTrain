import math

import numpy as np
import pytest

from robotcontrol.configs.joint_map import JointMap
from robotcontrol.execution.safety_guard import SafetyConfig, SafetyGuard
from robotcontrol.interfaces.base_interface import LowStateData, quat_to_rpy, projected_gravity_from_quat


@pytest.fixture(scope="module")
def jm():
    return JointMap.load()


def _state(jm, q=None, t=0.0, quat=(1, 0, 0, 0), dq=None):
    s = LowStateData.zeros(jm.num_joints, timestamp=t)
    s.q[:] = jm.defaults if q is None else q
    if dq is not None:
        s.dq[:] = dq
    s.imu_quat[:] = quat
    return s


def test_soft_limit_clip(jm):
    g = SafetyGuard(jm, SafetyConfig(torque_clip=False))
    st = _state(jm, t=0.0)
    g.apply(jm.defaults.copy(), st, t=0.0)  # prime
    target = jm.defaults.copy()
    i = jm.index_of("head_pitch_joint")
    target[i] = 10.0
    st2 = _state(jm, t=100.0)  # huge dt -> slew does not bind
    safe, rep = g.apply(target, st2, t=100.0)
    lo, hi = jm.soft_limits(0.9)
    assert math.isclose(safe[i], hi[i])
    assert rep.limit_clipped[i] and rep.limit_clipped.sum() == 1
    assert not rep.tripped


def test_slew_rate_limit(jm):
    g = SafetyGuard(jm, SafetyConfig(torque_clip=False))
    st = _state(jm, t=0.0)
    g.apply(jm.defaults.copy(), st, t=0.0)
    target = jm.defaults.copy()
    i_arm = jm.index_of("right_elbow_joint")
    i_leg = jm.index_of("left_knee_joint")
    target[i_arm] += 1.0
    target[i_leg] += 1.0
    dt = 0.02
    safe, rep = g.apply(target, _state(jm, t=dt), t=dt)
    assert math.isclose(safe[i_arm] - jm.defaults[i_arm], 6.0 * dt, rel_tol=1e-9)
    assert math.isclose(safe[i_leg] - jm.defaults[i_leg], 10.0 * dt, rel_tol=1e-9)
    assert rep.slew_limited[i_arm] and rep.slew_limited[i_leg]
    # keeps ramping from the last safe target
    safe2, _ = g.apply(target, _state(jm, t=2 * dt), t=2 * dt)
    assert math.isclose(safe2[i_arm] - jm.defaults[i_arm], 12.0 * dt, rel_tol=1e-9)


def test_torque_clip(jm):
    g = SafetyGuard(jm, SafetyConfig())
    i = jm.index_of("head_yaw_joint")  # kp 10, effort 0.86 -> max err 0.086
    st = _state(jm, t=0.0)
    g.apply(jm.defaults.copy(), st, t=0.0)
    target = jm.defaults.copy()
    target[i] = 0.1  # within soft limits and slew (dt=1s), but kp*err = 1.0 > 0.86
    safe, rep = g.apply(target, _state(jm, t=1.0), t=1.0)
    assert rep.torque_clipped[i]
    assert math.isclose(safe[i], 0.86 / 10.0, rel_tol=1e-9)


def test_tilt_trip_returns_damping(jm):
    g = SafetyGuard(jm, SafetyConfig())
    roll = math.radians(30.0)
    quat = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
    st = _state(jm, t=0.0, quat=quat)
    safe, rep = g.apply(jm.defaults.copy(), st, t=0.0)
    assert rep.tripped and "tilt" in rep.trip_reason
    assert np.allclose(rep.kp, 0.0) and np.allclose(rep.kd, jm.kd)
    assert np.allclose(safe, st.q)
    # latched: an upright state is still tripped until reset()
    _, rep2 = g.apply(jm.defaults.copy(), _state(jm, t=0.02), t=0.02)
    assert rep2.tripped
    g.reset()
    _, rep3 = g.apply(jm.defaults.copy(), _state(jm, t=0.04), t=0.04)
    assert not rep3.tripped


def test_tilt_below_threshold_ok(jm):
    g = SafetyGuard(jm, SafetyConfig())
    pitch = math.radians(20.0)
    quat = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
    _, rep = g.apply(jm.defaults.copy(), _state(jm, quat=quat), t=0.0)
    assert not rep.tripped
    assert math.isclose(rep.pitch_deg, 20.0, abs_tol=1e-6)


def test_stale_hold(jm):
    g = SafetyGuard(jm, SafetyConfig(torque_clip=False))
    target = jm.defaults.copy()
    target[jm.index_of("head_pitch_joint")] = 0.1
    safe0, _ = g.apply(target, _state(jm, t=0.0), t=0.0)
    safe1, rep1 = g.apply(None, _state(jm, t=0.05), t=0.05)
    assert not rep1.stale and np.allclose(safe1, safe0)
    safe2, rep2 = g.apply(None, _state(jm, t=0.2), t=0.2)
    assert rep2.stale and np.allclose(safe2, safe0) and not rep2.tripped


def test_accel_flag(jm):
    g = SafetyGuard(jm, SafetyConfig())
    g.apply(jm.defaults.copy(), _state(jm, t=0.0, dq=np.zeros(26)), t=0.0)
    dq = np.zeros(26)
    dq[3] = 2.0  # 2 rad/s in 20 ms -> 100 rad/s^2
    _, rep = g.apply(jm.defaults.copy(), _state(jm, t=0.02, dq=dq), t=0.02)
    assert rep.accel_flag[3] and rep.accel_flag.sum() == 1 and not rep.tripped


def test_quaternion_helpers():
    yaw = math.radians(90)
    q = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    rpy = quat_to_rpy(q)
    assert np.allclose(rpy, [0, 0, yaw])
    assert np.allclose(projected_gravity_from_quat(np.array([1.0, 0, 0, 0])), [0, 0, -1])
    pitch = math.radians(90)
    qp = np.array([math.cos(pitch / 2), 0, math.sin(pitch / 2), 0])
    assert np.allclose(projected_gravity_from_quat(qp), [1, 0, 0], atol=1e-9)
