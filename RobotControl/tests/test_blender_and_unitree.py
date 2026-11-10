import numpy as np
import pytest

from robotcontrol.configs.joint_map import JointMap
from robotcontrol.execution.policy_blender import PolicyBlender, blend_sequence
from robotcontrol.interfaces.base_interface import MotorCommand
from robotcontrol.interfaces.unitree_interface import FakeLowCmd, UnitreeSDK2Interface


def test_blender_linear_ramp():
    old = np.zeros(3)
    new = np.ones(3)
    traj = blend_sequence(old, new, num_steps=10)
    assert traj.shape == (10, 3)
    assert np.allclose(traj[:, 0], np.arange(1, 11) / 10.0)
    b = PolicyBlender(num_steps=4)
    b.start()
    outs = [b.blend(old, new) for _ in range(6)]
    assert np.allclose([o[0] for o in outs], [0.25, 0.5, 0.75, 1.0, 1.0, 1.0])
    assert b.done


def test_blender_slew_clamp():
    b = PolicyBlender(num_steps=2, max_slew=0.1)
    b.start(last_output=np.zeros(2))
    o1 = b.blend(np.zeros(2), np.ones(2))     # 0.5 requested -> clamped to 0.1
    o2 = b.blend(np.zeros(2), np.ones(2))     # 1.0 requested -> 0.2
    assert np.allclose(o1, 0.1) and np.allclose(o2, 0.2)
    with pytest.raises(ValueError):
        b.blend(np.zeros(2), np.ones(3))


def test_motor_command_slot_mapping():
    jm = JointMap.load()
    cmd = MotorCommand.from_targets(jm.defaults, jm.kp, jm.kd)
    slots = cmd.to_slot_arrays(jm)
    assert slots["mode"].sum() == 26
    assert all(slots["mode"][s] == 0 for s in (14, 20, 21, 27, 28))
    assert slots["q"][30] == jm.defaults[jm.index_of("head_yaw_joint")]
    assert slots["kp"][22] == jm.kp[jm.index_of("right_shoulder_pitch_joint")]
    with pytest.raises(ValueError):
        MotorCommand.from_targets(jm.defaults, jm.kp[:5], jm.kd)


def test_unitree_dry_run_fills_only_mapped_slots():
    jm = JointMap.load()
    with UnitreeSDK2Interface(jm, dry_run=True, log_every=0) as itf:
        assert itf.connected and isinstance(itf.low_cmd, FakeLowCmd)
        s = itf.get_low_state()
        assert np.allclose(s.q, jm.defaults)
        target = jm.defaults.copy()
        target[jm.index_of("head_yaw_joint")] = 0.4
        itf.send_low_cmd(target, jm.kp, jm.kd)
        lc = itf.low_cmd
        assert lc.mode_pr == 0
        for i, slot in enumerate(jm.motor_slots.tolist()):
            mc = lc.motor_cmd[slot]
            assert mc.mode == 1 and mc.q == target[i] and mc.kp == jm.kp[i] and mc.kd == jm.kd[i] and mc.tau == 0.0
        for slot in (14, 20, 21, 27, 28, 31, 32, 33, 34):
            assert lc.motor_cmd[slot].mode == 0 and lc.motor_cmd[slot].kp == 0.0
        # fake state tracks commanded targets for driven joints
        assert itf.get_low_state().q[jm.index_of("head_yaw_joint")] == 0.4
        # helpers
        d = itf.damping(kd=1.5)
        assert np.allclose(d.kp, 0.0) and np.allclose(d.kd, 1.5)
        z = itf.zero_torque()
        assert np.allclose(z.kp, 0.0) and np.allclose(z.kd, 0.0) and lc.motor_cmd[30].mode == 1
        itf.move_to(jm.defaults, duration_s=0.1, rate_hz=100)
        assert np.allclose(itf.get_low_state().q, jm.defaults)
        assert itf.stats()["cmds"] >= 13


def test_unitree_real_mode_without_sdk_gives_clear_error():
    pytest.importorskip("numpy")
    try:
        import unitree_sdk2py  # noqa: F401
        pytest.skip("unitree_sdk2py installed; skipping missing-SDK message test")
    except ImportError:
        pass
    itf = UnitreeSDK2Interface(JointMap.load(), dry_run=False)
    with pytest.raises(ImportError, match="unitree_sdk2py is not installed"):
        itf.connect()


def test_urlab_import_error_message():
    from robotcontrol.interfaces import urlab_interface
    try:
        urlab_interface._import_urlab()
    except ImportError as exc:
        assert "urlab" in str(exc).lower()
    # if it imports (deps present) that is fine too
