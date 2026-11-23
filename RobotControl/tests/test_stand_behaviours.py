"""Stand-safe upper-body behaviours and telemetry extraction."""

import math
import tempfile
import types

import pytest

from robotcontrol.behaviours.backends import RealBackend, extract_lowstate_telemetry
from robotcontrol.behaviours.move_joint import MoveJoint, validate_move_request
from robotcontrol.behaviours.move_joint import main as move_joint_main
from robotcontrol.behaviours.neck_turn import SweepParams
from robotcontrol.behaviours.runner import main as runner_main
from robotcontrol.behaviours.stand_behaviours import (
    BEHAVIOUR_SPECS,
    HEAD_PITCH,
    R_ELBOW,
    R_SHOULDER_PITCH,
    R_SHOULDER_ROLL,
    ArmRaise,
    HandWave,
    NeckNod,
)
from robotcontrol.configs.joint_map import JointMap


def _default_q(jm: JointMap) -> dict:
    return dict(zip(jm.names, jm.defaults.tolist()))


def test_behaviour_specs_cover_all_stand_behaviours():
    assert {k for k, s in BEHAVIOUR_SPECS.items() if s.kind == "scripted"} == {
        "neck_turn",
        "neck_nod",
        "arm_raise",
        "hand_wave",
    }


def test_neck_nod_caps_amplitude_and_returns_to_zero():
    jm = JointMap.load()
    beh = NeckNod(jm, SweepParams(amplitude_deg=30.0, duration_s=10.0))
    assert beh.eff_amp_deg == 20.0
    q0 = _default_q(jm)
    beh.start(q0)
    vals = [beh.nominal_targets(k * 0.05)[HEAD_PITCH] for k in range(int(beh.duration_s / 0.05) + 1)]
    assert max(abs(v) for v in vals) <= math.radians(20.0) + 1e-6
    assert abs(vals[-1]) < 1e-5


def test_arm_raise_starts_and_ends_at_initial_pose():
    jm = JointMap.load()
    beh = ArmRaise(jm, SweepParams(amplitude_deg=20.0, duration_s=6.0))
    q0 = _default_q(jm)
    q0[R_SHOULDER_PITCH] = 0.05  # relaxed arm pose
    q0[R_ELBOW] = 0.1
    beh.start(q0)
    t_end = beh.duration_s
    start = beh.nominal_targets(0.0)
    end = beh.nominal_targets(t_end)
    assert abs(start[R_SHOULDER_PITCH] - 0.05) < 1e-6
    assert abs(end[R_SHOULDER_PITCH] - 0.05) < 1e-5
    assert end[R_ELBOW] == pytest.approx(0.1)


def test_hand_wave_powers_only_right_arm_joints():
    jm = JointMap.load()
    beh = HandWave(jm, SweepParams(amplitude_deg=30.0, duration_s=6.0))
    assert beh.eff_amp_deg == 20.0
    assert set(beh.powered) == {R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_ELBOW}
    q0 = _default_q(jm)
    beh.start(q0)
    end = beh.nominal_targets(beh.duration_s)
    for jn in beh.powered:
        assert abs(end[jn] - q0[jn]) < 1e-5


def test_hand_wave_real_lowcmd_enables_only_three_arm_slots():
    jm = JointMap.load()
    sent = []

    class _Cmd:
        def __init__(self):
            self.mode_pr = 0
            self.mode_machine = 0
            self.crc = 0
            self.motor_cmd = [types.SimpleNamespace(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0) for _ in range(35)]

    b = RealBackend.__new__(RealBackend)
    b.jm = jm
    b.powered = list(HandWave.powered)
    b.pidx = [jm.index_of(n) for n in b.powered]
    b._setup(jm, True, _Cmd, types.SimpleNamespace(Crc=lambda m: 1))
    b._msg = types.SimpleNamespace(mode_machine=2)
    b._pub = types.SimpleNamespace(Write=sent.append)
    b.command({R_SHOULDER_PITCH: -0.2, R_SHOULDER_ROLL: -0.4, R_ELBOW: 0.5})
    cmd = sent[-1]
    on = [i for i, m in enumerate(cmd.motor_cmd) if m.mode != 0 or m.kp != 0 or m.kd != 0]
    assert set(on) == {22, 23, 25}


def test_extract_lowstate_telemetry():
    jm = JointMap.load()
    motors = [types.SimpleNamespace(temperature=[32, 34], vol=48.2, mode=0) for _ in range(35)]
    motors[30].temperature = [44, 47]
    motors[30].mode = 1
    msg = types.SimpleNamespace(
        motor_state=motors,
        imu_state=types.SimpleNamespace(rpy=[0.01, -0.02, 0.05]),
        mode_machine=4,
    )
    tel = extract_lowstate_telemetry(msg, jm)
    assert tel["max_temp_c"] == 47
    assert tel["hot_joint"] == "head_yaw"
    assert tel["bus_v"] == pytest.approx(48.2)
    assert tel["active_motors"] == 1
    assert tel["mode_machine"] == 4
    assert len(tel["temps_c"]) == 26
    assert len(tel["imu_rpy_deg"]) == 3


@pytest.mark.parametrize("behaviour", ["neck_nod", "arm_raise", "hand_wave"])
def test_sim_stand_behaviours_complete_cleanly(behaviour):
    with tempfile.TemporaryDirectory() as d:
        assert (
            runner_main(
                [behaviour, "--target", "sim", "--fast", "--amplitude", "20", "--duration", "3", "--log-dir", d]
            )
            == 0
        )


def test_move_joint_smooth_ramp_and_sim_execution(capsys):
    jm = JointMap.load()
    with pytest.raises(ValueError):
        validate_move_request(jm, "left_hip_pitch_joint", 10.0, "real")
    _, rad = validate_move_request(jm, "head_yaw_joint", 12.0, "real")
    beh = MoveJoint(jm, "head_yaw_joint", rad, speed_deg_s=24.0)
    beh.start(_default_q(jm))
    assert beh.targets(0.0, _default_q(jm), {n: 0.0 for n in jm.names}, 0.0)["head_yaw_joint"] == pytest.approx(0.0)
    # Effort cap keeps step within effort/kp (0.86 / 10 = 0.086 rad) above current q=0; near rad it reaches rad
    assert beh.targets(beh.duration_s, _default_q(jm), {n: 0.0 for n in jm.names}, 0.0)[
        "head_yaw_joint"
    ] == pytest.approx(0.086)
    q_near = dict(_default_q(jm), head_yaw_joint=rad)
    assert beh.targets(beh.duration_s, q_near, {n: 0.0 for n in jm.names}, 0.0)["head_yaw_joint"] == pytest.approx(rad)

    with tempfile.TemporaryDirectory() as tmp:
        code = move_joint_main(["head_yaw_joint", "8.0", "--target", "sim", "--log-dir", tmp])
    assert code == 0
    out = capsys.readouterr().out
    assert "@telemetry " in out and "head_yaw_joint" in out
