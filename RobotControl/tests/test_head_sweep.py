"""Neck turn behaviour: trajectory, aborts, and that the real LowCmd enables ONLY the head-yaw slot."""
import math
import types

import pytest

from robotcontrol.configs.joint_map import JointMap
from robotcontrol.behaviours.backends import RealBackend
from robotcontrol.behaviours.neck_turn import HEAD_YAW, HeadSweepController, SweepParams


def _ctl(**kw):
    jm = JointMap.load()
    i = jm.index_of(HEAD_YAW)
    return HeadSweepController(SweepParams(**kw), jm.kp[i], jm.kd[i], jm.effort[i], jm.limits_lo[i], jm.limits_hi[i])


def test_refuses_to_start_off_centre():
    with pytest.raises(RuntimeError, match="straight ahead"):
        _ctl().start(math.radians(115.0))


def test_amplitude_capped():
    with pytest.raises(ValueError):
        _ctl(amplitude_deg=45.0)


def test_trajectory_bounded_and_ends_centred():
    c = _ctl(amplitude_deg=10.0)
    c.start(math.radians(5.0))
    ts = [k * 0.01 for k in range(int(c.p.total_s * 100) + 1)]
    tg = [c.nominal_target(t) for t in ts]
    assert abs(tg[0] - math.radians(5.0)) < 1e-9
    assert max(abs(x) for x in tg) <= math.radians(10.0) + 1e-9
    assert abs(tg[-1]) < 1e-6
    assert max(abs(a - b) for a, b in zip(tg, tg[1:])) < 0.01   # smooth, no jumps


def test_torque_band_respected():
    c = _ctl()
    c.start(0.0)
    tgt = c.command(5.0, q=math.radians(-20.0), dq=0.0, state_age_s=0.0)
    assert abs(c.kp * (tgt - math.radians(-20.0))) <= c.effort + 1e-9


def test_aborts_when_head_does_not_follow():
    c = _ctl(amplitude_deg=30.0)
    c.start(0.0)
    out = None
    t = 0.0
    while t < 10.0 and (out is not None or t == 0.0):
        out = c.command(t, q=0.0, dq=0.0, state_age_s=0.0)   # head stuck at 0 while target sweeps to 30
        t += 0.01
    assert out is None and "tracking error" in c.abort_reason


def test_aborts_on_stale_state():
    c = _ctl()
    c.start(0.0)
    assert c.command(1.0, 0.0, 0.0, state_age_s=0.5) is None
    assert "stale" in c.abort_reason


class _Cmd:
    def __init__(self):
        self.mode_pr = 0
        self.mode_machine = 0
        self.crc = 0
        self.motor_cmd = [types.SimpleNamespace(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0) for _ in range(35)]


def test_real_lowcmd_enables_only_head_yaw_slot():
    jm = JointMap.load()
    sent = []
    b = RealBackend.__new__(RealBackend)
    b.jm, b.powered, b.pidx = jm, [HEAD_YAW], [jm.index_of(HEAD_YAW)]
    b._setup(jm, True, _Cmd, types.SimpleNamespace(Crc=lambda m: 1))
    b._msg = types.SimpleNamespace(mode_machine=3)
    b._pub = types.SimpleNamespace(Write=sent.append)
    b.command({HEAD_YAW: 0.1})
    assert b.pslots == [30]
    cmd = sent[-1]
    on = [i for i, m in enumerate(cmd.motor_cmd) if m.mode != 0 or m.kp != 0 or m.kd != 0]
    assert on == [30]
    assert cmd.motor_cmd[30].kp == 10.0 and cmd.motor_cmd[30].q == 0.1 and cmd.mode_machine == 3


def test_sim_neck_turn_moves_only_the_head():
    from robotcontrol.behaviours.runner import main
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        assert main(["neck_turn", "--target", "sim", "--duration", "5", "--log-dir", d]) == 0


def test_sim_refuses_off_centre_start():
    from robotcontrol.behaviours.runner import main
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        assert main(["neck_turn", "--target", "sim", "--duration", "2", "--sim-start-deg", "40", "--log-dir", d]) == 1
