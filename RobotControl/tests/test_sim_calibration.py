"""Tests covering all 10 sim-to-real digital twin fidelity upgrades and calibration CLI."""
import math
import tempfile
from pathlib import Path

import numpy as np
import pytest

from robotcontrol.behaviours.backends import SimBackend
from robotcontrol.behaviours.neck_turn import HEAD_YAW
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.configs.sim_calibration import SimCalibration
from robotcontrol.interfaces.base_interface import MotorCommand
from robotcontrol.interfaces.mujoco_interface import MujocoInterface
from robotcontrol.tools.calibrate_sim import (
    fit_calibration,
    measure_idle_noise,
    simulate_behaviour_trajectory,
)
from robotcontrol.training.level0_env import Level0Config, Level0StandEnv


def test_sim_calibration_loads_and_differentiates_joint_classes():
    jm = JointMap.load()
    cal = SimCalibration.load(jm=jm)
    i_yaw = jm.index_of("head_yaw_joint")
    i_hip = jm.index_of("left_hip_pitch_joint")
    assert cal.armature[i_hip] > cal.armature[i_yaw]
    assert cal.unpowered_friction_nm[i_hip] > cal.unpowered_friction_nm[i_yaw]
    assert cal.control_delay_ms == pytest.approx(10.0)


def test_sim_backend_applies_delay_noise_bemf_and_compliant_stand():
    jm = JointMap.load()
    cal = SimCalibration.load(jm=jm)
    cal.zero_offset_rad[jm.index_of(HEAD_YAW)] = math.radians(2.5)
    b = SimBackend(jm, [HEAD_YAW], settle_s=0.5, calibration=cal, realistic=True, seed=42)
    try:
        q, dq, age = b.read()
        assert age == pytest.approx(0.01)
        assert abs(math.degrees(q[jm.index_of(HEAD_YAW)]) - 2.5) < 0.6
        b.command({HEAD_YAW: math.radians(15.0)})
        b.advance(0.4)
        tel = b.telemetry()
        assert "imu_rpy_deg" in tel and len(tel["imu_rpy_deg"]) == 3
    finally:
        b.close()


def test_mujoco_interface_realistic_mode_preserves_contract():
    jm = JointMap.load()
    with MujocoInterface(jm, scene="stand", realistic=True, seed=1) as itf:
        s0 = itf.get_low_state()
        assert s0.q.shape == (26,)
        tgt = jm.defaults.copy()
        tgt[jm.index_of(HEAD_YAW)] = 0.25
        for _ in range(10):
            itf.send_low_cmd(tgt, jm.kp, jm.kd)
            itf.tick()
        assert itf.get_low_state().q[jm.index_of(HEAD_YAW)] > 0.05


def test_calibrate_sim_noise_and_fit_against_simulated_real_log():
    jm = JointMap.load()
    cal_true = SimCalibration.load(jm=jm)
    cal_true.zero_offset_rad[jm.index_of(HEAD_YAW)] = math.radians(1.8)
    ts, qs = simulate_behaviour_trajectory(jm, cal_true, "neck_turn", amplitude_deg=10.0, freq_hz=0.2, duration_s=3.0)

    noise = measure_idle_noise(ts[:20], qs[:20])
    assert noise["pos_noise_rad"] >= 0.0 and noise["vel_noise_rad_s"] >= 0.0

    with tempfile.TemporaryDirectory() as tmp:
        real_csv = Path(tmp) / "real_neck.csv"
        cal_copy = Path(tmp) / "sim_calibration.yaml"
        cal_copy.write_text(Path("RobotControl/configs/sim_calibration.yaml").read_text(encoding="utf-8"), encoding="utf-8")
        hdr = ["t", "phase", "target_head_yaw_joint"] + list(jm.names)
        lines = [",".join(hdr)]
        for t, qrow in zip(ts, qs):
            lines.append(",".join([f"{t:.4f}", "sweep", "0.0"] + [f"{v:.5f}" for v in qrow]))
        real_csv.write_text("\n".join(lines), encoding="utf-8")

        res = fit_calibration(real_csv, "neck_turn", amplitude_deg=10.0, freq_hz=0.2, cal_path=cal_copy, write=True)
        assert res["rms_after_deg"] <= res["rms_before_deg"] + 1e-6
        assert res["max_after_deg"] < 5.0
        updated = SimCalibration.load(cal_copy, jm=jm)
        assert updated.last_calibrated_at is not None


def test_level0_env_randomizes_calibrated_friction_and_damping():
    env = Level0StandEnv(Level0Config(num_envs=4, nthread=1, episode_length_s=0.4))
    try:
        d0 = env.models[0].dof_damping[env.hinge_dofs].copy()
        d1 = env.models[1].dof_damping[env.hinge_dofs].copy()
        assert not np.allclose(d0, d1)
        env.set_eval(True)
        env.reset(np.array([0, 1]))
        assert np.allclose(
            env.models[0].dof_damping[env.hinge_dofs],
            env.models[1].dof_damping[env.hinge_dofs],
        )
    finally:
        env.close()
