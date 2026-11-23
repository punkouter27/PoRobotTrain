"""Motor heat + battery model, action filter and jitter score (docs/sim_features.md #5, #15)."""

import json

import numpy as np
import pytest

from robotcontrol.sim_features.motor_model import (
    CONFIG_PATH,
    ActionFilter,
    MotorModel,
    MotorModelConfig,
    jitter_score,
)

CONTRACT = {
    "ambient_c": 25.0,
    "thermal_time_constant_s": 120.0,
    "heat_rate_c_per_s_at_full_torque": 0.8333,
    "warn_c": 70.0,
    "limit_c": 90.0,
    "battery_wh": 500.0,
    "idle_w": 60.0,
    "drive_efficiency": 0.8,
    "copper_loss_w_at_full_torque": 40.0,
    "torque_alert_fraction": 0.8,
    "action_filter_alpha": 0.5,
    "jitter_window_s": 1.0,
}


def test_config_file_has_the_contract_values():
    assert json.loads(CONFIG_PATH.read_text(encoding="utf-8")) == CONTRACT
    cfg = MotorModelConfig.load()
    assert cfg.heat_rate_c_per_s_at_full_torque == 0.8333 and cfg.battery_wh == 500.0


def test_unknown_config_key_is_rejected(tmp_path):
    p = tmp_path / "m.json"
    p.write_text(json.dumps(dict(CONTRACT, derate=True)), encoding="utf-8")
    with pytest.raises(ValueError, match="unknown keys"):
        MotorModelConfig.load(p)


def test_one_step_matches_the_formulas():
    cfg = MotorModelConfig()
    mm = MotorModel(["a", "b"], [10.0, 2.0], cfg)
    tau, dq, dt = np.array([5.0, -2.0]), np.array([2.0, 1.0]), 0.02
    mm.step(tau, dq, dt)
    f = np.abs(tau) / np.array([10.0, 2.0])
    np.testing.assert_allclose(mm.frac, f)
    np.testing.assert_allclose(mm.temp_c, 25.0 + dt * cfg.heat_rate_c_per_s_at_full_torque * f**2)
    p = cfg.idle_w + max(5.0 * 2.0, 0) / cfg.drive_efficiency + cfg.copper_loss_w_at_full_torque * np.sum(f**2)
    assert mm.power_w == pytest.approx(p)  # the braking joint (tau*dq < 0) adds no mechanical power
    assert mm.soc == pytest.approx(1.0 - p * dt / (cfg.battery_wh * 3600))
    assert mm.minutes_left() == pytest.approx(mm.soc * cfg.battery_wh / p * 60)


def test_full_torque_heats_towards_equilibrium_and_cools_back():
    cfg = MotorModelConfig()
    mm = MotorModel(["j"], [1.0], cfg)
    for _ in range(int(600 / 0.02)):  # 10 min at full torque
        mm.step(np.array([1.0]), np.array([0.0]), 0.02)
    eq = cfg.ambient_c + cfg.heat_rate_c_per_s_at_full_torque * cfg.thermal_time_constant_s  # 125 C
    assert mm.temp_c[0] == pytest.approx(eq, abs=1.0)
    th = mm.thermal_block()
    assert th["hottest"] == "j" and th["limit"] == ["j"] and th["warn"] == []
    for _ in range(int(1200 / 0.02)):
        mm.step(np.array([0.0]), np.array([0.0]), 0.02)
    assert mm.temp_c[0] < 30.0
    assert mm.torque_alert() == []
    mm.step(np.array([0.9]), np.array([0.0]), 0.02)
    assert mm.torque_alert() == ["j"]


def test_blocks_are_json_ready():
    mm = MotorModel(["x", "y"], [1.0, 1.0])
    mm.step(np.array([0.1, 0.2]), np.array([1.0, -1.0]), 0.02)
    out = {"motors": mm.motors_block(), "thermal": mm.thermal_block(), "battery": mm.battery_block()}
    assert json.loads(json.dumps(out))["motors"]["names"] == ["x", "y"]
    assert 0 < out["battery"]["soc_pct"] <= 100


def test_action_filter_ema_and_jitter():
    af = ActionFilter(2, alpha=0.5, window_s=0.1, dt=0.02)
    assert af.window == 5
    np.testing.assert_allclose(af.step(np.array([1.0, 0.0])), [1.0, 0.0])  # first sample passes through
    np.testing.assert_allclose(af.step(np.array([0.0, 0.0])), [0.5, 0.0])
    np.testing.assert_allclose(af.step(np.array([0.0, 0.0])), [0.25, 0.0])
    # second difference of [1, 0, 0] is [1, 0] -> mean square 0.5
    assert af.jitter() == pytest.approx(np.sqrt(0.5))
    b = af.block()
    assert b["raw"] == [0.0, 0.0] and b["filtered"] == [0.25, 0.0] and b["apply"] is False
    af.set(alpha=0.2, apply=True)
    assert af.alpha == 0.2 and af.apply
    with pytest.raises(ValueError):
        af.set(alpha=0.0)
    smooth = np.linspace(0, 1, 50)[:, None]
    assert jitter_score(smooth) == pytest.approx(0.0, abs=1e-12)
    assert jitter_score(np.array([0.0, 1.0, 0.0, 1.0])) == pytest.approx(2.0)
