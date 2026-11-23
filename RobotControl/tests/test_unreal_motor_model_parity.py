"""The Unreal lab's C++ motor model (R1MotorModel.h) uses the numbers of RobotControl/configs/motor_model.json."""

import json
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONFIG = ROOT / "RobotControl/configs/motor_model.json"
HEADER = ROOT / "UnrealProject/Source/R1Twin/Public/R1MotorModel.h"

FIELDS = {
    "ambient_c": "AmbientC",
    "thermal_time_constant_s": "ThermalTimeConstantS",
    "heat_rate_c_per_s_at_full_torque": "HeatRateCPerSAtFullTorque",
    "warn_c": "WarnC",
    "limit_c": "LimitC",
    "battery_wh": "BatteryWh",
    "idle_w": "IdleW",
    "drive_efficiency": "DriveEfficiency",
    "copper_loss_w_at_full_torque": "CopperLossWAtFullTorque",
    "torque_alert_fraction": "TorqueAlertFraction",
    "action_filter_alpha": "ActionFilterAlpha",
    "jitter_window_s": "JitterWindowS",
}


def test_cpp_defaults_match_motor_model_json():
    if not HEADER.exists():
        pytest.skip(f"{HEADER.name} not written yet")
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    assert set(cfg) == set(FIELDS)
    header = HEADER.read_text(encoding="utf-8")
    for key, name in FIELDS.items():
        m = re.search(rf"static constexpr float {name} = ([-+0-9.eE]+)f;", header)
        assert m, f"{name} not found in {HEADER.name}"
        assert float(m.group(1)) == pytest.approx(cfg[key], rel=1e-6), f"{name} != {key}"
