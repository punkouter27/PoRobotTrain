"""RobotControl/configs/reward_config.json is valid and is what the Level 0 trainer uses."""

from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace

from robotcontrol.training import level0_env

CONFIG = Path(__file__).resolve().parents[1] / "configs" / "reward_config.json"


def _raw():
    return json.loads(CONFIG.read_text(encoding="utf-8"))


def test_file_is_valid():
    raw = _raw()
    pen = raw["penalties"]
    assert set(pen) == {"torque_sq", "joint_ang_vel_sq", "action_rate_sq", "foot_slip"}
    assert all(math.isfinite(w) and w <= 0 for w in pen.values())
    stages = [s["stage"] for s in raw["curriculum"]]
    assert stages == ["L0_start", "L0_advance", "L0_full"]


def test_level0_config_reads_the_file():
    pen = _raw()["penalties"]
    cur = {s["stage"]: s for s in _raw()["curriculum"]}
    c = level0_env.Level0Config()
    assert (c.w_torque, c.w_joint_vel, c.w_action_rate, c.w_foot_slip) == (
        pen["torque_sq"],
        pen["joint_ang_vel_sq"],
        pen["action_rate_sq"],
        pen["foot_slip"],
    )
    assert c.curriculum_start == cur["L0_start"]["threshold"]
    assert c.curriculum_advance_err_deg == cur["L0_advance"]["threshold"]
    assert c.curriculum_step == cur["L0_advance"]["step"]
    assert c.curriculum_full == cur["L0_full"]["threshold"]


def test_curriculum_follows_the_thresholds():
    c = level0_env.Level0Config()
    env = SimpleNamespace(cfg=c, curriculum=c.curriculum_start)
    level0_env.Level0StandEnv.update_curriculum(env, c.curriculum_advance_err_deg + 0.5)
    assert env.curriculum == c.curriculum_start  # error too high: no advance
    for _ in range(100):
        level0_env.Level0StandEnv.update_curriculum(env, c.curriculum_advance_err_deg - 0.5)
    assert env.curriculum == c.curriculum_full
