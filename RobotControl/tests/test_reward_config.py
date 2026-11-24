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


def test_each_penalty_term_is_computed_and_weighted_separately():
    import numpy as np

    c = level0_env.Level0Config()
    frc = np.array([[2.0, 0.0], [0.0, 0.0]])
    dq = np.array([[0.0, 0.0], [3.0, 0.0]])
    a, prev = np.array([[1.0, 0.0], [0.0, 0.0]]), np.zeros((2, 2))
    pen = level0_env.penalty_terms(c, frc, dq, a, prev, foot_slip_sq=np.array([0.0, 0.5]))
    assert tuple(pen) == level0_env.PENALTY_KEYS
    np.testing.assert_allclose(pen["pen_torque_sq"], [4.0 * c.w_torque, 0.0])
    np.testing.assert_allclose(pen["pen_joint_ang_vel_sq"], [0.0, 9.0 * c.w_joint_vel])
    np.testing.assert_allclose(pen["pen_action_rate_sq"], [c.w_action_rate, 0.0])
    np.testing.assert_allclose(pen["pen_foot_slip"], [0.0, 0.5 * c.w_foot_slip])
    # on the stand there is no foot contact: foot slip is logged as 0
    assert not level0_env.penalty_terms(c, frc, dq, a, prev)["pen_foot_slip"].any()
    line = level0_env.format_penalties([{k: 1.0 for k in level0_env.PENALTY_KEYS}])
    assert all(k in line for k in level0_env.PENALTY_KEYS)


def test_level0_step_logs_every_penalty_term():
    import numpy as np

    env = level0_env.Level0StandEnv(level0_env.Level0Config(num_envs=2, nthread=1, episode_length_s=0.4))
    try:
        _, rew, _, info = env.step(np.zeros((2, env.num_actions)))
        assert np.isfinite(rew).all()
        for k in level0_env.PENALTY_KEYS:
            assert k in info and np.isfinite(info[k]) and info[k] <= 0.0
        assert info["pen_foot_slip"] == 0.0  # Level 0 hangs on the stand
        assert info["pen_torque_sq"] == info["r_tau"]  # the logged term is the one added to the reward
    finally:
        env.close()
