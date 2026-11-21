"""The Unreal DataAsset UR1RewardConfig mirrors RobotControl/configs/reward_config.json (Po Game Standard C10)."""

import json
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONFIG = ROOT / "RobotControl/configs/reward_config.json"
HEADER = ROOT / "UnrealProject/Source/R1Twin/Public/R1RewardConfig.h"
SOURCE = ROOT / "UnrealProject/Source/R1Twin/Private/R1RewardConfig.cpp"

FIELDS = {
    "torque_sq": "TorqueSqWeight",
    "joint_ang_vel_sq": "JointAngVelSqWeight",
    "action_rate_sq": "ActionRateSqWeight",
    "foot_slip": "FootSlipWeight",
}


def test_penalty_defaults_match_the_trainer_config():
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    header = HEADER.read_text(encoding="utf-8")
    for key, field in FIELDS.items():
        m = re.search(rf"float {field} = ([-+0-9.eE]+)f;", header)
        assert m, f"{field} default not found in {HEADER.name}"
        assert float(m.group(1)) == cfg["penalties"][key], f"{field} != penalties.{key}"


def test_curriculum_defaults_match_the_trainer_config():
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    source = SOURCE.read_text(encoding="utf-8")
    stages = [(n, float(t)) for n, t in re.findall(r'AddStage\(TEXT\("([^"]+)"\), ([-+0-9.eE]+)f', source)]
    assert stages == [(s["stage"], float(s["threshold"])) for s in cfg["curriculum"]]
