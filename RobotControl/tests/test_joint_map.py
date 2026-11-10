import numpy as np
import pytest

from robotcontrol.configs.joint_map import JointMap, actuator_name_for
from robotcontrol.configs.deploy_config import LEVEL0_JOINTS, level0_deploy_config


def test_joint_map_loads_26_joints():
    jm = JointMap.load()
    assert jm.num_joints == 26
    assert jm.names[0] == "left_hip_pitch_joint"
    assert jm.names[25] == "head_yaw_joint"
    assert list(jm.policy_indices) == list(range(26))
    for arr in (jm.motor_slots, jm.limits_lo, jm.limits_hi, jm.kp, jm.kd, jm.effort, jm.action_scale, jm.defaults):
        assert arr.shape == (26,)
    assert jm.axes.shape == (26, 3)
    assert jm.lowcmd_topic == "rt/lowcmd" and jm.lowstate_topic == "rt/lowstate" and jm.msg_type == "unitree_hg"


def test_motor_slots_unique_and_unmapped():
    jm = JointMap.load()
    slots = jm.motor_slots.tolist()
    assert len(set(slots)) == 26
    assert all(0 <= s < jm.lowcmd_num_slots for s in slots)
    assert jm.unmapped_slots == [14, 20, 21, 27, 28]
    assert jm.motor_slots[jm.index_of("left_shoulder_pitch_joint")] == 15
    assert jm.motor_slots[jm.index_of("right_shoulder_pitch_joint")] == 22
    assert jm.motor_slots[jm.index_of("head_yaw_joint")] == 30


def test_groups_cover_all_joints():
    jm = JointMap.load()
    all_idx = sorted(i for g in jm.groups.values() for i in g)
    assert all_idx == list(range(26))
    assert jm.groups["head"] == [24, 25]
    assert len(jm.group_indices("left_leg", "right_leg")) == 12


def test_limits_and_defaults_consistent():
    jm = JointMap.load()
    assert np.all(jm.limits_lo < jm.limits_hi)
    assert np.all(jm.defaults >= jm.limits_lo) and np.all(jm.defaults <= jm.limits_hi)
    lo, hi = jm.soft_limits(0.9)
    assert np.all(lo > jm.limits_lo) and np.all(hi < jm.limits_hi)
    assert np.all(jm.defaults >= lo) and np.all(jm.defaults <= hi)
    assert np.allclose(hi - lo, 0.9 * (jm.limits_hi - jm.limits_lo))


def test_select_submap_and_slot_arrays():
    jm = JointMap.load()
    sub = jm.select_names(LEVEL0_JOINTS)
    assert sub.names == LEVEL0_JOINTS
    assert sub.motor_slots.tolist() == [22, 23, 24, 25, 26, 29, 30]
    assert list(sub.policy_indices) == list(range(7))
    assert sub.groups == {"right_arm": [0, 1, 2, 3, 4], "head": [5, 6]}
    v = np.arange(26, dtype=float)
    slots = jm.to_slot_array(v, fill=-1.0)
    assert slots.shape == (31,)
    assert all(slots[s] == -1.0 for s in jm.unmapped_slots)
    assert np.array_equal(jm.from_slot_array(slots), v)


def test_actuator_names_strip_suffix():
    assert actuator_name_for("head_yaw_joint") == "head_yaw"
    assert actuator_name_for("foo") == "foo"
    jm = JointMap.load()
    assert jm.actuator_names[25] == "head_yaw"


def test_duplicate_slot_rejected():
    jm = JointMap.load()
    import yaml
    raw = yaml.safe_load(open(jm.source_path, encoding="utf-8"))
    raw["joints"][1]["motor_slot"] = raw["joints"][0]["motor_slot"]
    with pytest.raises(ValueError):
        JointMap.from_dict(raw)


def test_level0_deploy_config_valid():
    jm = JointMap.load()
    cfg = level0_deploy_config(jm)
    assert cfg.validate(jm) == []
    assert cfg.num_obs == 23 and cfg.num_actions == 7 and cfg.command_dim == 2
    assert len(cfg.passive_joints) == 19
    rt = cfg.to_dict()
    from robotcontrol.configs.deploy_config import DeployConfig
    cfg2 = DeployConfig.from_dict(rt)
    assert cfg2.validate(jm) == []
    assert cfg2.joint_names == LEVEL0_JOINTS
