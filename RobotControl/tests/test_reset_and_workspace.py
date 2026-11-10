"""Step 5 reset logic: simulated targets auto-reset on trip / workspace exit; real targets do not."""
import numpy as np

from robotcontrol.configs.joint_map import JointMap
from robotcontrol.execution.policy_runner import OnnxPolicy, PolicyRunner
from robotcontrol.interfaces.mujoco_interface import MujocoInterface
from robotcontrol.tests.fixtures import make_level0_bundle


def _runner(tmp_path, **kw):
    jm = JointMap.load()
    onnx_path, _yaml, deploy = make_level0_bundle(tmp_path, jm)
    itf = MujocoInterface(jm, scene="stand")
    itf.connect()
    return PolicyRunner(itf, OnnxPolicy(onnx_path), deploy, jm, log_dir=None, realtime=False,
                        status_hz=0, **kw), itf


def test_sim_auto_reset_defaults_on(tmp_path):
    r, itf = _runner(tmp_path)
    assert r.auto_reset is True
    itf.close()


def test_workspace_exit_triggers_reset(tmp_path):
    r, itf = _runner(tmp_path, workspace_min_z_m=5.0)  # pelvis at 1.0 m is "below the floor" -> must reset
    out = r.run(duration_s=0.2)
    assert out["resets"] >= 1
    itf.close()


def test_no_reset_inside_workspace(tmp_path):
    r, itf = _runner(tmp_path)
    out = r.run(duration_s=0.5)
    assert out["resets"] == 0 and not out["tripped"]
    itf.close()
