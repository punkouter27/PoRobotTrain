"""Command modes: head-sweep moves only the head yaw command and holds the shoulder at its default."""
import math
import threading

import numpy as np

from robotcontrol import MODELS_DIR
from robotcontrol.configs.deploy_config import DeployConfig
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.main_runner import _shoulder_default, build_parser, make_command_fn

DEPLOY = MODELS_DIR / "r1_level0_upperbody" / "deploy.yaml"


def _load():
    return DeployConfig.load(DEPLOY), JointMap.load()


def test_head_sweep_holds_shoulder_and_sweeps_head():
    deploy, jm = _load()
    fn = make_command_fn("head-sweep", deploy, jm, threading.Event())
    shoulder = _shoulder_default(deploy, jm)
    ts = np.arange(0.0, 8.0, 0.02)
    cmds = np.array([fn(t, None).copy() for t in ts])
    assert np.allclose(cmds[:, 1], shoulder)
    assert math.isclose(cmds[:, 0].max(), math.radians(30.0), rel_tol=1e-3)
    assert math.isclose(cmds[:, 0].min(), -math.radians(30.0), rel_tol=1e-3)


def test_head_sweep_amplitude_scales_head_only():
    deploy, jm = _load()
    fn = make_command_fn("head-sweep", deploy, jm, threading.Event(), amplitude=0.3)
    cmds = np.array([fn(t, None).copy() for t in np.arange(0.0, 4.0, 0.02)])
    assert np.abs(cmds[:, 0]).max() <= math.radians(9.0) + 1e-9
    assert np.allclose(cmds[:, 1], _shoulder_default(deploy, jm))


def test_parser_accepts_head_sweep():
    args = build_parser().parse_args(["--policy", "x.onnx", "--command-mode", "head-sweep"])
    assert args.command_mode == "head-sweep"
