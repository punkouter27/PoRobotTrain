"""Test helpers: build a tiny Level-0-shaped ONNX policy (with baked-in normalization)
and its deploy.yaml, without needing torch."""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from robotcontrol.configs.deploy_config import DeployConfig, level0_deploy_config
from robotcontrol.configs.joint_map import JointMap


def build_fake_onnx(path: Path, num_obs: int, num_actions: int, seed: int = 0, with_norm: bool = True,
                    weights: Optional[np.ndarray] = None, bias: Optional[np.ndarray] = None,
                    dynamic_batch: bool = False, mean: Optional[np.ndarray] = None, std: Optional[np.ndarray] = None) -> Path:
    """obs (1,N) -> [Sub mean -> Div std] -> MatMul W -> Add b -> actions (1,A)."""
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    rng = np.random.default_rng(seed)
    W = (rng.standard_normal((num_obs, num_actions)) * 0.1).astype(np.float32) if weights is None else weights.astype(np.float32)
    b = np.zeros(num_actions, dtype=np.float32) if bias is None else bias.astype(np.float32)
    mean = (rng.standard_normal(num_obs) * 0.1).astype(np.float32) if mean is None else mean.astype(np.float32)
    std = (1.0 + rng.random(num_obs)).astype(np.float32) if std is None else std.astype(np.float32)

    batch = "batch" if dynamic_batch else 1
    inp = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [batch, num_obs])
    out = helper.make_tensor_value_info("actions", TensorProto.FLOAT, [batch, num_actions])
    inits = [numpy_helper.from_array(W, "W"), numpy_helper.from_array(b, "b")]
    nodes = []
    x = "obs"
    if with_norm:
        inits += [numpy_helper.from_array(mean, "obs_mean"), numpy_helper.from_array(std, "obs_std")]
        nodes.append(helper.make_node("Sub", ["obs", "obs_mean"], ["obs_centered"]))
        nodes.append(helper.make_node("Div", ["obs_centered", "obs_std"], ["obs_norm"]))
        x = "obs_norm"
    nodes.append(helper.make_node("MatMul", [x, "W"], ["mm"]))
    nodes.append(helper.make_node("Add", ["mm", "b"], ["actions"]))
    graph = helper.make_graph(nodes, "fake_policy", [inp], [out], initializer=inits)
    model = helper.make_model(graph, producer_name="robotcontrol-tests", opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    model.metadata_props.append(onnx.StringStringEntryProto(key="policy_name", value="fake"))
    onnx.checker.check_model(model)
    path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(path))
    return path


def fake_policy_forward(path: Path, obs: np.ndarray) -> np.ndarray:
    """numpy reference of the fake graph (for replay tests)."""
    import onnx
    from onnx import numpy_helper

    m = onnx.load(str(path))
    inits = {t.name: numpy_helper.to_array(t) for t in m.graph.initializer}
    x = obs.astype(np.float32)
    if "obs_mean" in inits:
        x = (x - inits["obs_mean"]) / inits["obs_std"]
    return x @ inits["W"] + inits["b"]


def make_level0_bundle(out_dir: Path, joint_map: Optional[JointMap] = None, seed: int = 0,
                       zero_weights: bool = False, with_norm: bool = True) -> Tuple[Path, Path, DeployConfig]:
    """Write ``policy.onnx`` + ``deploy.yaml`` for the Level 0 layout; returns (onnx, yaml, cfg)."""
    jm = joint_map or JointMap.load()
    cfg = level0_deploy_config(jm, metadata={"source": "tests.fixtures", "seed": seed})
    out_dir.mkdir(parents=True, exist_ok=True)
    W = np.zeros((cfg.num_obs, cfg.num_actions), dtype=np.float32) if zero_weights else None
    onnx_path = build_fake_onnx(out_dir / "policy.onnx", cfg.num_obs, cfg.num_actions, seed=seed,
                                with_norm=with_norm, weights=W)
    yaml_path = cfg.save(out_dir / "deploy.yaml")
    return onnx_path, yaml_path, cfg
