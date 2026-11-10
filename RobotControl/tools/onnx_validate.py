"""Validate ``policy.onnx`` + ``deploy.yaml`` against the joint map.

Checks: graph io names/shapes vs num_obs/num_actions, joint names exist in the joint
map, motor_slots match, per-joint list lengths, defaults inside soft limits,
action_scale sanity, observation layout sums, passive joints. Exit code 1 on failure.

    python -m robotcontrol.tools.onnx_validate models/r1_level0_upperbody/policy.onnx [--deploy deploy.yaml] [--joint-map ...]
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Optional, Tuple

from ..configs.deploy_config import DeployConfig, find_deploy_yaml
from ..configs.joint_map import JointMap
from .onnx_inspect import inspect_onnx


def validate(onnx_path: str | Path, deploy_path: Optional[str | Path] = None,
             joint_map_path: Optional[str | Path] = None, run_session: bool = True) -> Tuple[List[str], List[str]]:
    """Return ``(errors, warnings)``."""
    errors: List[str] = []
    warnings: List[str] = []
    onnx_path = Path(onnx_path)
    if not onnx_path.exists():
        return [f"onnx file not found: {onnx_path}"], warnings
    try:
        deploy_path = Path(deploy_path) if deploy_path else find_deploy_yaml(onnx_path)
    except FileNotFoundError as exc:
        return [str(exc)], warnings
    jm = JointMap.load(joint_map_path)
    try:
        deploy = DeployConfig.load(deploy_path)
    except Exception as exc:
        return [f"deploy.yaml failed to load: {exc}"], warnings

    errors += [f"deploy: {e}" for e in deploy.validate(jm)]

    info = inspect_onnx(onnx_path)
    ins, outs = info["inputs"], info["outputs"]
    if len(ins) != 1:
        errors.append(f"onnx: expected exactly 1 input, got {[i['name'] for i in ins]}")
    if len(outs) != 1:
        errors.append(f"onnx: expected exactly 1 output, got {[o['name'] for o in outs]}")
    if ins:
        i = ins[0]
        if i["name"] != "obs":
            warnings.append(f"onnx: input is named {i['name']!r}, contract says 'obs'")
        if i["dtype"] != "FLOAT":
            errors.append(f"onnx: input dtype {i['dtype']} != FLOAT")
        if len(i["shape"]) != 2:
            errors.append(f"onnx: input rank {len(i['shape'])} != 2 (expected (1, N))")
        elif i["shape"][-1] != deploy.num_obs:
            errors.append(f"onnx: input dim {i['shape'][-1]} != deploy num_obs {deploy.num_obs}")
    if outs:
        o = outs[0]
        if o["name"] != "actions":
            warnings.append(f"onnx: output is named {o['name']!r}, contract says 'actions'")
        if o["dtype"] != "FLOAT":
            errors.append(f"onnx: output dtype {o['dtype']} != FLOAT")
        if len(o["shape"]) != 2:
            errors.append(f"onnx: output rank {len(o['shape'])} != 2 (expected (1, A))")
        elif o["shape"][-1] != deploy.num_actions:
            errors.append(f"onnx: output dim {o['shape'][-1]} != deploy num_actions {deploy.num_actions}")
    if not info["normalization"].get("detected"):
        warnings.append("onnx: no Sub/Div normalization found after the input (ok only if obs are already normalized)")
    if abs(deploy.step_dt - 0.02) > 1e-9:
        warnings.append(f"deploy: step_dt={deploy.step_dt} (project standard is 0.02)")
    if not deploy.motor_slots:
        warnings.append("deploy: motor_slots missing (derived from joint map at runtime)")

    if run_session and not errors:
        try:
            import numpy as np
            from ..execution.policy_runner import OnnxPolicy
            pol = OnnxPolicy(onnx_path)
            out = pol.infer(np.zeros(pol.num_obs, dtype=np.float32))
            if not np.all(np.isfinite(out)):
                errors.append("onnx: non-finite output for a zero observation")
        except Exception as exc:
            errors.append(f"onnxruntime failed to run the model: {exc}")
    return errors, warnings


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx", type=Path)
    ap.add_argument("--deploy", type=Path, default=None)
    ap.add_argument("--joint-map", type=Path, default=None)
    ap.add_argument("--no-run", action="store_true", help="skip the onnxruntime smoke inference")
    args = ap.parse_args(argv)
    errors, warnings = validate(args.onnx, args.deploy, args.joint_map, run_session=not args.no_run)
    for w in warnings:
        print(f"WARN  {w}")
    for e in errors:
        print(f"ERROR {e}")
    if errors:
        print(f"FAIL: {len(errors)} error(s)")
        return 1
    print("PASS: onnx + deploy.yaml consistent with joint map")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
