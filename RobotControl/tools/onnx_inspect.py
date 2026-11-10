"""Inspect an ONNX policy: inputs/outputs/shapes, opset, metadata_props, op histogram and
whether an observation-normalization ``Sub -> Div`` (or ``Mul``) pair follows the input.

    python -m robotcontrol.tools.onnx_inspect models/r1_level0_upperbody/policy.onnx
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np


def _dims(tensor_type) -> List[Any]:
    out: List[Any] = []
    for d in tensor_type.shape.dim:
        if d.HasField("dim_value"):
            out.append(int(d.dim_value))
        elif d.HasField("dim_param"):
            out.append(str(d.dim_param))
        else:
            out.append("?")
    return out


def inspect_onnx(path: str | Path) -> Dict[str, Any]:
    import onnx
    from onnx import TensorProto

    model = onnx.load(str(path), load_external_data=True)
    g = model.graph
    inits = {t.name: t for t in g.initializer}

    def tensor_info(vi) -> Dict[str, Any]:
        tt = vi.type.tensor_type
        return {"name": vi.name, "dtype": TensorProto.DataType.Name(tt.elem_type), "shape": _dims(tt)}

    inputs = [tensor_info(i) for i in g.input if i.name not in inits]
    outputs = [tensor_info(o) for o in g.output]
    ops = Counter(n.op_type for n in g.node)

    # normalization detection: input -> Sub(mean) -> Div(std) | Mul(inv_std)
    norm: Dict[str, Any] = {"detected": False}
    if inputs:
        in_name = inputs[0]["name"]
        consumers = [n for n in g.node if in_name in n.input]
        for n in consumers:
            if n.op_type != "Sub":
                continue
            other = [x for x in n.input if x != in_name]
            mean_name = other[0] if other else None
            sub_out = n.output[0]
            nxt = [m for m in g.node if sub_out in m.input and m.op_type in ("Div", "Mul")]
            if nxt:
                m = nxt[0]
                std_name = [x for x in m.input if x != sub_out]
                std_name = std_name[0] if std_name else None
                norm = {
                    "detected": True,
                    "pattern": f"Sub -> {m.op_type}",
                    "mean_initializer": mean_name,
                    "std_initializer": std_name,
                    "mean_shape": list(inits[mean_name].dims) if mean_name in inits else None,
                    "std_shape": list(inits[std_name].dims) if std_name in inits else None,
                }
                if std_name in inits:
                    std = onnx.numpy_helper.to_array(inits[std_name])
                    norm["std_min"] = float(np.min(std))
                    norm["std_max"] = float(np.max(std))
                break
        if not norm["detected"] and consumers:
            norm["first_consumer"] = consumers[0].op_type

    return {
        "path": str(path),
        "ir_version": int(model.ir_version),
        "producer": f"{model.producer_name} {model.producer_version}".strip(),
        "opsets": {(o.domain or "ai.onnx"): int(o.version) for o in model.opset_import},
        "metadata_props": {p.key: p.value for p in model.metadata_props},
        "inputs": inputs,
        "outputs": outputs,
        "num_nodes": len(g.node),
        "num_initializers": len(inits),
        "num_parameters": int(sum(int(np.prod(t.dims)) for t in g.initializer)),
        "ops": dict(ops.most_common()),
        "normalization": norm,
    }


def format_report(info: Dict[str, Any]) -> str:
    lines = [f"ONNX: {info['path']}",
             f"  ir_version {info['ir_version']}  producer '{info['producer']}'  opsets {info['opsets']}",
             f"  nodes {info['num_nodes']}  initializers {info['num_initializers']}  params {info['num_parameters']}"]
    lines.append("  inputs:")
    for i in info["inputs"]:
        lines.append(f"    {i['name']:12s} {i['dtype']:8s} {i['shape']}")
    lines.append("  outputs:")
    for o in info["outputs"]:
        lines.append(f"    {o['name']:12s} {o['dtype']:8s} {o['shape']}")
    lines.append(f"  ops: {info['ops']}")
    n = info["normalization"]
    if n.get("detected"):
        lines.append(f"  normalization: YES ({n['pattern']}) mean{n['mean_shape']} std{n['std_shape']}"
                     + (f" std in [{n['std_min']:.3g}, {n['std_max']:.3g}]" if "std_min" in n else ""))
    else:
        lines.append(f"  normalization: not detected after input (first consumer: {n.get('first_consumer')})")
    if info["metadata_props"]:
        lines.append("  metadata_props:")
        for k, v in info["metadata_props"].items():
            v = v if len(v) < 200 else v[:197] + "..."
            lines.append(f"    {k}: {v}")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx", type=Path)
    ap.add_argument("--json", action="store_true", help="print JSON instead of text")
    args = ap.parse_args(argv)
    info = inspect_onnx(args.onnx)
    print(json.dumps(info, indent=2) if args.json else format_report(info))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
