"""Bundle a policy for deployment::

    dist/<policy_name>_<yyyymmdd_hhmm>/
        policy.onnx (+ policy.onnx.data if external data is used)
        deploy.yaml
        r1_joint_map.yaml
        MANIFEST.json   (sha256 + size per file, timestamp, policy_name, metadata)

    python -m robotcontrol.tools.deploy_pack models/r1_level0_upperbody/policy.onnx [--out dist] [--scp user@host:/path]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

from .. import PROJECT_ROOT
from ..configs.deploy_config import DeployConfig, find_deploy_yaml
from ..configs.joint_map import DEFAULT_JOINT_MAP_PATH


def sha256_file(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def pack(onnx_path: str | Path, deploy_path: Optional[str | Path] = None, joint_map_path: Optional[str | Path] = None,
         out_dir: str | Path = PROJECT_ROOT / "dist", validate: bool = True, timestamp: Optional[datetime] = None,
         extra_files: Optional[List[Path]] = None) -> Path:
    onnx_path = Path(onnx_path).resolve()
    deploy_path = Path(deploy_path) if deploy_path else find_deploy_yaml(onnx_path)
    joint_map_path = Path(joint_map_path) if joint_map_path else DEFAULT_JOINT_MAP_PATH
    deploy = DeployConfig.load(deploy_path)
    if validate:
        from .onnx_validate import validate as _validate
        errors, _ = _validate(onnx_path, deploy_path, joint_map_path)
        if errors:
            raise SystemExit("refusing to pack an invalid policy:\n  " + "\n  ".join(errors))

    ts = (timestamp or datetime.now()).strftime("%Y%m%d_%H%M")
    bundle = Path(out_dir) / f"{deploy.policy_name}_{ts}"
    bundle.mkdir(parents=True, exist_ok=True)

    files: Dict[str, Path] = {"policy.onnx": onnx_path, "deploy.yaml": deploy_path, "r1_joint_map.yaml": joint_map_path}
    ext = onnx_path.with_name(onnx_path.name + ".data")
    if ext.exists():
        files["policy.onnx.data"] = ext
    for p in extra_files or []:
        files[Path(p).name] = Path(p)

    manifest = {
        "policy_name": deploy.policy_name,
        "created": datetime.now().isoformat(timespec="seconds"),
        "step_dt": deploy.step_dt,
        "num_obs": deploy.num_obs,
        "num_actions": deploy.num_actions,
        "joint_names": deploy.joint_names,
        "metadata": deploy.metadata,
        "files": {},
    }
    for name, src in files.items():
        dst = bundle / name
        shutil.copyfile(src, dst)
        manifest["files"][name] = {"sha256": sha256_file(dst), "bytes": dst.stat().st_size, "source": str(src)}
    with open(bundle / "MANIFEST.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    return bundle


def verify_bundle(bundle: str | Path) -> List[str]:
    bundle = Path(bundle)
    with open(bundle / "MANIFEST.json", "r", encoding="utf-8") as f:
        manifest = json.load(f)
    problems = []
    for name, meta in manifest["files"].items():
        p = bundle / name
        if not p.exists():
            problems.append(f"missing {name}")
        elif sha256_file(p) != meta["sha256"]:
            problems.append(f"sha256 mismatch {name}")
    return problems


def scp_bundle(bundle: Path, dest: str) -> int:
    cmd = ["scp", "-r", str(bundle), dest]
    print("+", " ".join(cmd))
    return subprocess.call(cmd)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx", type=Path)
    ap.add_argument("--deploy", type=Path, default=None)
    ap.add_argument("--joint-map", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT / "dist")
    ap.add_argument("--no-validate", action="store_true")
    ap.add_argument("--scp", default=None, help="user@host:/path -- copy the bundle with system scp")
    ap.add_argument("--extra", type=Path, nargs="*", default=None, help="additional files to include")
    args = ap.parse_args(argv)
    bundle = pack(args.onnx, args.deploy, args.joint_map, args.out, validate=not args.no_validate, extra_files=args.extra)
    print(f"bundle: {bundle}")
    for name, meta in json.load(open(bundle / "MANIFEST.json", encoding="utf-8"))["files"].items():
        print(f"  {name:22s} {meta['bytes']:>10d} B  sha256 {meta['sha256'][:16]}...")
    if args.scp:
        return scp_bundle(bundle, args.scp)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
