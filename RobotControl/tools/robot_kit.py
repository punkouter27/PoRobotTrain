"""Build the self-contained robot kit: one .tar.gz to copy onto the R1's Jetson.

Contents (unpacked into ~/porobot on the robot):
  robotcontrol/        the runner package (interfaces, safety guard, policy runner, tools) - no training code
  bundle/              policy.onnx + deploy.yaml + r1_joint_map.yaml + MANIFEST.json (from deploy_pack)
  wheels/cp38|cp310    offline Python wheels for linux aarch64 (numpy, pyyaml, onnxruntime)
  src/                 CycloneDDS C source + Python sdist 0.10.2, unitree_sdk2_python
  setup_robot.sh       offline install into ./.venv, then a software-only preflight
  run.sh               runs a robotcontrol module with the right environment
  KIT_MANIFEST.json    policy name, bundle checksums, build time

Usage (laptop):
  .venv/Scripts/python.exe scripts/fetch_robot_vendor.py         # once, needs internet
  .venv/Scripts/python.exe -m robotcontrol.tools.robot_kit       # newest bundle in dist/
"""
from __future__ import annotations

import argparse
import io
import json
import tarfile
import time
from pathlib import Path
from typing import List, Optional

from .. import PACKAGE_DIR, PROJECT_ROOT
from .deploy_pack import verify_bundle

EXCLUDE_DIRS = {"__pycache__", "training", "models", "tests", ".pytest_cache", "robot_kit_files"}
FILES_DIR = PACKAGE_DIR / "tools" / "robot_kit_files"


def latest_bundle(dist: Path) -> Path:
    cands = sorted([p for p in dist.iterdir() if p.is_dir() and (p / "MANIFEST.json").exists()],
                   key=lambda p: p.stat().st_mtime)
    if not cands:
        raise FileNotFoundError(f"no deploy bundle in {dist}; run robotcontrol.tools.deploy_pack first")
    return cands[-1]


def _add_text(tar: tarfile.TarFile, arcname: str, text: str, mode: int = 0o644) -> None:
    data = text.replace("\r\n", "\n").encode("utf-8")
    info = tarfile.TarInfo(arcname)
    info.size = len(data)
    info.mode = mode
    info.mtime = int(time.time())
    tar.addfile(info, io.BytesIO(data))


def _add_tree(tar: tarfile.TarFile, src: Path, arc: str, exclude=frozenset()) -> int:
    n = 0
    for p in sorted(src.rglob("*")):
        rel = p.relative_to(src)
        if any(part in exclude for part in rel.parts) or p.suffix == ".pyc":
            continue
        if p.is_file():
            tar.add(p, arcname=f"{arc}/{rel.as_posix()}", recursive=False)
            n += 1
    return n


def build(bundle: Path, out_dir: Path, vendor: Path) -> Path:
    problems = verify_bundle(bundle)
    if problems:
        raise RuntimeError(f"bundle {bundle} failed verification: {problems}")
    for need in ("wheels/cp38", "wheels/cp310", "src/unitree_sdk2_python", "src/cyclonedds"):
        if not (vendor / need).exists():
            raise FileNotFoundError(f"{vendor / need} missing - run scripts/fetch_robot_vendor.py first")
    manifest = json.loads((bundle / "MANIFEST.json").read_text(encoding="utf-8"))
    name = manifest.get("policy_name", bundle.name)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"r1_robot_kit_{name}_{time.strftime('%Y%m%d_%H%M')}.tar.gz"
    with tarfile.open(out, "w:gz") as tar:
        n_pkg = _add_tree(tar, PACKAGE_DIR, "robotcontrol", EXCLUDE_DIRS)
        _add_tree(tar, bundle, "bundle")
        _add_tree(tar, vendor / "wheels", "wheels")
        _add_tree(tar, vendor / "src", "src", {"build", "install"})
        for script in ("setup_robot.sh", "run.sh"):
            _add_text(tar, script, (FILES_DIR / script).read_text(encoding="utf-8"), mode=0o755)
        kit_manifest = {"policy_name": name, "bundle": bundle.name, "bundle_files": manifest["files"],
                        "built": time.strftime("%Y-%m-%dT%H:%M:%S"), "robotcontrol_files": n_pkg}
        _add_text(tar, "KIT_MANIFEST.json", json.dumps(kit_manifest, indent=2))
    return out


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Build the offline robot kit (.tar.gz) for the R1 Jetson")
    ap.add_argument("--bundle", type=Path, default=None, help="deploy bundle dir (default: newest in dist/)")
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT / "dist")
    ap.add_argument("--vendor", type=Path, default=PROJECT_ROOT / "vendor" / "robot")
    args = ap.parse_args(argv)
    bundle = args.bundle or latest_bundle(PROJECT_ROOT / "dist")
    out = build(bundle, args.out, args.vendor)
    print(f"robot kit: {out}  ({out.stat().st_size / 1e6:.1f} MB)  bundle: {bundle.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
