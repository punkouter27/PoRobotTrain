"""Download everything the R1's Jetson needs to run the policy runner OFFLINE into vendor/robot/.

  wheels/cp38   numpy, pyyaml, onnxruntime (+deps) for Python 3.8, linux aarch64 (Ubuntu 20.04 / JetPack 5)
  wheels/cp310  same for Python 3.10 (Ubuntu 22.04 / JetPack 6)
  src/cyclonedds-0.10.2.tar.gz   Python binding source (no aarch64 wheel exists on PyPI)
  src/cyclonedds/                C library, branch releases/0.10.x (built on the robot only if not preinstalled)
  src/unitree_sdk2_python/       Unitree's Python SDK (copied from external/)

Needs internet on the laptop and `uv` (uses `uvx pip download`). Run once; robot_kit.py packs the result.
  .venv/Scripts/python.exe scripts/fetch_robot_vendor.py
"""
from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parents[1]
VENDOR = ROOT / "vendor/robot"
PKGS = ["pip<25.1", "numpy<2", "pyyaml", "onnxruntime<1.20",
        "rich-click", "setuptools<70", "wheel"]  # pip: Ubuntu 20.04 pip 20.0 cannot read manylinux_2_28 tags; last three: cyclonedds 0.10.2 build + runtime deps
PLATFORMS = ["manylinux2014_aarch64", "manylinux_2_17_aarch64", "manylinux_2_28_aarch64"]


def run(cmd):
    print("$", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main():
    (VENDOR / "src").mkdir(parents=True, exist_ok=True)
    for py in ("38", "310"):
        dest = VENDOR / "wheels" / f"cp{py}"
        dest.mkdir(parents=True, exist_ok=True)
        # Run pip under the TARGET Python version so environment markers (e.g. typing-extensions for
        # python_version < "3.11") resolve as they will on the Jetson, not as on this laptop.
        cmd = ["uvx", "--python", f"{py[0]}.{py[1:]}", "pip", "download", "--only-binary=:all:",
               "--python-version", py, "--implementation", "cp", "-d", str(dest)]
        for p in PLATFORMS:
            cmd += ["--platform", p]
        run(cmd + PKGS)

    meta = json.load(urllib.request.urlopen("https://pypi.org/pypi/cyclonedds/0.10.2/json"))
    url = next(u["url"] for u in meta["urls"] if u["packagetype"] == "sdist")
    sdist = VENDOR / "src" / pathlib.Path(url).name
    if not sdist.exists():
        print("downloading", url)
        urllib.request.urlretrieve(url, sdist)

    cdds = VENDOR / "src" / "cyclonedds"
    if not cdds.exists():
        run(["git", "clone", "-q", "--depth", "1", "-b", "releases/0.10.x",
             "https://github.com/eclipse-cyclonedds/cyclonedds.git", str(cdds)])
        shutil.rmtree(cdds / ".git", ignore_errors=True)

    sdk_src = ROOT / "external/unitree_sdk2_python"
    sdk_dst = VENDOR / "src" / "unitree_sdk2_python"
    if sdk_dst.exists():
        shutil.rmtree(sdk_dst)
    shutil.copytree(sdk_src, sdk_dst, ignore=shutil.ignore_patterns(".git", "__pycache__", "*.egg-info"))
    print("vendor ready:", VENDOR)


if __name__ == "__main__":
    main()
