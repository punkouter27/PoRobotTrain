"""robotcontrol -- Unitree R1 sim-to-real control package.

Layout (the ``RobotControl/`` directory is the package root):

* ``configs``       joint map (single source of truth) + deploy.yaml loader
* ``interfaces``    hardware abstraction: MuJoCo twin, URLab (UE5), Unitree SDK2
* ``execution``     safety guard, ONNX policy runner, policy blender
* ``tools``         ONNX inspect / validate / replay, deploy packaging, sim-real diff
* ``perception``, ``state_machine``, ``training``  stubs owned by other work-streams
"""
from __future__ import annotations

from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
CONFIGS_DIR = PACKAGE_DIR / "configs"
MODELS_DIR = PACKAGE_DIR / "models"
LOGS_DIR = PROJECT_ROOT / "logs"
MJCF_DIR = PROJECT_ROOT / "UnrealProject" / "MJCF"  # outside Content/: UE must not auto-import MJCF or URLab *_ue.xml

__version__ = "0.1.0"
__all__ = ["PACKAGE_DIR", "PROJECT_ROOT", "CONFIGS_DIR", "MODELS_DIR", "LOGS_DIR", "MJCF_DIR"]
