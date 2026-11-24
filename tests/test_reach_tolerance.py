"""The WebGPU reach kernel's per-adapter tolerance (0.1 mm discrete, 0.5 mm integrated): node --test on the JS module."""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
TEST = ROOT / "src" / "PoRobotTrain.Client" / "wwwroot" / "js" / "twin" / "reach_tolerance.test.mjs"


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_reach_tolerance_per_adapter():
    p = subprocess.run(
        ["node", "--test", "--test-reporter=tap", str(TEST)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert p.returncode == 0, p.stdout + p.stderr
    assert "# fail 0" in p.stdout
