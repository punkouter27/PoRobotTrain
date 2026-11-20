"""web/build.py builds the showcase (into a temp dir) and every /diag check points at a file it built."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def test_web_build_check():
    p = subprocess.run(
        [sys.executable, "web/build.py", "--check"], cwd=ROOT, capture_output=True, text=True, timeout=120, check=False
    )
    assert p.returncode == 0, p.stdout + p.stderr
    assert "web build OK (v" in p.stdout
