"""Run the whole test-suite: ``python -m robotcontrol.tests.run_all [pytest args]``."""
from __future__ import annotations

import sys
from pathlib import Path


def main(argv=None) -> int:
    try:
        import pytest
    except ImportError:
        print("pytest missing: uv pip install --python .venv/Scripts/python.exe pytest")
        return 2
    here = Path(__file__).resolve().parent
    args = [str(here), "--import-mode=importlib", "-q"] + list(argv or sys.argv[1:])
    return pytest.main(args)


if __name__ == "__main__":
    raise SystemExit(main())
