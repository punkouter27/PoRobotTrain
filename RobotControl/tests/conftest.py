"""pytest bootstrap: make ``robotcontrol`` importable even without the editable install
(the directory is named RobotControl but the package is robotcontrol)."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _bootstrap() -> None:
    try:
        import robotcontrol  # noqa: F401
        return
    except ImportError:
        pass
    spec = importlib.util.spec_from_file_location(
        "robotcontrol", PACKAGE_ROOT / "__init__.py", submodule_search_locations=[str(PACKAGE_ROOT)]
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules["robotcontrol"] = mod
    spec.loader.exec_module(mod)


_bootstrap()
