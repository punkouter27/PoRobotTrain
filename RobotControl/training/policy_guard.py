"""The export guard this repo puts in front of every new `.onnx` (RULES.md R10/R11/R23).

It shells out to `global/UNITE/tools/verify_policy.py` rather than importing it, so the repo references
the shared tool by path and hash instead of vendoring a copy (R26/R27).

Three behaviours, deliberately different:

* **legacy shape** (matches one of the recorded `legacy_interfaces`) -> warn. The existing Level 0
  (23/7), head-skill (9/2) and walk (83/24) interfaces keep exporting, because the twin cannot yet feed
  131 observations. This is the grandfathering in R22.
* **conforming shape** -> the sidecar must verify; a mismatch is fatal.
* **anything else** (neither conforming nor a known legacy interface) -> fatal. That is a new interface
  nobody signed off on, which is exactly the leak R23 closes.

`STRICT=1` (or `--strict`) turns legacy warnings into errors. Flip it the day the trainer moves to the
role-ordered interface.

The tool is optional in CI: a fresh clone has no `global/UNITE`, and then the check reports
"not verified" instead of failing the build. `contracts.lock.json` still pins the shapes, so nothing is
silently unguarded.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

ROOT = Path(__file__).resolve().parents[2]
VERIFY = ROOT.parent / "GLOBAL" / "rl_training" / "standards" / "unite" / "tools" / "verify_policy.py"

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "configs"))
import contract as contract_mod  # noqa: E402


def _run(policy: Path, creature: str, extra: List[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(VERIFY), str(policy), "--creature", creature, *extra],
        capture_output=True,
        text=True,
    )


def check(
    policy: Path, *, creature: Optional[str] = None, strict: Optional[bool] = None, write_sidecar: bool = False
) -> str:
    """Gate one exported policy. Returns the status line; raises SystemExit on a fatal problem."""
    creature = creature or contract_mod.CREATURE
    strict = os.environ.get("UNITE_STRICT", "0") == "1" if strict is None else strict
    c = contract_mod.load(creature)

    if not VERIFY.exists():
        print(
            f"policy guard: {policy.name} NOT VERIFIED — {VERIFY} is absent (no global/UNITE here). "
            f"Lock still pins n_obs {c.n_obs} / n_action {c.n_action}."
        )
        return "unverified"

    r = _run(policy, creature, ["--check"])
    conforms = r.returncode == 0

    if conforms:
        if write_sidecar:
            w = _run(policy, creature, ["--write"])
            if w.returncode != 0:
                print(w.stdout, w.stderr)
                raise SystemExit(f"policy guard: sidecar write failed for {policy.name}")
            print(f"policy guard: {policy.name} CONFORMS ({c.n_obs}/{c.n_action}); sidecar written")
        else:
            print(f"policy guard: {policy.name} CONFORMS ({c.n_obs}/{c.n_action})")
        return "conforming"

    # Not conforming. Is it a known legacy shape, or something new?
    n_obs, n_action = _shape(policy, c)
    status = c.describe_io(n_obs, n_action) if n_obs is not None else "unknown"

    if status == "legacy" and not strict:
        li = next(i for i in c.legacy_interfaces if int(i["n_obs"]) == n_obs and int(i["n_action"]) == n_action)
        print(
            f"policy guard: {policy.name} is LEGACY ({n_obs}/{n_action} = {li['name']}). "
            f"Export allowed; it is not cross-compatible and cannot be promoted (R22). "
            f"Set UNITE_STRICT=1 to make this fatal."
        )
        return "legacy"

    print(r.stdout.rstrip())
    if status == "legacy":
        raise SystemExit(f"policy guard: {policy.name} is legacy ({n_obs}/{n_action}) and strict mode is on")
    raise SystemExit(
        f"policy guard: {policy.name} does not conform and is not a known legacy interface "
        f"({n_obs}/{n_action}); see RULES.md R6/R7/R23"
    )


def _shape(policy: Path, c) -> tuple:
    """Cheap shape probe that does not need onnx imported into this process."""
    try:
        import onnx  # noqa: F401
    except ImportError:
        return None, None
    from onnx import load

    m = load(str(policy))
    g = m.graph
    init = {i.name for i in g.initializer}
    ins = [i for i in g.input if i.name not in init]
    outs = list(g.output)
    if len(ins) != 1 or len(outs) != 1:
        return None, None
    return _flat(ins[0]), _flat(outs[0])


def _flat(value_info) -> Optional[int]:
    """The single non-batch dimension of a graph tensor.

    Batch is dim 0 and may be 1 or symbolic (0), so it is dropped positionally rather than by value —
    a 1-actuator skill legitimately has an output of [batch, 1].
    """
    dims = [d.dim_value for d in value_info.type.tensor_type.shape.dim]
    body = dims[1:] if len(dims) > 1 else dims
    body = [d for d in body if d != 0]
    return body[0] if len(body) == 1 else None
