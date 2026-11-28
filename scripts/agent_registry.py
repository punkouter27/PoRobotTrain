"""Build `agents/registry.json` -- every policy in this repo, keyed by content hash.

    .venv/Scripts/python.exe scripts/agent_registry.py          # write agents/registry.json
    .venv/Scripts/python.exe scripts/agent_registry.py --check   # exit 1 if it is stale

Why by hash: the same brain is checked in at several paths (a `policy.onnx` and its `versions/v1`
copy, and the web twin's copy). Grouping by path overcounts; grouping by sha256 gives the real number
of distinct policies, which is what a migration has to reason about.

Status is decided against the pinned contract (`RobotControl/configs/contract.py`):
  * `conforming` -- n_obs/n_action equal the contract (R6/R7);
  * `legacy`     -- matches a recorded `legacy_interfaces` entry; kept, never promoted (R22);
  * `foreign`    -- neither; should not exist, and is reported as a problem (R23).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "agents" / "registry.json"
SKIP = ("\\.venv", "\\Library", "\\Intermediate", "\\Saved", "\\DerivedDataCache", "\\node_modules", "\\.git")

sys.path.insert(0, str(ROOT / "RobotControl" / "configs"))
import contract as contract_mod  # noqa: E402


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sidecar_shape(policy: Path):
    """n_obs / n_action from a neighbouring deploy.json or deploy.yaml, if one exists."""
    for name in ("deploy.json", "deploy.yaml"):
        p = policy.with_name(name)
        if not p.exists():
            continue
        if p.suffix == ".json":
            d = json.loads(p.read_text(encoding="utf-8"))
        else:
            import yaml

            d = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        if "num_obs" in d and "num_actions" in d:
            return int(d["num_obs"]), int(d["num_actions"]), p.relative_to(ROOT).as_posix()
    return None, None, None


def _sidecar_interface(paths):
    """interface_sha256 from a UNITE sidecar beside any copy of the policy (policy.unite.json / policy.json)."""
    for rel in paths:
        p = ROOT / rel
        for sc in (p.with_suffix(".unite.json"), p.with_suffix(".json")):
            if sc.exists():
                try:
                    d = json.loads(sc.read_text(encoding="utf-8"))
                except ValueError:
                    continue
                if "interface_sha256" in d:
                    return d["interface_sha256"]
    return None


def onnx_shape(policy: Path):
    try:
        from onnx import load
    except ImportError:
        return None, None
    g = load(str(policy)).graph
    init = {i.name for i in g.initializer}
    ins = [i for i in g.input if i.name not in init]
    outs = list(g.output)
    if len(ins) != 1 or len(outs) != 1:
        return None, None

    def flat(vi):
        # dim 0 is batch: 1, or symbolic ('batch' -> dim_value 0). Drop it by position, as policy_guard does.
        dims = [d.dim_value for d in vi.type.tensor_type.shape.dim][1:]
        return dims[0] if len(dims) == 1 and dims[0] > 0 else None

    return flat(ins[0]), flat(outs[0])


def _tracked_onnx():
    """Policies under version control only, so the registry is the same locally and in CI.

    Untracked files (vendored third-party exports under external/, scratch runs under logs/) are not this
    repo's policies. Falls back to a disk walk when git is unavailable."""
    import subprocess

    try:
        out = subprocess.run(
            ["git", "ls-files", "-z", "--", "*.onnx"], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout
        return sorted(ROOT / f for f in out.split("\0") if f)
    except (OSError, subprocess.CalledProcessError):
        return sorted(ROOT.rglob("*.onnx"))


def build() -> dict:
    c = contract_mod.load()
    policies = {}
    for p in _tracked_onnx():
        rel = p.relative_to(ROOT).as_posix()
        if any(
            part in p.parts
            for part in (".venv", "Library", "Intermediate", "Saved", "DerivedDataCache", "node_modules")
        ):
            continue
        digest = sha256(p)
        entry = policies.setdefault(digest, {"sha256": digest, "paths": [], "sidecars": []})
        entry["paths"].append(rel)
        n_obs, n_action, sc = sidecar_shape(p)
        if n_obs is None:
            n_obs, n_action = onnx_shape(p)
        if n_obs is not None:
            entry.setdefault("n_obs", n_obs)
            entry.setdefault("n_action", n_action)
        if sc:
            entry["sidecars"].append(sc)

    out = []
    for digest, e in sorted(policies.items(), key=lambda kv: kv[1]["paths"][0]):
        n_obs = e.get("n_obs")
        n_action = e.get("n_action")
        if n_obs is None or n_action is None:
            status, legacy_name = "foreign", None
        elif (n_obs, n_action) == (c.n_obs, c.n_action):
            # Same shape is not the same interface: a UNITE sidecar records which layout it was exported
            # against. 131/28 policies pinned to an older contract are legacy (R13/R22), not conforming.
            pinned = _sidecar_interface(e["paths"])
            if pinned is not None and pinned != c.hashes["interface_sha256"]:
                status, legacy_name = "legacy", f"unite_131_28@{pinned[:16]}"
            else:
                status, legacy_name = "conforming", None
        else:
            match = [i for i in c.legacy_interfaces if int(i["n_obs"]) == n_obs and int(i["n_action"]) == n_action]
            status = "legacy" if match else "foreign"
            legacy_name = match[0]["name"] if match else None
        primary = e["paths"][0]
        name = Path(primary).parent.name
        out.append(
            {
                "name": name,
                "sha256": digest,
                "n_obs": n_obs,
                "n_action": n_action,
                "status": status,
                "legacy_interface": legacy_name,
                "paths": e["paths"],
                "sidecars": e["sidecars"],
                "note": "not cross-compatible; do not reuse"
                if status != "conforming"
                else "conforming to the pinned contract",
            }
        )

    return {
        "_comment": "Generated by scripts/agent_registry.py. Every distinct policy in this repo, keyed by "
        "sha256 so duplicate copies collapse. status=legacy policies are grandfathered (R22): "
        "kept and runnable, never promoted, never ported.",
        "contract": {
            "creature": c.creature,
            "n_obs": c.n_obs,
            "n_action": c.n_action,
            "interface_sha256_16": c.hashes["interface_sha256_16"],
        },
        "policies": out,
        "summary": {
            "files_on_disk": sum(len(e["paths"]) for e in out),
            "distinct_policies": len(out),
            "conforming": sum(1 for e in out if e["status"] == "conforming"),
            "legacy": sum(1 for e in out if e["status"] == "legacy"),
            "foreign": sum(1 for e in out if e["status"] == "foreign"),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()

    data = build()
    text = json.dumps(data, indent=2) + "\n"
    if a.check:
        if not OUT.exists() or OUT.read_text(encoding="utf-8") != text:
            print("STALE agents/registry.json")
            return 1
        print("up to date agents/registry.json")
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding="utf-8")
    s = data["summary"]
    print(
        f"wrote {OUT.relative_to(ROOT)}: {s['distinct_policies']} distinct policies from "
        f"{s['files_on_disk']} files — {s['conforming']} conforming, {s['legacy']} legacy, {s['foreign']} foreign"
    )
    for e in data["policies"]:
        print(f"  {e['status']:10s} {e['n_obs']}/{e['n_action']}  {e['name']:22s} {e['sha256'][:12]}")
    if s["foreign"]:
        print("\nFOREIGN policies are neither conforming nor a known legacy interface — investigate (R23).")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
