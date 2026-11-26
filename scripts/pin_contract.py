"""Re-pin `contracts.lock.json` from the UNITE contract.

    .venv/Scripts/python.exe scripts/pin_contract.py            # re-pin from global/UNITE
    .venv/Scripts/python.exe scripts/pin_contract.py --check     # exit 1 if the lock is stale

Pinning is a deliberate act (RULES.md R27). Run it only when you mean to move to a new contract, and
expect a retrain: a changed `interface_sha256` invalidates every policy trained against the old one.
The script prints what changed so that decision is visible.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOCK = ROOT / "contracts.lock.json"
CREATURE = "unitree_r1_a5"
CONTRACT = ROOT.parent / "global" / "UNITE" / "creatures" / CREATURE / "contract.json"


def build() -> dict:
    c = json.loads(CONTRACT.read_text(encoding="utf-8"))
    return {
        "_comment": "Pinned reference to the UNITE contract this repo's trainers and runtimes must match. "
        "Required fields let CI check shapes and hashes without global/UNITE present. "
        "Regenerate with scripts/pin_contract.py after a contract change (that is a breaking change).",
        "contract_version": 1,
        "creature": c["name"],
        "contract_path": f"global/UNITE/creatures/{c['name']}/contract.json",
        "n_obs": c["observation"]["n_obs"],
        "n_action": c["action"]["n_action"],
        "control_dt": c["physics"]["control_dt"],
        "decimation": c["physics"]["decimation"],
        "timestep": c["physics"]["timestep"],
        "mujoco": c["physics"]["mujoco"],
        "hashes": c["hashes"],
        "legacy_interfaces": c.get("legacy_interfaces", []),
        "hardware": {
            "owner_app": c["hardware"]["owner_app"],
            "policy_rate_hz": c["hardware"]["policy_rate_hz"],
            "num_slots": c["hardware"]["bus"]["num_slots"],
        },
        "pinned_at": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()

    if not CONTRACT.exists():
        print(f"no contract at {CONTRACT}; nothing to pin", file=sys.stderr)
        return 2

    new = build()
    old = json.loads(LOCK.read_text(encoding="utf-8")) if LOCK.exists() else None

    if a.check:
        if old is None:
            print("MISSING contracts.lock.json")
            return 1
        for k in ("n_obs", "n_action", "hashes", "legacy_interfaces", "mujoco"):
            if old.get(k) != new.get(k):
                print(f"STALE contracts.lock.json: {k} differs")
                return 1
        print("up to date contracts.lock.json")
        return 0

    if old is not None:
        for k in ("n_obs", "n_action", "mujoco"):
            if old.get(k) != new.get(k):
                print(f"CHANGED {k}: {old.get(k)} -> {new.get(k)}")
        if old.get("hashes", {}).get("interface_sha256") != new["hashes"]["interface_sha256"]:
            print("CHANGED interface_sha256 — every existing policy is now out of date; a retrain is required")
        added = [
            e["name"]
            for e in new["legacy_interfaces"]
            if e["name"] not in {x["name"] for x in old.get("legacy_interfaces", [])}
        ]
        if added:
            print(f"recorded new legacy interfaces: {added} (no retrain needed; they were already in the wild)")

    LOCK.write_text(json.dumps(new, indent=2) + "\n", encoding="utf-8")
    print(
        f"pinned {LOCK.relative_to(ROOT)}: {new['n_obs']}/{new['n_action']} "
        f"interface {new['hashes']['interface_sha256_16']}, "
        f"{len(new['legacy_interfaces'])} legacy interfaces"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
