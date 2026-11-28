"""The UNITE contract for this repo's creature -- the single source of interface shape.

RULES.md R1/R2 in `PoPunkouterSoftware/docs/unite/RULES.md`; the pinned copy is `contracts.lock.json` at the repo root.

Two modes, on purpose:

* **Lock-only (always available, including CI from a fresh clone).** `contracts.lock.json` carries
  `n_obs`, `n_action`, the three hashes, the timing and the legacy interface list. Everything that only
  needs shapes and rates works without `PoPunkouterSoftware/docs/unite` on disk.
* **Full contract (local only).** When `../PoPunkouterSoftware/docs/unite/creatures/<id>/contract.json` exists, the roles,
  joint table, motor slots and per-slot rules are loaded too, and its hashes are checked against the lock.
  A mismatch raises: the lock is stale, and pinning is a deliberate act (R27).

Nothing here imports mujoco or torch, so it is safe to import from web, test and tooling code.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[2]
LOCK_PATH = ROOT / "contracts.lock.json"
CREATURE = "unitree_r1_a5"


class ContractMismatch(RuntimeError):
    """The thing being checked does not match the pinned contract."""


def _global_contract_path(pinned: Dict[str, Any]) -> Path:
    return ROOT.parent / pinned["contract_path"]


def read_lock() -> Dict[str, Any]:
    if not LOCK_PATH.exists():
        raise ContractMismatch(
            f"{LOCK_PATH} is missing. Restore it from version control; do not hand-write interface constants."
        )
    return json.loads(LOCK_PATH.read_text(encoding="utf-8"))


def _full_contract(pinned: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    p = _global_contract_path(pinned)
    if not p.exists():
        return None
    return json.loads(p.read_text(encoding="utf-8"))


@dataclass(frozen=True)
class Contract:
    """Everything downstream reads from here instead of hard-coding numbers."""

    creature: str
    n_obs: int
    n_action: int
    control_dt: float
    decimation: int
    timestep: float
    mujoco: str
    hashes: Dict[str, str]
    policy_rate_hz: float
    legacy_interfaces: Tuple[Dict[str, Any], ...]
    full: Optional[Dict[str, Any]] = None

    # -- derived views (None when only the lock is available) -------------------------------------

    @property
    def has_full(self) -> bool:
        return self.full is not None

    @property
    def roles(self) -> Tuple[str, ...]:
        return tuple(self.full["roles"]["canonical_order"]) if self.full else ()

    @property
    def mask(self) -> Tuple[int, ...]:
        return tuple(self.full["roles"]["mask"]) if self.full else ()

    @property
    def joints(self) -> Tuple[Dict[str, Any], ...]:
        return tuple(self.full["morphology"]["joints"]) if self.full else ()

    def role_of(self, joint_name: str) -> Optional[str]:
        for j in self.joints:
            if j["name"] == joint_name:
                return j.get("role")
        return None

    def motor_slot(self, joint_name: str) -> Optional[int]:
        for j in self.joints:
            if j["name"] == joint_name:
                return j.get("motor_slot")
        return None

    # -- guards -----------------------------------------------------------------------------------

    def assert_io(self, n_obs: int, n_action: int, *, what: str = "policy") -> None:
        """R6/R7: raise when a network (or an observation builder) has the wrong width.

        Call this in every export path and every observation builder. It is the cheapest possible
        guard and it catches the most common cross-app bug.
        """
        problems = []
        if int(n_obs) != self.n_obs:
            problems.append(f"{what} n_obs={n_obs}, contract expects {self.n_obs}")
        if int(n_action) != self.n_action:
            problems.append(f"{what} n_action={n_action}, contract expects {self.n_action}")
        if problems:
            raise ContractMismatch("; ".join(problems))

    def is_legacy_io(self, n_obs: int, n_action: int) -> bool:
        return any(
            int(i["n_obs"]) == int(n_obs) and int(i["n_action"]) == int(n_action) for i in self.legacy_interfaces
        )

    def describe_io(self, n_obs: int, n_action: int) -> str:
        if (n_obs, n_action) == (self.n_obs, self.n_action):
            return "conforming"
        if self.is_legacy_io(n_obs, n_action):
            return "legacy"
        return "foreign"


def load(creature: str = CREATURE) -> Contract:
    """Load the pinned contract. Raises ContractMismatch when the lock is stale or missing."""
    pinned = read_lock()
    if pinned["creature"] != creature:
        raise ContractMismatch(f"lock pins {pinned['creature']!r}, asked for {creature!r}")

    full = _full_contract(pinned)
    if full is not None:
        for key in ("interface_sha256", "morphology_sha256", "physics_sha256"):
            if full["hashes"][key] != pinned["hashes"][key]:
                raise ContractMismatch(
                    f"contracts.lock.json is stale: {key} pinned {pinned['hashes'][key][:16]}..., "
                    f"contract has {full['hashes'][key][:16]}... . Re-pin deliberately; a contract change "
                    f"is a breaking change that needs a retrain."
                )
        if full["observation"]["n_obs"] != pinned["n_obs"] or full["action"]["n_action"] != pinned["n_action"]:
            raise ContractMismatch("lock n_obs/n_action disagree with the full contract")

    return Contract(
        creature=pinned["creature"],
        n_obs=int(pinned["n_obs"]),
        n_action=int(pinned["n_action"]),
        control_dt=float(pinned["control_dt"]),
        decimation=int(pinned["decimation"]),
        timestep=float(pinned["timestep"]),
        mujoco=str(pinned["mujoco"]),
        hashes=dict(pinned["hashes"]),
        policy_rate_hz=float(pinned.get("hardware", {}).get("policy_rate_hz", 1.0 / float(pinned["control_dt"]))),
        legacy_interfaces=tuple(pinned.get("legacy_interfaces", ())),
        full=full,
    )


def verify_lock() -> List[str]:
    """Problems with the pin itself. Empty list = the lock matches the contract on disk."""
    try:
        load()
    except ContractMismatch as e:
        return [str(e)]
    return []


if __name__ == "__main__":  # small CLI: python -m robotcontrol.configs.contract
    c = load()
    print(
        f"{c.creature}: n_obs {c.n_obs}, n_action {c.n_action}, {c.policy_rate_hz:.0f} Hz, "
        f"interface {c.hashes['interface_sha256_16']}"
    )
    print(f"full contract: {'present' if c.has_full else 'absent (lock-only mode)'}")
    if c.has_full:
        mapped = sum(c.mask)
        print(f"roles mapped {mapped}/{len(c.roles)}, joints {len(c.joints)}")
    for li in c.legacy_interfaces:
        print(f"  legacy: {li['name']} {li['n_obs']}/{li['n_action']} ({li['status']})")
    problems = verify_lock()
    print("lock: OK" if not problems else f"lock: STALE -- {problems[0]}")
