"""The pinned contract is the single source of interface shape (RULES.md R1/R2/R6/R7).

These tests exist because every cross-app bug this repo can have starts with a number that was
hard-coded in one place and changed in another. They are fast and need no mujoco/torch.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from robotcontrol.configs import contract as C


def test_lock_matches_the_contract_on_disk():
    assert C.verify_lock() == []


def test_shape_and_rate_come_from_the_lock():
    c = C.load()
    assert (c.n_obs, c.n_action) == (131, 28)
    assert c.policy_rate_hz == pytest.approx(50.0)
    assert c.control_dt == pytest.approx(0.02)
    assert c.decimation == 4
    assert c.timestep == pytest.approx(0.005)
    assert c.hashes["interface_sha256_16"] == "94377ebae772ff6c"


def test_assert_io_accepts_the_contract_width_and_rejects_everything_else():
    c = C.load()
    c.assert_io(131, 28)  # does not raise
    with pytest.raises(C.ContractMismatch) as e:
        c.assert_io(125, 29)
    assert "n_obs" in str(e.value) and "n_action" in str(e.value)


def test_every_existing_interface_is_classified():
    c = C.load()
    assert c.describe_io(131, 28) == "conforming"
    # the three shapes that are already deployed and grandfathered (R22)
    assert c.describe_io(23, 7) == "legacy", "Level 0 upper body"
    assert c.describe_io(9, 2) == "legacy", "two-actuator head skills"
    assert c.describe_io(6, 1) == "legacy", "single-actuator head skills (nod_yes, shake_no)"
    assert c.describe_io(83, 24) == "legacy", "walk sandbox"
    # not a known interface: a new shape nobody signed off on (R23)
    assert c.describe_io(125, 29) == "foreign"


def test_full_contract_exposes_roles_and_motor_slots():
    c = C.load()
    if not c.has_full:
        pytest.skip("global/UNITE not present in this checkout (lock-only mode)")
    assert len(c.roles) == 28
    assert sum(c.mask) == 26
    assert len(c.joints) == 26
    assert c.role_of("left_hip_pitch_joint") == "hip_pitch_l"
    assert c.motor_slot("left_hip_pitch_joint") == 0
    assert c.motor_slot("head_yaw_joint") == 30


def test_lock_only_mode_still_answers_shapes(monkeypatch):
    """CI has no global/UNITE, so the lock alone must be enough for the guards."""
    monkeypatch.setattr(C, "_global_contract_path", lambda pinned: Path("Z:/definitely/absent.json"))
    c = C.load()
    assert c.has_full is False
    assert (c.n_obs, c.n_action) == (131, 28)
    assert c.roles == () and c.joints == ()
    c.assert_io(131, 28)
    assert c.describe_io(23, 7) == "legacy"


def test_a_stale_lock_is_refused(tmp_path, monkeypatch):
    pinned = json.loads(C.LOCK_PATH.read_text(encoding="utf-8"))
    pinned["hashes"]["interface_sha256"] = "0" * 64
    bad = tmp_path / "contracts.lock.json"
    bad.write_text(json.dumps(pinned), encoding="utf-8")
    monkeypatch.setattr(C, "LOCK_PATH", bad)
    real = C.ROOT.parent / "global" / "UNITE" / "creatures" / C.CREATURE / "contract.json"
    if not real.exists():
        pytest.skip("needs the full contract to prove the hash comparison")
    with pytest.raises(C.ContractMismatch) as e:
        C.load()
    assert "stale" in str(e.value)
