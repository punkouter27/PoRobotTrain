"""The role-ordered interface: 131 observations in, 28 role actions out, straight from ``contract.json``.

RULES.md R6-R8 (layout) and R13/R14 (adopt-or-refuse) in ``PoPunkouterSoftware/docs/unite/RULES.md``. This is the app-side
half of the contract: it builds the observation the creature contract defines, then turns the 28 actions
into joint targets with ``clip(default + a*scale, soft_lo, soft_hi)``. Role order, mask, defaults, gains,
scales, limits and the one-hot index all come from the pinned contract — nothing is hand-copied, so a
contract bump breaks the *load* instead of the robot.

The legacy ``ObservationBuilder`` is untouched. This path is chosen only when the ONNX is exactly
``contract.n_obs -> contract.n_action`` **and** a sidecar next to it matches; otherwise the runner keeps
using the deploy.yaml layout, which is what every grandfathered policy needs (R22).
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from ..configs import contract as C
from ..configs.joint_map import JointMap
from ..interfaces.base_interface import LowStateData, projected_gravity_from_quat

# The nine blocks of the universal observation, in the order the contract lists them.
ROLE_SLOTS = (
    "joint_pos_rel",
    "joint_vel",
    "prev_action",
    "base_ang_vel",
    "projected_gravity",
    "velocity_commands",
    "gait_phase",
    "role_mask",
    "morphology_onehot",
)
DEFAULT_GAIT_PERIOD_S = 0.8
MOVING_COMMAND_THRESHOLD = 0.1


class RoleObs:
    """131 floats, role-ordered, zero-filled where the creature has no joint for a role (R6)."""

    def __init__(self, contract: dict, joint_index: Dict[str, int]):
        roles = contract["roles"]["canonical_order"]
        self.roles: List[str] = list(roles)
        self.n_roles = len(self.roles)
        self.mask = np.asarray(contract["roles"]["mask"], dtype=np.float32)
        self.slots = {
            s["name"]: (int(s["start"]), int(s["size"]), float(s["scale"])) for s in contract["observation"]["slots"]
        }
        self.n_obs = int(contract["observation"]["n_obs"])
        self.onehot_size = int(contract["roles"].get("morphology_onehot_size", 8))
        self.morphology_id = int(contract["roles"].get("morphology_id", 0))
        self.joint_index = dict(joint_index)

        self.joint_of_role: Dict[str, int] = {}
        self.default_of_role = np.zeros(self.n_roles, dtype=np.float64)
        for j in contract["morphology"]["joints"]:
            role = j.get("role")
            i = self.joint_index.get(j["name"])
            if role in self.roles and i is not None:
                self.joint_of_role[role] = i
                self.default_of_role[self.roles.index(role)] = float(j.get("default") or 0.0)

        missing = [name for name in ROLE_SLOTS if name not in self.slots]
        if missing:
            raise ValueError(f"contract is missing observation slots {missing}")
        self.obs = np.zeros(self.n_obs, dtype=np.float32)

    def slice_of(self, obs: np.ndarray, name: str) -> np.ndarray:
        start, size, _ = self.slots[name]
        return obs[start : start + size]

    def build(
        self,
        state: LowStateData,
        last_action: np.ndarray,
        command: Optional[np.ndarray] = None,
        t: float = 0.0,
        gait_period_s: float = DEFAULT_GAIT_PERIOD_S,
    ) -> np.ndarray:
        obs = self.obs
        pos = np.zeros(self.n_roles, dtype=np.float64)
        vel = np.zeros(self.n_roles, dtype=np.float64)
        act = np.zeros(self.n_roles, dtype=np.float64)
        prev = np.asarray(last_action, dtype=np.float64).reshape(-1)

        for i, role in enumerate(self.roles):
            if self.mask[i] == 0:
                continue
            j = self.joint_of_role.get(role)
            if j is None:
                continue
            pos[i] = float(state.q[j]) - self.default_of_role[i]
            vel[i] = float(state.dq[j])
            if i < prev.shape[0]:
                act[i] = prev[i]

        cmd = np.zeros(3, dtype=np.float64)
        if command is not None:
            c = np.asarray(command, dtype=np.float64).reshape(-1)
            n = min(3, c.shape[0])
            cmd[:n] = c[:n]
        if float(np.linalg.norm(cmd)) >= MOVING_COMMAND_THRESHOLD:
            phase = (t / float(gait_period_s)) % 1.0
            gait = np.array([math.sin(2.0 * math.pi * phase), math.cos(2.0 * math.pi * phase)])
        else:
            gait = np.zeros(2)

        onehot = np.zeros(self.onehot_size, dtype=np.float64)
        if 0 <= self.morphology_id < self.onehot_size:
            onehot[self.morphology_id] = 1.0

        blocks = {
            "joint_pos_rel": pos,
            "joint_vel": vel,
            "prev_action": act,
            "base_ang_vel": np.asarray(state.gyro, dtype=np.float64).reshape(3),
            "projected_gravity": projected_gravity_from_quat(state.imu_quat),
            "velocity_commands": cmd,
            "gait_phase": gait,
            "role_mask": self.mask,
            "morphology_onehot": onehot,
        }
        for name, values in blocks.items():
            start, size, scale = self.slots[name]
            if len(values) != size:
                raise ValueError(f"slot {name}: built {len(values)} values, contract says {size}")
            obs[start : start + size] = np.asarray(values, dtype=np.float64) * scale
        return obs


class RoleActionMap:
    """28 role actions -> joint targets, and the gains that come with them (R7, layer 6)."""

    def __init__(self, contract: dict, joint_map: JointMap):
        self.roles: List[str] = list(contract["roles"]["canonical_order"])
        self.mask = np.asarray(contract["roles"]["mask"], dtype=np.float64)
        self.n_roles = len(self.roles)
        self.n_joints = int(joint_map.num_joints)
        self.index = {name: i for i, name in enumerate(joint_map.names)}

        factor = float(contract["action"].get("soft_limit_factor", 0.9))
        self.q_default = np.asarray(joint_map.defaults, dtype=np.float64).copy()
        self.kp = np.asarray(joint_map.kp, dtype=np.float64).copy()
        self.kd = np.asarray(joint_map.kd, dtype=np.float64).copy()
        self.scale = np.asarray(joint_map.action_scale, dtype=np.float64).copy()
        self.lo, self.hi = (np.asarray(a, dtype=np.float64).copy() for a in joint_map.soft_limits(factor))

        self.joint_of_role: Dict[str, int] = {}
        for j in contract["morphology"]["joints"]:
            role = j.get("role")
            i = self.index.get(j["name"])
            if role not in self.roles or i is None:
                continue
            self.joint_of_role[role] = i
            self.q_default[i] = float(j.get("default") or 0.0)
            self.kp[i] = float(j.get("kp") or 0.0)
            self.kd[i] = float(j.get("kd") or 0.0)
            self.scale[i] = float(j.get("action_scale") or 0.0)
            lo, hi = (float(x) for x in j["range"])
            pad = (1.0 - factor) * 0.5 * (hi - lo)
            self.lo[i] = lo + pad
            self.hi[i] = hi - pad

    @property
    def unmapped_joints(self) -> List[str]:
        """Joints no role drives; they are PD-held at their default."""
        driven = set(self.joint_of_role.values())
        return [n for i, n in enumerate(self.index) if i not in driven]

    def targets(self, action: np.ndarray) -> np.ndarray:
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.shape[0] != self.n_roles:
            raise ValueError(f"action has {a.shape[0]} values, the role interface wants {self.n_roles}")
        q = self.q_default.copy()
        for i, role in enumerate(self.roles):
            j = self.joint_of_role.get(role)
            if j is None or self.mask[i] == 0:
                continue
            q[j] = min(max(self.q_default[j] + a[i] * self.scale[j], self.lo[j]), self.hi[j])
        return q


class RoleInterface:
    """Both halves plus the gains, ready for ``PolicyRunner``."""

    def __init__(self, contract: dict, joint_map: JointMap):
        self.contract = contract
        self.obs = RoleObs(contract, {n: i for i, n in enumerate(joint_map.names)})
        self.action = RoleActionMap(contract, joint_map)
        self.n_obs = self.obs.n_obs
        self.n_action = self.action.n_roles
        self.roles = self.obs.roles
        self.mask = self.obs.mask
        self.kp = self.action.kp.copy()
        self.kd = self.action.kd.copy()
        self.foreign = False
        self.trained_for: Optional[str] = None
        if self.n_action != int(contract["action"]["n_action"]):
            raise ValueError(
                f"contract roles ({self.n_action}) disagree with action.n_action ({contract['action']['n_action']})"
            )

    def build(
        self,
        state: LowStateData,
        last_action: np.ndarray,
        command: Optional[np.ndarray] = None,
        t: float = 0.0,
        gait_period_s: float = DEFAULT_GAIT_PERIOD_S,
    ) -> np.ndarray:
        return self.obs.build(state, last_action, command, t, gait_period_s)

    def targets(self, action: np.ndarray) -> np.ndarray:
        return self.action.targets(action)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sidecar_problems(onnx_path: str | Path, contract: C.Contract, *, foreign: bool = False) -> List[str]:
    """What a runtime checks before it trusts a policy (R10/R14). Empty list = loadable.

    ``foreign=True`` is R13: a brain trained for another body may be driven here when the *interface*
    hash matches, because all six creatures publish one layout. Its morphology and physics hashes
    describing a different body is not a defect — it is loadable-but-not-correct, and the caller says so.
    """
    path = Path(onnx_path)
    sidecar = path.with_suffix(".json")
    if not sidecar.is_file():
        return [f"{path.name} has no sidecar ({sidecar.name}); R10 requires one beside every onnx"]
    sc = json.loads(sidecar.read_text(encoding="utf-8"))
    problems: List[str] = []
    for key in ("interface_sha256", "morphology_sha256", "physics_sha256", "policy_sha256"):
        if not sc.get(key):
            problems.append(f"sidecar is missing {key}")
    keys = ("interface_sha256",) if foreign else ("interface_sha256", "morphology_sha256", "physics_sha256")
    for key in keys:
        want = contract.hashes[key]
        got = sc.get(key)
        if got and got != want:
            problems.append(
                f"sidecar {key} = {str(got)[:16]}... but the contract has {want[:16]}... — "
                "the policy was trained against a different contract"
            )
    if sc.get("policy_sha256") and sc["policy_sha256"] != _sha256(path):
        problems.append("policy_sha256 does not match the onnx; the file changed after the sidecar was written")
    if int(sc.get("n_obs", -1)) != contract.n_obs or int(sc.get("n_action", -1)) != contract.n_action:
        problems.append(
            f"sidecar io {sc.get('n_obs')}/{sc.get('n_action')} != contract {contract.n_obs}/{contract.n_action}"
        )
    return problems


def for_policy(policy, joint_map: JointMap, contract: Optional[C.Contract] = None) -> Optional[RoleInterface]:
    """The role interface for a conforming ONNX, or ``None`` to keep the legacy deploy.yaml path.

    A brain trained for a different creature is accepted when its interface hash matches (R13) and is
    marked ``foreign``: loadable here, correct only on the body it was trained for.

    Raises ``ContractMismatch`` when the ONNX *is* the contract's shape but its sidecar does not match:
    that is a policy the runtime must refuse, not silently degrade (R14).
    """
    c = contract if contract is not None else C.load()
    if not c.has_full:
        return None  # lock-only install (fresh clone / CI): no roles, no role path
    if (int(policy.num_obs), int(policy.num_actions)) != (c.n_obs, c.n_action):
        return None
    sidecar = Path(policy.path).with_suffix(".json")
    sc = json.loads(sidecar.read_text(encoding="utf-8")) if sidecar.is_file() else {}
    foreign = bool(sc.get("creature")) and sc["creature"] != c.creature
    problems = sidecar_problems(policy.path, c, foreign=foreign)
    if problems:
        raise C.ContractMismatch(f"role policy refused: {problems[0]}")
    iface = RoleInterface(c.full, joint_map)
    iface.foreign = foreign
    iface.trained_for = sc.get("creature")
    return iface
