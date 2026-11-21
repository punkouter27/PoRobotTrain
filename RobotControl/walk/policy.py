"""Level 2 velocity policy: ONNX + deploy.yaml, observation builder, and the validation marker.

Contract (Unitree mjlab ``r1_velocity``, docs/service_api.md section 3): obs 83 =
base_ang_vel(3), projected_gravity(3), velocity_commands(3), gait_phase(2), joint_pos_rel(24), joint_vel_rel(24),
last_action(24); 24 actions in joint-map policy_index 0..23 order; target = default + action_scale * action.
The term order, dims, scales and clips come from deploy.yaml ``obs_terms``; this module only knows how to compute
each named term from simulator (or robot) signals.

A policy is *available* to the sandbox only when ``validate_level2.py`` passed for exactly this file: deploy.yaml
carries ``validation: {passed: true, policy_sha256: <sha of policy.onnx>}``.
"""

from __future__ import annotations

import hashlib
import pathlib
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import yaml

from .. import PROJECT_ROOT

MODEL_DIR = PROJECT_ROOT / "RobotControl/models/r1_level2_velocity"
NUM_POLICY_JOINTS = 24
TERM_DIMS = {
    "base_ang_vel": 3,
    "projected_gravity": 3,
    "velocity_commands": 3,
    "gait_phase": 2,
    "joint_pos_rel": NUM_POLICY_JOINTS,
    "joint_vel_rel": NUM_POLICY_JOINTS,
    "last_action": NUM_POLICY_JOINTS,
}
# mjlab's own term names (as written by scripts/level2/export_level2.py) -> ours
TERM_ALIASES = {
    "command": "velocity_commands",
    "phase": "gait_phase",
    "joint_pos": "joint_pos_rel",
    "joint_vel": "joint_vel_rel",
    "actions": "last_action",
}


@dataclass
class ObsSignals:
    """Raw signals for one observation. Frames: angular velocity in the pelvis (base) frame."""

    base_ang_vel: np.ndarray  # (3,) rad/s, body frame
    base_quat: np.ndarray  # (4,) w x y z, world <- body
    command: np.ndarray  # (3,) vx, vy m/s, wz rad/s
    phase_time: float  # s since the gait clock started
    q: np.ndarray  # (24,) rad, policy joints
    dq: np.ndarray  # (24,) rad/s
    last_action: np.ndarray  # (24,)


def quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate world vector v into the body frame of unit quaternion q (w, x, y, z)."""
    w, x, y, z = q_wxyz
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    return r.T @ v


@dataclass
class Level2Deploy:
    joint_names: list
    default: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    action_scale: np.ndarray
    obs_terms: list  # [{"name", "dim", "scale" (scalar|list), "clip" ([lo, hi]|None), ...}]
    step_dt: float = 0.02
    physics_dt: float = 0.005
    gait_period: float = 0.6
    gait_order: tuple = ("sin", "cos")
    cmd_limits: dict = field(default_factory=lambda: {"vx": [-0.5, 1.0], "vy": [-0.5, 0.5], "wz": [-1.0, 1.0]})
    zero_phase_below: Optional[float] = None  # if set: gait_phase obs = 0 when |command| is below this (standing)
    action_clip: Optional[float] = None
    raw: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: pathlib.Path) -> Level2Deploy:
        d = yaml.safe_load(path.read_text(encoding="utf-8"))
        terms = [dict(t, name=TERM_ALIASES.get(t["name"], t["name"])) for t in d["obs_terms"]]
        for t in terms:
            if t["name"] not in TERM_DIMS:
                raise ValueError(f"unsupported observation term {t['name']!r} in {path}")
            if int(t.get("dim", TERM_DIMS[t["name"]])) != TERM_DIMS[t["name"]]:
                raise ValueError(f"term {t['name']} dim {t.get('dim')} != {TERM_DIMS[t['name']]}")
        gp = d.get("gait_phase", {}) or {}
        cr = d.get("command_ranges") or d.get("commands") or {}
        limits = {
            "vx": list(cr.get("lin_vel_x", [-0.5, 1.0])),
            "vy": list(cr.get("lin_vel_y", [-0.5, 0.5])),
            "wz": list(cr.get("ang_vel_z", [-1.0, 1.0])),
        }
        return cls(
            joint_names=list(d["joint_names"]),
            default=np.asarray(d["default_joint_pos"], dtype=np.float64),
            kp=np.asarray(d["kp"], dtype=np.float64),
            kd=np.asarray(d["kd"], dtype=np.float64),
            action_scale=np.asarray(d["action_scale"], dtype=np.float64) * np.ones(NUM_POLICY_JOINTS),
            obs_terms=terms,
            step_dt=float(d.get("step_dt", 0.02)),
            physics_dt=float(d.get("physics_dt", 0.005)),
            gait_period=float(gp.get("period", d.get("gait_period", 0.6))),
            gait_order=tuple(gp.get("order", ("sin", "cos"))),
            cmd_limits=limits,
            # mjlab: both phase values are 0 while |command| < standing_threshold (the robot stands still)
            zero_phase_below=gp.get("zero_below_command", cr.get("standing_threshold")),
            action_clip=d.get("action_clip"),
            raw=d,
        )

    @property
    def num_obs(self) -> int:
        return sum(TERM_DIMS[t["name"]] for t in self.obs_terms)


def build_obs(dep: Level2Deploy, s: ObsSignals) -> np.ndarray:
    parts = []
    for t in dep.obs_terms:
        n = t["name"]
        if n == "base_ang_vel":
            v = np.asarray(s.base_ang_vel, dtype=np.float64)
        elif n == "projected_gravity":
            v = quat_rotate_inverse(np.asarray(s.base_quat, dtype=np.float64), np.array([0.0, 0.0, -1.0]))
        elif n == "velocity_commands":
            v = np.asarray(s.command, dtype=np.float64)
        elif n == "gait_phase":
            ph = (s.phase_time % dep.gait_period) / dep.gait_period
            vals = {"sin": np.sin(2 * np.pi * ph), "cos": np.cos(2 * np.pi * ph)}
            v = np.array([vals[k] for k in dep.gait_order])
            if dep.zero_phase_below is not None and np.linalg.norm(s.command) < float(dep.zero_phase_below):
                v = np.zeros(2)
        elif n == "joint_pos_rel":
            v = np.asarray(s.q, dtype=np.float64) - dep.default
        elif n == "joint_vel_rel":
            v = np.asarray(s.dq, dtype=np.float64)
        else:  # last_action
            v = np.asarray(s.last_action, dtype=np.float64)
        v = v * np.asarray(t.get("scale", 1.0), dtype=np.float64)
        if t.get("clip") is not None:
            v = np.clip(v, t["clip"][0], t["clip"][1])
        parts.append(v)
    return np.concatenate(parts).astype(np.float32)


def rel_path(p: pathlib.Path) -> str:
    """Repo-relative posix path for messages (Python 3.8: no Path.is_relative_to)."""
    try:
        return pathlib.Path(p).resolve().relative_to(PROJECT_ROOT.resolve()).as_posix()
    except ValueError:
        return str(p)


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def policy_status(model_dir: pathlib.Path = MODEL_DIR) -> tuple[bool, Optional[str]]:
    """(available, reason-if-not). Available = validated by validate_level2.py for exactly this policy.onnx."""
    rel = rel_path(model_dir)
    onnx_p, dep_p = model_dir / "policy.onnx", model_dir / "deploy.yaml"
    if not onnx_p.exists() or not dep_p.exists():
        return False, f"no validated Level 2 policy in {rel} (train it, then run validate_level2.py --promote)"
    try:
        d = yaml.safe_load(dep_p.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as e:
        return False, f"unreadable {rel}/deploy.yaml: {e}"
    v = d.get("validation") or {}
    if not v.get("passed"):
        return False, f"{rel}/deploy.yaml has no passed validation (run validate_level2.py)"
    if v.get("policy_sha256") != sha256(onnx_p):
        return False, f"{rel}/policy.onnx changed after validation (re-run validate_level2.py)"
    return True, None


class Level2Policy:
    """ONNX session + deploy contract. obs (83,) -> action (24,)."""

    def __init__(self, model_dir: pathlib.Path = MODEL_DIR, require_validated: bool = True):
        import onnxruntime as ort

        if require_validated:
            ok, reason = policy_status(model_dir)
            if not ok:
                raise RuntimeError(reason)
        self.dir = model_dir
        self.deploy = Level2Deploy.load(model_dir / "deploy.yaml")
        self.sess = ort.InferenceSession(str(model_dir / "policy.onnx"), providers=["CPUExecutionProvider"])
        i, o = self.sess.get_inputs()[0], self.sess.get_outputs()[0]
        if i.name != "obs" or o.name != "actions":
            raise RuntimeError(f"ONNX tensor names {i.name}/{o.name} violate the contract (obs/actions)")
        if i.shape[-1] != self.deploy.num_obs or o.shape[-1] != NUM_POLICY_JOINTS:
            raise RuntimeError(f"ONNX shapes {i.shape}->{o.shape} do not match deploy.yaml ({self.deploy.num_obs}->24)")
        self._buf = np.zeros((1, self.deploy.num_obs), dtype=np.float32)

    def act(self, obs: np.ndarray) -> np.ndarray:
        self._buf[0] = obs
        a = self.sess.run(None, {"obs": self._buf})[0][0].astype(np.float64)
        if self.deploy.action_clip is not None:
            a = np.clip(a, -self.deploy.action_clip, self.deploy.action_clip)
        return a
