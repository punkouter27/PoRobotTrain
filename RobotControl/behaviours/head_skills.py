"""Trained head skills: five RL policies (ONNX) that power ONLY the head joints, on the stand.

Each skill is trained in MuJoCo (``robotcontrol.training.train_head_skill``), exported to ONNX, and must pass
``robotcontrol.training.validate_head_skill`` before it can run anywhere. They plug into the same runner,
service, Unreal panel and Blazor client as the scripted behaviours, so a trained skill follows the same path
(twin preview -> read-only check -> pre-flight -> confirm -> real robot) and the same safety envelope
(``JointSafetyChannel``: stale state, tracking timeout, torque cap, joint limits, plus the runner's other-joint check).

Contract shared by training and runtime (this module is the single source; keep it numpy-only and Python 3.8
compatible because it runs on the R1 Jetson):

  joints   : the skill's ``powered`` joints, in this order (subset of head_pitch_joint, head_yaw_joint)
  obs      : [q - default (k), dq (k), last_action (k), command (3)]           -> 3k + 3 floats
  action   : target = default + action_scale * clip(a, -5, 5), clipped to the soft limits
  rate     : 50 Hz policy (POLICY_DT); the backends tick faster and hold the last target between steps
  command  : hold_centre    [0, 0, 0]
             look_at        [yaw_target_rad, pitch_target_rad, 0]
             nod / shake / glance   [sin(2 pi t / T), cos(2 pi t / T), size]   size = amplitude / max amplitude

Policy files: ``RobotControl/models/head_skills/<id>/`` on the PC, ``~/porobot/head_skills/<id>/`` on the robot
(pushed by the service's read-only check, or shipped in the robot kit). A policy loads only when deploy.yaml says
``validation.passed: true`` and ``validation.policy_sha256`` matches the ONNX file.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .. import MODELS_DIR, PROJECT_ROOT
from .neck_turn import SweepParams

HEAD_PITCH = "head_pitch_joint"
HEAD_YAW = "head_yaw_joint"
POLICY_DT = 0.02
ACTION_CLIP = 5.0
SOFT_LIMIT_FACTOR = 0.9
NUM_CMD = 3
SKILLS_SUBDIR = "head_skills"

# UNITE cross-platform portable contract (unitree_r1_a5: 131 obs -> 28 role-ordered actions)
UNITE_N_OBS = 131
UNITE_N_ACT = 28
UNITE_ROLE_INDEX = {HEAD_PITCH: 25, HEAD_YAW: 26}
UNITE_ROLE_MASK = np.array(
    [1.0 if i not in (13, 27) else 0.0 for i in range(UNITE_N_ACT)],
    dtype=np.float32,
)


def pack_unite_obs(
    powered: Sequence[str],
    q_rel: np.ndarray,
    dq: np.ndarray,
    last_action: np.ndarray,
    cmd: np.ndarray,
) -> np.ndarray:
    """Pack head-skill observations into the canonical UNITE 131-dim role-ordered vector."""
    q_rel = np.asarray(q_rel, dtype=np.float32)
    batched = q_rel.ndim == 2
    n = q_rel.shape[0] if batched else 1
    q2 = q_rel if batched else q_rel[None, :]
    dq2 = np.asarray(dq, dtype=np.float32).reshape(n, -1)
    la2 = np.asarray(last_action, dtype=np.float32).reshape(n, -1)
    c2 = np.asarray(cmd, dtype=np.float32).reshape(n, -1)
    obs = np.zeros((n, UNITE_N_OBS), dtype=np.float32)
    obs[:, 8] = -1.0  # projected_gravity = [0, 0, -1] upright on stand
    for k, j in enumerate(powered):
        r = UNITE_ROLE_INDEX[j]
        obs[:, 9 + r] = q2[:, k]
        obs[:, 37 + r] = dq2[:, k]
        obs[:, 65 + r] = la2[:, k]
    obs[:, 93:121] = UNITE_ROLE_MASK
    obs[:, 121 : 121 + c2.shape[1]] = c2
    return obs if batched else obs[0]


def unpack_unite_action(powered: Sequence[str], action: np.ndarray) -> np.ndarray:
    """Extract the powered head-joint actions from a 28-dim UNITE action vector."""
    a = np.asarray(action, dtype=np.float64)
    idx = [UNITE_ROLE_INDEX[j] for j in powered]
    return a[:, idx] if a.ndim == 2 else a[idx]


@dataclass(frozen=True)
class HeadSkill:
    id: str
    label: str
    desc: str
    powered: Tuple[str, ...]
    primary_joint: str
    primary_label: str
    max_deg: float  # largest size the panel may select (and the trained amplitude for the periodic skills)
    period_s: float = 0.0  # periodic skills: one cycle; 0 = command-driven
    # training command ranges (rad) for look_at; periodic skills train size in [size_lo, 1]
    yaw_range: float = 0.0
    pitch_range: float = 0.0
    size_lo: float = 0.3
    # taught pose (behaviours/taught_poses.py): fixed (yaw, pitch) rad reference, command [0, 0, 0] like Hold Centre
    pose: Optional[Tuple[float, float]] = None


SKILLS: Dict[str, HeadSkill] = {
    s.id: s
    for s in (
        HeadSkill(
            "hold_centre",
            "Hold Centre",
            "Trained: holds the head still, facing forward. Head pitch + yaw only.",
            (HEAD_PITCH, HEAD_YAW),
            HEAD_YAW,
            "HEAD YAW",
            30.0,
        ),
        HeadSkill(
            "look_at",
            "Look At",
            "Trained: turns the head to a target angle and settles. Head pitch + yaw only.",
            (HEAD_PITCH, HEAD_YAW),
            HEAD_YAW,
            "HEAD YAW",
            30.0,
            yaw_range=math.radians(40.0),
            pitch_range=math.radians(20.0),
        ),
        HeadSkill(
            "nod_yes",
            "Nod Yes",
            "Trained: nods 'yes' (max 15 deg). Only the neck pitch motor moves.",
            (HEAD_PITCH,),
            HEAD_PITCH,
            "HEAD PITCH",
            15.0,
            period_s=2.0,
        ),
        HeadSkill(
            "shake_no",
            "Shake No",
            "Trained: shakes 'no' (max 20 deg). Only the neck yaw motor moves.",
            (HEAD_YAW,),
            HEAD_YAW,
            "HEAD YAW",
            20.0,
            period_s=3.0,
        ),
        HeadSkill(
            "glance_around",
            "Glance Around",
            "Trained: glances left, right and down, pausing at each. Head pitch + yaw only.",
            (HEAD_PITCH, HEAD_YAW),
            HEAD_YAW,
            "HEAD YAW",
            25.0,
            period_s=12.0,
        ),
    )
}

# glance_around keyframes (yaw, pitch) as a fraction of the size; positive pitch looks down. 3 s per segment:
# hold the key, then move to the next one during the last GLANCE_MOVE of the segment (starts and ends at centre).
GLANCE_KEYS = ((0.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (0.0, 0.4))
GLANCE_MOVE = 0.4


def obs_size(skill: HeadSkill) -> int:
    return 3 * len(skill.powered) + NUM_CMD


# ----------------------------------------------------------------------------- reference motion (vectorised)
def reference(skill: HeadSkill, cmd: np.ndarray, phase: Optional[np.ndarray] = None) -> np.ndarray:
    """Desired angles (n, k) for the skill's powered joints, from the command (n, 3).

    ``phase`` (n,) in [0, 1) is the cycle fraction for periodic skills; it is recovered from sin/cos when omitted.
    """
    cmd = np.atleast_2d(np.asarray(cmd, dtype=np.float64))
    n = cmd.shape[0]
    if skill.id == "hold_centre":
        return np.zeros((n, len(skill.powered)))
    if skill.pose is not None:
        want = {HEAD_YAW: np.full(n, skill.pose[0]), HEAD_PITCH: np.full(n, skill.pose[1])}
        return np.stack([want[j] for j in skill.powered], axis=1)
    if skill.id == "look_at":
        want = {HEAD_YAW: cmd[:, 0], HEAD_PITCH: cmd[:, 1]}
        return np.stack([want[j] for j in skill.powered], axis=1)
    if phase is None:
        phase = np.mod(np.arctan2(cmd[:, 0], cmd[:, 1]) / (2 * np.pi), 1.0)
    amp = math.radians(skill.max_deg) * cmd[:, 2]
    if skill.id == "nod_yes":
        return (amp * np.sin(2 * np.pi * phase))[:, None]
    if skill.id == "shake_no":
        return (amp * np.sin(2 * np.pi * phase))[:, None]
    if skill.id == "glance_around":
        nk = len(GLANCE_KEYS)
        seg_f = phase * nk
        seg = np.floor(seg_f).astype(int) % nk
        u = np.clip((seg_f - np.floor(seg_f) - (1.0 - GLANCE_MOVE)) / GLANCE_MOVE, 0.0, 1.0)
        u = u * u * (3.0 - 2.0 * u)
        keys = np.array(GLANCE_KEYS)
        a, b = keys[seg], keys[(seg + 1) % nk]  # hold this key, then move to the next one
        yp = (a + (b - a) * u[:, None]) * amp[:, None]
        want = {HEAD_YAW: yp[:, 0], HEAD_PITCH: yp[:, 1]}
        return np.stack([want[j] for j in skill.powered], axis=1)
    raise KeyError(skill.id)


def phase_command(skill: HeadSkill, t: np.ndarray, size: np.ndarray) -> np.ndarray:
    """Periodic skills: the [sin, cos, size] command at time t (s)."""
    ph = 2 * np.pi * np.asarray(t, dtype=np.float64) / skill.period_s
    return np.stack([np.sin(ph), np.cos(ph), np.broadcast_to(size, ph.shape)], axis=-1)


def look_at_script(t: float, amp_deg: float) -> Tuple[float, float]:
    """Demo target sequence for Look At until the camera provides targets: (yaw, pitch) rad, 2.5 s each."""
    a = math.radians(amp_deg)
    p = math.radians(min(amp_deg, 20.0) * 0.5)
    seq = ((0.0, 0.0), (a, 0.0), (-a, 0.0), (0.0, p), (0.0, -p), (0.5 * a, 0.5 * p))
    return seq[int(max(t, 0.0) / 2.5) % len(seq)]


# ----------------------------------------------------------------------------- policy files + validation gate
def skill_dirs(skill_id: str) -> List[Path]:
    """Candidate policy folders: PC models dir first, then the robot kit layout (~/porobot/head_skills/<id>)."""
    return [MODELS_DIR / SKILLS_SUBDIR / skill_id, PROJECT_ROOT / SKILLS_SUBDIR / skill_id]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _load_yaml(path: Path) -> dict:
    import yaml

    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def skill_status(skill_id: str, version: Optional[int] = None) -> dict:
    """{"state": "untrained"|"unvalidated"|"failed"|"ready", "reason", "dir", "metrics", "trained_at"}.

    ``version`` checks a stored training session (PC only: models/head_skills/<id>/versions/v<N>) instead of the active
    copy; the same gate applies (validation passed for that exact ONNX file)."""
    dirs = (
        [MODELS_DIR / SKILLS_SUBDIR / skill_id / "versions" / f"v{int(version)}"] if version else skill_dirs(skill_id)
    )
    for d in dirs:
        onnx_p, dep_p = d / "policy.onnx", d / "deploy.yaml"
        if not onnx_p.exists() or not dep_p.exists():
            continue
        dep = _load_yaml(dep_p)
        val = dep.get("validation") or {}
        out = {
            "dir": str(d),
            "metrics": val.get("metrics") or {},
            "trained_at": (dep.get("metadata") or {}).get("trained_at"),
            "validated_at": val.get("validated_at"),
        }
        if not val:
            return dict(out, state="unvalidated", reason="trained but not validated yet")
        if not val.get("passed"):
            return dict(out, state="failed", reason="validation failed: " + "; ".join(val.get("failures") or ["?"]))
        if val.get("policy_sha256") != sha256_file(onnx_p):
            return dict(out, state="failed", reason="policy.onnx changed after validation (sha256 mismatch)")
        return dict(out, state="ready", reason="")
    return {"state": "untrained", "reason": "not trained yet", "dir": None, "metrics": {}, "trained_at": None}


def load_validated(skill_id: str, version: Optional[int] = None) -> Tuple[Path, dict]:
    st = skill_status(skill_id, version)
    if st["state"] != "ready":
        raise RuntimeError(f"{SKILLS[skill_id].label}: {st['reason']} - train it first (Train button)")
    d = Path(st["dir"])
    return d / "policy.onnx", _load_yaml(d / "deploy.yaml")


class OnnxPolicy:
    def __init__(self, path: Path, num_obs: int):
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        self.sess = ort.InferenceSession(str(path), sess_options=opts, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        shape = self.sess.get_inputs()[0].shape
        got = shape[-1]
        if got not in (num_obs, UNITE_N_OBS):
            raise RuntimeError(f"{path.name} expects {got} observations, this skill builds {num_obs} (or {UNITE_N_OBS})")
        self.num_obs = int(got)
        self.is_unite = self.num_obs == UNITE_N_OBS

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return self.sess.run(None, {self.inp: obs.reshape(1, -1).astype(np.float32)})[0][0]


# ----------------------------------------------------------------------------- runtime behaviour
class TrainedHeadBehaviour:
    """Runs a validated head-skill ONNX policy through the runner (sim | urlab | real).

    ``params.amplitude_deg`` is the size: the Look At reach, or the nod / shake / glance size (capped at the skill's
    max). ``params.duration_s`` is how long the skill runs (periodic skills round up to whole cycles so the head ends
    near centre). Hold Centre ignores the size.
    """

    skill: HeadSkill = SKILLS["hold_centre"]
    name = "hold_centre"
    powered: Sequence[str] = ()

    def __init__(self, joint_map, params: SweepParams, check_tracking: bool = True, policy=None, version=None):
        from .stand_behaviours import JointSafetyChannel  # lazy: stand_behaviours imports this module

        sk = self.skill
        onnx_path, dep = (None, {}) if policy is not None else load_validated(sk.id, version)
        if dep and list(dep.get("joint_names", [])) != list(sk.powered):
            raise RuntimeError(f"{sk.id}: deploy.yaml joints {dep.get('joint_names')} != {list(sk.powered)}")
        self.policy = policy if policy is not None else OnnxPolicy(onnx_path, obs_size(sk))
        self.jm = joint_map
        self.amp_deg = min(float(params.amplitude_deg), sk.max_deg)
        self.size = self.amp_deg / sk.max_deg if sk.max_deg > 0 else 1.0
        self.check_tracking = check_tracking
        dur = max(float(params.duration_s), 2.0)
        if sk.period_s > 0:
            dur = math.ceil(dur / sk.period_s) * sk.period_s
        self.run_s = dur
        self.settle_s = 1.0  # Hold Centre for 1 s at the end of every skill, so the head is released at centre
        idx = [joint_map.index_of(j) for j in sk.powered]
        self.default = np.array([joint_map.defaults[i] for i in idx], dtype=np.float64)
        self.scale = np.array([joint_map.action_scale[i] for i in idx], dtype=np.float64)
        lo = np.array([joint_map.limits_lo[i] for i in idx])
        hi = np.array([joint_map.limits_hi[i] for i in idx])
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo) * SOFT_LIMIT_FACTOR
        self.soft_lo, self.soft_hi = mid - half, mid + half
        # Tracking timeout against the REFERENCE motion (head blocked, wrong direction, policy misbehaving). The smooth
        # skills track within ~1 deg after validation, so they get a tight window; Look At jumps between targets (up
        # to 60 deg, which takes the weak 0.86 Nm yaw motor most of a second), so it gets a wide one.
        steps = sk.id == "look_at" or sk.pose is not None  # a taught pose starts with a move of up to 30 deg
        safety = SweepParams(
            amplitude_deg=max(self.amp_deg, 1.0),
            max_amplitude_deg=90.0,
            max_err_deg=15.0 if steps else 8.0,
            max_err_time_s=1.0 if steps else 0.4,
            stale_s=params.stale_s,
            start_tol_deg=params.start_tol_deg,
        )
        self.p = safety
        self.channels = {
            j: JointSafetyChannel(
                joint_map.kp[i], joint_map.effort[i], joint_map.limits_lo[i], joint_map.limits_hi[i], safety,
                check_tracking=check_tracking,
            )
            for j, i in zip(sk.powered, idx)
        }  # fmt: skip
        self.last_action = np.zeros(len(sk.powered))
        self.target = self.default.copy()
        self.next_policy_t = 0.0
        self.abort_reason: Optional[str] = None
        self.cmd = np.zeros(NUM_CMD)

    @property
    def duration_s(self) -> float:
        return self.run_s + self.settle_s

    def start(self, q: Dict[str, float]) -> None:
        tol = math.radians(self.p.start_tol_deg)
        for j in (HEAD_PITCH, HEAD_YAW):
            v = q[j]
            if not math.isfinite(v):
                raise RuntimeError(f"{j} angle is not finite")
            if abs(v) > tol:
                raise RuntimeError(
                    f"{j.replace('_joint', '').replace('_', ' ')} is {math.degrees(v):+.1f} deg; face the head "
                    f"straight ahead (within +-{self.p.start_tol_deg:.0f} deg) and run again"
                )

    def phase(self, t: float) -> str:
        return "skill" if t < self.run_s else ("settle" if t < self.duration_s else "done")

    def command_at(self, t: float) -> np.ndarray:
        sk = self.skill
        if t >= self.run_s or sk.id == "hold_centre" or sk.pose is not None:
            return np.zeros(NUM_CMD)
        if sk.id == "look_at":
            yaw, pitch = look_at_script(t, self.amp_deg)
            return np.array([yaw, pitch, 0.0])
        return phase_command(sk, np.array(t), np.array(self.size)).reshape(NUM_CMD)

    def reference_at(self, t: float) -> np.ndarray:
        """What the policy is asked to do now (for the panel's target readout and the logs)."""
        if t >= self.run_s and self.skill.pose is None:  # a taught pose settles at its pose, not at centre
            return np.zeros(len(self.skill.powered))
        cmd = self.command_at(t)
        ph = None if self.skill.period_s <= 0 else np.array([(t / self.skill.period_s) % 1.0])
        return reference(self.skill, cmd[None, :], ph)[0]

    def _policy_step(self, t: float, q: np.ndarray, dq: np.ndarray) -> None:
        self.cmd = self.command_at(t)
        if getattr(self.policy, "is_unite", False):
            obs = pack_unite_obs(self.skill.powered, q - self.default, dq, self.last_action, self.cmd)
            raw = np.asarray(self.policy(obs), dtype=np.float64).reshape(-1)
            a = np.clip(unpack_unite_action(self.skill.powered, raw), -ACTION_CLIP, ACTION_CLIP)
        else:
            obs = np.concatenate([q - self.default, dq, self.last_action, self.cmd]).astype(np.float32)
            a = np.clip(np.asarray(self.policy(obs), dtype=np.float64).reshape(-1), -ACTION_CLIP, ACTION_CLIP)
        if not np.all(np.isfinite(a)):
            raise FloatingPointError("policy produced a non-finite action")
        self.last_action = a
        self.target = np.clip(self.default + self.scale * a, self.soft_lo, self.soft_hi)

    def targets(self, t: float, q: Dict[str, float], dq: Dict[str, float], age: float) -> Optional[Dict[str, float]]:
        names = self.skill.powered
        qv = np.array([q[j] for j in names], dtype=np.float64)
        dqv = np.array([dq[j] for j in names], dtype=np.float64)
        if t + 1e-9 >= self.next_policy_t:
            if np.all(np.isfinite(qv)) and np.all(np.isfinite(dqv)):
                try:
                    self._policy_step(t, qv, dqv)
                except FloatingPointError as exc:
                    self.abort_reason = str(exc)
                    return None
            self.next_policy_t += POLICY_DT
            while self.next_policy_t <= t:  # a slow tick: skip ahead rather than burst
                self.next_policy_t += POLICY_DT
        out: Dict[str, float] = {}
        ref = self.reference_at(t)
        for k, j in enumerate(names):
            ch = self.channels[j]
            # The tracking check compares the measured angle with the REFERENCE motion (what the skill should do),
            # but the command sent is the policy's target, filtered by the same torque cap and limits.
            if ch.filter(j, t, float(ref[k]), q[j], dq[j], age) is None:
                self.abort_reason = ch.abort_reason
                return None
            band = ch.effort / max(ch.kp, 1e-6)
            v = min(max(float(self.target[k]), q[j] - band), q[j] + band)
            out[j] = min(max(v, ch.q_lo), ch.q_hi)
        return out

    def describe(self) -> str:
        sk = self.skill
        size = "" if sk.id == "hold_centre" or sk.pose is not None else f" size {self.amp_deg:.0f} deg"
        return f"trained skill {sk.label}{size} for {self.run_s:.0f} s (+{self.settle_s:.0f} s settle), ONNX policy"


def _make_class(sk: HeadSkill) -> type:
    cls_name = "".join(w.capitalize() for w in sk.id.split("_")) + "Skill"
    return type(cls_name, (TrainedHeadBehaviour,), {"skill": sk, "name": sk.id, "powered": sk.powered})


SKILL_CLASSES: Dict[str, type] = {k: _make_class(s) for k, s in SKILLS.items()}


# ----------------------------------------------------------------------------- taught poses (pose_<id> skills)
BUILTIN_SKILLS = tuple(SKILLS)


def pose_skill(pose: dict) -> HeadSkill:
    """HeadSkill for a taught pose {"id", "name", "yaw_deg", "pitch_deg"} (behaviours/taught_poses.py)."""
    y, p = float(pose["yaw_deg"]), float(pose["pitch_deg"])
    return HeadSkill(
        "pose_" + str(pose["id"]),
        f"Pose: {pose['name']}",
        f"Taught pose: holds the head at yaw {y:+.0f} deg, pitch {p:+.0f} deg. Head pitch + yaw only.",
        (HEAD_PITCH, HEAD_YAW),
        HEAD_YAW,
        "HEAD YAW",
        30.0,
        pose=(math.radians(y), math.radians(p)),
    )


def sync_taught_poses(poses: Optional[List[dict]] = None) -> None:
    """Make the pose_<id> entries of SKILLS / SKILL_CLASSES match the stored poses (built-in skills untouched)."""
    if poses is None:
        from .taught_poses import load_poses

        poses = load_poses()
    want = {}
    for p in poses:
        sk = pose_skill(p)
        want[sk.id] = sk
    for sid in [s for s in SKILLS if s not in BUILTIN_SKILLS and s not in want]:
        del SKILLS[sid]
        SKILL_CLASSES.pop(sid, None)
    for sid, sk in want.items():
        if sid in BUILTIN_SKILLS:
            continue
        if SKILLS.get(sid) != sk:
            SKILLS[sid] = sk
            SKILL_CLASSES[sid] = _make_class(sk)


sync_taught_poses()
