"""Headless single-env MuJoCo stand scene for the skill tools (duel, training montage, synthetic camera data).

Same physics setup as the head-skill training env in eval mode (``training/head_skill_env.py``): the R1 welded to its
stand, ``sim_calibration.yaml`` applied, legs passive (kp 0, kd 3), every other joint PD-held at its default with the
joint-map gains; the skill's powered head joints get the policy targets. 5 ms physics, 4 substeps per 20 ms policy
step. ``r1_twin_stand_train.xml`` (collision only, fast) by default, ``r1_twin_stand.xml`` (meshes, camera) to render.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

import mujoco
import numpy as np

from .. import MJCF_DIR
from ..configs.joint_map import JointMap
from ..configs.sim_calibration import SimCalibration

TRAIN_XML = MJCF_DIR / "r1_twin_stand_train.xml"
VISUAL_XML = MJCF_DIR / "r1_twin_stand.xml"
SUBSTEPS = 4
POLICY_DT = 0.02
PASSIVE_KD_LEGS = 3.0
HEAD_LINK = "head_yaw_link"


def apply_stand_gains(m: mujoco.MjModel, jm: JointMap, strength: Optional[Sequence[int]] = None, scale: float = 1.0):
    """Joint-map gains on every actuator, legs passive; ``strength`` joint indices get kp/kd * scale."""
    kp, kd = jm.kp.astype(np.float64).copy(), jm.kd.astype(np.float64).copy()
    legs = list(jm.groups["left_leg"]) + list(jm.groups["right_leg"])
    kp[legs], kd[legs] = 0.0, PASSIVE_KD_LEGS
    if strength is not None:
        idx = list(strength)
        kp[idx] *= scale
        kd[idx] *= scale
    m.actuator_gainprm[:, 0] = kp
    m.actuator_biasprm[:, 1] = -kp
    m.actuator_biasprm[:, 2] = -kd


def build_model(xml: Path = TRAIN_XML, jm: Optional[JointMap] = None, spec_hook=None) -> mujoco.MjModel:
    """Compile the stand scene (``spec_hook(spec)`` may add things first), apply calibration + stand gains."""
    jm = jm or JointMap.load()
    if spec_hook is None:
        m = mujoco.MjModel.from_xml_path(str(xml))
    else:
        spec = mujoco.MjSpec.from_file(str(xml))
        spec_hook(spec)
        m = spec.compile()
    hinge = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in hinge][: jm.num_joints]
    if names != list(jm.names):
        raise RuntimeError(f"{Path(xml).name}: joint order differs from r1_joint_map.yaml")
    dofs = np.array([m.jnt_dofadr[i] for i in hinge[: jm.num_joints]], dtype=np.int64)
    SimCalibration.load(jm=jm).apply_to_model(mujoco, m, dofs)
    apply_stand_gains(m, jm)
    return m


class StandSim:
    def __init__(self, xml: Path = TRAIN_XML, spec_hook=None):
        self.jm = JointMap.load()
        self.m = build_model(xml, self.jm, spec_hook)
        self.d = mujoco.MjData(self.m)
        m = self.m
        hinge = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE][: self.jm.num_joints]
        self.qadr = np.array([m.jnt_qposadr[i] for i in hinge])
        self.vadr = np.array([m.jnt_dofadr[i] for i in hinge])
        self.head_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, HEAD_LINK)
        self.default_all = np.asarray(self.jm.defaults, dtype=np.float64)
        self.reset()

    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key("home").id)
        self.d.ctrl[:] = self.default_all
        mujoco.mj_forward(self.m, self.d)
        self.t = 0.0

    @property
    def q(self) -> np.ndarray:
        return self.d.qpos[self.qadr].copy()

    @property
    def dq(self) -> np.ndarray:
        return self.d.qvel[self.vadr].copy()

    def set_head(self, yaw: float, pitch: float) -> None:
        """Teleport the head joints (synthetic camera poses), then forward kinematics."""
        jm = self.jm
        iy, ip = jm.index_of("head_yaw_joint"), jm.index_of("head_pitch_joint")
        self.d.qpos[self.qadr[iy]], self.d.qpos[self.qadr[ip]] = yaw, pitch
        self.d.ctrl[iy], self.d.ctrl[ip] = yaw, pitch
        mujoco.mj_forward(self.m, self.d)

    def set_strength(self, joints: Sequence[int], pct: float) -> None:
        apply_stand_gains(self.m, self.jm, joints, pct / 100.0)

    def step(self, targets: dict, head_torque: Optional[Sequence[float]] = None) -> None:
        """One 20 ms policy step: {joint index: target}, others at default; optional world torque on the head."""
        self.d.ctrl[:] = self.default_all
        for i, v in targets.items():
            self.d.ctrl[i] = v
        self.d.xfrc_applied[self.head_body, :] = 0.0
        if head_torque is not None:
            self.d.xfrc_applied[self.head_body, 3:6] = head_torque
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.m, self.d)
        self.t += POLICY_DT
