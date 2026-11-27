"""Head-skill training environment: R1 hanging on the stand, the policy drives ONLY the skill's head joints.

Same physics as Level 0 (``r1_twin_stand_train.xml`` with ``sim_calibration.yaml`` applied, vectorised with
``mujoco.rollout``, one MjModel per env for domain randomisation). Legs are passive (kp 0, kd 3); every other joint,
including an unpowered head joint, is held at its default with the joint-map gains (the real head gearbox holds an
unpowered joint; the runner's other-joint check aborts if one moves more than 5 deg).

Observation, action, command and reference motion come from ``robotcontrol.behaviours.head_skills`` so training and
the runtime (PC, Unreal, Jetson) cannot drift apart.
"""

from __future__ import annotations

import os
import pathlib
from dataclasses import dataclass, field

import mujoco
import numpy as np
from mujoco import rollout

from ..behaviours import head_skills as hs
from ..configs.joint_map import JointMap
from ..configs.sim_calibration import SimCalibration
from .level0_env import _PEN, penalty_terms

ROOT = pathlib.Path(__file__).resolve().parents[2]
MODEL_XML = ROOT / "UnrealProject/MJCF/r1_twin_stand_train.xml"
SUBSTEPS = 4  # 4 x 5 ms physics = one 20 ms policy step
PASSIVE_KD_LEGS = 3.0


@dataclass
class HeadSkillConfig:
    skill: str = "look_at"
    num_envs: int = 512
    episode_length_s: float = 10.0
    cmd_resample_s: tuple = (1.5, 3.0)
    init_offset_deg: tuple = (20.0, 15.0)  # (yaw, pitch) start-pose spread for hold_centre / look_at
    init_offset_small_deg: float = 5.0  # periodic skills
    # domain randomisation (per env, resampled on reset)
    dr_gain_scale: tuple = (0.8, 1.2)
    dr_mass_scale: tuple = (0.9, 1.1)
    dr_friction_scale: tuple = (0.6, 1.4)
    dr_damping_scale: tuple = (0.7, 1.3)
    dr_armature_scale: tuple = (0.8, 1.2)
    dr_action_delay_steps: int = 1
    obs_noise_pos: float = 0.005
    obs_noise_vel: float = 0.3
    # reward
    w_track: float = 1.0
    w_track_fine: float = 1.0
    sigma_coarse: float = 0.15
    sigma_fine: float = 0.035
    w_action_rate: float = _PEN["action_rate_sq"]
    w_joint_vel: float = _PEN["joint_ang_vel_sq"]
    w_torque: float = _PEN["torque_sq"]
    w_limits: float = -1.0
    w_joint_acc: float = -1e-6
    acc_limit: float = 80.0
    w_acc_violation: float = -0.1
    seed: int = 0
    portable: bool = False
    nthread: int = field(default_factory=lambda: max(1, (os.cpu_count() or 4) - 1))


class HeadSkillEnv:
    def __init__(self, cfg: HeadSkillConfig):
        self.cfg = cfg
        self.skill = hs.SKILLS[cfg.skill]
        self.rng = np.random.default_rng(cfg.seed)
        jm = self.jm = JointMap.load()
        m = self.base_model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
        hinge = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
        assert [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in hinge] == jm.names
        self.hinge_dofs = np.array([m.jnt_dofadr[i] for i in hinge], dtype=np.int64)
        cal = SimCalibration.load(jm=jm)
        cal.apply_to_model(mujoco, m, self.hinge_dofs)
        self.n, self.nu = cfg.num_envs, m.nu
        self.ctrl_idx = np.array([jm.index_of(j) for j in self.skill.powered])
        self.k = len(self.ctrl_idx)
        self.passive_idx = np.array(jm.groups["left_leg"] + jm.groups["right_leg"])
        self.portable = bool(cfg.portable)
        self.num_actions = hs.UNITE_N_ACT if self.portable else self.k
        self.num_obs = hs.UNITE_N_OBS if self.portable else hs.obs_size(self.skill)

        self.kp_all, self.kd_all, self.effort_all = jm.kp.copy(), jm.kd.copy(), jm.effort.copy()
        self.default_all = jm.defaults.copy()
        self.default = self.default_all[self.ctrl_idx]
        self.action_scale = jm.action_scale[self.ctrl_idx]
        mid = 0.5 * (jm.limits_lo + jm.limits_hi)
        half = 0.5 * (jm.limits_hi - jm.limits_lo) * hs.SOFT_LIMIT_FACTOR
        self.soft_lo, self.soft_hi = (mid - half)[self.ctrl_idx], (mid + half)[self.ctrl_idx]

        self.spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        self.nstate = mujoco.mj_stateSize(m, self.spec)
        self.qpos_off, self.qvel_off = 1, 1 + m.nq
        self.q_cols = self.qpos_off + 7 + self.ctrl_idx
        self.dq_cols = self.qvel_off + 6 + self.ctrl_idx
        self.q_all_cols = self.qpos_off + 7 + np.arange(self.nu)
        self.frc_cols = np.array(
            [
                m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, j.replace("_joint", "") + "_frc")]
                for j in self.skill.powered
            ]
        )
        self.models = [mujoco.MjModel.from_xml_path(str(MODEL_XML)) for _ in range(self.n)]
        for mi in self.models:
            cal.apply_to_model(mujoco, mi, self.hinge_dofs)
        self.datas = [mujoco.MjData(m) for _ in range(cfg.nthread)]
        self.home_qpos = m.key_qpos[0].copy()

        n, k = self.n, self.k
        self.state = np.zeros((n, self.nstate))
        self.ctrl = np.zeros((n, 1, self.nu))
        self.q, self.dq, self.prev_dq = np.zeros((n, k)), np.zeros((n, k)), np.zeros((n, k))
        self.last_action, self.prev_action = np.zeros((n, k)), np.zeros((n, k))
        self.delayed_target = np.tile(self.default, (n, 1))
        self.frc = np.zeros((n, k))
        self.cmd = np.zeros((n, hs.NUM_CMD))
        self.cmd_timer = np.zeros(n)
        self.t_skill = np.zeros(n)  # periodic skills: time along the cycle
        self.size = np.ones(n)
        self.ep_step = np.zeros(n, dtype=np.int64)
        self.delay = np.zeros(n, dtype=np.int64)
        self.max_steps = int(cfg.episode_length_s / hs.POLICY_DT)
        self._eval = False
        self.reset(np.arange(n))

    # ------------------------------------------------------------------ domain randomisation
    def _randomise(self, ids):
        c, base = self.cfg, self.base_model
        for i in ids:
            mi = self.models[i]
            ev = self._eval
            g = np.ones(self.nu) if ev else self.rng.uniform(*c.dr_gain_scale, size=self.nu)
            kp, kd = self.kp_all * g, self.kd_all * g
            kp[self.passive_idx], kd[self.passive_idx] = 0.0, PASSIVE_KD_LEGS
            mi.actuator_gainprm[:, 0] = kp
            mi.actuator_biasprm[:, 1] = -kp
            mi.actuator_biasprm[:, 2] = -kd
            ms = 1.0 if ev else self.rng.uniform(*c.dr_mass_scale)
            mi.body_mass[:] = base.body_mass * ms
            mi.body_inertia[:] = base.body_inertia * ms
            d = self.hinge_dofs
            for attr, rng in (
                ("dof_frictionloss", c.dr_friction_scale),
                ("dof_damping", c.dr_damping_scale),
                ("dof_armature", c.dr_armature_scale),
            ):
                s = 1.0 if ev else self.rng.uniform(*rng, size=self.nu)
                getattr(mi, attr)[d] = getattr(base, attr)[d] * s
            self.delay[i] = 0 if ev else self.rng.integers(0, c.dr_action_delay_steps + 1)

    def set_eval(self, flag: bool = True):
        """Nominal physics, no noise, no delay (validation)."""
        self._eval = flag

    # ------------------------------------------------------------------ commands
    def _sample_cmd(self, ids):
        sk, n = self.skill, len(ids)
        if sk.id == "look_at":
            yaw = self.rng.uniform(-1, 1, n) * sk.yaw_range
            pitch = self.rng.uniform(-1, 1, n) * sk.pitch_range
            zero = self.rng.random(n) < 0.15
            yaw[zero], pitch[zero] = 0.0, 0.0
            self.cmd[ids] = np.stack([yaw, pitch, np.zeros(n)], axis=1)
            self.cmd_timer[ids] = self.rng.uniform(*self.cfg.cmd_resample_s, size=n)
        elif sk.period_s > 0:
            self.size[ids] = self.rng.uniform(sk.size_lo, 1.0, n)
            self.cmd_timer[ids] = 1e9
        else:
            self.cmd[ids] = 0.0
            self.cmd_timer[ids] = 1e9

    def set_command(self, cmd: np.ndarray):
        """External command (validation): look_at / hold targets; periodic skills use set_size + their clock."""
        self.cmd[:] = cmd
        self.cmd_timer[:] = 1e9

    def set_size(self, size: float):
        self.size[:] = size

    def _update_periodic_cmd(self):
        if self.skill.period_s > 0:
            self.cmd[:] = hs.phase_command(self.skill, self.t_skill, self.size)

    def reference(self) -> np.ndarray:
        sk = self.skill
        ph = (self.t_skill / sk.period_s) % 1.0 if sk.period_s > 0 else None
        return hs.reference(sk, self.cmd, ph)

    # ------------------------------------------------------------------ reset / step
    def reset(self, ids, start_offset: np.ndarray = None):
        ids = np.asarray(ids)
        if ids.size == 0:
            return
        self._randomise(ids)
        sk, c = self.skill, self.cfg
        for i in ids:
            qpos = self.home_qpos.copy()
            if start_offset is not None:
                qpos[7 + self.ctrl_idx] += start_offset
            elif not self._eval:
                if sk.period_s > 0:
                    off = np.deg2rad(c.init_offset_small_deg) * self.rng.uniform(-1, 1, self.k)
                else:
                    spread = {hs.HEAD_YAW: c.init_offset_deg[0], hs.HEAD_PITCH: c.init_offset_deg[1]}
                    off = np.deg2rad([spread[j] for j in sk.powered]) * self.rng.uniform(-1, 1, self.k)
                qpos[7 + self.ctrl_idx] += off
            self.state[i] = 0.0
            self.state[i, self.qpos_off : self.qpos_off + len(qpos)] = qpos
        self.q[ids] = self.state[ids][:, self.q_cols]
        self.dq[ids] = self.prev_dq[ids] = 0.0
        self.last_action[ids] = self.prev_action[ids] = 0.0
        self.delayed_target[ids] = self.default
        self.ep_step[ids] = 0
        if sk.period_s > 0:  # half the episodes start at the beginning of the cycle (like the runtime), half anywhere
            start = self.rng.uniform(0, sk.period_s, len(ids)) * (self.rng.random(len(ids)) < 0.5)
            self.t_skill[ids] = 0.0 if self._eval else start
        self._sample_cmd(ids)
        self._update_periodic_cmd()
        self.ctrl[ids, 0, :] = self.default_all

    def get_obs(self) -> np.ndarray:
        q, dq = self.q, self.dq
        if not self._eval:
            q = q + self.rng.uniform(-self.cfg.obs_noise_pos, self.cfg.obs_noise_pos, q.shape)
            dq = dq + self.rng.uniform(-self.cfg.obs_noise_vel, self.cfg.obs_noise_vel, dq.shape)
        if self.portable:
            return hs.pack_unite_obs(self.skill.powered, q - self.default, dq, self.last_action, self.cmd)
        return np.concatenate([q - self.default, dq, self.last_action, self.cmd], axis=1).astype(np.float32)

    def targets_from_action(self, action: np.ndarray) -> np.ndarray:
        a = np.clip(action, -hs.ACTION_CLIP, hs.ACTION_CLIP)
        return np.clip(self.default + self.action_scale * a, self.soft_lo, self.soft_hi)

    def step(self, action: np.ndarray):
        c = self.cfg
        action = np.asarray(action, dtype=np.float64)
        if self.portable and action.shape[-1] == hs.UNITE_N_ACT:
            action = hs.unpack_unite_action(self.skill.powered, action)
        ref = self.reference()  # what this step's observation asked for
        self.prev_action[:] = self.last_action
        self.last_action[:] = action
        target = self.targets_from_action(action)
        applied = np.where(self.delay[:, None] > 0, self.delayed_target, target)
        self.delayed_target[:] = target
        self.ctrl[:, 0, self.ctrl_idx] = applied
        state, sensordata = rollout.rollout(
            self.models,
            self.datas,
            self.state,
            self.ctrl,
            control_spec=mujoco.mjtState.mjSTATE_CTRL,
            nstep=SUBSTEPS,
            persistent_pool=True,
        )
        self.state[:] = state[:, -1, :]
        self.prev_dq[:] = self.dq
        self.q[:] = self.state[:, self.q_cols]
        self.dq[:] = self.state[:, self.dq_cols]
        self.frc[:] = sensordata[:, -1, self.frc_cols]
        acc = (self.dq - self.prev_dq) / hs.POLICY_DT

        # advance the skill clock, then score against the reference for the NEW time (the policy must lead)
        self.t_skill += hs.POLICY_DT
        self._update_periodic_cmd()
        ref_next = self.reference() if self.skill.period_s > 0 else ref
        e = self.q - ref_next
        r_track = c.w_track * np.mean(np.exp(-(e**2) / c.sigma_coarse**2), axis=1)
        r_fine = c.w_track_fine * np.mean(np.exp(-(e**2) / c.sigma_fine**2), axis=1)
        pen = penalty_terms(c, self.frc, self.dq, self.last_action, self.prev_action)
        r_rate = pen["pen_action_rate_sq"]
        r_vel = pen["pen_joint_ang_vel_sq"]
        r_acc = c.w_joint_acc * np.sum(acc**2, axis=1)
        over = np.clip(self.q - self.soft_hi, 0, None) + np.clip(self.soft_lo - self.q, 0, None)
        r_lim = c.w_limits * np.sum(over, axis=1)
        r_tau = pen["pen_torque_sq"]
        r_accv = c.w_acc_violation * np.sum(np.abs(acc) > c.acc_limit, axis=1)
        reward = r_track + r_fine + r_rate + r_vel + r_acc + r_lim + r_tau + r_accv

        self.ep_step += 1
        self.cmd_timer -= hs.POLICY_DT
        resample = np.where(self.cmd_timer <= 0)[0]
        if resample.size:
            self._sample_cmd(resample)
        timeout = self.ep_step >= self.max_steps
        bad = ~np.isfinite(self.state).all(axis=1)
        done = timeout | bad
        info = {
            "track_err_deg": float(np.rad2deg(np.abs(e)).mean()),
            "max_acc": float(np.abs(acc).max()),
            "r_track": float((r_track + r_fine).mean()),
            "max_frc": float(np.abs(self.frc).max()),
            **{k: float(v.mean()) for k, v in pen.items()},
        }
        obs = self.get_obs()
        done_ids = np.where(done)[0]
        if done_ids.size:
            self.reset(done_ids)
            obs[done_ids] = self.get_obs()[done_ids]
        return obs, reward.astype(np.float32), done, info

    def other_joint_drift(self, q_start_all: np.ndarray) -> np.ndarray:
        """Max |change| of every non-powered, non-leg joint since q_start_all (n,)."""
        keep = [i for i in range(self.nu) if i not in set(self.ctrl_idx) and i not in set(self.passive_idx)]
        q_all = self.state[:, self.q_all_cols]
        return np.abs(q_all[:, keep] - q_start_all[:, keep]).max(axis=1)

    def close(self):
        rollout.shutdown_persistent_pool()
