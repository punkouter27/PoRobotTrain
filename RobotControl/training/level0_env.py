"""Level 0 training environment: R1 hanging on the stand, policy drives right arm (5) + head (2).
Legs are passive (kp 0, kd 3); waist and left arm are held at default with joint-map gains.

Vectorised with mujoco.rollout: every environment owns its own MjModel copy so gains, masses and
friction can be domain-randomised per environment, while stepping runs multithreaded in C.

Observation (23) : joint_pos_rel(7), joint_vel(7), last_action(7), command(2)
Action      (7)  : target = default + action_scale * a  (clipped to 0.9 * joint range)
Command     (2)  : [head_yaw_target_rad, right_shoulder_pitch_target_rad]
Policy rate      : 50 Hz (dt 0.02 s = 4 physics substeps of 5 ms)
"""
from __future__ import annotations

import os
import pathlib
from dataclasses import dataclass, field

import mujoco
import numpy as np
import yaml
from mujoco import rollout

ROOT = pathlib.Path(__file__).resolve().parents[2]
# mesh-free variant (identical physics; visual geoms carry no mass/collision) so 512 per-env model copies fit in RAM
MODEL_XML = ROOT / "UnrealProject/MJCF/r1_twin_stand_train.xml"
JOINT_MAP = ROOT / "RobotControl/configs/r1_joint_map.yaml"

CONTROLLED_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
]
CMD_HEAD_YAW, CMD_SHOULDER_PITCH = 0, 1
POLICY_DT = 0.02
SUBSTEPS = 4
PASSIVE_KD_LEGS = 3.0  # Unitree passive-mode damping


@dataclass
class Level0Config:
    num_envs: int = 512
    episode_length_s: float = 10.0
    cmd_resample_s: tuple[float, float] = (2.0, 4.0)
    head_yaw_amp: float = np.deg2rad(60.0)        # full-curriculum amplitude
    shoulder_pitch_range: float = 1.6             # arm raised forward: default - range .. default
    curriculum_start: float = 0.3                 # fraction of amplitude at iteration 0
    # domain randomisation (per env, resampled on reset)
    dr_gain_scale: tuple[float, float] = (0.8, 1.2)
    dr_mass_scale: tuple[float, float] = (0.9, 1.1)
    dr_action_delay_steps: int = 1
    obs_noise_pos: float = 0.01
    obs_noise_vel: float = 0.5
    # reward weights
    w_track: float = 1.0
    track_sigma: float = 0.10
    w_hold: float = -0.5
    w_action_rate: float = -0.05
    w_joint_vel: float = -0.001
    w_joint_acc: float = -1e-6
    w_limits: float = -1.0
    w_torque: float = -1e-5
    acc_limit: float = 80.0                       # rad/s^2 hard penalty threshold
    w_acc_violation: float = -0.1
    seed: int = 0
    nthread: int = field(default_factory=lambda: max(1, (os.cpu_count() or 4) - 1))


class Level0StandEnv:
    def __init__(self, cfg: Level0Config):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.jm = yaml.safe_load(JOINT_MAP.read_text())
        self.base_model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
        m = self.base_model
        self.n = cfg.num_envs
        self.nu = m.nu
        names = [j["name"] for j in self.jm["joints"]]
        assert names == [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
                         for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
        self.ctrl_idx = np.array([names.index(j) for j in CONTROLLED_JOINTS])
        # Level 0 on the stand: legs passive (damping only); waist + left arm HELD at default with map gains
        # (a passive waist lets the torso twist under arm reaction torque and drags the hand into the thigh).
        self.passive_idx = np.array(self.jm["groups"]["left_leg"] + self.jm["groups"]["right_leg"])
        self.held_idx = np.array([i for i in range(self.nu) if i not in self.ctrl_idx and i not in self.passive_idx])
        self.num_actions = len(self.ctrl_idx)
        self.num_obs = 3 * self.num_actions + 2

        jarr = lambda k: np.array([j[k] for j in self.jm["joints"]], dtype=np.float64)
        self.kp_all, self.kd_all, self.effort_all = jarr("kp"), jarr("kd"), jarr("effort")
        self.default_all = jarr("default")
        limits = np.array([j["limits"] for j in self.jm["joints"]], dtype=np.float64)
        self.lim_lo, self.lim_hi = limits[:, 0], limits[:, 1]
        self.action_scale = jarr("action_scale")[self.ctrl_idx]
        self.default = self.default_all[self.ctrl_idx]
        mid = 0.5 * (self.lim_lo + self.lim_hi)
        half = 0.5 * (self.lim_hi - self.lim_lo) * 0.9
        self.soft_lo, self.soft_hi = (mid - half)[self.ctrl_idx], (mid + half)[self.ctrl_idx]
        self.passive_kd = np.full(len(self.passive_idx), PASSIVE_KD_LEGS)

        # state layout (mjSTATE_FULLPHYSICS = TIME | QPOS | QVEL | ACT)
        self.spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        self.nstate = mujoco.mj_stateSize(m, self.spec)
        self.qpos_off, self.qvel_off = 1, 1 + m.nq
        self.q_cols = self.qpos_off + 7 + self.ctrl_idx          # 7 = free joint qpos
        self.dq_cols = self.qvel_off + 6 + self.ctrl_idx         # 6 = free joint qvel
        self.q_all_cols = self.qpos_off + 7 + np.arange(self.nu)
        self.frc_sensor_cols = np.array([
            m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, j.replace("_joint", "") + "_frc")]
            for j in CONTROLLED_JOINTS])

        self.models = [mujoco.MjModel.from_xml_path(str(MODEL_XML)) for _ in range(self.n)]
        self.datas = [mujoco.MjData(m) for _ in range(cfg.nthread)]
        self.home_qpos = m.key_qpos[0].copy()

        # buffers
        self.state = np.zeros((self.n, self.nstate))
        self.ctrl = np.zeros((self.n, 1, self.nu))
        self.q = np.zeros((self.n, self.num_actions)); self.dq = np.zeros_like(self.q)
        self.prev_dq = np.zeros_like(self.q)
        self.last_action = np.zeros_like(self.q); self.prev_action = np.zeros_like(self.q)
        self.delayed_target = np.tile(self.default, (self.n, 1))
        self.cmd = np.zeros((self.n, 2))
        self.cmd_timer = np.zeros(self.n)
        self.ep_step = np.zeros(self.n, dtype=np.int64)
        self.delay = np.zeros(self.n, dtype=np.int64)
        self.max_steps = int(cfg.episode_length_s / POLICY_DT)
        self.curriculum = cfg.curriculum_start
        self.frc = np.zeros((self.n, self.num_actions))
        self._eval = False
        self.reset(np.arange(self.n))

    # ------------------------------------------------------------------ domain randomisation
    def _randomise(self, ids):
        for i in ids:
            mi = self.models[i]
            g = self.rng.uniform(*self.cfg.dr_gain_scale, size=self.nu) if not self._eval else np.ones(self.nu)
            kp = self.kp_all * g
            kd = self.kd_all * g
            kp[self.passive_idx] = 0.0
            kd[self.passive_idx] = self.passive_kd
            mi.actuator_gainprm[:, 0] = kp
            mi.actuator_biasprm[:, 1] = -kp
            mi.actuator_biasprm[:, 2] = -kd
            ms = self.rng.uniform(*self.cfg.dr_mass_scale) if not self._eval else 1.0
            mi.body_mass[:] = self.base_model.body_mass * ms
            mi.body_inertia[:] = self.base_model.body_inertia * ms
            self.delay[i] = self.rng.integers(0, self.cfg.dr_action_delay_steps + 1) if not self._eval else 0

    def set_eval(self, flag: bool = True):
        """Nominal physics, no noise, no delay (used for validation rollouts)."""
        self._eval = flag

    # ------------------------------------------------------------------ commands
    def _sample_cmd(self, ids):
        c = self.curriculum
        n = len(ids)
        self.cmd[ids, CMD_HEAD_YAW] = self.rng.uniform(-1, 1, n) * self.cfg.head_yaw_amp * c
        lo = self.default[CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")] - self.cfg.shoulder_pitch_range * c
        hi = self.default[CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")]
        self.cmd[ids, CMD_SHOULDER_PITCH] = self.rng.uniform(lo, hi, n)
        self.cmd_timer[ids] = self.rng.uniform(*self.cfg.cmd_resample_s, size=n)

    def set_command(self, cmd: np.ndarray):
        """External command override (validation / sweep)."""
        self.cmd[:] = cmd
        self.cmd_timer[:] = 1e9

    # ------------------------------------------------------------------ reset / step
    def reset(self, ids):
        ids = np.asarray(ids)
        if ids.size == 0:
            return
        self._randomise(ids)
        for i in ids:
            qpos = self.home_qpos.copy()
            if not self._eval:
                qpos[7 + self.ctrl_idx] += self.rng.uniform(-0.1, 0.1, self.num_actions)
            self.state[i] = 0.0
            self.state[i, self.qpos_off:self.qpos_off + len(qpos)] = qpos
        self.q[ids] = self.state[ids][:, self.q_cols]
        self.dq[ids] = 0.0; self.prev_dq[ids] = 0.0
        self.last_action[ids] = 0.0; self.prev_action[ids] = 0.0
        self.delayed_target[ids] = self.default
        self.ep_step[ids] = 0
        self._sample_cmd(ids)
        self.ctrl[ids, 0, :] = self.default_all  # controlled joints' defaults are already inside default_all

    def _obs(self) -> np.ndarray:
        q, dq = self.q, self.dq
        if not self._eval:
            q = q + self.rng.uniform(-self.cfg.obs_noise_pos, self.cfg.obs_noise_pos, q.shape)
            dq = dq + self.rng.uniform(-self.cfg.obs_noise_vel, self.cfg.obs_noise_vel, dq.shape)
        return np.concatenate([q - self.default, dq, self.last_action, self.cmd], axis=1).astype(np.float32)

    def get_obs(self) -> np.ndarray:
        return self._obs()

    def targets_from_action(self, action: np.ndarray) -> np.ndarray:
        a = np.clip(action, -5.0, 5.0)
        return np.clip(self.default + self.action_scale * a, self.soft_lo, self.soft_hi)

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64)
        self.prev_action[:] = self.last_action
        self.last_action[:] = action
        target = self.targets_from_action(action)
        # action delay: envs with delay=1 apply the previous target this tick
        applied = np.where(self.delay[:, None] > 0, self.delayed_target, target)
        self.delayed_target[:] = target
        self.ctrl[:, 0, self.ctrl_idx] = applied

        state, sensordata = rollout.rollout(self.models, self.datas, self.state, self.ctrl,
                                            control_spec=mujoco.mjtState.mjSTATE_CTRL,
                                            nstep=SUBSTEPS, persistent_pool=True)
        self.state[:] = state[:, -1, :]
        self.prev_dq[:] = self.dq
        self.q[:] = self.state[:, self.q_cols]
        self.dq[:] = self.state[:, self.dq_cols]
        self.frc[:] = sensordata[:, -1, self.frc_sensor_cols]
        acc = (self.dq - self.prev_dq) / POLICY_DT

        # ---- reward
        e_yaw = self.q[:, CONTROLLED_JOINTS.index("head_yaw_joint")] - self.cmd[:, CMD_HEAD_YAW]
        e_pitch = self.q[:, CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")] - self.cmd[:, CMD_SHOULDER_PITCH]
        s2 = self.cfg.track_sigma ** 2
        r_track = self.cfg.w_track * (np.exp(-e_yaw ** 2 / s2) + np.exp(-e_pitch ** 2 / s2))
        hold_mask = np.ones(self.num_actions); hold_mask[[CONTROLLED_JOINTS.index("head_yaw_joint"),
                                                           CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")]] = 0
        r_hold = self.cfg.w_hold * np.sum(((self.q - self.default) ** 2) * hold_mask, axis=1)
        r_rate = self.cfg.w_action_rate * np.sum((self.last_action - self.prev_action) ** 2, axis=1)
        r_vel = self.cfg.w_joint_vel * np.sum(self.dq ** 2, axis=1)
        r_acc = self.cfg.w_joint_acc * np.sum(acc ** 2, axis=1)
        over = np.clip(self.q - self.soft_hi, 0, None) + np.clip(self.soft_lo - self.q, 0, None)
        r_lim = self.cfg.w_limits * np.sum(over, axis=1)
        r_tau = self.cfg.w_torque * np.sum(self.frc ** 2, axis=1)
        r_accv = self.cfg.w_acc_violation * np.sum(np.abs(acc) > self.cfg.acc_limit, axis=1)
        reward = r_track + r_hold + r_rate + r_vel + r_acc + r_lim + r_tau + r_accv

        # ---- commands / episodes
        self.ep_step += 1
        self.cmd_timer -= POLICY_DT
        resample = np.where(self.cmd_timer <= 0)[0]
        if resample.size:
            self._sample_cmd(resample)
        timeout = self.ep_step >= self.max_steps
        bad = ~np.isfinite(self.state).all(axis=1)
        done = timeout | bad
        info = {
            "track_err_yaw_deg": np.rad2deg(np.abs(e_yaw)).mean(),
            "track_err_pitch_deg": np.rad2deg(np.abs(e_pitch)).mean(),
            "max_acc": np.abs(acc).max(),
            "r_track": r_track.mean(), "r_hold": r_hold.mean(), "r_rate": r_rate.mean(),
            "r_acc": r_acc.mean(), "r_lim": r_lim.mean(), "r_tau": r_tau.mean(),
            "timeouts": timeout, "curriculum": self.curriculum,
        }
        obs_before_reset = self._obs()
        done_ids = np.where(done)[0]
        if done_ids.size:
            self.reset(done_ids)
        return obs_before_reset, reward.astype(np.float32), done, info

    def presettle(self, seconds: float = 2.0):
        """Hold controlled joints at default (action 0) while passive limbs settle under gravity.
        Mirrors the real procedure: enter passive/damping mode on the stand, wait, then start the policy."""
        for _ in range(int(seconds / POLICY_DT)):
            self.step(np.zeros((self.n, self.num_actions)))
        self.last_action[:] = 0.0; self.prev_action[:] = 0.0
        self.prev_dq[:] = self.dq
        self.ep_step[:] = 0

    def update_curriculum(self, mean_track_err_deg: float):
        if mean_track_err_deg < 3.0 and self.curriculum < 1.0:
            self.curriculum = min(1.0, self.curriculum + 0.05)

    def close(self):
        rollout.shutdown_persistent_pool()
