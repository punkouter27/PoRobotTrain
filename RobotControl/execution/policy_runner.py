"""ONNX policy execution: ``OnnxPolicy`` (onnxruntime, zero per-step allocation),
``ObservationBuilder`` (deploy.yaml-driven), ``TrajectoryLogger`` and the 50 Hz
``PolicyRunner`` loop::

    state -> obs -> ONNX -> action -> q* = default + scale * action -> SafetyGuard -> interface
"""
from __future__ import annotations

import csv
import logging
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from .. import LOGS_DIR
from ..configs.deploy_config import DeployConfig, find_deploy_yaml
from ..configs.joint_map import JointMap
from ..interfaces.base_interface import LowStateData, RobotHardwareInterface
from .safety_guard import SafetyConfig, SafetyGuard, SafetyReport

log = logging.getLogger(__name__)

CommandFn = Callable[[float, LowStateData], np.ndarray]


# ============================================================================
class OnnxPolicy:
    """onnxruntime wrapper. Input ``obs`` float32 (1, N) -> output ``actions`` float32 (1, A).
    Buffers are pre-allocated and bound once; ``infer`` does no per-step allocation."""

    def __init__(self, path: str | Path, use_cuda: bool = False, input_name: Optional[str] = None,
                 output_name: Optional[str] = None, intra_op_threads: int = 1):
        import onnxruntime as ort

        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(self.path)
        so = ort.SessionOptions()
        so.intra_op_num_threads = int(intra_op_threads)
        so.inter_op_num_threads = 1
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        available = ort.get_available_providers()
        providers = ["CPUExecutionProvider"]
        if use_cuda:
            if "CUDAExecutionProvider" in available:
                providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            else:
                log.warning("CUDA requested but CUDAExecutionProvider not available (%s); using CPU", available)
        self.session = ort.InferenceSession(str(self.path), so, providers=providers)
        self.providers = self.session.get_providers()

        inputs = {i.name: i for i in self.session.get_inputs()}
        outputs = {o.name: o for o in self.session.get_outputs()}
        self.input_name = input_name or ("obs" if "obs" in inputs else next(iter(inputs)))
        self.output_name = output_name or ("actions" if "actions" in outputs else next(iter(outputs)))
        if self.input_name not in inputs:
            raise ValueError(f"input {self.input_name!r} not in {list(inputs)}")
        if self.output_name not in outputs:
            raise ValueError(f"output {self.output_name!r} not in {list(outputs)}")
        self.input_shape = list(inputs[self.input_name].shape)
        self.output_shape = list(outputs[self.output_name].shape)
        self.num_obs = _last_static_dim(self.input_shape, "input")
        self.num_actions = _last_static_dim(self.output_shape, "output")
        self.batch_dynamic = not isinstance(self.input_shape[0], int)

        self.obs_buf = np.zeros((1, self.num_obs), dtype=np.float32)
        self.act_buf = np.zeros((1, self.num_actions), dtype=np.float32)
        self._obs_view = self.obs_buf[0]
        self._act_view = self.act_buf[0]
        self._io = None
        try:
            self._in_val = ort.OrtValue.ortvalue_from_numpy(self.obs_buf)
            self._out_val = ort.OrtValue.ortvalue_from_numpy(self.act_buf)
            io = self.session.io_binding()
            io.bind_ortvalue_input(self.input_name, self._in_val)
            io.bind_ortvalue_output(self.output_name, self._out_val)
            self._io = io
        except Exception as exc:  # pragma: no cover - fallback path
            log.warning("io_binding unavailable (%s); falling back to session.run", exc)
            self._io = None
        self.infer_count = 0
        self.last_infer_ms = 0.0

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """Returns a (A,) float32 view into the internal output buffer (copy it if you keep it)."""
        t0 = time.perf_counter()
        np.copyto(self._obs_view, obs, casting="unsafe")
        if self._io is not None:
            self.session.run_with_iobinding(self._io)
        else:
            out = self.session.run([self.output_name], {self.input_name: self.obs_buf})[0]
            np.copyto(self.act_buf, out)
        self.infer_count += 1
        self.last_infer_ms = (time.perf_counter() - t0) * 1e3
        return self._act_view

    def infer_batch(self, obs: np.ndarray) -> np.ndarray:
        """(T, N) -> (T, A). Uses one batched run when the graph has a dynamic batch dim."""
        obs = np.ascontiguousarray(obs, dtype=np.float32)
        if obs.ndim == 1:
            obs = obs[None]
        if self.batch_dynamic:
            return self.session.run([self.output_name], {self.input_name: obs})[0]
        return np.stack([self.infer(o).copy() for o in obs])

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return self.infer(obs)

    def warmup(self, n: int = 5) -> None:
        for _ in range(n):
            self.infer(np.zeros(self.num_obs, dtype=np.float32))


def _last_static_dim(shape: list, what: str) -> int:
    d = shape[-1]
    if not isinstance(d, int):
        raise ValueError(f"ONNX {what} last dim must be static, got {shape}")
    return int(d)


# ============================================================================
class ObservationBuilder:
    """Assemble the observation vector from ``deploy.observations`` in order.

    Supported terms: joint_pos_rel, joint_vel, joint_vel_rel, last_action, command,
    base_ang_vel, projected_gravity, gait_phase (param ``period``, default 0.8 s).
    Optional per-term params: ``scale`` (float), ``joints`` (explicit joint list)."""

    def __init__(self, deploy: DeployConfig, joint_map: JointMap):
        self.deploy = deploy
        self.jm = joint_map
        self.act_idx = joint_map.indices_of(deploy.joint_names)
        self.num_actions = len(self.act_idx)
        self.command_dim = deploy.command_dim
        self._plan: List[tuple] = []
        offset = 0
        for term in deploy.observations:
            name = term.name
            p = term.params
            scale = float(p.get("scale", 1.0))
            if name in ("joint_pos_rel", "joint_vel", "joint_vel_rel"):
                if p.get("joints"):
                    idx = joint_map.indices_of(p["joints"])
                    defaults = joint_map.defaults[idx].copy()
                else:
                    idx = self.act_idx
                    defaults = np.asarray(deploy.default_joint_pos, dtype=np.float64)
                size = len(idx) if term.size < 0 else term.size
                if size != len(idx):
                    raise ValueError(f"{name}: size {size} but {len(idx)} joints")
                self._plan.append((name, slice(offset, offset + size), idx, defaults, scale))
            elif name == "last_action":
                size = self.num_actions if term.size < 0 else term.size
                if size != self.num_actions:
                    raise ValueError(f"last_action size {size} != num_actions {self.num_actions}")
                self._plan.append((name, slice(offset, offset + size), None, None, scale))
            elif name == "command":
                size = self.command_dim if term.size < 0 else term.size
                if size != self.command_dim:
                    raise ValueError(f"command size {size} != sum(commands) {self.command_dim}")
                self._plan.append((name, slice(offset, offset + size), None, None, scale))
            elif name in ("base_ang_vel", "projected_gravity"):
                size = 3
                self._plan.append((name, slice(offset, offset + size), None, None, scale))
            elif name == "gait_phase":
                size = 2
                period = float(p.get("period", 0.8))
                self._plan.append((name, slice(offset, offset + size), None, period, scale))
            else:
                raise ValueError(f"unknown observation term {name!r}")
            offset += size
        self.num_obs = offset
        if deploy.num_obs and deploy.num_obs != offset:
            raise ValueError(f"deploy.num_obs={deploy.num_obs} but terms sum to {offset}")
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        self._gravity = np.array([0.0, 0.0, -1.0])

    @property
    def term_slices(self) -> Dict[str, slice]:
        return {name: sl for name, sl, *_ in self._plan}

    def build(self, state: LowStateData, last_action: np.ndarray, command: Optional[np.ndarray] = None,
              t: float = 0.0) -> np.ndarray:
        obs = self.obs
        for name, sl, idx, aux, scale in self._plan:
            if name == "joint_pos_rel":
                obs[sl] = (state.q[idx] - aux) * scale
            elif name == "joint_vel" or name == "joint_vel_rel":
                obs[sl] = state.dq[idx] * scale
            elif name == "last_action":
                obs[sl] = last_action * scale
            elif name == "command":
                if command is None:
                    obs[sl] = 0.0
                else:
                    obs[sl] = np.asarray(command, dtype=np.float32)[: sl.stop - sl.start] * scale
            elif name == "base_ang_vel":
                obs[sl] = state.gyro * scale
            elif name == "projected_gravity":
                obs[sl] = state.projected_gravity() * scale
            elif name == "gait_phase":
                ph = 2.0 * math.pi * t / aux
                obs[sl.start] = math.sin(ph) * scale
                obs[sl.start + 1] = math.cos(ph) * scale
        return obs


# ============================================================================
class TrajectoryLogger:
    """Collects per-step arrays and writes ``<stem>.npz`` + ``<stem>.csv``."""

    def __init__(self, log_dir: str | Path = LOGS_DIR, stem: Optional[str] = None, joint_names: Optional[List[str]] = None,
                 write_csv: bool = True):
        self.log_dir = Path(log_dir)
        self.stem = stem or f"traj_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.joint_names = joint_names
        self.write_csv = write_csv
        self.rows: Dict[str, List[np.ndarray]] = {k: [] for k in
                                                  ("t", "q", "dq", "tau", "target_q", "action", "obs", "command", "kp", "kd", "flags")}
        self.meta: Dict[str, object] = {}

    def add(self, t: float, state: LowStateData, target_q: np.ndarray, action: np.ndarray, obs: np.ndarray,
            command: Optional[np.ndarray], kp: np.ndarray, kd: np.ndarray, report: Optional[SafetyReport] = None) -> None:
        r = self.rows
        r["t"].append(np.float64(t))
        r["q"].append(state.q.astype(np.float64, copy=True))
        r["dq"].append(state.dq.astype(np.float64, copy=True))
        r["tau"].append(state.tau.astype(np.float64, copy=True))
        r["target_q"].append(np.asarray(target_q, dtype=np.float64).copy())
        r["action"].append(np.asarray(action, dtype=np.float64).copy())
        r["obs"].append(np.asarray(obs, dtype=np.float32).copy())
        r["command"].append(np.zeros(0) if command is None else np.asarray(command, dtype=np.float64).copy())
        r["kp"].append(np.asarray(kp, dtype=np.float64).copy())
        r["kd"].append(np.asarray(kd, dtype=np.float64).copy())
        flags = 0
        if report is not None:
            flags = (1 if report.tripped else 0) | (2 if report.stale else 0) | (4 if report.limit_clipped.any() else 0) \
                | (8 if report.slew_limited.any() else 0) | (16 if report.torque_clipped.any() else 0) | (32 if report.accel_flag.any() else 0)
        r["flags"].append(np.int64(flags))

    def __len__(self) -> int:
        return len(self.rows["t"])

    def save(self) -> Dict[str, Path]:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        out: Dict[str, Path] = {}
        if not self.rows["t"]:
            return out
        arrays = {k: np.stack(v) if len(v) and np.ndim(v[0]) > 0 else np.asarray(v) for k, v in self.rows.items()}
        if self.joint_names:
            arrays["joint_names"] = np.array(self.joint_names)
        for k, v in self.meta.items():
            arrays[f"meta_{k}"] = np.asarray(v)
        npz = self.log_dir / f"{self.stem}.npz"
        np.savez_compressed(npz, **arrays)
        out["npz"] = npz
        if self.write_csv:
            csv_path = self.log_dir / f"{self.stem}.csv"
            names = self.joint_names or [f"j{i}" for i in range(arrays["q"].shape[1])]
            header = ["t"] + [f"q_{n}" for n in names] + [f"dq_{n}" for n in names] + \
                [f"target_{n}" for n in names] + [f"action_{i}" for i in range(arrays["action"].shape[1])] + \
                [f"cmd_{i}" for i in range(arrays["command"].shape[1])] + ["flags"] + \
                [f"obs_{i}" for i in range(arrays["obs"].shape[1])]
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(header)
                for i in range(len(arrays["t"])):
                    w.writerow([f"{arrays['t'][i]:.4f}"] + [f"{x:.6f}" for x in arrays["q"][i]] +
                               [f"{x:.6f}" for x in arrays["dq"][i]] + [f"{x:.6f}" for x in arrays["target_q"][i]] +
                               [f"{x:.6f}" for x in arrays["action"][i]] + [f"{x:.6f}" for x in arrays["command"][i]] +
                               [int(arrays["flags"][i])] + [f"{x:.6f}" for x in arrays["obs"][i]])
            out["csv"] = csv_path
        return out


# ============================================================================
def build_interface(target: str, joint_map: JointMap, scene: str = "stand", address: str = "tcp://localhost",
                    iface: Optional[str] = None, dry_run: bool = False, render: bool = False,
                    policy_dt: float = 0.02, **kw) -> RobotHardwareInterface:
    target = target.lower()
    if target in ("sim", "mujoco"):
        from ..interfaces.mujoco_interface import MujocoInterface
        return MujocoInterface(joint_map, scene=scene, policy_dt=policy_dt, render=render, **kw)
    if target == "urlab":
        from ..interfaces.urlab_interface import URLabInterface
        return URLabInterface(joint_map, address=address, policy_dt=policy_dt, **kw)
    if target == "real":
        from ..interfaces.unitree_interface import UnitreeSDK2Interface
        return UnitreeSDK2Interface(joint_map, network_interface=iface, dry_run=dry_run, **kw)
    raise ValueError(f"unknown target {target!r}; choose sim|urlab|real")


class PolicyRunner:
    """Fixed-rate control loop (``deploy.step_dt``, 50 Hz by default)."""

    def __init__(self, interface: RobotHardwareInterface, policy: OnnxPolicy, deploy: DeployConfig,
                 joint_map: Optional[JointMap] = None, command_fn: Optional[CommandFn] = None,
                 safety: Optional[SafetyGuard] = None, log_dir: Optional[str | Path] = LOGS_DIR,
                 log_stem: Optional[str] = None, realtime: bool = True, status_hz: float = 2.0,
                 status_fn: Optional[Callable[[dict], None]] = None, exit_on_trip: bool = True,
                 hold_non_policy_joints: bool = True, auto_reset: Optional[bool] = None,
                 workspace_radius_m: float = 3.0, workspace_min_z_m: float = 0.25):
        self.itf = interface
        self.policy = policy
        self.deploy = deploy
        self.jm = joint_map or interface.joint_map
        self.command_fn = command_fn
        self.safety = safety or SafetyGuard(self.jm, SafetyConfig())
        self.realtime = realtime
        self.status_hz = status_hz
        self.status_fn = status_fn or (lambda d: print(format_status(d), flush=True))
        self.exit_on_trip = exit_on_trip
        # Reset logic: simulated targets reset to the home keyframe on a safety trip or when the root leaves
        # the workspace (radius around the origin, minimum pelvis height). Real hardware never auto-resets:
        # it damps and exits.
        self.auto_reset = bool(getattr(interface, 'is_simulation', False)) if auto_reset is None else auto_reset
        self.workspace_radius_m = float(workspace_radius_m)
        self.workspace_min_z_m = float(workspace_min_z_m)
        self.reset_count = 0
        self.dt = float(deploy.step_dt)

        problems = deploy.validate(self.jm)
        if problems:
            raise ValueError("deploy.yaml invalid:\n  " + "\n  ".join(problems))
        if policy.num_obs != deploy.num_obs or policy.num_actions != deploy.num_actions:
            raise ValueError(f"ONNX io ({policy.num_obs}->{policy.num_actions}) != deploy "
                             f"({deploy.num_obs}->{deploy.num_actions})")

        n = self.jm.num_joints
        self.act_idx = deploy.policy_indices(self.jm)
        self.passive_idx = deploy.passive_indices(self.jm)
        self.default_act = np.asarray(deploy.default_joint_pos, dtype=np.float64)
        self.scale_act = np.asarray(deploy.action_scale, dtype=np.float64)
        self.clip_act = None if deploy.action_clip is None else np.asarray(deploy.action_clip, dtype=np.float64)
        self.soft_lo, self.soft_hi = self.jm.soft_limits(self.safety.cfg.soft_limit_fraction)

        self.kp_full = self.jm.kp.copy()
        self.kd_full = self.jm.kd.copy()
        self.kp_full[self.act_idx] = deploy.kp
        self.kd_full[self.act_idx] = deploy.kd
        self.kp_full[self.passive_idx] = 0.0
        self.kd_full[self.passive_idx] = (deploy.passive_kd if deploy.passive_kd is not None
                                          else self.jm.kd[self.passive_idx])
        self.hold_full = self.jm.defaults.copy()
        self.hold_non_policy = hold_non_policy_joints

        self.obs_builder = ObservationBuilder(deploy, self.jm)
        self.last_action = np.zeros(deploy.num_actions, dtype=np.float32)
        self.command = np.zeros(deploy.command_dim)
        self.logger = TrajectoryLogger(log_dir, log_stem, joint_names=self.jm.names) if log_dir else None
        if self.logger is not None:
            self.logger.meta["policy_name"] = deploy.policy_name
            self.logger.meta["step_dt"] = self.dt

        self.step_count = 0
        self.t_elapsed = 0.0
        self.tripped = False
        self.last_report: Optional[SafetyReport] = None
        self.last_state: Optional[LowStateData] = None
        self.last_target = self.hold_full.copy()
        self.loop_hz = 0.0
        self._stop = threading.Event()

    # ------------------------------------------------------------------ loop
    def step_once(self, t: float) -> dict:
        state = self.itf.get_low_state()
        self.last_state = state
        if self.command_fn is not None:
            self.command = np.asarray(self.command_fn(t, state), dtype=np.float64)
        obs = self.obs_builder.build(state, self.last_action, self.command, t)
        action = self.policy.infer(obs)
        if self.clip_act is not None:
            np.clip(action, -self.clip_act, self.clip_act, out=action)
        target_act = self.default_act + self.scale_act * action
        target_act = np.clip(target_act, self.soft_lo[self.act_idx], self.soft_hi[self.act_idx])

        target = self.hold_full.copy() if self.hold_non_policy else state.q.copy()
        target[self.passive_idx] = state.q[self.passive_idx]
        target[self.act_idx] = target_act

        safe_q, rep = self.safety.apply(target, state, self.kp_full, self.kd_full, t=state.timestamp)
        self.itf.send_low_cmd(safe_q, rep.kp, rep.kd)
        self.itf.tick()

        self.last_action[:] = action
        self.last_report = rep
        self.last_target = safe_q
        self.tripped = rep.tripped
        if self.logger is not None:
            self.logger.add(t, state, safe_q, action, obs, self.command, rep.kp, rep.kd, rep)
        self.step_count += 1
        return {"t": t, "state": state, "target": safe_q, "action": action, "report": rep, "command": self.command}

    def run(self, duration_s: Optional[float] = None, stop_event: Optional[threading.Event] = None) -> dict:
        stop = stop_event or self._stop
        self.policy.warmup()
        t_start = time.perf_counter()
        next_tick = t_start
        next_status = 0.0                      # in loop time, so --no-realtime runs report too
        trip_since: Optional[float] = None
        hz_count, hz_t0 = 0, t_start
        try:
            while not stop.is_set():
                now = time.perf_counter()
                self.t_elapsed = now - t_start if self.realtime else self.step_count * self.dt
                if duration_s is not None and self.t_elapsed >= duration_s:
                    break
                info = self.step_once(self.t_elapsed)
                hz_count += 1
                if self.auto_reset:
                    reason = info['report'].trip_reason if self.tripped else self.out_of_workspace(info['state'])
                    if reason:
                        self.reset_episode(reason)
                        trip_since = None
                        continue
                if self.tripped:
                    if trip_since is None:
                        trip_since = self.t_elapsed
                        log.error("SAFETY TRIP: %s -- damping", info["report"].trip_reason)
                    if self.exit_on_trip and (self.t_elapsed - trip_since) > 0.5:
                        break
                if now - hz_t0 >= 1.0:
                    self.loop_hz = hz_count / (now - hz_t0)
                    hz_count, hz_t0 = 0, now
                if self.status_hz > 0 and self.t_elapsed >= next_status:
                    next_status = self.t_elapsed + 1.0 / self.status_hz
                    self.status_fn(self.status())
                if self.realtime:
                    next_tick += self.dt
                    slack = next_tick - time.perf_counter()
                    if slack > 0:
                        time.sleep(slack)
                    elif slack < -self.dt:
                        next_tick = time.perf_counter()  # fell behind; resync
        finally:
            self.shutdown()
        return self.status()

    def out_of_workspace(self, state: LowStateData) -> Optional[str]:
        pos = state.extras.get('root_pos') if state.extras else None
        if pos is None:
            return None
        r = float(np.hypot(pos[0], pos[1]))
        if r > self.workspace_radius_m:
            return f'root left workspace (r={r:.2f} m > {self.workspace_radius_m} m)'
        if pos[2] < self.workspace_min_z_m:
            return f'root below workspace floor (z={pos[2]:.2f} m < {self.workspace_min_z_m} m)'
        return None

    def reset_episode(self, reason: str) -> None:
        """Simulation only: back to the home keyframe, clear policy and safety state, keep running."""
        self.reset_count += 1
        log.warning('AUTO-RESET #%d: %s', self.reset_count, reason)
        reset = getattr(self.itf, 'reset', None)
        if reset is not None:
            reset()
        self.safety.reset()
        self.last_action[:] = 0.0
        self.tripped = False

    def stop(self) -> None:
        self._stop.set()

    def shutdown(self) -> dict:
        paths: dict = {}
        try:
            if self.itf.connected:
                for _ in range(3):
                    self.itf.send_damping()
                    self.itf.tick()
        except Exception as exc:  # pragma: no cover
            log.warning("damping on shutdown failed: %s", exc)
        if self.logger is not None and len(self.logger):
            paths = self.logger.save()
            if paths:
                log.info("trajectory log: %s", paths.get("npz"))
            self.logger = None
        return paths

    def status(self) -> dict:
        st = self.last_state
        rep = self.last_report
        return {
            "t": self.t_elapsed, "steps": self.step_count, "hz": self.loop_hz,
            "infer_ms": self.policy.last_infer_ms,
            "command": self.command.copy(),
            "q_act": None if st is None else st.q[self.act_idx].copy(),
            "target_act": self.last_target[self.act_idx].copy(),
            "safety": "n/a" if rep is None else rep.summary(),
            "tripped": self.tripped,
            "resets": self.reset_count,
            "rpy_deg": None if st is None else np.degrees(st.rpy()),
        }


def format_status(d: dict) -> str:
    cmd = " ".join(f"{c:+.2f}" for c in d["command"]) if len(d["command"]) else "-"
    q = "-" if d["q_act"] is None else " ".join(f"{x:+.2f}" for x in d["q_act"])
    tq = " ".join(f"{x:+.2f}" for x in d["target_act"])
    return (f"t={d['t']:7.2f}s step={d['steps']:6d} {d['hz']:5.1f}Hz inf={d['infer_ms']:.2f}ms "
            f"cmd=[{cmd}] q=[{q}] q*=[{tq}] safety={d['safety']}")


def load_policy_bundle(onnx_path: str | Path, deploy_path: Optional[str | Path] = None, use_cuda: bool = False):
    """Return ``(OnnxPolicy, DeployConfig)`` for a policy directory."""
    onnx_path = Path(onnx_path)
    deploy = DeployConfig.load(deploy_path or find_deploy_yaml(onnx_path))
    policy = OnnxPolicy(onnx_path, use_cuda=use_cuda)
    return policy, deploy
