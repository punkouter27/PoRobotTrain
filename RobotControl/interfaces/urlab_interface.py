"""UE5 / Unreal Robotics Lab interface through the ``urlab_client`` Python package
(``external/urlab_bridge/src/urlab_client``).

API used (checked against client.py / articulation.py):

* ``URLabClient(address, step_mode="direct", transport="zmq"|"shm", ...)``
* ``client.connect(observations="full")``, ``client.articulations`` (dict prefix -> URLabArticulation),
  ``client.articulation(prefix=None)``, ``client.step(n_steps, target_hz=...)``, ``client.reset()``, ``client.close()``
* ``art.actuators`` / ``art.joints`` (keys are unprefixed MJCF names), ``art.resolve_actuator(name)``,
  ``art.resolve_joint(name)`` (both strip a trailing ``_joint``)
* ``art.set_ctrl({actuator_name: value})``, ``art.qpos_array`` / ``art.qvel_array`` with per-joint
  ``Joint.qpos_local_offset`` / ``qvel_local_offset``, ``art.dof_qpos`` / ``art.dof_qvel``
* ``art.get_sensors()`` -> {name: ndarray}, ``art.root_quat_w`` (wxyz), ``art.root_ang_vel_b``
* ``art.push_gains(joint_names, stiffness, damping, torque_limit)`` -> URLabPDController.set_gains
* ``art.control_mode`` ("ue_controller" uses UE's PD controller with pushed gains; "raw" sends ctrl
  straight into the MuJoCo actuators, so kp/kd are whatever the MJCF baked in)

``urlab_client`` is imported lazily so the rest of the package works without it.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from .. import PROJECT_ROOT
from ..configs.joint_map import JointMap, actuator_name_for
from .base_interface import LowStateData, MotorCommand, RobotHardwareInterface

log = logging.getLogger(__name__)

_URLAB_SRC = PROJECT_ROOT / "external" / "urlab_bridge" / "src"


def _import_urlab():
    try:
        import urlab_client  # type: ignore
    except ImportError:
        if _URLAB_SRC.exists() and str(_URLAB_SRC) not in sys.path:
            sys.path.insert(0, str(_URLAB_SRC))
            try:
                import urlab_client  # type: ignore
            except ImportError as exc:
                raise ImportError(
                    "urlab_client is present at external/urlab_bridge/src but failed to import "
                    f"({exc}). Install its deps: `uv pip install -e external/urlab_bridge` "
                    "(needs pyzmq, msgpack, mujoco==3.8.1)."
                ) from exc
        else:
            raise ImportError(
                "urlab_client not installed. Clone URLab-Sim/urlab_bridge into external/urlab_bridge "
                "and run `uv pip install -e external/urlab_bridge`, or `pip install urlab-bridge`."
            )
    return urlab_client


class URLabInterface(RobotHardwareInterface):
    """Talks to a running UE5 URLab session that has the R1 articulation spawned."""

    def __init__(self, joint_map: Optional[JointMap] = None, address: str = "tcp://localhost",
                 step_mode: str = "direct", articulation: Optional[str] = None,
                 control_mode: Optional[str] = None, policy_dt: float = 0.02,
                 sim_dt: float = 0.005, transport: str = "zmq", observations: str = "full",
                 target_hz: Optional[float] = None, mujoco_version_check: bool = True,
                 imu_sensor_names: Optional[Dict[str, str]] = None,
                 sync_options_xml: Optional[str] = None, **client_kwargs):
        super().__init__(joint_map)
        if sync_options_xml is None:
            from .. import MJCF_DIR
            sync_options_xml = str(Path(MJCF_DIR) / "scene_stand.xml")
        self.sync_options_xml = sync_options_xml or None
        self.address = address
        self.step_mode = step_mode
        self.articulation_prefix = articulation
        self.requested_control_mode = control_mode
        self.policy_dt = float(policy_dt)
        self.sim_dt = float(sim_dt)
        self.substeps = max(1, int(round(self.policy_dt / self.sim_dt)))
        self.transport = transport
        self.observations = observations
        self.target_hz = target_hz
        self.mujoco_version_check = mujoco_version_check
        self.client_kwargs = client_kwargs
        self.imu_sensor_names = {"quat": "imu_quat", "gyro": "imu_ang_vel", "accel": "imu_lin_acc"}
        if imu_sensor_names:
            self.imu_sensor_names.update(imu_sensor_names)
        self.client = None
        self.art = None
        self._act_keys: list[str] = []
        self._qpos_idx: np.ndarray
        self._qvel_idx: np.ndarray
        self._last_kp: Optional[np.ndarray] = None
        self._last_kd: Optional[np.ndarray] = None
        self._warned_raw = False
        self._tick = 0
        self._ue_controller = False

    # ------------------------------------------------------------- lifecycle
    def connect(self) -> None:
        urlab = _import_urlab()
        self.client = urlab.URLabClient(
            self.address, step_mode=self.step_mode, transport=self.transport,
            mujoco_version_check=self.mujoco_version_check, **self.client_kwargs,
        )
        self.client.connect(observations=self.observations)
        if not self.client.articulations:
            raise RuntimeError("URLab connected but no articulation is present (is PIE running?)")
        self.art = self.client.articulation(self.articulation_prefix)
        art = self.art

        # Physics parity: push the training MJCF's solver options (5 ms step, implicitfast, iterations)
        # into the live UE model. URLab otherwise runs its manager default (2 ms).
        if self.sync_options_xml:
            import mujoco
            opt_model = mujoco.MjModel.from_xml_path(str(self.sync_options_xml))
            self.client.runtime.set_sim_options_from_model(opt_model)
            self.sim_dt = float(opt_model.opt.timestep)
            self.substeps = max(1, int(round(self.policy_dt / self.sim_dt)))
            log.info("URLab sim options synced from %s (dt=%.4f, %d substeps/tick)",
                     Path(self.sync_options_xml).name, self.sim_dt, self.substeps)

        # resolve actuators and joints in policy order
        act_keys, qpos_idx, qvel_idx = [], [], []
        for jn in self.joint_map.names:
            a_key = art.resolve_actuator(jn) or art.resolve_actuator(actuator_name_for(jn))
            if a_key is None:
                raise RuntimeError(f"URLab articulation has no actuator for {jn!r}; "
                                   f"available: {list(art.actuators)}")
            j_key = art.resolve_joint(jn)
            if j_key is None:
                raise RuntimeError(f"URLab articulation has no joint {jn!r}; available: {list(art.joints)}")
            act_keys.append(a_key)
            j = art.joints[j_key]
            qpos_idx.append(int(j.qpos_local_offset))
            qvel_idx.append(int(j.qvel_local_offset))
        self._act_keys = act_keys
        self._qpos_idx = np.array(qpos_idx, dtype=np.int64)
        self._qvel_idx = np.array(qvel_idx, dtype=np.int64)

        # control mode
        has_ctrl = getattr(art, "controller", None) is not None
        mode = self.requested_control_mode or ("ue_controller" if has_ctrl else "raw")
        if mode == "ue_controller" and not has_ctrl:
            log.warning("URLab articulation exposes no controller; falling back to raw ctrl")
            mode = "raw"
        self._ue_controller = mode == "ue_controller"
        try:
            art.control_mode = urlab.ControlMode(mode)
        except Exception:  # older bridge: plain string
            art.control_mode = mode

        # sensors available?
        sensors = art.get_sensors()
        for k, sname in self.imu_sensor_names.items():
            if sname not in sensors:
                log.warning("URLab sensor %r (%s) not found; falling back to root state", sname, k)
        self.connected = True
        # Start from the same 'home' keyframe as the headless twin and training (PIE starts at qpos0).
        try:
            # URLab prefixes imported keyframe names with the actor name.
            names = [k.name for k in self.client.runtime.list_keyframes()]
            key = next((n for n in names if n == "home" or n.endswith("_home")), None)
            if key is None:
                raise RuntimeError(f"no home keyframe in {names}")
            self._home_key = key
            self.client.reset(keyframe_name=key)
        except Exception as e:  # keyframe missing on an older import
            log.warning("URLab reset to keyframe 'home' failed: %s", e)
        log.info("URLabInterface connected: prefix=%s mode=%s step_mode=%s substeps=%d",
                 art.prefix, mode, self.step_mode, self.substeps)

    def close(self) -> None:
        if self.client is not None:
            try:
                self.client.close()
            except Exception:  # pragma: no cover
                pass
        self.client = None
        self.connected = False

    def reset(self, keyframe: Optional[str] = "home") -> None:
        # URLab prefixes keyframe names with the actor name; map the logical 'home' to the resolved key.
        if keyframe in (None, "home"):
            keyframe = getattr(self, "_home_key", None) or keyframe
        self.client.reset(keyframe_name=keyframe)
        self._tick = 0

    # ----------------------------------------------------------------- state
    def get_low_state(self) -> LowStateData:
        art = self.art
        qpos = np.asarray(art.qpos_array, dtype=np.float64)
        qvel = np.asarray(art.qvel_array, dtype=np.float64)
        q = qpos[self._qpos_idx].copy()
        dq = qvel[self._qvel_idx].copy()
        tau = np.zeros(self.num_joints)
        for i, k in enumerate(self._act_keys):
            f = art.actuators[k].force
            if f is not None:
                tau[i] = float(f)
        sensors = art.get_sensors()
        n = self.imu_sensor_names
        if n["quat"] in sensors and sensors[n["quat"]].shape[0] == 4:
            quat = np.asarray(sensors[n["quat"]], dtype=np.float64)
        else:
            quat = np.asarray(art.root_quat_w, dtype=np.float64)
        if n["gyro"] in sensors:
            gyro = np.asarray(sensors[n["gyro"]], dtype=np.float64)
        else:
            gyro = np.asarray(art.root_ang_vel_b, dtype=np.float64)
        accel = np.asarray(sensors.get(n["accel"], np.zeros(3)), dtype=np.float64)
        return LowStateData(q=q, dq=dq, tau=tau, imu_quat=quat, gyro=gyro, accel=accel,
                            timestamp=float(self.client.sim_time), tick=self._tick,
                            extras={"root_pos": np.asarray(art.root_pos_w, dtype=np.float64)})

    # --------------------------------------------------------------- command
    def _send(self, cmd: MotorCommand) -> None:
        art = self.art
        if self._ue_controller:
            if (self._last_kp is None or not np.array_equal(self._last_kp, cmd.kp)
                    or not np.array_equal(self._last_kd, cmd.kd)):
                art.push_gains(self.joint_map.names, cmd.kp, cmd.kd, torque_limit=self.joint_map.effort)
                self._last_kp = cmd.kp.copy()
                self._last_kd = cmd.kd.copy()
        elif not self._warned_raw:
            model_kp = np.array([art.actuators[k].kp or 0.0 for k in self._act_keys])
            if not np.allclose(model_kp, cmd.kp, atol=1e-6):
                log.warning("URLab raw control mode: MJCF actuator gains are fixed (kp=%s); requested kp ignored",
                            np.round(model_kp, 1).tolist())
            self._warned_raw = True
        art.set_ctrl({k: float(cmd.q[i]) for i, k in enumerate(self._act_keys)})

    # ---------------------------------------------------------------- timing
    def tick(self) -> None:
        self.client.step(n_steps=self.substeps, target_hz=self.target_hz)
        self._tick += 1

    @property
    def is_simulation(self) -> bool:
        return True

    def now(self) -> float:
        return float(self.client.sim_time) if self.client is not None else 0.0
