"""Real-time walk sandbox for the behaviour service: one WalkSim stepped at 50 Hz on a worker thread.

* ``/walk/command`` sets the velocity command; a watchdog ramps it to zero when no command arrives for 0.5 s,
  so a client that disappears stops the robot.
* Every policy step the full pose (floating base + 26 joints) is handed to a puppet thread that pushes it into
  the Unreal twin (URLab puppet mode, the live-mirror path). Unreal never simulates the robot.
* Simulation only: there is no robot backend here.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np

from .course import COURSES, course
from .policy import MODEL_DIR, Level2Policy, policy_status
from .sim import POLICY_DT, WalkSim

WATCHDOG_S = 0.5
WATCHDOG_RAMP_S = 0.3
PUPPET_RETRY_S = 3.0


class WalkSandbox:
    def __init__(
        self,
        mirror_factory: Optional[Callable[[], object]] = None,
        log: Callable[[str], None] = print,
        model_dir=MODEL_DIR,
    ):
        self.mirror_factory = mirror_factory  # () -> object with push_full(root7, q) and close(); None = no Unreal
        self.log = log
        self.model_dir = model_dir
        self.lock = threading.Lock()
        self.sim: Optional[WalkSim] = None
        self.policy: Optional[Level2Policy] = None
        self.running = False
        self.course_name = "loop"
        self.cmd_target = np.zeros(3)
        self.cmd_at = 0.0
        self.stop_evt = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.puppet_thread: Optional[threading.Thread] = None
        self.puppet_pose: Optional[tuple] = None
        self.puppet_evt = threading.Event()
        self.puppet_connected = False
        self.overruns = 0

    # ------------------------------------------------------------------ queries
    def policy_status(self) -> tuple[bool, Optional[str]]:
        return policy_status(self.model_dir)

    def state(self) -> dict:
        ok, reason = self.policy_status()
        with self.lock:
            sim = self.sim
            base = {
                "ok": True,
                "policy_available": ok,
                "policy_reason": reason,
                "running": self.running,
                "courses": sorted(COURSES),
                "puppet": {"connected": self.puppet_connected},
            }
            if sim is None:
                return {**base, "mode": None, "course": self.course_name, "geometry": course(self.course_name),
                        "cmd_limits": {"vx": [-0.5, 1.0], "vy": [-0.5, 0.5], "wz": [-1.0, 1.0]}}  # fmt: skip
            lim = sim.policy.deploy.cmd_limits if sim.policy else {"vx": [0, 0], "vy": [0, 0], "wz": [0, 0]}
            return {
                **base,
                **sim.snapshot(),
                "cmd_limits": lim,
                "joint_names": list(sim.jm.names),
                "geometry": sim.geometry,
            }

    # ------------------------------------------------------------------ commands
    def start(self, course_name: str = "loop", mode: str = "policy") -> dict:
        if course_name not in COURSES:
            return {"ok": False, "error": f"unknown course {course_name!r}"}
        if mode not in ("policy", "hold"):
            return {"ok": False, "error": f"mode must be 'policy' or 'hold', got {mode!r}"}
        self.stop()
        policy = None
        if mode == "policy":
            ok, reason = self.policy_status()
            if not ok:
                return {"ok": False, "error": reason, "policy_reason": reason}
            try:
                policy = self.policy if self.policy is not None else Level2Policy(self.model_dir)
                self.policy = policy
            except Exception as e:
                return {"ok": False, "error": f"Level 2 policy failed to load: {e}"}
        try:
            sim = WalkSim(course_name, mode=mode, policy=policy)
        except Exception as e:
            return {"ok": False, "error": f"walk scene failed to load: {e}"}
        with self.lock:
            self.sim = sim
            self.course_name = course_name
            self.cmd_target[:] = 0.0
            self.cmd_at = time.monotonic()
            self.running = True
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._loop, name="walk-sim", daemon=True)
        self.thread.start()
        if self.mirror_factory is not None:
            self.puppet_thread = threading.Thread(target=self._puppet_loop, name="walk-puppet", daemon=True)
            self.puppet_thread.start()
        self.log(f"walk: started course={course_name} mode={mode}")
        return {"ok": True}

    def command(self, vx: float, vy: float, wz: float) -> dict:
        with self.lock:
            if not self.running:
                return {"ok": False, "error": "walk sandbox is not running"}
            self.cmd_target[:] = [vx, vy, wz]
            self.cmd_at = time.monotonic()
            if self.sim is not None and self.sim.mode == "hold":
                return {"ok": True, "note": "hold mode ignores velocity commands"}
        return {"ok": True}

    def reset(self) -> dict:
        with self.lock:
            if self.sim is None:
                return {"ok": False, "error": "walk sandbox is not running"}
            self.sim.reset()
            self.sim.falls = self.sim.laps = 0
            self.sim.last_lap_s = self.sim.best_lap_s = None
            self.cmd_target[:] = 0.0
        return {"ok": True}

    def stop(self) -> dict:
        self.stop_evt.set()
        self.puppet_evt.set()
        for t in (self.thread, self.puppet_thread):
            if t is not None and t.is_alive() and t is not threading.current_thread():
                t.join(timeout=2.0)
        self.thread = self.puppet_thread = None
        with self.lock:
            was = self.running
            self.running = False
        if was:
            self.log("walk: stopped")
        return {"ok": True}

    # ------------------------------------------------------------------ threads
    def _loop(self) -> None:
        next_t = time.perf_counter()
        while not self.stop_evt.is_set():
            with self.lock:
                sim = self.sim
                age = time.monotonic() - self.cmd_at
                target = self.cmd_target.copy()
                if age > WATCHDOG_S:
                    fade = max(0.0, 1.0 - (age - WATCHDOG_S) / WATCHDOG_RAMP_S)
                    target *= fade
                    if fade == 0.0:
                        self.cmd_target[:] = 0.0
                if sim.mode == "policy":
                    sim.set_command(*target)
                sim.step()
                root, q = sim.full_qpos()
            self.puppet_pose = (root, q)
            self.puppet_evt.set()
            next_t += POLICY_DT
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.25:  # fell far behind (debugger, laptop sleep): resync instead of fast-forwarding
                self.overruns += 1
                next_t = time.perf_counter()

    def _puppet_loop(self) -> None:
        mirror = None
        next_try = 0.0
        warned = False
        while not self.stop_evt.is_set():
            self.puppet_evt.wait(timeout=0.2)
            self.puppet_evt.clear()
            if self.stop_evt.is_set():
                break
            if mirror is None:
                if time.monotonic() < next_try:
                    continue
                try:
                    mirror = self.mirror_factory()
                    self.puppet_connected = True
                    self.log("walk: Unreal twin connected (puppet mode)")
                except Exception as e:
                    next_try = time.monotonic() + PUPPET_RETRY_S
                    if not warned:  # once per run; it keeps retrying quietly
                        self.log(f"walk: Unreal twin not reachable ({str(e).splitlines()[0][:80]}); web view only")
                        warned = True
                    self.puppet_connected = False
                    continue
            pose = self.puppet_pose
            if pose is None:
                continue
            try:
                mirror.push_full(*pose)
            except Exception as e:
                self.log(f"walk: Unreal puppet lost ({str(e).splitlines()[0][:80]})")
                try:
                    mirror.close()
                except Exception:
                    pass
                mirror = None
                self.puppet_connected = False
                next_try = time.monotonic() + PUPPET_RETRY_S
        if mirror is not None:
            try:
                mirror.close()
            except Exception:
                pass
        self.puppet_connected = False
