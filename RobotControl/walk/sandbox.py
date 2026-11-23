"""Real-time walk sandbox for the behaviour service: one WalkSim stepped at 50 Hz on a worker thread.

* ``/walk/command`` sets the velocity command; a watchdog ramps it to zero when no command arrives for 0.5 s,
  so a client that disappears stops the robot.
* Every policy step the full pose (floating base + 26 joints) is handed to a puppet thread that pushes it into
  the Unreal twin (URLab puppet mode, the live-mirror path). Unreal never simulates the robot.
* Replay (docs/sim_features.md #12): the simulation pauses and the last 1..10 s of recorded frames are played back
  (puppet + ``/walk/state``) at 0.1..1x; ``auto_on_fall`` replays 3 s at 0.25x one second after every fall.
* Scenario scripts (#18): ``RobotControl/scenarios/<id>.json`` steps run on sim time inside this loop.
* Simulation only: there is no robot backend here.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np

from .course import COURSES, course
from .policy import MODEL_DIR, Level2Policy, policy_status
from .sim import MODES, POLICY_DT, WalkSim

WATCHDOG_S = 0.5
WATCHDOG_RAMP_S = 0.3
PUPPET_RETRY_S = 3.0
AUTO_REPLAY_DELAY_S = 1.0
AUTO_REPLAY = {"seconds": 3.0, "speed": 0.25}


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
        self.seed, self.difficulty = 0, 0.5
        self.cmd_target = np.zeros(3)
        self.cmd_at = 0.0
        self.stop_evt = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.puppet_thread: Optional[threading.Thread] = None
        self.puppet_pose: Optional[tuple] = None
        self.puppet_evt = threading.Event()
        self.puppet_connected = False
        self.overruns = 0
        self.auto_on_fall = False
        self.replay: Optional[dict] = None  # {"frames", "speed", "pos", "duration_s"} while active
        self.replay_speed = AUTO_REPLAY["speed"]
        self.replay_duration = AUTO_REPLAY["seconds"]
        self.pending_auto: Optional[float] = None  # sim time to start the automatic replay at
        self.seen_falls = 0
        self.scenario: Optional[dict] = None

    # ------------------------------------------------------------------ queries
    def policy_status(self) -> tuple:
        return policy_status(self.model_dir)

    def _replay_block(self) -> dict:
        r = self.replay
        return {
            "active": r is not None,
            "speed": r["speed"] if r else self.replay_speed,
            "t": round(r["pos"], 3) if r else 0.0,
            "duration_s": round(r["duration_s"], 3) if r else self.replay_duration,
            "auto_on_fall": self.auto_on_fall,
        }

    def _scenario_block(self) -> Optional[dict]:
        sc = self.scenario
        if sc is None or self.sim is None:
            return None
        return {
            "id": sc["id"],
            "t": round(max(0.0, self.sim.t - sc["t0"]), 2),
            "step": sc["idx"],
            "steps": len(sc["steps"]),
            "done": sc["done"],
        }

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
                "modes": list(MODES),
                "puppet": {"connected": self.puppet_connected},
            }
            if sim is None:
                empty = ("physics", "motors", "thermal", "battery", "action_filter", "props", "fall_analysis",
                         "falls_log", "replay", "scenario")  # fmt: skip
                proc = self.course_name == "procedural"
                return {**base, "mode": None, "course": self.course_name,
                        "seed": self.seed if proc else None, "difficulty": self.difficulty if proc else None,
                        "geometry": course(self.course_name, seed=self.seed, difficulty=self.difficulty),
                        "cmd_limits": {"vx": [-0.5, 1.0], "vy": [-0.5, 0.5], "wz": [-1.0, 1.0]},
                        **{k: None for k in empty}}  # fmt: skip
            lim = sim.policy.deploy.cmd_limits if sim.policy else {"vx": [0, 0], "vy": [0, 0], "wz": [0, 0]}
            snap = sim.snapshot()
            if self.replay is not None:  # the replayed frame replaces the pose fields
                fr = self.replay["frames"][self._replay_index()]
                snap.update({k: fr[k] for k in ("t", "base", "q", "props", "physics")})
            return {
                **base,
                **snap,
                "cmd_limits": lim,
                "joint_names": list(sim.jm.names),
                "geometry": sim.geometry,
                "replay": self._replay_block(),
                "scenario": self._scenario_block(),
            }

    # ------------------------------------------------------------------ commands
    def start(self, course_name: str = "loop", mode: str = "policy", seed: int = 0, difficulty: float = 0.5) -> dict:
        if course_name not in COURSES:
            return {"ok": False, "error": f"unknown course {course_name!r}"}
        if mode not in MODES:
            return {"ok": False, "error": f"mode must be one of {', '.join(MODES)}, got {mode!r}"}
        try:
            seed, difficulty = int(seed), float(difficulty)
        except (TypeError, ValueError):
            return {"ok": False, "error": "seed must be an integer and difficulty a number"}
        if not 0.0 <= difficulty <= 1.0:
            return {"ok": False, "error": "difficulty must be in [0, 1]"}
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
            sim = WalkSim(course_name, mode=mode, policy=policy, seed=seed, difficulty=difficulty)
        except Exception as e:
            return {"ok": False, "error": f"walk scene failed to load: {e}"}
        with self.lock:
            self.sim = sim
            self.course_name = course_name
            self.seed, self.difficulty = seed, difficulty
            self.cmd_target[:] = 0.0
            self.cmd_at = time.monotonic()
            self.running = True
            self.replay = None
            self.pending_auto = None
            self.seen_falls = 0
            self.scenario = None
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._loop, name="walk-sim", daemon=True)
        self.thread.start()
        if self.mirror_factory is not None:
            self.puppet_thread = threading.Thread(target=self._puppet_loop, name="walk-puppet", daemon=True)
            self.puppet_thread.start()
        extra = f" seed={seed} difficulty={difficulty:g}" if course_name == "procedural" else ""
        self.log(f"walk: started course={course_name} mode={mode}{extra}")
        return {"ok": True}

    def command(self, vx: float, vy: float, wz: float) -> dict:
        with self.lock:
            if not self.running:
                return {"ok": False, "error": "walk sandbox is not running"}
            self.cmd_target[:] = [vx, vy, wz]
            self.cmd_at = time.monotonic()
            if self.sim is not None and self.sim.mode != "policy":
                return {"ok": True, "note": f"{self.sim.mode.replace('_', ' ')} mode ignores velocity commands"}
        return {"ok": True}

    def reset(self) -> dict:
        with self.lock:
            if self.sim is None:
                return {"ok": False, "error": "walk sandbox is not running"}
            self.replay = None
            self.pending_auto = None
            self.sim.reset()
            self.sim.falls = self.sim.laps = 0
            self.seen_falls = 0
            self.sim.last_lap_s = self.sim.best_lap_s = None
            self.cmd_target[:] = 0.0
            if self.scenario is not None:
                self.scenario["t0"] = 0.0
        return {"ok": True}

    def throw(self, kind: str = "ball", speed: float = 4.0, from_deg: float = 0.0) -> dict:
        with self.lock:
            if not self.running or self.sim is None:
                return {"ok": False, "error": "walk sandbox is not running"}
            try:
                return self.sim.throw(str(kind), float(speed), float(from_deg))
            except (TypeError, ValueError) as e:
                return {"ok": False, "error": str(e)}

    def set_filter(self, alpha=None, apply=None) -> dict:
        with self.lock:
            if self.sim is None:
                return {"ok": False, "error": "walk sandbox is not running"}
            try:
                self.sim.filter.set(None if alpha is None else float(alpha), None if apply is None else bool(apply))
            except (TypeError, ValueError) as e:
                return {"ok": False, "error": str(e)}
            return {"ok": True, "action_filter": self.sim.filter.block()}

    def start_replay(self, seconds: float = 3.0, speed: float = 0.25) -> dict:
        with self.lock:
            return self._start_replay_locked(seconds, speed)

    def _start_replay_locked(self, seconds: float, speed: float) -> dict:
        if not self.running or self.sim is None:
            return {"ok": False, "error": "walk sandbox is not running"}
        seconds, speed = float(seconds), float(speed)
        if not 1.0 <= seconds <= 10.0:
            return {"ok": False, "error": "seconds must be in [1, 10]"}
        if not 0.1 <= speed <= 1.0:
            return {"ok": False, "error": "speed must be in [0.1, 1]"}
        frames = list(self.sim.frames)
        t_end = frames[-1]["t"]
        frames = [f for f in frames if f["t"] >= t_end - seconds - 1e-9]
        if len(frames) < 2:
            return {"ok": False, "error": "nothing recorded yet"}
        self.replay = {"frames": frames, "speed": speed, "pos": 0.0, "duration_s": frames[-1]["t"] - frames[0]["t"]}
        self.replay_speed, self.replay_duration = speed, seconds
        return {"ok": True, "frames": len(frames), "duration_s": round(self.replay["duration_s"], 3)}

    def stop_replay(self) -> dict:
        with self.lock:
            self.replay = None
        return {"ok": True}

    def replay_post(self, body: dict) -> dict:
        """POST /walk/replay: {"seconds", "speed"} | {"stop": true} | {"auto_on_fall": bool} (may combine)."""
        res: dict = {"ok": True}
        if "auto_on_fall" in body:
            self.auto_on_fall = bool(body["auto_on_fall"])
            res["auto_on_fall"] = self.auto_on_fall
            if "seconds" not in body and "speed" not in body and not body.get("stop"):
                return res
        if body.get("stop"):
            return self.stop_replay()
        try:
            return self.start_replay(float(body.get("seconds", 3.0)), float(body.get("speed", 0.25)))
        except (TypeError, ValueError):
            return {"ok": False, "error": "seconds and speed must be numbers"}

    def _replay_index(self) -> int:
        r = self.replay
        t = r["frames"][0]["t"] + r["pos"]
        fr = r["frames"]
        lo, hi = 0, len(fr) - 1
        while lo < hi:  # last frame with t <= replay time
            mid = (lo + hi + 1) // 2
            if fr[mid]["t"] <= t + 1e-9:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def run_scenario(self, sc: dict) -> dict:
        """(Re)start the sandbox with the scenario's course/mode/seed and play its steps on sim time."""
        mode = sc.get("mode", "hold")
        note = None
        if mode == "policy" and not self.policy_status()[0] and sc.get("fallback_mode"):
            note = f"no validated Level 2 policy: running in {sc['fallback_mode']} mode"
            mode = sc["fallback_mode"]
        res = self.start(sc.get("course", "flat"), mode, sc.get("seed", 0), sc.get("difficulty", 0.5))
        if not res.get("ok"):
            return res
        with self.lock:
            self.scenario = {"id": sc["id"], "steps": list(sc["steps"]), "idx": 0, "t0": self.sim.t,
                             "duration_s": float(sc["duration_s"]), "done": False, "cmd": None}  # fmt: skip
        self.log(f"walk: scenario {sc['id']} ({len(sc['steps'])} steps)")
        return dict(res, scenario=sc["id"], mode=mode, **({"note": note} if note else {}))

    def stop_scenario(self) -> dict:
        with self.lock:
            if self.scenario is not None:
                self.scenario["done"] = True
                self.scenario["cmd"] = None
        return {"ok": True}

    def _scenario_tick(self) -> None:
        """Called with the lock held, before each sim step: run every due step."""
        sc, sim = self.scenario, self.sim
        if sc is None or sc["done"]:
            return
        t = sim.t - sc["t0"]
        while sc["idx"] < len(sc["steps"]) and float(sc["steps"][sc["idx"]]["t"]) <= t + 1e-9:
            st = sc["steps"][sc["idx"]]
            sc["idx"] += 1
            kind = st["do"]
            try:
                if kind == "command":
                    sc["cmd"] = [float(st.get(k, 0.0)) for k in ("vx", "vy", "wz")]
                elif kind == "throw":
                    sim.throw(st.get("kind", "ball"), float(st.get("speed", 4.0)), float(st.get("from_deg", 0.0)))
                elif kind == "reset":
                    sim.reset(keep_time=True)
                elif kind == "replay":
                    self._start_replay_locked(float(st.get("seconds", 3.0)), float(st.get("speed", 0.25)))
                elif kind == "filter":
                    sim.filter.set(st.get("alpha"), st.get("apply"))
            except (TypeError, ValueError) as e:
                self.log(f"walk: scenario step {kind} failed: {e}")
            if self.replay is not None:
                break  # the sim pauses: the remaining due steps run after the replay
        if sc["cmd"] is not None:
            self.cmd_target[:] = sc["cmd"]
            self.cmd_at = time.monotonic()
        if t >= sc["duration_s"] and sc["idx"] >= len(sc["steps"]):
            sc["done"] = True
            sc["cmd"] = None

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
            self.replay = None
        if was:
            self.log("walk: stopped")
        return {"ok": True}

    # ------------------------------------------------------------------ threads
    def _tick_locked(self):
        """One 20 ms tick (lock held): advance the replay, or step the simulation. Returns the pose to push."""
        sim = self.sim
        if self.replay is not None:
            r = self.replay
            r["pos"] += POLICY_DT * r["speed"]
            if r["pos"] > r["duration_s"] + 1e-9:
                self.replay = None  # done: the simulation resumes on the next tick
            else:
                fr = r["frames"][self._replay_index()]
                return fr["root"], fr["qarr"]
        self._scenario_tick()
        if self.replay is not None:  # the scenario just started one
            fr = self.replay["frames"][0]
            return fr["root"], fr["qarr"]
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
        if sim.falls > self.seen_falls:
            self.seen_falls = sim.falls
            if self.auto_on_fall and sim.fallen_at is not None:
                self.pending_auto = sim.fallen_at + AUTO_REPLAY_DELAY_S
        if self.pending_auto is not None and sim.t >= self.pending_auto - 1e-9:
            self.pending_auto = None
            self._start_replay_locked(AUTO_REPLAY["seconds"], AUTO_REPLAY["speed"])
        return sim.full_qpos()

    def _loop(self) -> None:
        next_t = time.perf_counter()
        while not self.stop_evt.is_set():
            with self.lock:
                root, q = self._tick_locked()
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
