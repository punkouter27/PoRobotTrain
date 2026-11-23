"""Motor heat + battery model, action filter and jitter score (docs/sim_features.md, features 5 and 15).

Display only: the model never derates a motor. Parameters come from ``RobotControl/configs/motor_model.json``
(the Unreal lab's C++ port ``R1MotorModel.h`` mirrors the same numbers; a pytest checks parity).

Per joint i, every policy step dt::

    f_i  = |tau_i| / effort_i
    T_i += dt * (heat * f_i^2 - (T_i - ambient) / time_constant)
    P    = idle_w + sum(max(tau_i * dq_i, 0)) / drive_efficiency + sum(copper_loss * f_i^2)
    soc -= P * dt / (battery_wh * 3600)

Jitter score = RMS of the action second difference ``a_t - 2 a_{t-1} + a_{t-2}`` over ``jitter_window_s``.
Filtered action = EMA ``y += alpha * (a - y)``.
"""

from __future__ import annotations

import collections
import json
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Deque, List, Optional, Sequence

import numpy as np

from .. import CONFIGS_DIR

CONFIG_PATH = CONFIGS_DIR / "motor_model.json"


@dataclass(frozen=True)
class MotorModelConfig:
    ambient_c: float = 25.0
    thermal_time_constant_s: float = 120.0
    heat_rate_c_per_s_at_full_torque: float = 0.8333
    warn_c: float = 70.0
    limit_c: float = 90.0
    battery_wh: float = 500.0
    idle_w: float = 60.0
    drive_efficiency: float = 0.8
    copper_loss_w_at_full_torque: float = 40.0
    torque_alert_fraction: float = 0.8
    action_filter_alpha: float = 0.5
    jitter_window_s: float = 1.0

    @classmethod
    def load(cls, path: Optional[Path] = None) -> MotorModelConfig:
        raw = json.loads(Path(path or CONFIG_PATH).read_text(encoding="utf-8"))
        known = {f.name for f in fields(cls)}
        unknown = set(raw) - known
        if unknown:
            raise ValueError(f"motor_model.json: unknown keys {sorted(unknown)}")
        return cls(**{k: float(v) for k, v in raw.items()})


class MotorModel:
    """Vectorised thermal + battery state for n joints. ``step`` takes torque and joint speed arrays (n,)."""

    def __init__(self, names: Sequence[str], effort: Sequence[float], cfg: Optional[MotorModelConfig] = None):
        self.cfg = cfg or MotorModelConfig.load()
        self.names = list(names)
        self.effort = np.maximum(np.asarray(effort, dtype=np.float64), 1e-9)
        if self.effort.shape != (len(self.names),):
            raise ValueError("effort must have one value per joint")
        self.reset()

    def reset(self) -> None:
        self.temp_c = np.full(len(self.names), self.cfg.ambient_c, dtype=np.float64)
        self.soc = 1.0
        self.power_w = self.cfg.idle_w
        self.torque = np.zeros(len(self.names))
        self.speed = np.zeros(len(self.names))
        self.frac = np.zeros(len(self.names))

    def step(self, tau: np.ndarray, dq: np.ndarray, dt: float) -> None:
        c = self.cfg
        tau = np.asarray(tau, dtype=np.float64)
        dq = np.asarray(dq, dtype=np.float64)
        f = np.abs(tau) / self.effort
        self.torque, self.speed, self.frac = tau.copy(), dq.copy(), f
        self.temp_c = self.temp_c + dt * (
            c.heat_rate_c_per_s_at_full_torque * f**2 - (self.temp_c - c.ambient_c) / c.thermal_time_constant_s
        )
        mech = float(np.sum(np.maximum(tau * dq, 0.0)))
        self.power_w = c.idle_w + mech / c.drive_efficiency + float(np.sum(c.copper_loss_w_at_full_torque * f**2))
        self.soc = max(0.0, self.soc - self.power_w * dt / (c.battery_wh * 3600.0))

    # ------------------------------------------------------------------ views
    def minutes_left(self) -> Optional[float]:
        if self.power_w <= 0:
            return None
        return self.soc * self.cfg.battery_wh / self.power_w * 60.0

    def motors_block(self) -> dict:
        return {
            "names": list(self.names),
            "speed": [round(float(v), 3) for v in self.speed],
            "torque": [round(float(v), 3) for v in self.torque],
            "torque_frac": [round(float(v), 3) for v in self.frac],
        }

    def thermal_block(self) -> dict:
        c = self.cfg
        i = int(np.argmax(self.temp_c))
        return {
            "temp_c": [round(float(v), 2) for v in self.temp_c],
            "hottest": self.names[i],
            "hottest_c": round(float(self.temp_c[i]), 2),
            "warn": [n for n, t in zip(self.names, self.temp_c) if c.warn_c <= t < c.limit_c],
            "limit": [n for n, t in zip(self.names, self.temp_c) if t >= c.limit_c],
        }

    def battery_block(self) -> dict:
        ml = self.minutes_left()
        return {
            "soc_pct": round(self.soc * 100.0, 3),
            "power_w": round(self.power_w, 1),
            "minutes_left": None if ml is None else round(ml, 1),
        }

    def torque_alert(self) -> List[str]:
        return [n for n, f in zip(self.names, self.frac) if f > self.cfg.torque_alert_fraction]


class ActionFilter:
    """EMA low-pass of the policy action plus the jitter score of the RAW action (what the policy asks for)."""

    def __init__(self, n: int, alpha: float = 0.5, window_s: float = 1.0, dt: float = 0.02, apply: bool = False):
        self.n = n
        self.alpha = float(alpha)
        self.apply = bool(apply)
        self.window = max(1, int(round(window_s / dt)))
        self.reset()

    def reset(self) -> None:
        self.y: Optional[np.ndarray] = None
        self.raw = np.zeros(self.n)
        self.hist: Deque[np.ndarray] = collections.deque(maxlen=3)
        self.d2: Deque[float] = collections.deque(maxlen=self.window)

    def set(self, alpha: Optional[float] = None, apply: Optional[bool] = None) -> None:
        if alpha is not None:
            if not 0.0 < float(alpha) <= 1.0:
                raise ValueError("alpha must be in (0, 1]")
            self.alpha = float(alpha)
        if apply is not None:
            self.apply = bool(apply)

    def step(self, a: np.ndarray) -> np.ndarray:
        """Feed the raw action; returns the filtered one (callers decide whether to apply it)."""
        a = np.asarray(a, dtype=np.float64).reshape(self.n)
        self.raw = a.copy()
        self.y = a.copy() if self.y is None else self.y + self.alpha * (a - self.y)
        self.hist.append(a.copy())
        if len(self.hist) == 3:
            d = self.hist[2] - 2.0 * self.hist[1] + self.hist[0]
            self.d2.append(float(np.mean(d**2)))
        return self.y.copy()

    @property
    def filtered(self) -> np.ndarray:
        return self.raw.copy() if self.y is None else self.y.copy()

    def jitter(self) -> float:
        return float(np.sqrt(np.mean(self.d2))) if self.d2 else 0.0

    def block(self) -> dict:
        return {
            "alpha": self.alpha,
            "apply": self.apply,
            "raw": [round(float(v), 4) for v in self.raw],
            "filtered": [round(float(v), 4) for v in self.filtered],
            "jitter": round(self.jitter(), 5),
        }


def jitter_score(actions: np.ndarray) -> float:
    """RMS of the second difference over a whole action sequence (T, n) (duel tool)."""
    a = np.asarray(actions, dtype=np.float64)
    if a.ndim == 1:
        a = a[:, None]
    if len(a) < 3:
        return 0.0
    d = a[2:] - 2.0 * a[1:-1] + a[:-2]
    return float(np.sqrt(np.mean(d**2)))
