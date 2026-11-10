"""Alpha-blend between two action (or target) streams when switching policies.

    a_t = (1 - alpha_k) * a_old + alpha_k * a_new,   alpha_k = k / N  (k = 1..N)

optionally with a smoothstep ramp and a per-step slew clamp on the output.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


def linear_ramp(k: int, n: int) -> float:
    return float(np.clip(k / max(1, n), 0.0, 1.0))


def smoothstep_ramp(k: int, n: int) -> float:
    x = linear_ramp(k, n)
    return float(x * x * (3.0 - 2.0 * x))


class PolicyBlender:
    def __init__(self, num_steps: int = 10, max_slew: Optional[float | np.ndarray] = None,
                 ramp: str = "linear"):
        if num_steps < 1:
            raise ValueError("num_steps must be >= 1")
        self.num_steps = int(num_steps)
        self.max_slew = None if max_slew is None else np.asarray(max_slew, dtype=np.float64)
        self._ramp = smoothstep_ramp if ramp == "smoothstep" else linear_ramp
        self._k = 0
        self._active = False
        self._last: Optional[np.ndarray] = None

    # -------------------------------------------------------------- control
    def start(self, last_output: Optional[np.ndarray] = None) -> None:
        """Begin a new blend. ``last_output`` seeds the slew clamp."""
        self._k = 0
        self._active = True
        self._last = None if last_output is None else np.asarray(last_output, dtype=np.float64).copy()

    @property
    def active(self) -> bool:
        return self._active

    @property
    def done(self) -> bool:
        return not self._active

    @property
    def alpha(self) -> float:
        if not self._active:
            return 1.0
        return self._ramp(self._k, self.num_steps)

    # -------------------------------------------------------------- blending
    def blend(self, old_action: np.ndarray, new_action: np.ndarray) -> np.ndarray:
        """Advance one step and return the blended output."""
        old = np.asarray(old_action, dtype=np.float64)
        new = np.asarray(new_action, dtype=np.float64)
        if old.shape != new.shape:
            raise ValueError(f"shape mismatch {old.shape} vs {new.shape}")
        if self._active:
            self._k += 1
            a = self._ramp(self._k, self.num_steps)
            out = (1.0 - a) * old + a * new
            if self._k >= self.num_steps:
                self._active = False
        else:
            out = new.copy()
        out = self._slew(out)
        self._last = out.copy()
        return out

    def _slew(self, out: np.ndarray) -> np.ndarray:
        if self.max_slew is None or self._last is None:
            return out
        delta = np.clip(out - self._last, -self.max_slew, self.max_slew)
        return self._last + delta

    def reset(self) -> None:
        self._k = 0
        self._active = False
        self._last = None


def blend_sequence(old_action: np.ndarray, new_action: np.ndarray, num_steps: int = 10,
                   ramp: str = "linear") -> np.ndarray:
    """Offline helper: the full (num_steps, A) blended trajectory."""
    b = PolicyBlender(num_steps, ramp=ramp)
    b.start()
    return np.stack([b.blend(old_action, new_action) for _ in range(num_steps)])
