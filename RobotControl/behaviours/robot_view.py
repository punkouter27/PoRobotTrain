"""Live 3D MuJoCo offscreen render of the Unitree R1 mounted on its test stand."""
from __future__ import annotations

import math
import queue
import struct
import threading
import zlib
from typing import Optional, Sequence

import numpy as np

from .. import MJCF_DIR
from ..configs.joint_map import JointMap

CAMERA_PRESETS = {
    "iso": (136.0, -14.0, 2.35),
    "front": (180.0, -10.0, 2.25),
    "side": (90.0, -10.0, 2.30),
    "back": (25.0, -14.0, 2.35),
}


def _rgb_to_png(rgb: np.ndarray) -> bytes:
    """Encode an HxWx3 uint8 numpy array into a PNG without requiring Pillow."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, level=4))
        + chunk(b"IEND", b"")
    )


class _GlRenderWorker:
    """Owns the MuJoCo OpenGL Renderer on one dedicated thread (Windows GL contexts are thread-affine)."""

    def __init__(self, width: int = 520, height: int = 420):
        self.width = width
        self.height = height
        self._jobs: queue.Queue = queue.Queue()
        self._thread: Optional[threading.Thread] = None
        self._start_lock = threading.Lock()
        self._cache_lock = threading.Lock()
        self._last_key = None
        self._last_png: Optional[bytes] = None

    def _ensure_started(self) -> None:
        if self._thread is not None:
            return
        with self._start_lock:
            if self._thread is None:
                t = threading.Thread(target=self._run, name="mujoco-gl-render", daemon=True)
                t.start()
                self._thread = t

    def _run(self) -> None:
        import mujoco
        jm = JointMap.load()
        m = mujoco.MjModel.from_xml_path(str(MJCF_DIR / "scene_stand.xml"))
        d = mujoco.MjData(m)
        home_k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in jm.names]
        qadr = np.array([m.jnt_qposadr[j] for j in jid], dtype=np.int64)
        renderer = mujoco.Renderer(m, height=self.height, width=self.width)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [-0.08, 0.0, 0.78]

        while True:
            q_key, az, el, dist, cache_key, done_ev, box = self._jobs.get()
            try:
                with self._cache_lock:
                    if cache_key == self._last_key and self._last_png is not None:
                        box["png"] = self._last_png
                        continue
                if home_k >= 0:
                    mujoco.mj_resetDataKeyframe(m, d, home_k)
                else:
                    mujoco.mj_resetData(m, d)
                if q_key:
                    d.qpos[qadr] = [math.radians(v) for v in q_key]
                mujoco.mj_forward(m, d)
                cam.distance, cam.azimuth, cam.elevation = dist, az, el
                renderer.update_scene(d, camera=cam)
                png = _rgb_to_png(renderer.render())
                with self._cache_lock:
                    self._last_key = cache_key
                    self._last_png = png
                box["png"] = png
            except Exception as exc:
                box["err"] = exc
            finally:
                done_ev.set()

    def render(self, q_key: tuple, az: float, el: float, dist: float) -> bytes:
        cache_key = (q_key, az, el, dist)
        with self._cache_lock:
            if cache_key == self._last_key and self._last_png is not None:
                return self._last_png
        self._ensure_started()
        done_ev = threading.Event()
        box: dict = {}
        self._jobs.put((q_key, az, el, dist, cache_key, done_ev, box))
        done_ev.wait(timeout=15.0)
        if "err" in box:
            raise box["err"]
        if "png" not in box:
            raise TimeoutError("MuJoCo render worker timed out")
        return box["png"]


_WORKER = _GlRenderWorker()


class StandRenderer:
    """Thread-safe facade over the dedicated MuJoCo OpenGL render thread."""

    def __init__(self, jm: JointMap, width: int = 520, height: int = 420):
        self.jm = jm
        self.width = width
        self.height = height

    def render_png(self, q_deg: Optional[Sequence[float]] = None, cam_name: str = "iso") -> bytes:
        az, el, dist = CAMERA_PRESETS.get(cam_name, CAMERA_PRESETS["iso"])
        q_key = tuple(round(float(v), 1) for v in q_deg) if q_deg and len(q_deg) == self.jm.num_joints else ()
        return _WORKER.render(q_key, az, el, dist)
