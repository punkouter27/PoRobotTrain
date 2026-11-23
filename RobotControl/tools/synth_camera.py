"""Synthetic head-camera data (docs/sim_features.md #7): labelled RGB / depth / segmentation frames from the R1's
``head_rgbd`` camera in the MuJoCo stand scene.

    python -m robotcontrol.tools.synth_camera --frames 200 --out datasets/head_cam

Every frame randomises the head pose (yaw +-30 deg, pitch -10..20 deg), a light (position, colour, intensity) and up
to MAX_OBJECTS primitive objects (ball, box, cylinder, capsule; size, colour, placed on the floor or floating in the
camera's view). Writes ``rgb/00000.png``, ``depth/00000.npy`` (float32 metres), ``seg/00000.png`` (uint8 object id
per pixel, 0 = background / robot / floor) and ``labels.json`` (per frame: head yaw/pitch, light, objects with class,
colour, bbox [x0, y0, x1, y1] from the segmentation; occluded objects are left out). ``datasets/`` is git-ignored.
Simulation only (headless MuJoCo, offscreen rendering).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

from .. import PROJECT_ROOT

MAX_OBJECTS = 6
CLASSES = ("ball", "box", "cylinder", "capsule")
COLOURS = {
    "red": (0.85, 0.15, 0.12),
    "green": (0.15, 0.7, 0.25),
    "blue": (0.15, 0.3, 0.85),
    "yellow": (0.95, 0.85, 0.15),
    "orange": (0.95, 0.5, 0.1),
    "purple": (0.55, 0.2, 0.75),
    "white": (0.92, 0.92, 0.92),
    "black": (0.08, 0.08, 0.08),
}
CAMERA = "head_rgbd"
FAR = (100.0, 100.0, -50.0)  # unused object slots


def _hook(spec) -> None:
    import mujoco

    spec.worldbody.add_geom(name="synth_floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05],
                            rgba=[0.45, 0.45, 0.48, 1], contype=0, conaffinity=0)  # fmt: skip
    spec.worldbody.add_light(name="synth_light", pos=[1.0, 0.0, 3.0], dir=[0.0, 0.0, -1.0], castshadow=True)
    for k in range(MAX_OBJECTS):
        spec.worldbody.add_geom(name=f"synth_obj_{k}", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1, 0.1, 0.1],
                                pos=list(FAR), rgba=[1, 1, 1, 1], contype=0, conaffinity=0)  # fmt: skip


class SynthCamera:
    def __init__(self, width: int = 320, height: int = 240, seed: int = 0):
        import mujoco

        from ..sim_features.stand_sim import VISUAL_XML, StandSim

        self.mj = mujoco
        self.sim = StandSim(VISUAL_XML, spec_hook=_hook)
        m = self.sim.m
        if width > m.vis.global_.offwidth or height > m.vis.global_.offheight:
            m.vis.global_.offwidth, m.vis.global_.offheight = max(width, 640), max(height, 480)
        self.w, self.h = width, height
        self.rng = np.random.default_rng(seed)
        self.cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA)
        self.light = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "synth_light")
        self.obj = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"synth_obj_{k}") for k in range(MAX_OBJECTS)]
        self.types = {
            "ball": mujoco.mjtGeom.mjGEOM_SPHERE,
            "box": mujoco.mjtGeom.mjGEOM_BOX,
            "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
            "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE,
        }
        self.rgb_r = mujoco.Renderer(m, height, width)
        self.depth_r = mujoco.Renderer(m, height, width)
        self.depth_r.enable_depth_rendering()
        self.seg_r = mujoco.Renderer(m, height, width)
        self.seg_r.enable_segmentation_rendering()

    def close(self) -> None:
        for r in (self.rgb_r, self.depth_r, self.seg_r):
            r.close()

    def _ray(self, u: float, v: float) -> tuple:
        """World origin + unit direction of the camera ray through pixel (u, v)."""
        m, d = self.sim.m, self.sim.d
        f = (self.h / 2) / math.tan(math.radians(m.cam_fovy[self.cam]) / 2)
        local = np.array([(u - self.w / 2) / f, -(v - self.h / 2) / f, -1.0])  # camera looks along -z, y up
        R = d.cam_xmat[self.cam].reshape(3, 3)
        dvec = R @ local
        return d.cam_xpos[self.cam].copy(), dvec / np.linalg.norm(dvec)

    def randomise(self) -> dict:
        rng, m = self.rng, self.sim.m
        yaw, pitch = rng.uniform(-30, 30), rng.uniform(-10, 20)
        self.sim.set_head(math.radians(yaw), math.radians(pitch))
        lpos = [float(rng.uniform(-1, 3)), float(rng.uniform(-2, 2)), float(rng.uniform(2, 4))]
        tint = rng.uniform(0.7, 1.0, 3) * rng.uniform(0.4, 1.0)
        m.light_pos[self.light] = lpos
        m.light_dir[self.light] = -np.array(lpos) / np.linalg.norm(lpos)
        m.light_diffuse[self.light] = tint
        m.vis.headlight.diffuse[:] = rng.uniform(0.1, 0.4)
        objs = []
        n = int(rng.integers(1, MAX_OBJECTS + 1))
        for k, g in enumerate(self.obj):
            if k >= n:
                m.geom_pos[g] = FAR
                continue
            cls = CLASSES[int(rng.integers(len(CLASSES)))]
            cname = list(COLOURS)[int(rng.integers(len(COLOURS)))]
            rgb = np.clip(np.array(COLOURS[cname]) + rng.uniform(-0.06, 0.06, 3), 0, 1)
            s = float(rng.uniform(0.05, 0.2))
            size = {"ball": [s, s, s], "box": [s, s * rng.uniform(0.6, 1.4), s * rng.uniform(0.6, 1.4)],
                    "cylinder": [s, s * rng.uniform(0.8, 2.0), 0], "capsule": [s * 0.6, s * 1.5, 0]}[cls]  # fmt: skip
            o, dvec = self._ray(rng.uniform(0.1, 0.9) * self.w, rng.uniform(0.1, 0.9) * self.h)
            if rng.random() < 0.5 and dvec[2] < -0.05:  # on the floor where the ray meets it
                dist = (o[2] - size[0]) / -dvec[2]
            else:
                dist = rng.uniform(0.8, 3.0)
            pos = o + dvec * min(dist, 4.0)
            pos[2] = max(pos[2], size[0])
            m.geom_type[g] = self.types[cls]
            m.geom_size[g] = size
            m.geom_rbound[g] = float(np.linalg.norm(size)) + 0.05
            m.geom_pos[g] = pos
            q = rng.normal(size=4)
            m.geom_quat[g] = q / np.linalg.norm(q)
            m.geom_rgba[g] = [*rgb, 1.0]
            objs.append({"id": k + 1, "class": cls, "colour": cname, "rgb": [round(float(c), 3) for c in rgb],
                         "pos": [round(float(c), 3) for c in pos], "size": [round(float(c), 3) for c in size]})  # fmt: skip
        self.mj.mj_forward(m, self.sim.d)
        return {
            "head": {"yaw_deg": round(float(yaw), 2), "pitch_deg": round(float(pitch), 2)},
            "light": {"pos": [round(v, 3) for v in lpos], "diffuse": [round(float(c), 3) for c in tint]},
            "objects": objs,
        }

    def render(self) -> tuple:
        d = self.sim.d
        out = []
        for r in (self.rgb_r, self.depth_r, self.seg_r):
            r.update_scene(d, camera=self.cam)
            out.append(r.render())
        rgb, depth, seg_raw = out
        seg = np.zeros((self.h, self.w), dtype=np.uint8)
        geom_ids = seg_raw[..., 0]
        is_geom = seg_raw[..., 1] == int(self.mj.mjtObj.mjOBJ_GEOM)
        for k, g in enumerate(self.obj):
            seg[(geom_ids == g) & is_geom] = k + 1
        return rgb, depth.astype(np.float32), seg


def bbox(seg: np.ndarray, obj_id: int) -> Optional[List[int]]:
    ys, xs = np.nonzero(seg == obj_id)
    if xs.size == 0:
        return None
    return [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]


def generate(out: Path, frames: int, width: int = 320, height: int = 240, seed: int = 0, log=print) -> dict:
    from PIL import Image

    out = Path(out)
    for sub in ("rgb", "depth", "seg"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    cam = SynthCamera(width, height, seed)
    labels = {"camera": CAMERA, "width": width, "height": height, "seed": seed, "classes": list(CLASSES),
              "fovy_deg": float(cam.sim.m.cam_fovy[cam.cam]), "depth_unit": "m", "frames": []}  # fmt: skip
    try:
        for i in range(frames):
            info = cam.randomise()
            rgb, depth, seg = cam.render()
            name = f"{i:05d}"
            Image.fromarray(rgb).save(out / "rgb" / f"{name}.png")
            np.save(out / "depth" / f"{name}.npy", depth)
            Image.fromarray(seg, mode="L").save(out / "seg" / f"{name}.png")
            visible = []
            for o in info["objects"]:
                b = bbox(seg, o["id"])
                if b is not None:
                    visible.append(dict(o, bbox=b, pixels=int((seg == o["id"]).sum())))
            labels["frames"].append(
                {"frame": i, "rgb": f"rgb/{name}.png", "depth": f"depth/{name}.npy", "seg": f"seg/{name}.png",
                 "head": info["head"], "light": info["light"], "objects": visible}
            )  # fmt: skip
            if (i + 1) % 50 == 0:
                log(f"{i + 1}/{frames} frames")
    finally:
        cam.close()
    (out / "labels.json").write_text(json.dumps(labels, indent=1), encoding="utf-8")
    return labels


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Synthetic head-camera dataset (RGB, depth, segmentation, labels)")
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT / "datasets" / "head_cam")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    if not 1 <= args.frames <= 100000:
        ap.error("--frames must be in [1, 100000]")
    if not (16 <= args.width <= 1280 and 16 <= args.height <= 960):
        ap.error("--width/--height out of range")
    labels = generate(args.out, args.frames, args.width, args.height, args.seed)
    n_obj = sum(len(f["objects"]) for f in labels["frames"])
    print(f"wrote {args.frames} frames ({n_obj} visible objects) to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
