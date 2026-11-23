"""Training montage (docs/sim_features.md #11): how a head skill learned, one short clip per training checkpoint.

    python -m robotcontrol.training.train_head_skill look_at --checkpoint-every 25     # writes checkpoints
    python -m robotcontrol.tools.training_montage look_at                               # montage.gif + montage.json

Each checkpoint (``checkpoints/iter_XXXXX.pt``, model.pt's format) drives the head in the headless MuJoCo stand
scene (``r1_twin_stand.xml``, offscreen ``mujoco.Renderer``) for a few seconds of the skill's demo commands, through
the runtime's observation / action code. Frames are captioned "iter N" and written as one small GIF (320x240, 10 fps)
plus ``montage.json`` into the skill folder (or ``--dir``).
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

from ..behaviours import head_skills as hs
from ..behaviours.neck_turn import SweepParams

WIDTH, HEIGHT = 320, 240
FPS = 10
SECONDS_PER_CHECKPOINT = 3.0
MAX_CHECKPOINTS = 8
CKPT_RE = re.compile(r"^iter_(\d+)\.pt$")


def skill_folder(skill_id: str) -> Path:
    from ..behaviours.skill_versions import active_dir

    return active_dir(skill_id)


def checkpoints(folder: Path) -> List[tuple]:
    """[(iteration, path)] in ``folder/checkpoints``, by iteration."""
    d = folder / "checkpoints"
    if not d.is_dir():
        return []
    out = []
    for p in d.iterdir():
        m = CKPT_RE.match(p.name)
        if m:
            out.append((int(m.group(1)), p))
    return sorted(out)


def pick(cks: List[tuple], n: int = MAX_CHECKPOINTS) -> List[tuple]:
    """At most n checkpoints, evenly spread, always the first and the last."""
    if len(cks) <= n:
        return cks
    idx = sorted({int(round(i)) for i in np.linspace(0, len(cks) - 1, n)})
    return [cks[i] for i in idx]


def load_policy(skill: hs.HeadSkill, path: Path):
    import torch

    from ..training.ppo import ActorCritic

    ac = ActorCritic(hs.obs_size(skill), len(skill.powered), hidden=(128, 64))
    ac.load_state_dict(torch.load(path, map_location="cpu"))
    ac.eval()

    def pol(obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            return ac.act_inference(torch.from_numpy(np.asarray(obs, dtype=np.float32).reshape(1, -1))).numpy()[0]

    return pol


def _demo_command(skill: hs.HeadSkill, beh, t: float) -> np.ndarray:
    if skill.id == "look_at":  # start mid-way so every clip shows a target jump
        y, p = hs.look_at_script(t + 1.5, 30.0)
        return np.array([y, p, 0.0])
    if skill.period_s > 0:
        return hs.phase_command(skill, np.array(t), np.array(1.0)).reshape(hs.NUM_CMD)
    return np.zeros(hs.NUM_CMD)


def render_clip(skill: hs.HeadSkill, policy, renderer, sim, caption: str, seconds: float) -> List:
    from PIL import Image, ImageDraw

    import mujoco

    sim.reset()
    beh = hs.SKILL_CLASSES[skill.id](
        sim.jm, SweepParams(amplitude_deg=skill.max_deg, duration_s=seconds), check_tracking=False, policy=policy
    )
    beh.run_s = 1e9
    beh.command_at = lambda t: _demo_command(skill, beh, t)
    idx = [sim.jm.index_of(j) for j in skill.powered]
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    head = sim.d.xpos[sim.head_body].copy()
    cam.lookat[:] = head + np.array([0.0, 0.0, -0.05])
    cam.distance, cam.azimuth, cam.elevation = 1.1, 150.0, -12.0
    every = max(1, int(round(1.0 / (FPS * 0.02))))
    frames = []
    for k in range(int(round(seconds / 0.02))):
        t = k * 0.02
        q, dq = sim.q[idx], sim.dq[idx]
        beh._policy_step(t, q, dq)
        sim.step({i: float(v) for i, v in zip(idx, beh.target)})
        if k % every == 0:
            renderer.update_scene(sim.d, camera=cam)
            img = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(img)
            draw.rectangle([0, 0, WIDTH, 18], fill=(11, 15, 23))
            draw.text((6, 3), caption, fill=(230, 233, 242))
            frames.append(img)
    return frames


def make_montage(skill_id: str, folder: Optional[Path] = None, seconds: float = SECONDS_PER_CHECKPOINT) -> dict:
    """Render the montage; returns montage.json's content. Raises ValueError when there are no checkpoints."""
    import mujoco

    from ..sim_features.stand_sim import VISUAL_XML, StandSim

    if skill_id not in hs.SKILLS:
        raise ValueError(f"{skill_id!r} is not a trained skill")
    skill = hs.SKILLS[skill_id]
    folder = Path(folder) if folder else skill_folder(skill_id)
    cks = pick(checkpoints(folder))
    if not cks:
        raise ValueError("no checkpoints: train with --checkpoint-every N")
    sim = StandSim(VISUAL_XML)
    renderer = mujoco.Renderer(sim.m, HEIGHT, WIDTH)
    try:
        frames = []
        last = cks[-1][0]
        for it, path in cks:
            frames += render_clip(skill, load_policy(skill, path), renderer, sim, f"{skill.label}  iter {it} / {last}",
                                  seconds)  # fmt: skip
    finally:
        renderer.close()
    gif = folder / "montage.gif"
    pal = [f.convert("P", palette=1, colors=128) for f in frames]  # 1 = Image.ADAPTIVE
    pal[0].save(gif, save_all=True, append_images=pal[1:], duration=int(1000 / FPS), loop=0, optimize=True)
    meta = {
        "skill": skill_id,
        "checkpoints": [it for it, _ in cks],
        "frames": len(frames),
        "fps": FPS,
        "size": [WIDTH, HEIGHT],
        "seconds_per_checkpoint": seconds,
        "gif_bytes": gif.stat().st_size,
        "created": dt.datetime.now().isoformat(timespec="seconds"),
    }
    (folder / "montage.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    return meta


def montage_status(skill_id: str, folder: Optional[Path] = None, build: bool = True) -> dict:
    """For GET /skills/montage: the GIF (rebuilt first when a checkpoint is newer), or ok false with a reason."""
    import base64

    if skill_id not in hs.SKILLS:
        return {"ok": False, "error": f"{skill_id!r} is not a trained skill"}
    folder = Path(folder) if folder else skill_folder(skill_id)
    cks = checkpoints(folder)
    gif, js = folder / "montage.gif", folder / "montage.json"
    if not cks and not gif.exists():
        return {"ok": False, "error": "no checkpoints: train with --checkpoint-every N"}
    stale = not gif.exists() or not js.exists() or any(p.stat().st_mtime > gif.stat().st_mtime for _, p in cks)
    if stale and cks:
        if not build:
            return {"ok": False, "error": "montage not built yet"}
        try:
            make_montage(skill_id, folder)
        except Exception as e:  # no OpenGL, broken checkpoint
            return {"ok": False, "error": f"montage failed: {str(e).splitlines()[0][:120]}"}
    meta = json.loads(js.read_text(encoding="utf-8"))
    return {
        "ok": True,
        "skill": skill_id,
        "frames": meta["frames"],
        "checkpoints": meta["checkpoints"],
        "fps": meta.get("fps", FPS),
        "gif_base64": base64.b64encode(gif.read_bytes()).decode("ascii"),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Render a training montage GIF from a skill's checkpoints")
    ap.add_argument("skill", choices=sorted(hs.SKILLS))
    ap.add_argument("--dir", type=Path, default=None, help="skill folder (default: models/head_skills/<skill>)")
    ap.add_argument("--seconds", type=float, default=SECONDS_PER_CHECKPOINT)
    args = ap.parse_args(argv)
    try:
        meta = make_montage(args.skill, args.dir, args.seconds)
    except ValueError as e:
        print(e)
        return 1
    print(json.dumps(meta, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
