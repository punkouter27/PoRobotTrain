"""Training montage (#11) and synthetic head-camera data (#7). Offscreen rendering: skipped without OpenGL."""

import json

import numpy as np
import pytest

from robotcontrol.behaviours import head_skills as hs


def _need_gl():
    import mujoco

    try:
        m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
        r = mujoco.Renderer(m, 8, 8)
        r.render()
        r.close()
    except Exception as e:  # no OpenGL context on this machine
        pytest.skip(f"offscreen OpenGL unavailable: {e}")


def test_checkpoint_format_and_montage(tmp_path):
    torch = pytest.importorskip("torch")
    from robotcontrol.tools import training_montage as M
    from robotcontrol.training.ppo import ActorCritic
    from robotcontrol.training.train_head_skill import save_checkpoint

    sk = hs.SKILLS["hold_centre"]
    assert M.montage_status("hold_centre", tmp_path)["ok"] is False  # no checkpoints yet
    torch.manual_seed(0)
    ac = ActorCritic(hs.obs_size(sk), len(sk.powered), hidden=(128, 64))
    for it in (1, 5, 10):
        save_checkpoint(ac, tmp_path / "checkpoints", it)
    assert [i for i, _ in M.checkpoints(tmp_path)] == [1, 5, 10]
    ac2 = ActorCritic(hs.obs_size(sk), len(sk.powered), hidden=(128, 64))
    ac2.load_state_dict(torch.load(tmp_path / "checkpoints" / "iter_00005.pt", map_location="cpu"))  # model.pt format
    x = torch.zeros(1, hs.obs_size(sk))
    assert torch.allclose(ac.act_inference(x), ac2.act_inference(x))
    assert (
        M.pick([(i, None) for i in range(20)], 4)[0][0] == 0 and M.pick([(i, None) for i in range(20)], 4)[-1][0] == 19
    )
    _need_gl()
    meta = M.make_montage("hold_centre", tmp_path, seconds=0.3)
    assert meta["checkpoints"] == [1, 5, 10] and meta["frames"] == 3 * 3 and meta["size"] == [320, 240]
    assert json.loads((tmp_path / "montage.json").read_text(encoding="utf-8"))["frames"] == 9
    from PIL import Image

    gif = Image.open(tmp_path / "montage.gif")
    assert gif.size == (320, 240) and gif.n_frames == 9
    st = M.montage_status("hold_centre", tmp_path)
    assert st["ok"] and st["checkpoints"] == [1, 5, 10] and len(st["gif_base64"]) > 100


def test_synth_camera_frames(tmp_path):
    _need_gl()
    from PIL import Image

    from robotcontrol.tools import synth_camera as SC

    labels = SC.generate(tmp_path / "cam", 2, width=96, height=72, seed=1, log=lambda s: None)
    assert len(labels["frames"]) == 2 and labels["camera"] == "head_rgbd"
    for f in labels["frames"]:
        rgb = np.array(Image.open(tmp_path / "cam" / f["rgb"]))
        seg = np.array(Image.open(tmp_path / "cam" / f["seg"]))
        depth = np.load(tmp_path / "cam" / f["depth"])
        assert rgb.shape == (72, 96, 3) and seg.shape == (72, 96) and depth.shape == (72, 96)
        assert depth.dtype == np.float32 and np.all(depth > 0)
        assert -30 <= f["head"]["yaw_deg"] <= 30 and -10 <= f["head"]["pitch_deg"] <= 20
        for o in f["objects"]:
            x0, y0, x1, y1 = o["bbox"]
            assert o["class"] in SC.CLASSES and o["colour"] in SC.COLOURS
            assert 0 <= x0 < x1 <= 96 and 0 <= y0 < y1 <= 72
            assert (seg[y0:y1, x0:x1] == o["id"]).any() and (seg == o["id"]).sum() == o["pixels"]
    saved = json.loads((tmp_path / "cam" / "labels.json").read_text(encoding="utf-8"))
    assert saved == labels
    assert SC.bbox(np.zeros((4, 4), np.uint8), 1) is None
