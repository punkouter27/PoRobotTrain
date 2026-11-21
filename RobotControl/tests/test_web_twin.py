"""Contract checks for the browser twin assets written by scripts/build_web_twin.py.

The Blazor UI's 3D view, Rapier pose check, reach kernel and in-browser policy-parity check all read these files,
so they must stay aligned with r1_joint_map.yaml and with the validated Level 0 policy.
"""

from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path

import numpy as np
import pytest

from RobotControl import MODELS_DIR, PROJECT_ROOT
from RobotControl.configs.joint_map import JointMap

TWIN_DIR = PROJECT_ROOT / "src" / "PoRobotTrain.Client" / "wwwroot" / "twin"
POLICY = MODELS_DIR / "r1_level0_upperbody" / "policy.onnx"

pytestmark = pytest.mark.skipif(not (TWIN_DIR / "twin_manifest.json").exists(), reason="run scripts/build_web_twin.py")


def _manifest() -> dict:
    return json.loads((TWIN_DIR / "twin_manifest.json").read_text(encoding="utf-8"))


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def test_joint_order_limits_and_defaults_follow_the_joint_map():
    jm = JointMap.load()
    joints = _manifest()["joints"]
    assert [j["name"] for j in joints] == jm.names
    assert [j["policyIndex"] for j in joints] == list(range(len(jm.names)))
    np.testing.assert_allclose([j["minDeg"] for j in joints], np.degrees(jm.limits_lo), atol=1e-3)
    np.testing.assert_allclose([j["maxDeg"] for j in joints], np.degrees(jm.limits_hi), atol=1e-3)
    np.testing.assert_allclose([j["defaultDeg"] for j in joints], np.degrees(jm.defaults), atol=1e-3)


def test_browser_policy_is_the_validated_policy_byte_for_byte():
    man = _manifest()
    served = TWIN_DIR / "policy.onnx"
    assert _sha(served) == _sha(POLICY), "wwwroot/twin/policy.onnx differs from the validated policy; rerun the build"
    assert man["policy"]["sha256"] == man["policy"]["sourceSha256"] == _sha(POLICY)


def test_policy_golden_actions_reproduce_with_onnxruntime():
    ort = pytest.importorskip("onnxruntime")
    golden = json.loads((TWIN_DIR / "policy_golden.json").read_text(encoding="utf-8"))
    sess = ort.InferenceSession(str(POLICY), providers=["CPUExecutionProvider"])
    obs = np.asarray(golden["obs"], dtype=np.float32)
    assert obs.shape[1] == golden["obsDim"] == 23
    act = sess.run([golden["output"]], {golden["input"]: obs})[0]
    assert act.shape[1] == golden["actDim"] == 7
    np.testing.assert_allclose(act, np.asarray(golden["actions"], dtype=np.float32), atol=golden["tolerance"])


def test_glb_is_valid_draco_gltf_with_every_body():
    man = _manifest()
    data = (TWIN_DIR / man["glb"]["file"]).read_bytes()
    magic, version, total = struct.unpack_from("<III", data, 0)
    assert magic == 0x46546C67 and version == 2 and total == len(data)
    json_len, chunk_type = struct.unpack_from("<II", data, 12)
    assert chunk_type == 0x4E4F534A
    gltf = json.loads(data[20 : 20 + json_len])
    node_names = {n.get("name") for n in gltf["nodes"]}
    assert {b["name"] for b in man["bodies"]} <= node_names
    assert man["glb"]["sha256"] == hashlib.sha256(data).hexdigest()
    if man["glb"]["draco"]:
        assert "KHR_draco_mesh_compression" in gltf.get("extensionsUsed", [])
        assert len(data) < 2_000_000, "Draco glb should stay small for the browser"


def test_fk_golden_matches_mujoco_now():
    mujoco = pytest.importorskip("mujoco")
    from RobotControl import MJCF_DIR

    man = _manifest()
    m = mujoco.MjModel.from_xml_path(str(MJCF_DIR / "scene_stand.xml"))
    d = mujoco.MjData(m)
    home = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
    for g in man["fkGolden"]:
        mujoco.mj_resetDataKeyframe(m, d, home)
        for j in man["joints"]:
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j["name"])
            d.qpos[m.jnt_qposadr[jid]] = np.radians(g["qDeg"][j["policyIndex"]])
        mujoco.mj_kinematics(m, d)
        for name, ref in g["bodies"].items():
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            np.testing.assert_allclose(d.xpos[bid], ref["pos"], atol=1e-5, err_msg=f"{name} moved: rerun the build")
