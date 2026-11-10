import json

import numpy as np
import pytest

from robotcontrol.configs.deploy_config import DeployConfig
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.execution.policy_runner import OnnxPolicy, PolicyRunner
from robotcontrol.tools import deploy_pack, onnx_inspect, onnx_replay, onnx_validate, sim_real_diff

from .fixtures import build_fake_onnx, fake_policy_forward, make_level0_bundle

pytest.importorskip("onnx")
pytest.importorskip("onnxruntime")


@pytest.fixture(scope="module")
def jm():
    return JointMap.load()


@pytest.fixture(scope="module")
def bundle(tmp_path_factory, jm):
    out = tmp_path_factory.mktemp("level0_policy")
    return make_level0_bundle(out, jm, seed=1)


def test_onnx_policy_infer_matches_reference(bundle):
    onnx_path, _, cfg = bundle
    pol = OnnxPolicy(onnx_path)
    assert pol.input_name == "obs" and pol.output_name == "actions"
    assert pol.num_obs == 23 and pol.num_actions == 7
    rng = np.random.default_rng(0)
    for _ in range(5):
        obs = rng.standard_normal(23).astype(np.float32)
        out = pol.infer(obs)
        ref = fake_policy_forward(onnx_path, obs[None])[0]
        assert np.allclose(out, ref, atol=1e-5)
    # the returned array is the internal buffer (no allocation per step)
    a = pol.infer(np.zeros(23, np.float32))
    b = pol.infer(np.ones(23, np.float32))
    assert a is b


def test_onnx_inspect_detects_normalization(bundle):
    onnx_path, _, _ = bundle
    info = onnx_inspect.inspect_onnx(onnx_path)
    assert info["inputs"][0]["name"] == "obs" and info["inputs"][0]["shape"] == [1, 23]
    assert info["outputs"][0]["name"] == "actions" and info["outputs"][0]["shape"] == [1, 7]
    assert info["normalization"]["detected"] and info["normalization"]["pattern"] == "Sub -> Div"
    assert info["metadata_props"]["policy_name"] == "fake"
    txt = onnx_inspect.format_report(info)
    assert "normalization: YES" in txt
    assert onnx_inspect.main([str(onnx_path)]) == 0


def test_onnx_validate_pass_and_fail(bundle, tmp_path, jm):
    onnx_path, yaml_path, cfg = bundle
    errors, warnings = onnx_validate.validate(onnx_path, yaml_path)
    assert errors == [], errors
    assert onnx_validate.main([str(onnx_path), "--deploy", str(yaml_path)]) == 0

    # wrong num_obs
    bad = DeployConfig.load(yaml_path)
    bad.num_obs = 24
    bad_path = bad.save(tmp_path / "bad_obs.yaml")
    errors, _ = onnx_validate.validate(onnx_path, bad_path)
    assert any("num_obs" in e for e in errors)
    assert onnx_validate.main([str(onnx_path), "--deploy", str(bad_path)]) == 1

    # unknown joint name and wrong slots
    bad2 = DeployConfig.load(yaml_path)
    bad2.joint_names[0] = "not_a_joint"
    bad2_path = bad2.save(tmp_path / "bad_joint.yaml")
    errors, _ = onnx_validate.validate(onnx_path, bad2_path)
    assert any("not in joint map" in e for e in errors)

    bad3 = DeployConfig.load(yaml_path)
    bad3.motor_slots[0] = 15
    bad3_path = bad3.save(tmp_path / "bad_slots.yaml")
    errors, _ = onnx_validate.validate(onnx_path, bad3_path)
    assert any("motor_slots" in e for e in errors)

    # wrong graph shape vs deploy
    other = build_fake_onnx(tmp_path / "other.onnx", 23, 6)
    errors, _ = onnx_validate.validate(other, yaml_path)
    assert any("num_actions" in e for e in errors)


def test_onnx_replay(bundle, tmp_path):
    onnx_path, _, _ = bundle
    rng = np.random.default_rng(3)
    obs = rng.standard_normal((50, 23)).astype(np.float32)
    acts = fake_policy_forward(onnx_path, obs)
    npz = tmp_path / "replay.npz"
    np.savez(npz, obs=obs, actions=acts)
    res = onnx_replay.replay(onnx_path, npz, tol=1e-4)
    assert res["passed"] and res["max_abs_err"] < 1e-4
    assert onnx_replay.main([str(onnx_path), str(npz)]) == 0
    np.savez(npz, obs=obs, actions=acts + 0.01)
    assert onnx_replay.main([str(onnx_path), str(npz)]) == 1


def test_deploy_pack(bundle, tmp_path):
    onnx_path, yaml_path, cfg = bundle
    out = deploy_pack.pack(onnx_path, yaml_path, out_dir=tmp_path / "dist")
    assert out.name.startswith("r1_level0_upperbody_")
    manifest = json.load(open(out / "MANIFEST.json", encoding="utf-8"))
    assert set(manifest["files"]) == {"policy.onnx", "deploy.yaml", "r1_joint_map.yaml"}
    assert manifest["files"]["policy.onnx"]["sha256"] == deploy_pack.sha256_file(onnx_path)
    assert deploy_pack.verify_bundle(out) == []
    (out / "deploy.yaml").write_text("tampered", encoding="utf-8")
    assert deploy_pack.verify_bundle(out) == ["sha256 mismatch deploy.yaml"]


def test_policy_runner_in_sim_logs_trajectory(bundle, tmp_path, jm):
    pytest.importorskip("mujoco")
    from robotcontrol.interfaces.mujoco_interface import MujocoInterface

    out = tmp_path / "zero_policy"
    onnx_path, yaml_path, cfg = make_level0_bundle(out, jm, zero_weights=True)
    pol = OnnxPolicy(onnx_path)
    deploy = DeployConfig.load(yaml_path)
    statuses = []
    with MujocoInterface(jm, scene="stand") as itf:
        runner = PolicyRunner(itf, pol, deploy, jm, command_fn=lambda t, s: np.array([0.2, 0.35]),
                              log_dir=tmp_path / "logs", log_stem="sim_run", realtime=False,
                              status_hz=1000.0, status_fn=statuses.append)
        # all 19 non-policy joints are passive: kp 0, kd from the map
        assert np.count_nonzero(runner.kp_full == 0.0) == 19
        assert np.allclose(runner.kd_full[runner.passive_idx], jm.kd[runner.passive_idx])
        status = runner.run(duration_s=1.0)
    assert status["steps"] == 50 and not status["tripped"]
    assert statuses
    npz = tmp_path / "logs" / "sim_run.npz"
    csv_path = tmp_path / "logs" / "sim_run.csv"
    assert npz.exists() and csv_path.exists()
    d = np.load(npz)
    assert d["q"].shape == (50, 26) and d["obs"].shape == (50, 23) and d["action"].shape == (50, 7)
    assert d["target_q"].shape == (50, 26) and d["command"].shape == (50, 2)
    # zero-weight policy -> action 0 -> active joints hold their defaults
    act_idx = deploy.policy_indices(jm)
    assert np.allclose(d["target_q"][-1, act_idx], deploy.default_joint_pos, atol=1e-6)
    assert np.degrees(np.abs(d["q"][-1, act_idx] - deploy.default_joint_pos)).max() < 3.0

    # sim_real_diff on the log against itself (shifted) must pass
    t, q, names = sim_real_diff.load_log(npz)
    res = sim_real_diff.compare(t, q, t, q + np.radians(1.0))
    assert np.allclose(res["max_deg"], 1.0, atol=1e-6)
    assert sim_real_diff.main([str(npz), str(csv_path), "--tol-deg", "0.5"]) == 0
