"""Sim-to-real gap check (docs/sim_features.md #4): run log listing, safe names, comparison, downsampling."""

import numpy as np
import pytest

from robotcontrol.behaviours import service as S
from robotcontrol.configs.joint_map import JointMap
from robotcontrol.tools import gap_check as G


def _write_log(path, n, offset_deg=0.0, dt=0.005, t0=0.0):
    names = JointMap.load().names
    head = ["t", "phase", "target_head_pitch_joint", "target_head_yaw_joint"] + list(names)
    rows = []
    for k in range(n):
        t = t0 + k * dt
        q = np.zeros(len(names))
        q[names.index("head_yaw_joint")] = 0.3 * np.sin(t - t0) + np.radians(offset_deg)  # both runs start at t0
        rows.append([f"{t:.4f}", "skill", "0", "0"] + [f"{v:.5f}" for v in q])
    path.write_text("\n".join(",".join(r) for r in [head] + rows) + "\n", encoding="utf-8")


def test_gap_between_two_logs(tmp_path):
    _write_log(tmp_path / "look_at_sim_20261122_100000.csv", 2000)
    _write_log(tmp_path / "look_at_real_20261122_110000.csv", 1500, offset_deg=2.0, t0=5.0)
    (tmp_path / "notes.csv").write_text("x\n", encoding="utf-8")
    runs = G.list_runs(tmp_path)
    assert [r["target"] for r in runs] == ["real", "sim"]  # newest first; non-run files ignored
    assert runs[0] == {"file": "look_at_real_20261122_110000.csv", "skill": "look_at", "target": "real",
                       "stamp": "20261122_110000"}  # fmt: skip
    res = G.gap("look_at_sim_20261122_100000.csv", "look_at_real_20261122_110000.csv", 5.0, tmp_path)
    assert res["ok"] and res["pass"] and len(res["joints"]) == 26 and len(res["t"]) <= G.MAX_POINTS
    j = res["joints"].index("head_yaw_joint")
    assert res["max_deg"][j] == pytest.approx(2.0, abs=0.05) and res["rms_deg"][j] == pytest.approx(2.0, abs=0.05)
    assert len(res["a_deg"]) == len(res["t"]) == len(res["err_deg"]) and len(res["a_deg"][0]) == 26
    assert res["worst"] == "head_yaw_joint" and res["overlap_s"] == pytest.approx(1499 * 0.005, abs=0.01)
    assert not G.gap("look_at_sim_20261122_100000.csv", "look_at_real_20261122_110000.csv", 1.0, tmp_path)["pass"]


@pytest.mark.parametrize("name", ["../secret.csv", "a/b.csv", "..\\x.csv", "C:\\x.csv", "x.txt", ""])
def test_paths_are_refused(name, tmp_path):
    with pytest.raises(ValueError):
        G.safe_log_path(name, tmp_path)


def test_service_gap_and_runs():
    svc = S.Service(mirror_enabled=False)
    assert svc.runs()["ok"]
    res = svc.gap({"a": "../pyproject.toml", "b": "x.csv"})
    assert not res["ok"] and res["_code"] == 400
