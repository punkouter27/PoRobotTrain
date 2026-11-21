"""Robustness explorer: case layout, verdict rule, and a short end-to-end sweep against the deployed policy."""

import json

import numpy as np
import pytest

from robotcontrol.tools import robustness_sweep as R


def test_cases_layout():
    for quick, levels in ((True, 5), (False, 7)):
        cases, sens, grid = R.build_cases(quick)
        assert cases[0] == R.NOMINAL
        assert len(sens) == len(R.FACTORS) * levels and len(grid) == levels * levels
        assert len(cases) == 1 + len(sens) + len(grid)
        # each sensitivity case changes exactly one factor
        for c in cases[1 : 1 + len(sens)]:
            assert sum(c[k] != v for k, v in R.NOMINAL.items()) <= 1


def test_sweep_command_matches_validation_sweep():
    t = np.linspace(0, 8, 33)
    c = R.sweep_command(t, 0.35)
    np.testing.assert_allclose(c[:, 0], np.deg2rad(30) * np.sin(2 * np.pi * 0.25 * t))
    np.testing.assert_allclose(c[:, 1], 0.35 - 0.6 * (1 - np.cos(2 * np.pi * 0.15 * t)))


@pytest.mark.parametrize(
    "cell,v",
    [
        ({"finite": True, "trips": 0, "mean_err_deg": 1.0, "max_acc": 20.0}, "pass"),
        ({"finite": True, "trips": 0, "mean_err_deg": 2.5, "max_acc": 20.0}, "warn"),
        ({"finite": True, "trips": 0, "mean_err_deg": 1.0, "max_acc": 70.0}, "warn"),
        ({"finite": True, "trips": 0, "mean_err_deg": 3.5, "max_acc": 20.0}, "fail"),
        ({"finite": True, "trips": 0, "mean_err_deg": 1.0, "max_acc": 90.0}, "fail"),
        ({"finite": True, "trips": 2, "mean_err_deg": 1.0, "max_acc": 20.0}, "fail"),
        ({"finite": False, "trips": 0, "mean_err_deg": 0.0, "max_acc": 0.0}, "fail"),
    ],
)
def test_verdict(cell, v):
    assert R.verdict(cell) == v


def test_short_sweep_writes_contract_report(tmp_path):
    rep = R.run_sweep(quick=True, rollout_s=1.5, nthread=4)
    R.write_report(rep, tmp_path)
    data = json.loads((tmp_path / "latest.json").read_text(encoding="utf-8"))
    assert data["available"] and data["quick"] and data["num_cases"] == 56
    assert [f["id"] for f in data["sensitivity"]] == [f.id for f in R.FACTORS]
    assert all(len(f["cells"]) == len(f["levels"]) == 5 for f in data["sensitivity"])
    it = data["interaction"]
    assert (it["x"]["id"], it["y"]["id"]) == ("gain", "latency_ms")
    assert len(it["cells"]) == 5 and all(len(r) == 5 for r in it["cells"])
    for c in [data["nominal"]] + [c for f in data["sensitivity"] for c in f["cells"]]:
        assert c["verdict"] in ("pass", "warn", "fail")
        assert c["acc_joint"].endswith("_joint")
    s = data["summary"]
    assert s["pass"] + s["warn"] + s["fail"] == 55 and s["worst"]
    assert (tmp_path / "report.md").exists()
    try:  # the heatmap PNG is optional (CI installs no matplotlib)
        import matplotlib  # noqa: F401
    except ImportError:
        assert not (tmp_path / "heatmap.png").exists()
    else:
        assert (tmp_path / "heatmap.png").exists()
