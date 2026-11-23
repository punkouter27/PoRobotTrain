"""Policy-vs-policy duel (docs/sim_features.md #10): two versions of a head skill, the same seeded lab scenario.

    python -m robotcontrol.tools.skill_duel look_at --a current --b 2 --scenario lab_stress

Headless MuJoCo stand scene (``sim_features/stand_sim.py``, the training env's physics). The policy runs through
the runtime code (``TrainedHeadBehaviour._policy_step``: observation, action clip, soft limits) plus the runtime's
torque-cap band; a copy of the runtime's safety channels (``JointSafetyChannel``) tells whether the real runner
would have stopped. Lab scenario steps: ``target`` (look_at target), ``push`` (0.25 s torque on the head, the lab
panel's Push), ``noise`` (uniform sensor noise on the observed angles), ``delay`` (extra action delay, steps),
``strength`` (head kp/kd scaled). Same seed for both versions, so both see identical noise.

Score (lower wins) = sum of WEIGHTS[k] * metric[k]; ``would_stop`` adds STOP_PENALTY.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from ..behaviours import head_skills as hs
from ..behaviours import skill_versions as sv
from ..behaviours.neck_turn import SweepParams
from ..sim_features.motor_model import jitter_score
from ..sim_features.scenarios import load as load_scenario
from ..sim_features.stand_sim import POLICY_DT, StandSim

PUSH_S = 0.25
PUSH_TORQUE = (1.2, 5.0)  # Nm about the yaw / pitch axis (R1SkillBrain.cpp Push)
SETTLE_S = 1.0  # tracking error counts from 1 s after a target change
WEIGHTS = {"err_mean_deg": 1.0, "err_max_deg": 0.1, "torque_rms": 0.2, "jitter": 10.0, "energy_j": 0.02}
STOP_PENALTY = 100.0
TIE = 0.02  # relative score difference below which the duel is a tie


def policy_dir(skill_id: str, version: Optional[int]) -> Path:
    if version is None:
        return sv.active_dir(skill_id)
    return sv.versions_dir(skill_id) / f"v{int(version)}"


def run_one(skill_id: str, version: Optional[int], sc: dict, policy=None) -> dict:
    """Play the lab scenario with one policy version. ``policy`` (obs -> action) overrides the ONNX file (tests)."""
    if skill_id not in hs.SKILLS:
        raise ValueError(f"{skill_id!r} is not a trained skill")
    sk = hs.SKILLS[skill_id]
    folder = policy_dir(skill_id, version)
    if policy is None:
        if not (folder / "policy.onnx").exists():
            label = "current" if version is None else f"v{version}"
            raise ValueError(f"{sk.label} has no {label} policy")
        policy = hs.OnnxPolicy(folder / "policy.onnx", hs.obs_size(sk))
    sim = StandSim()
    jm = sim.jm
    size_deg = min(float(sc.get("size_deg", sk.max_deg)), sk.max_deg)
    beh = hs.SKILL_CLASSES[skill_id](
        jm, SweepParams(amplitude_deg=max(size_deg, 1.0), duration_s=sc["duration_s"]), check_tracking=True,
        policy=policy,
    )  # fmt: skip
    beh.run_s = 1e9  # the scenario decides the commands
    idx = [jm.index_of(j) for j in sk.powered]
    head = [jm.index_of(hs.HEAD_PITCH), jm.index_of(hs.HEAD_YAW)]
    st = {"target": (0.0, 0.0), "changed": 0.0, "noise": 0.0, "delay": 0, "push_until": -1.0, "push": (0.0, 0.0)}

    def command_at(t: float) -> np.ndarray:
        if sk.id == "look_at":
            y, p = st["target"]
            return np.array([y, p, 0.0])
        if sk.period_s > 0:
            return hs.phase_command(sk, np.array(t), np.array(beh.size)).reshape(hs.NUM_CMD)
        return np.zeros(hs.NUM_CMD)

    beh.command_at = command_at
    rng = np.random.default_rng(int(sc.get("seed", 0)))
    steps = list(sc["steps"])
    ring = collections.deque(maxlen=5)
    n = int(round(float(sc["duration_s"]) / POLICY_DT))
    errs, settled, taus, dqs, acts = [], [], [], [], []
    would_stop = ""
    k_step = 0
    for k in range(n):
        t = k * POLICY_DT
        while k_step < len(steps) and float(steps[k_step]["t"]) <= t + 1e-9:
            s = steps[k_step]
            k_step += 1
            if s["do"] == "target":
                y = math.radians(float(s["yaw_deg"]))
                p = math.radians(float(s["pitch_deg"]))
                if sk.yaw_range > 0:
                    y = float(np.clip(y, -sk.yaw_range, sk.yaw_range))
                    p = float(np.clip(p, -sk.pitch_range, sk.pitch_range))
                st["target"], st["changed"] = (y, p), t
            elif s["do"] == "push":
                st["push"] = (float(s.get("yaw", 0.0)), float(s.get("pitch", 0.0)))
                st["push_until"] = t + PUSH_S
            elif s["do"] == "noise":
                st["noise"] = math.radians(float(s["deg"]))
            elif s["do"] == "delay":
                st["delay"] = int(s["steps"])
            elif s["do"] == "strength":
                sim.set_strength(head, float(s["pct"]))
        q_all, dq_all = sim.q, sim.dq
        q, dq = q_all[idx], dq_all[idx]
        noise = rng.uniform(-st["noise"], st["noise"], len(idx)) if st["noise"] > 0 else np.zeros(len(idx))
        beh._policy_step(t, q + noise, dq)
        acts.append(beh.last_action.copy())
        band = np.array([beh.channels[j].effort / max(beh.channels[j].kp, 1e-6) for j in sk.powered])
        target = np.clip(beh.target, q - band, q + band)
        ring.append(target)
        applied = ring[max(0, len(ring) - 1 - st["delay"])]
        torque = None
        if t < st["push_until"]:
            yaw_dir, pitch_dir = st["push"]
            torque = [0.0, PUSH_TORQUE[1] * pitch_dir, PUSH_TORQUE[0] * yaw_dir]
        sim.step({i: float(v) for i, v in zip(idx, applied)}, head_torque=torque)
        q_new = sim.q[idx]
        ref = beh.reference_at(t + POLICY_DT)
        e = np.degrees(np.abs(q_new - ref))
        errs.append(float(e.mean()))
        settled.append(sk.id != "look_at" or (t + POLICY_DT - st["changed"]) >= SETTLE_S)
        taus.append(sim.d.actuator_force[idx].copy())
        dqs.append(sim.dq[idx])
        if not would_stop:  # the runtime safety channels, fed the measured (noisy) angle like on the robot
            for k_j, j in enumerate(sk.powered):
                ch = beh.channels[j]
                if ch.filter(j, t, float(ref[k_j]), float(q_new[k_j] + noise[k_j]), float(dqs[-1][k_j]), 0.0) is None:
                    would_stop = f"t={t:.1f}s {ch.abort_reason}"
                    break
    errs_a, mask = np.array(errs), np.array(settled)
    if not mask.any():
        mask[:] = True
    tau, dqa = np.array(taus), np.array(dqs)
    metrics = {
        "err_mean_deg": round(float(errs_a[mask].mean()), 3),
        "err_max_deg": round(float(errs_a[mask].max()), 3),
        "torque_rms": round(float(np.sqrt(np.mean(tau**2))), 4),
        "jitter": round(jitter_score(np.array(acts)), 4),
        "energy_j": round(float(np.sum(np.maximum(tau * dqa, 0.0)) * POLICY_DT), 4),
    }
    score = sum(WEIGHTS[k] * metrics[k] for k in WEIGHTS) + (STOP_PENALTY if would_stop else 0.0)
    ver = version if version is not None else sv.active_version(skill_id)
    return {"version": ver, **metrics, "would_stop": would_stop, "score": round(score, 4)}


def _why(a: dict, b: dict, winner: str) -> str:
    if winner == "tie":
        return f"scores within {TIE * 100:.0f} % ({a['score']:.2f} vs {b['score']:.2f})"
    w, lo = (a, b) if winner == "a" else (b, a)
    if bool(lo["would_stop"]) != bool(w["would_stop"]):
        return f"{'b' if winner == 'a' else 'a'} would have been stopped by the safety check ({lo['would_stop']})"
    labels = {"err_mean_deg": "tracks {:.2f} deg closer on average", "err_max_deg": "has a {:.2f} deg smaller worst error",
              "torque_rms": "uses {:.3f} Nm less torque (RMS)", "jitter": "jitters {:.3f} less",
              "energy_j": "spends {:.2f} J less energy"}  # fmt: skip
    gains = sorted(((WEIGHTS[k] * (lo[k] - w[k]), k) for k in WEIGHTS), reverse=True)
    parts = [labels[k].format(lo[k] - w[k]) for g, k in gains[:2] if g > 0]
    return (
        f"{winner} " + (" and ".join(parts) if parts else "scores lower") + f" ({w['score']:.2f} vs {lo['score']:.2f})"
    )


def duel(skill_id: str, a: Optional[int], b: Optional[int], scenario: str = "lab_stress") -> dict:
    sc = load_scenario(scenario)
    if sc["target"] != "lab":
        raise ValueError(f"scenario {scenario!r} is not a lab scenario")
    ra = run_one(skill_id, a, sc)
    rb = run_one(skill_id, b, sc)
    hi = max(abs(ra["score"]), abs(rb["score"]), 1e-9)
    if abs(ra["score"] - rb["score"]) <= TIE * hi:
        winner = "tie"
    else:
        winner = "a" if ra["score"] < rb["score"] else "b"
    return {"skill": skill_id, "scenario": scenario, "a": ra, "b": rb, "winner": winner, "why": _why(ra, rb, winner)}


def _ver(s: str) -> Optional[int]:
    return None if s in ("current", "none", "null", "") else int(s)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Duel two versions of a head skill under a lab scenario")
    ap.add_argument("skill", choices=sorted(hs.SKILLS))
    ap.add_argument("--a", default="current", help="version number or 'current'")
    ap.add_argument("--b", default="current")
    ap.add_argument("--scenario", default="lab_stress")
    args = ap.parse_args(argv)
    print(json.dumps(duel(args.skill, _ver(args.a), _ver(args.b), args.scenario), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
