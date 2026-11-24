"""Train one head skill (PPO, domain randomisation), export ONNX + deploy.yaml, then run the validation gate.

    .venv/Scripts/python.exe -u -m robotcontrol.training.train_head_skill look_at
    .venv/Scripts/python.exe -u -m robotcontrol.training.train_head_skill all --iters 250
    .venv/Scripts/python.exe -u -m robotcontrol.training.train_head_skill look_at --checkpoint-every 25

``--checkpoint-every N`` saves the policy weights every N iterations (same format as model.pt: the ActorCritic
state_dict on the CPU) into ``models/head_skills/<skill>/checkpoints/iter_XXXXX.pt`` for the training montage
(``tools/training_montage.py``); a new run with checkpoints replaces the previous run's. ``--scratch DIR`` trains,
exports and validates into DIR only (checkpoints in DIR/checkpoints): no version is stored or activated.

Outputs (RobotControl/models/head_skills/<skill>/): policy.onnx, deploy.yaml (with the validation block),
model.pt, train_log.csv, validation.json. Prints "@progress k/n" lines that the behaviour service turns into the
Train progress bar in the Blazor client and the Unreal panel. The previous policy is kept until the new one is
exported, and a skill only runs when its validation passed (behaviours/head_skills.py: load_validated).
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np
import torch
import yaml

from .. import LOGS_DIR, MODELS_DIR, PROJECT_ROOT
from ..behaviours import head_skills as hs
from .head_skill_env import HeadSkillConfig, HeadSkillEnv
from .level0_env import format_penalties
from .ppo import PPO, ActorCritic

DEFAULT_ITERS = {"hold_centre": 120, "look_at": 300, "nod_yes": 250, "shake_no": 250, "glance_around": 300}
POSE_ITERS = 150  # taught poses (pose_<id>)
CHECKPOINTS = "checkpoints"


class ExportWrapper(torch.nn.Module):
    def __init__(self, ac: ActorCritic):
        super().__init__()
        self.ac = ac

    def forward(self, obs):
        return self.ac.act_inference(obs)


def out_dir(skill_id: str) -> Path:
    return MODELS_DIR / hs.SKILLS_SUBDIR / skill_id


def checkpoint_dir(skill_id: str) -> Path:
    return out_dir(skill_id) / CHECKPOINTS


def save_checkpoint(ac: ActorCritic, folder: Path, it: int) -> Path:
    """Weights at iteration ``it`` in model.pt's format (CPU state_dict), loadable by ActorCritic.load_state_dict."""
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"iter_{it:05d}.pt"
    torch.save({k: v.detach().cpu().clone() for k, v in ac.state_dict().items()}, path)
    return path


def _git() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except Exception:
        return "nogit"


def export(ac: ActorCritic, env: HeadSkillEnv, out: Path, meta: dict) -> Path:
    import onnx

    out.mkdir(parents=True, exist_ok=True)
    ac = ac.cpu().eval()
    torch.save(ac.state_dict(), out / "model.pt")
    onnx_path = out / "policy.onnx"
    torch.onnx.export(
        ExportWrapper(ac),
        torch.zeros(1, env.num_obs),
        str(onnx_path),
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=17,
        dynamo=False,
    )
    model = onnx.load(str(onnx_path))
    for k, v in meta.items():
        p = model.metadata_props.add()
        p.key, p.value = k, str(v)
    onnx.save(model, str(onnx_path))
    sk, jm = env.skill, env.jm
    idx = [jm.index_of(j) for j in sk.powered]
    deploy = {
        "policy_name": f"r1_head_{sk.id}",
        "skill": sk.id,
        "label": sk.label,
        "step_dt": hs.POLICY_DT,
        "num_obs": env.num_obs,
        "num_actions": env.num_actions,
        "joint_names": list(sk.powered),
        "motor_slots": [int(jm.motor_slots[i]) for i in idx],
        "default_joint_pos": [float(jm.defaults[i]) for i in idx],
        "kp": [float(jm.kp[i]) for i in idx],
        "kd": [float(jm.kd[i]) for i in idx],
        "action_scale": [float(jm.action_scale[i]) for i in idx],
        "action_clip": [hs.ACTION_CLIP] * env.num_actions,
        "soft_limit_factor": hs.SOFT_LIMIT_FACTOR,
        "observations": [
            {"name": "joint_pos_rel", "size": env.num_actions},
            {"name": "joint_vel", "size": env.num_actions},
            {"name": "last_action", "size": env.num_actions},
            {"name": "command", "size": hs.NUM_CMD},
        ],
        "command": "hold: [0,0,0]; look_at: [yaw, pitch, 0] rad; periodic: [sin, cos, size]",
        "period_s": sk.period_s,
        "max_deg": sk.max_deg,
        "metadata": meta,
    }
    (out / "deploy.yaml").write_text(yaml.safe_dump(deploy, sort_keys=False), encoding="utf-8")
    return onnx_path


def train(
    skill_id: str,
    iters: int,
    num_envs: int = 512,
    horizon: int = 24,
    seed: int = 0,
    progress=(0, 1),
    checkpoint_every: int = 0,
    scratch: Optional[Path] = None,
) -> Path:
    """Train, export into a staging folder, validate, then move it into place. Returns the policy folder.

    ``scratch``: export + validate into that folder only (no version stored or activated)."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(seed)
    env = HeadSkillEnv(HeadSkillConfig(skill=skill_id, num_envs=num_envs, seed=seed))
    ac = ActorCritic(env.num_obs, env.num_actions, hidden=(128, 64), init_noise_std=0.8)
    ppo = PPO(ac, device)
    N, T = num_envs, horizon
    print(f"[{skill_id}] device={device} envs={N} obs={env.num_obs} act={env.num_actions} iters={iters}", flush=True)
    staging = LOGS_DIR / "head_skills" / "staging"  # an interrupted run leaves nothing in models/
    staging.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f"{skill_id}_", dir=str(staging)))
    ckpt_dir = (Path(scratch) / CHECKPOINTS) if scratch else checkpoint_dir(skill_id)
    if checkpoint_every > 0:  # one run per montage: drop the previous run's checkpoints
        for old in ckpt_dir.glob("iter_*.pt") if ckpt_dir.is_dir() else []:
            old.unlink()
    logf = open(stage / "train_log.csv", "w", newline="", encoding="utf-8")
    w = csv.writer(logf)
    w.writerow(["iter", "time_s", "reward_mean", "track_err_deg", "max_acc", "max_frc", "kl", "lr", "std", "fps"])
    shp = lambda *s: torch.zeros(*s, device=device)  # noqa: E731
    obs_b, act_b, mu_b, sig_b = shp(T, N, env.num_obs), shp(T, N, env.num_actions), None, None
    mu_b, sig_b = torch.zeros_like(act_b), torch.zeros_like(act_b)
    logp_b, val_b, rew_b, done_b = shp(T, N), shp(T, N), shp(T, N), shp(T, N)
    obs = torch.from_numpy(env.get_obs()).to(device)
    t0 = time.time()
    k0, ktot = progress
    try:
        for it in range(1, iters + 1):
            t_it = time.time()
            infos = []
            for t in range(T):
                ac.obs_norm.update(obs)
                a, logp, v, mu, sig = ppo.act(obs)
                nobs, r, d, info = env.step(a.cpu().numpy())
                obs_b[t], act_b[t], logp_b[t], val_b[t], mu_b[t], sig_b[t] = obs, a, logp, v, mu, sig
                rew_b[t] = torch.from_numpy(r).to(device)
                done_b[t] = torch.from_numpy(d.astype(np.float32)).to(device)
                obs = torch.from_numpy(nobs).to(device)
                infos.append(info)
            with torch.no_grad():
                last_v = ac.value(obs)
            adv, ret = ppo.compute_gae(rew_b, done_b, val_b, last_v)
            st = ppo.update((obs_b, act_b, logp_b, val_b, adv, ret, mu_b, sig_b))
            err = float(np.mean([i["track_err_deg"] for i in infos]))
            std = ac.log_std.detach().exp().mean().item()
            fps = N * T / (time.time() - t_it)
            row = [it, round(time.time() - t0, 1), float(rew_b.mean()), err, max(i["max_acc"] for i in infos),
                   max(i["max_frc"] for i in infos), st["kl"], st["lr"], std, fps]  # fmt: skip
            w.writerow([f"{x:.5g}" if isinstance(x, float) else x for x in row])
            logf.flush()
            print(f"@progress {k0 + it}/{ktot}", flush=True)
            if checkpoint_every > 0 and (it == 1 or it % checkpoint_every == 0 or it == iters):
                save_checkpoint(ac, ckpt_dir, it)
            if it == 1 or it % 10 == 0 or it == iters:
                print(
                    f"[{skill_id}] it {it:4d}/{iters} | reward {row[2]:6.3f} | track err {err:5.2f} deg | "
                    f"std {std:.3f} | {fps:6.0f} steps/s | {row[1]:6.0f} s",
                    flush=True,
                )
                print(f"[{skill_id}] penalties | {format_penalties(infos)}", flush=True)
    finally:
        logf.close()
        env.close()
    meta = {
        "trained_at": dt.datetime.now().isoformat(timespec="seconds"),
        "skill": skill_id,
        "iters": iters,
        "num_envs": N,
        "horizon": T,
        "seed": seed,
        "git": _git(),
        "device": str(device),
        "env": "HeadSkillEnv",
        "train_time_s": round(time.time() - t0, 1),
        "joint_map_sha256": hashlib.sha256((PROJECT_ROOT / "RobotControl/configs/r1_joint_map.yaml").read_bytes())
        .hexdigest()[:16],
    }  # fmt: skip
    export(ac, env, stage, meta)
    from ..behaviours import skill_versions as sv
    from .validate_head_skill import validate

    report = validate(skill_id, stage)
    if scratch:
        dest = Path(scratch)
        dest.mkdir(parents=True, exist_ok=True)
        for f in stage.iterdir():
            shutil.move(str(f), str(dest / f.name))
        stage.rmdir()
        verdict = "PASSED" if report["passed"] else "FAILED: " + "; ".join(report["failures"])
        print(f"[{skill_id}] scratch run in {dest}: validation {verdict} (nothing stored or activated)", flush=True)
        return dest
    version = sv.next_version(skill_id)
    stored = sv.store(skill_id, stage, version)
    verdict = "PASSED" if report["passed"] else "FAILED: " + "; ".join(report["failures"])
    print(f"[{skill_id}] v{version} validation {verdict}", flush=True)
    if report["passed"]:
        sv.activate(skill_id, version)  # a failed retrain never replaces a working version
        print(f"[{skill_id}] v{version} is now the active version", flush=True)
    else:
        print(f"[{skill_id}] kept the active version v{sv.active_version(skill_id)}", flush=True)
    print(f"[{skill_id}] stored: {stored}", flush=True)
    return out_dir(skill_id)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Train a head skill and export a validated ONNX policy")
    ap.add_argument("skill", choices=sorted(hs.SKILLS) + ["all"])
    ap.add_argument("--iters", type=int, default=None, help="PPO iterations (default: per skill)")
    ap.add_argument("--num-envs", type=int, default=512)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--checkpoint-every", type=int, default=0, help="save weights every N iterations (montage)")
    ap.add_argument("--scratch", type=Path, default=None, help="train into this folder only; store no version")
    args = ap.parse_args(argv)
    if args.checkpoint_every < 0:
        ap.error("--checkpoint-every must be >= 0")
    (MODELS_DIR / hs.SKILLS_SUBDIR).mkdir(parents=True, exist_ok=True)
    skills = list(hs.SKILLS) if args.skill == "all" else [args.skill]
    plan = [(s, args.iters or DEFAULT_ITERS.get(s, POSE_ITERS)) for s in skills]
    total, done, ok = sum(n for _, n in plan), 0, True
    if args.scratch and len(plan) > 1:
        ap.error("--scratch trains one skill")
    for s, n in plan:
        dest = train(s, n, args.num_envs, seed=args.seed, progress=(done, total),
                     checkpoint_every=args.checkpoint_every, scratch=args.scratch)  # fmt: skip
        done += n
        if args.scratch:
            return 0
        ok = ok and hs.skill_status(s)["state"] == "ready"
        print(f"[{s}] status: {hs.skill_status(s)['state']} ({dest})", flush=True)
    return 0 if ok else 4


if __name__ == "__main__":
    sys.exit(main())
