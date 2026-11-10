"""Train the Level 0 upper-body policy and export policy.onnx + deploy.yaml + baseline rollout.

Usage:
  .venv/Scripts/python.exe -u RobotControl/training/train_level0.py --iters 1500 --num-envs 512
Outputs (RobotControl/models/r1_level0_upperbody/):
  policy.onnx, deploy.yaml, model.pt, baseline_rollout.npz, train_log.csv
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time

import numpy as np
import torch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from export_level0 import OUT, export, run_meta  # noqa: E402
from level0_env import Level0Config, Level0StandEnv  # noqa: E402
from ppo import PPO, ActorCritic  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=1500)
    ap.add_argument("--num-envs", type=int, default=512)
    ap.add_argument("--horizon", type=int, default=24)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=pathlib.Path, default=OUT)
    ap.add_argument("--resume", type=pathlib.Path, default=None)
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(args.seed)
    env = Level0StandEnv(Level0Config(num_envs=args.num_envs, seed=args.seed))
    ac = ActorCritic(env.num_obs, env.num_actions)
    if args.resume:
        ac.load_state_dict(torch.load(args.resume, map_location="cpu"))
    ppo = PPO(ac, device)
    N, T = args.num_envs, args.horizon
    print(f"device={device} envs={N} horizon={T} obs={env.num_obs} act={env.num_actions} threads={env.cfg.nthread}")

    args.out.mkdir(parents=True, exist_ok=True)
    logf = open(args.out / "train_log.csv", "w", newline="")
    writer = csv.writer(logf)
    writer.writerow(["iter", "time_s", "reward_mean", "ep_reward", "track_err_yaw_deg", "track_err_pitch_deg",
                     "max_acc", "curriculum", "loss_v", "loss_pi", "kl", "entropy", "lr", "std", "fps"])

    obs_buf = torch.zeros(T, N, env.num_obs, device=device)
    act_buf = torch.zeros(T, N, env.num_actions, device=device)
    logp_buf = torch.zeros(T, N, device=device)
    val_buf = torch.zeros(T, N, device=device)
    rew_buf = torch.zeros(T, N, device=device)
    done_buf = torch.zeros(T, N, device=device)
    mu_buf = torch.zeros_like(act_buf)
    sig_buf = torch.zeros_like(act_buf)

    obs = torch.from_numpy(env.get_obs()).to(device)
    ep_ret = np.zeros(N)
    ep_ret_hist: list[float] = []
    t0 = time.time()
    for it in range(1, args.iters + 1):
        t_it = time.time()
        infos = []
        for t in range(T):
            ac.obs_norm.update(obs)
            a, logp, v, mu, sig = ppo.act(obs)
            nobs, r, d, info = env.step(a.cpu().numpy())
            obs_buf[t] = obs; act_buf[t] = a; logp_buf[t] = logp; val_buf[t] = v
            mu_buf[t] = mu; sig_buf[t] = sig
            rew_buf[t] = torch.from_numpy(r).to(device)
            done_buf[t] = torch.from_numpy(d.astype(np.float32)).to(device)
            ep_ret += r
            if d.any():
                ep_ret_hist += list(ep_ret[d])
                ep_ret[d] = 0
            obs = torch.from_numpy(nobs).to(device)
            infos.append(info)
        with torch.no_grad():
            last_v = ac.value(obs)
        adv, ret = ppo.compute_gae(rew_buf, done_buf, val_buf, last_v)
        stats = ppo.update((obs_buf, act_buf, logp_buf, val_buf, adv, ret, mu_buf, sig_buf))

        err_yaw = float(np.mean([i["track_err_yaw_deg"] for i in infos]))
        err_pitch = float(np.mean([i["track_err_pitch_deg"] for i in infos]))
        env.update_curriculum(0.5 * (err_yaw + err_pitch))
        fps = N * T / (time.time() - t_it)
        row = [it, round(time.time() - t0, 1), float(rew_buf.mean()),
               float(np.mean(ep_ret_hist[-100:])) if ep_ret_hist else 0.0,
               err_yaw, err_pitch, max(i["max_acc"] for i in infos), env.curriculum,
               stats["loss_v"], stats["loss_pi"], stats["kl"], stats["entropy"], stats["lr"],
               ac.log_std.detach().exp().mean().item(), fps]
        writer.writerow([f"{x:.5g}" if isinstance(x, float) else x for x in row])
        logf.flush()
        if it % 10 == 0 or it == 1:
            print(f"it {it:5d} | rew {row[2]:7.3f} | ep {row[3]:7.1f} | err yaw {err_yaw:5.2f} pitch {err_pitch:5.2f} deg | "
                  f"maxacc {row[6]:6.1f} | cur {env.curriculum:.2f} | std {row[13]:.3f} | lr {stats['lr']:.1e} | {fps:7.0f} fps",
                  flush=True)
        if it % 200 == 0:
            torch.save(ac.state_dict(), args.out / f"model_{it}.pt")
    logf.close()

    export(ac, env, args.out, run_meta(args.iters, N, T, args.seed, device))
    env.close()


if __name__ == "__main__":
    main()
