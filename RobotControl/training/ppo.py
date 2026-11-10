"""Compact PPO (rsl_rl-style: ELU MLP actor-critic, learnable log-std, GAE, adaptive LR by KL,
empirical observation normalisation). Device: CUDA if available."""
from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn


class EmpiricalNormalization(nn.Module):
    """Running mean/var normaliser; frozen at export so the ONNX graph carries it."""

    def __init__(self, shape, eps=1e-2, until=int(1e8)):
        super().__init__()
        self.eps, self.until = eps, until
        self.register_buffer("mean", torch.zeros(shape))
        self.register_buffer("var", torch.ones(shape))
        self.register_buffer("count", torch.zeros((), dtype=torch.long))

    def forward(self, x):
        return (x - self.mean) / torch.sqrt(self.var + self.eps)

    @torch.no_grad()
    def update(self, x):
        if self.count >= self.until:
            return
        n = x.shape[0]
        bmean, bvar = x.mean(0), x.var(0, unbiased=False)
        delta = bmean - self.mean
        tot = self.count + n
        self.mean += delta * n / tot
        self.var = (self.var * self.count + bvar * n + delta ** 2 * self.count * n / tot) / tot
        self.count = tot


def mlp(inp, hidden, out):
    layers, d = [], inp
    for h in hidden:
        layers += [nn.Linear(d, h), nn.ELU()]
        d = h
    layers.append(nn.Linear(d, out))
    return nn.Sequential(*layers)


class ActorCritic(nn.Module):
    def __init__(self, num_obs, num_actions, hidden=(256, 128, 64), init_noise_std=1.0):
        super().__init__()
        self.obs_norm = EmpiricalNormalization(num_obs)
        self.actor = mlp(num_obs, hidden, num_actions)
        self.critic = mlp(num_obs, hidden, 1)
        self.log_std = nn.Parameter(torch.full((num_actions,), float(np.log(init_noise_std))))

    def dist(self, obs):
        mean = self.actor(self.obs_norm(obs))
        return torch.distributions.Normal(mean, self.log_std.exp().expand_as(mean))

    def act_inference(self, obs):
        return self.actor(self.obs_norm(obs))

    def value(self, obs):
        return self.critic(self.obs_norm(obs)).squeeze(-1)


class PPO:
    def __init__(self, ac: ActorCritic, device, lr=1e-3, gamma=0.99, lam=0.95, clip=0.2,
                 entropy_coef=0.005, value_coef=1.0, epochs=5, minibatches=4, desired_kl=0.01,
                 max_grad_norm=1.0):
        self.ac, self.device = ac.to(device), device
        self.opt = torch.optim.Adam(ac.parameters(), lr=lr)
        self.lr, self.gamma, self.lam, self.clip = lr, gamma, lam, clip
        self.entropy_coef, self.value_coef = entropy_coef, value_coef
        self.epochs, self.minibatches, self.desired_kl, self.max_grad_norm = epochs, minibatches, desired_kl, max_grad_norm

    @torch.no_grad()
    def act(self, obs):
        d = self.ac.dist(obs)
        a = d.sample()
        return a, d.log_prob(a).sum(-1), self.ac.value(obs), d.mean, d.stddev

    def update(self, buf):
        obs, act, logp_old, val_old, adv, ret, mu_old, sigma_old = buf
        T, N = obs.shape[:2]
        flat = lambda x: x.reshape(T * N, *x.shape[2:])
        obs, act, logp_old, val_old, adv, ret, mu_old, sigma_old = map(flat, (obs, act, logp_old, val_old, adv, ret, mu_old, sigma_old))
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        n = T * N
        mb = n // self.minibatches
        stats = {"loss_v": 0.0, "loss_pi": 0.0, "kl": 0.0, "entropy": 0.0}
        cnt = 0
        for _ in range(self.epochs):
            perm = torch.randperm(n, device=self.device)
            for k in range(self.minibatches):
                idx = perm[k * mb:(k + 1) * mb]
                d = self.ac.dist(obs[idx])
                logp = d.log_prob(act[idx]).sum(-1)
                ent = d.entropy().sum(-1).mean()
                # adaptive LR (rsl_rl)
                with torch.no_grad():
                    kl = torch.sum(torch.log(d.stddev / sigma_old[idx] + 1e-5)
                                   + (sigma_old[idx] ** 2 + (mu_old[idx] - d.mean) ** 2) / (2 * d.stddev ** 2) - 0.5, -1).mean()
                    if kl > self.desired_kl * 2.0:
                        self.lr = max(1e-5, self.lr / 1.5)
                    elif kl < self.desired_kl / 2.0 and kl > 0:
                        self.lr = min(1e-2, self.lr * 1.5)
                    for g in self.opt.param_groups:
                        g["lr"] = self.lr
                ratio = torch.exp(logp - logp_old[idx])
                s1 = -adv[idx] * ratio
                s2 = -adv[idx] * torch.clamp(ratio, 1 - self.clip, 1 + self.clip)
                loss_pi = torch.max(s1, s2).mean()
                v = self.ac.value(obs[idx])
                v_clipped = val_old[idx] + (v - val_old[idx]).clamp(-self.clip, self.clip)
                loss_v = torch.max((v - ret[idx]) ** 2, (v_clipped - ret[idx]) ** 2).mean()
                loss = loss_pi + self.value_coef * loss_v - self.entropy_coef * ent
                self.opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.ac.parameters(), self.max_grad_norm)
                self.opt.step()
                stats["loss_v"] += loss_v.item(); stats["loss_pi"] += loss_pi.item()
                stats["kl"] += kl.item(); stats["entropy"] += ent.item(); cnt += 1
        return {k: v / cnt for k, v in stats.items()} | {"lr": self.lr}

    def compute_gae(self, rewards, dones, values, last_value):
        T = rewards.shape[0]
        adv = torch.zeros_like(rewards)
        gae = torch.zeros_like(last_value)
        for t in reversed(range(T)):
            next_v = last_value if t == T - 1 else values[t + 1]
            nonterm = 1.0 - dones[t]
            delta = rewards[t] + self.gamma * next_v * nonterm - values[t]
            gae = delta + self.gamma * self.lam * nonterm * gae
            adv[t] = gae
        return adv, adv + values
