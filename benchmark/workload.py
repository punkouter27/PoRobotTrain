"""PoRobotTrain Workload: the real RobotControl/training/train_level0.py (own PPO, torch) with the README's main
Level 0 run settings (512 envs, horizon 24, 1500 iterations).

Run by serve.py with the repo venv:  .venv/Scripts/python.exe benchmark/workload.py
The training loop has no timing callback, so the hooks below split each PPO iteration: env stepping + policy
inference up to PPO.compute_gae is Sim time, GAE + PPO.update is Learn time, and Level0StandEnv.update_curriculum
(called once per iteration, after the update) ends the block. One iteration = `horizon` Steps.
Output goes to logs/benchmark/level0 (gitignored, wiped first), never to RobotControl/models/.
Level 2 (scripts/level2/train_level2.ps1, mjlab in Docker) is not covered: it runs inside a Linux container.
"""

from __future__ import annotations

import argparse
import dataclasses
import os
import runpy
import shutil
import sys
import time
from importlib import metadata
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import core  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
TRAINING = ROOT / "RobotControl" / "training"
OUT = "logs/benchmark/level0"  # relative to ROOT, so the Workload (and its ID) is the same on every laptop
# Same settings as the main Level 0 run (README "Quick start" step 3). Change these and the Workload ID changes.
ARGS = ["--iters", "1500", "--num-envs", "512", "--horizon", "24", "--seed", "0", "--out", OUT]


def version(pkg: str) -> str | None:
    try:
        return metadata.version(pkg)
    except metadata.PackageNotFoundError:
        return None


def main() -> None:
    import torch

    # train_level0.py imports its siblings as top-level modules, but level0_env.py uses package-relative imports;
    # alias the package modules so the script runs unchanged.
    from robotcontrol.training import level0_env, ppo

    sys.modules.setdefault("level0_env", level0_env)
    sys.modules.setdefault("ppo", ppo)

    meter: core.Meter | None = None
    seen: dict = {}

    parse = argparse.ArgumentParser.parse_args

    def parse_(self, *a, **k):
        ns = parse(self, *a, **k)
        seen.setdefault("args", ns)
        return ns

    env_init = level0_env.Level0StandEnv.__init__

    def env_init_(self, cfg, *a, **k):
        env_init(self, cfg, *a, **k)
        seen["env"] = self

    ppo_init = ppo.PPO.__init__

    def ppo_init_(self, ac, device, *a, **k):
        nonlocal meter
        ppo_init(self, ac, device, *a, **k)
        args, env = seen["args"], seen["env"]
        cfg = {k: v for k, v in dataclasses.asdict(env.cfg).items() if k != "nthread"}  # nthread = CPU count
        hyper = {k: v for k, v in vars(self).items() if k not in ("ac", "opt", "device")}
        net = [m.out_features for m in ac.actor if isinstance(m, torch.nn.Linear)][:-1]
        workload = dict(
            repo="PoRobotTrain",
            trainer=f"PoRobotTrain PPO (RobotControl/training/ppo.py), torch {version('torch')}",
            script="RobotControl/training/train_level0.py",
            args=" ".join(ARGS),
            num_envs=args.num_envs,
            horizon=args.horizon,
            net=net,
            obs=env.num_obs,
            actions=env.num_actions,
            config_hash=core.workload_id({"env": cfg, "ppo": hyper}),
        )
        versions = dict(
            python=sys.version.split()[0],
            torch=version("torch"),
            mujoco=version("mujoco"),
            device=str(device),
            mujoco_threads=env.cfg.nthread,
        )
        meter = core.Meter(workload, versions)
        seen["t_block"] = time.perf_counter()

    gae = ppo.PPO.compute_gae

    def gae_(self, *a, **k):
        seen["t_learn"] = time.perf_counter()
        return gae(self, *a, **k)

    curriculum = level0_env.Level0StandEnv.update_curriculum

    def curriculum_(self, *a, **k):
        t = time.perf_counter()
        if meter is not None and "t_learn" in seen:
            meter.block(seen["t_learn"] - seen["t_block"], t - seen["t_learn"], meter.workload["horizon"])
        result = curriculum(self, *a, **k)
        seen["t_block"] = time.perf_counter()
        return result

    argparse.ArgumentParser.parse_args = parse_
    level0_env.Level0StandEnv.__init__ = env_init_
    level0_env.Level0StandEnv.update_curriculum = curriculum_
    ppo.PPO.__init__ = ppo_init_
    ppo.PPO.compute_gae = gae_

    out = (ROOT / OUT).resolve()
    assert (ROOT / "logs") in out.parents, out  # the throwaway run dir, never a real model folder
    shutil.rmtree(out, ignore_errors=True)
    os.chdir(ROOT)
    sys.path.insert(0, str(TRAINING))
    sys.argv = ["RobotControl/training/train_level0.py", *ARGS]
    try:
        runpy.run_path(str(TRAINING / "train_level0.py"), run_name="__main__")
        raise RuntimeError("training ended before the measured window finished")
    except BaseException as e:  # noqa: BLE001 - every failure becomes a failed row, never a silent miss
        if meter is None:
            meter = core.Meter({"repo": "PoRobotTrain", "args": " ".join(ARGS)}, {"python": sys.version.split()[0]})
        meter.finish(ok=False, error=core.fail_text(e))


if __name__ == "__main__":
    main()
