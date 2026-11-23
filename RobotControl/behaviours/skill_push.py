"""Copy a validated head skill (policy.onnx + deploy.yaml) to the robot, then run the given command.

    python -m robotcontrol.behaviours.skill_push look_at                      # copy only (Library: Copy to robot)
    python -m robotcontrol.behaviours.skill_push look_at -- ssh unitree@192.168.123.164 "cd ~/porobot && ..."

Used by the behaviour service's read-only check for trained skills, so the robot always checks (and later runs) the
exact file that passed validation; the runner on the robot re-verifies the sha256 before loading it. Copies nothing
that is not validated, and moves no motor itself.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from .head_skills import SKILLS, SKILLS_SUBDIR, skill_status
from .mirror import ROBOT, ROBOT_DIR

SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]


def push(skill_id: str) -> None:
    st = skill_status(skill_id)
    if st["state"] != "ready":
        raise RuntimeError(f"{SKILLS[skill_id].label}: {st['reason']}")
    remote = f"{ROBOT_DIR}/{SKILLS_SUBDIR}/{skill_id}"
    subprocess.run(["ssh", *SSH_OPTS, ROBOT, f"mkdir -p {remote}"], check=True, timeout=20)
    files = [f"{st['dir']}/policy.onnx", f"{st['dir']}/deploy.yaml"]
    subprocess.run(["scp", "-q", *SSH_OPTS, *files, f"{ROBOT}:{remote}/"], check=True, timeout=60)
    import yaml

    from .head_skills import sha256_file

    version = (yaml.safe_load(open(f"{st['dir']}/deploy.yaml", encoding="utf-8")) or {}).get("version") or 1
    print(f"pushed {skill_id} policy to the robot ({remote})", flush=True)
    # parsed by the behaviour service: which version the robot now has (Library: "On the robot: v2")
    print(f"@pushed {skill_id} v{version} {sha256_file(Path(st['dir']) / 'policy.onnx')}", flush=True)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) < 1 or argv[0] not in SKILLS:
        print(f"usage: skill_push <{'|'.join(SKILLS)}> [-- command ...]")
        return 2
    rest = argv[2:] if len(argv) > 1 and argv[1] == "--" else argv[1:]
    try:
        push(argv[0])
    except (RuntimeError, subprocess.SubprocessError, OSError) as exc:
        print(f"NOT STARTED: could not copy the skill to the robot: {exc}", flush=True)
        return 1
    return subprocess.call(rest) if rest else 0


if __name__ == "__main__":
    sys.exit(main())
