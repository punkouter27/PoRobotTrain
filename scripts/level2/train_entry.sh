#!/usr/bin/env bash
# Container entrypoint.
#   train  [extra train.py args...]   -> python scripts/train.py Unitree-R1-Flat ...
#   export [export_level2.py args...] -> python /opt/porobot/export_level2.py ...
#   anything else                     -> exec as a command
set -euo pipefail
cd /opt/unitree_rl_mjlab
mode="${1:-train}"; shift || true
case "$mode" in
  train)  exec python scripts/train.py "${L2_TASK:-Unitree-R1-Flat}" "$@" ;;
  export) exec python /opt/porobot/export_level2.py "$@" ;;
  *)      exec "$mode" "$@" ;;
esac
