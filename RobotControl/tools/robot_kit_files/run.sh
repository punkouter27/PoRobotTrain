#!/usr/bin/env bash
# Run a robotcontrol module with the kit's venv and (if it was built from source) CycloneDDS on the path.
#   ./run.sh robotcontrol.tools.robot_preflight --bundle bundle
#   ./run.sh robotcontrol.main_runner --target real --observe --policy bundle/policy.onnx --command-mode sweep
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$KIT"
if [ -f env.sh ]; then
  # shellcheck disable=SC1091
  source env.sh
  export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
fi
exec "$KIT/.venv/bin/python" -m "$@"
