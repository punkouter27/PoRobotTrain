#!/usr/bin/env bash
# Runs ON THE R1's development computer (Jetson, Ubuntu). Installs the policy runner into ./.venv
# entirely OFFLINE from the files in this kit. Safe to run more than once. Sends nothing to the motors.
#
#   cd ~/porobot && bash setup_robot.sh
set -euo pipefail
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$KIT"
PY=python3
PYVER=$($PY -c 'import sys; print(f"{sys.version_info[0]}{sys.version_info[1]}")')
echo "== kit: $KIT   python: $($PY --version)   arch: $(uname -m)"
[ "$(uname -m)" = "aarch64" ] || echo "WARNING: expected aarch64 (Jetson); wheels in this kit are aarch64-only"
WHEELS="$KIT/wheels/cp$PYVER"
[ -d "$WHEELS" ] || { echo "ERROR: no wheels for Python $PYVER in kit (have: $(ls wheels))"; exit 1; }

# 1. virtualenv that can also see system packages (keeps any preinstalled cyclonedds / unitree_sdk2py)
if [ ! -x .venv/bin/python ]; then
  $PY -m venv --system-site-packages .venv || {
    echo "ERROR: python3 -m venv failed. Install it once with internet:  sudo apt install python3-venv"; exit 1; }
fi
VPY="$KIT/.venv/bin/python"
"$VPY" -m pip --version >/dev/null 2>&1 || { echo "ERROR: pip missing in venv (sudo apt install python3-pip)"; exit 1; }

# 2. newer pip first (Ubuntu 20.04 pip 20.0 cannot read manylinux_2_28 wheel tags), then runner deps; all offline
"$VPY" -m pip install --no-index --find-links "$WHEELS" --upgrade pip
"$VPY" -m pip install --no-index --find-links "$WHEELS" numpy pyyaml onnxruntime

# 3. CycloneDDS Python binding 0.10.2 (Unitree requirement). Use the system one if present, else build offline.
if "$VPY" -c "import cyclonedds" 2>/dev/null; then
  echo "== cyclonedds already available: $("$VPY" -c 'import cyclonedds,os;print(os.path.dirname(cyclonedds.__file__))')"
else
  echo "== building CycloneDDS C library 0.10.x from kit source (needs cmake + gcc, ~5 min)"
  command -v cmake >/dev/null || { echo "ERROR: cmake not found (sudo apt install cmake build-essential)"; exit 1; }
  mkdir -p src/cyclonedds/build src/cyclonedds/install
  (cd src/cyclonedds/build && cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF >/dev/null \
     && cmake --build . --target install -j"$(nproc)" >/dev/null)
  export CYCLONEDDS_HOME="$KIT/src/cyclonedds/install"
  "$VPY" -m pip install --no-index --find-links "$WHEELS" setuptools wheel rich-click
  "$VPY" -m pip install --no-index --find-links "$WHEELS" --no-build-isolation src/cyclonedds-0.10.2.tar.gz
  echo "export CYCLONEDDS_HOME=$CYCLONEDDS_HOME" > "$KIT/env.sh"
fi

# 4. Unitree SDK2 Python (opencv is only needed by camera examples, so install without deps)
if ! "$VPY" -c "import unitree_sdk2py" 2>/dev/null; then
  "$VPY" -m pip install --no-index --find-links "$WHEELS" setuptools wheel
  "$VPY" -m pip install --no-index --no-deps --no-build-isolation -e src/unitree_sdk2_python
fi

# 5. offline self-check (software + bundle only; the robot test is a separate step)
"$KIT/run.sh" robotcontrol.tools.robot_preflight --bundle bundle --skip-robot
echo
echo "Setup done. Next (robot on its stand, powered, in zero-torque):"
echo "  cd $KIT && ./run.sh robotcontrol.tools.robot_preflight --bundle bundle"
