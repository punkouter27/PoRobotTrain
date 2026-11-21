# Level 2 (walking) trainer: Unitree unitree_rl_mjlab (mjlab + MuJoCo Warp) on the GPU.
# Build context = repo root. Built and run by scripts/level2/train_level2.ps1.
# Blackwell GPUs (RTX 50xx, sm_120): the default PyPI torch (CUDA 13 build) supports sm_120; needs NVIDIA driver >= 580.
FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
    MUJOCO_GL=egl \
    WANDB_MODE=disabled

RUN apt-get update && apt-get install -y --no-install-recommends \
        git libegl1 libgl1 libglib2.0-0 ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_DEFAULT_TIMEOUT=300 PIP_RETRIES=10
# torch in its own layer (largest download, cached across rebuilds); mjlab's deps need a recent torch
RUN pip install "torch>=2.9"
# Versions pinned to unitree_rl_mjlab's setup.py (mjlab 1.2.0, mujoco-warp 3.5.0); mujoco matched to mujoco-warp.
RUN pip install "mjlab==1.2.0" "mujoco-warp==3.5.0" "mujoco==3.5.0" "warp-lang==1.12.*" scipy onnx onnxruntime pyyaml  # scipy: imported by mjlab.terrains but undeclared; warp 1.12: mjlab 1.2.0 uses wp.context (removed later)

# unitree_rl_mjlab source (reference clone, pinned by train_level2.ps1) copied in at build time
WORKDIR /opt/unitree_rl_mjlab
COPY external/unitree_rl_mjlab/ /opt/unitree_rl_mjlab/
RUN pip install --no-deps -e .
COPY scripts/level2/export_level2.py scripts/level2/train_entry.sh /opt/porobot/
RUN sed -i 's/\r$//' /opt/porobot/train_entry.sh && chmod +x /opt/porobot/train_entry.sh

# training output: /opt/unitree_rl_mjlab/logs -> mounted to <repo>/logs/level2_training
VOLUME ["/opt/unitree_rl_mjlab/logs"]
ENTRYPOINT ["/opt/porobot/train_entry.sh"]
