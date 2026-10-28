#!/usr/bin/env bash
set -euo pipefail
# Run in the existing container; this venv is separate from /usr/bin/python3 + ROS.
python3 -m venv .venv-learning
.venv-learning/bin/python -m pip install --upgrade pip
.venv-learning/bin/python -m pip install \
  torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
.venv-learning/bin/python -m pip install 'lerobot==0.4.3'
.venv-learning/bin/python -m pip freeze > learning-environment.lock.txt
.venv-learning/bin/python - <<'PY'
import torch
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.datasets.lerobot_dataset import LeRobotDataset
print('Torch:', torch.__version__, 'CUDA available:', torch.cuda.is_available())
print('ACT and dataset imports OK. Actual CPU train/deploy still require data and Gazebo.')
PY
