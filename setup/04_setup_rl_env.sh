#!/usr/bin/env bash
# 创建强化学习 Python 环境（建议在带 GPU 的机器上执行）
# 用法：bash setup/04_setup_rl_env.sh
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
VENV="${ROOT}/.venv"

# 必须先 source ROS，才能在虚拟环境中 import rclpy
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

python3 -m venv --system-site-packages "${VENV}"
# shellcheck disable=SC1091
source "${VENV}/bin/activate"
python -m pip install --upgrade pip

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  echo "检测到 NVIDIA GPU，安装默认 CUDA 版 PyTorch"
  python -m pip install -r "${HERE}/requirements-rl.txt"
else
  echo "未检测到 NVIDIA GPU，安装 CPU 版 PyTorch（节省磁盘）"
  python -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu
  # 官方源下载卡住时，可改用阿里云镜像：
  # python -m pip install torch==2.8.0 --index-url https://mirrors.aliyun.com/pytorch-wheels/cpu
  python -m pip install stable-baselines3==2.9.2 gymnasium==1.4.0 tensorboard==2.21.0 \
    numpy==2.2.6 opencv-python-headless "coverage>=7.4"
fi

python - <<'EOF'
import torch, gymnasium, stable_baselines3 as sb3
print("torch:", torch.__version__, "| CUDA 可用:", torch.cuda.is_available())
print("gymnasium:", gymnasium.__version__)
print("stable-baselines3:", sb3.__version__)
EOF

echo
echo "如需在 ROS 2 节点中使用该环境："
echo "  source /opt/ros/humble/setup.bash && source ${VENV}/bin/activate"
