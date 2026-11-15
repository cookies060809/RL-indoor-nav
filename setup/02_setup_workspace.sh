#!/usr/bin/env bash
# 初始化 ROS 2 工作空间、拉取课程仿真包并编译
# 用法：bash setup/02_setup_workspace.sh
set -eo pipefail

ROS_DISTRO=humble
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS="${HOME}/ros2_ws"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "未找到 ${ROS_SETUP}，请先运行：sudo bash setup/01_install_ros2_humble.sh" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"

echo "[1/4] 创建工作空间"
mkdir -p "${WS}/src"

echo "[2/4] 获取课程仿真包 wpr_simulation2（Gitee 镜像）"
if [[ ! -d "${WS}/src/wpr_simulation2/.git" ]]; then
  git clone https://gitee.com/s-robot/wpr_simulation2.git "${WS}/src/wpr_simulation2"
else
  echo "已存在，跳过 clone"
fi

echo "[3/4] 注册 wpr_simulation2 的 Gazebo 模型"
GAZEBO_MODEL_DIR="${HOME}/.gazebo/models/wpr_simulation2"
mkdir -p "${GAZEBO_MODEL_DIR}"
cp -r "${WS}/src/wpr_simulation2/models" "${GAZEBO_MODEL_DIR}/"
cp -r "${WS}/src/wpr_simulation2/meshes" "${GAZEBO_MODEL_DIR}/"
cp -r "${WS}/src/wpr_simulation2/worlds" "${GAZEBO_MODEL_DIR}/"

echo "[4/4] 编译工作空间"
cd "${WS}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

BASHRC="${HOME}/.bashrc"
if ! grep -q '>>> ros2-humble >>>' "${BASHRC}" 2>/dev/null; then
  cat >> "${BASHRC}" <<'EOF'

# >>> ros2-humble >>>
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
# 默认不设置 ROS_LOCALHOST_ONLY；所有终端保持一致即可
# export ROS_LOCALHOST_ONLY=1
# <<< ros2-humble <<<
EOF
  echo "已写入 ~/.bashrc"
else
  echo "~/.bashrc 中已有配置，跳过"
fi

echo
echo "工作空间就绪：${WS}"
echo "新开终端会自动 source；当前终端执行：source ${WS}/install/setup.bash"
