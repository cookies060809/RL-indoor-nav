#!/usr/bin/env bash
# ROS 2 Humble + 课程仿真依赖安装脚本（Ubuntu 22.04）
# 用法：sudo bash setup/01_install_ros2_humble.sh
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "需要 root 权限，请执行：sudo bash $0" >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-qiqi}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
export DEBIAN_FRONTEND=noninteractive

source /etc/os-release
if [[ "${VERSION_ID}" != "22.04" ]]; then
  echo "当前系统为 ${VERSION_ID}，本脚本只适用于 Ubuntu 22.04 + ROS 2 Humble" >&2
  exit 1
fi

echo "[1/6] 安装基础工具"
apt-get update
apt-get install -y curl gnupg lsb-release ca-certificates locales

echo "[2/6] 检查 UTF-8 locale"
if ! locale | grep -qi 'utf-\?8'; then
  locale-gen en_US.UTF-8
  update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
fi

echo "[3/6] 配置 ROS 2 清华镜像源"
install -d -m 0755 /usr/share/keyrings
if ! curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; then
  echo "GitHub 下载密钥失败，改用 Ubuntu keyserver"
  rm -f /usr/share/keyrings/ros-archive-keyring.gpg
  gpg --keyserver keyserver.ubuntu.com --recv-keys \
    C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  gpg --export C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    > /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  > /etc/apt/sources.list.d/ros2.list

echo "[4/6] 安装 ROS 2 Humble Desktop"
apt-get update
apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete \
  python3-pip

echo "[5/6] 安装项目所需组件（Gazebo Classic / Nav2 / SLAM / ros2_control / 视觉）"
apt-get install -y \
  gazebo \
  pcl-tools \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros2-control \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-pcl-ros

echo "[6/6] 初始化 rosdep"
rosdep init 2>/dev/null || echo "rosdep 已初始化，跳过"
sudo -u "$TARGET_USER" -H bash -lc "rosdep update" || \
  echo "rosdep update 失败，可稍后手动执行（不影响基本使用）"

echo
echo "ROS 2 Humble 安装完成。下一步（普通用户）："
echo "  bash setup/02_setup_workspace.sh"
echo "  bash setup/03_verify_env.sh"
