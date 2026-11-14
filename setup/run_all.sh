#!/usr/bin/env bash
# 一键完成 ROS 2 安装 + 工作空间编译
# 用法：sudo bash setup/run_all.sh
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "需要 root 权限，请执行：sudo bash $0" >&2
  exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_USER="${SUDO_USER:-qiqi}"

bash "${HERE}/01_install_ros2_humble.sh"
sudo -u "${TARGET_USER}" -H bash "${HERE}/02_setup_workspace.sh"

echo
echo "全部完成。请重新登录或执行：source ~/.bashrc"
echo "然后自检：bash ${HERE}/03_verify_env.sh"
