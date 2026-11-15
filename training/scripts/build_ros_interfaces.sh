#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
  echo 'First run: bash training/scripts/setup_ubuntu.sh' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
"$ROOT/.venv/bin/python" -c 'import rclpy, colcon_core'
cd "$ROOT/ros2_ws"
"$ROOT/.venv/bin/python" -c 'from colcon_core.command import main; raise SystemExit(main())' build --symlink-install \
  --packages-select project_msgs bev_pkg safety_pkg rl_nav_pkg project_bringup \
  --cmake-args "-DPython3_EXECUTABLE=$ROOT/.venv/bin/python"
echo 'Build completed. Source this workspace before running ROS nodes.'
