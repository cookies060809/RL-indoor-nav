#!/usr/bin/env bash
# Overlay the course workspace first, then the project workspace, and start the
# PPO navigation demo. Extra arguments are forwarded to ros2 launch.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 launch project_bringup rl_nav.launch.py "project_root:=$ROOT" "$@"
