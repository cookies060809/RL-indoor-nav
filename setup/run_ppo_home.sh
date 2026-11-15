#!/usr/bin/env bash
# Start PPO navigation on the home map with Nav2 planning and a local
# lookahead goal. Extra arguments are forwarded to ros2 launch.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 launch project_bringup ppo_nav.launch.py "project_root:=$ROOT" "$@"
