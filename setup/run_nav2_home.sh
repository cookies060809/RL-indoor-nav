#!/usr/bin/env bash
# Start the Nav2 baseline in robocup_home with the course map, AMCL and the
# safety node as the only /cmd_vel publisher. Extra arguments are forwarded
# to ros2 launch.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 launch project_bringup nav2_nav.launch.py "$@"