#!/usr/bin/env bash
# First v2 upgrade: install Python dependencies, rebuild all changed ROS nodes,
# then launch the model-owned runtime. Existing setup/run_ppo_home.sh skips setup.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble on Ubuntu 22.04 is required.' >&2
  exit 1
fi
bash "$ROOT/setup/04_setup_rl_env.sh"
bash "$ROOT/training/scripts/build_ros_interfaces.sh"
bash "$ROOT/training/scripts/verify_ros_runtime.sh"
exec bash "$ROOT/setup/run_ppo_home.sh" "$@"
