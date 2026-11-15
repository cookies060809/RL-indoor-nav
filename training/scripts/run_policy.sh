#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ID="${1:-ppo_v2_recommended}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ ! "$MODEL_ID" =~ ^[A-Za-z0-9_-]+$ ]] || [[ ! -f "$ROOT/models/$MODEL_ID/policy.zip" ]]; then
  echo 'Choose an existing model ID from models/.' >&2
  exit 1
fi
if [[ ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo 'First run: bash training/scripts/build_ros_interfaces.sh' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
exec "$ROOT/.venv/bin/python" "$ROOT/ros2_ws/src/rl_nav_pkg/rl_nav_pkg/policy_node.py" \
  --ros-args -p use_sim_time:=true -p "model_path:=$ROOT/models/$MODEL_ID/policy.zip" "$@"
