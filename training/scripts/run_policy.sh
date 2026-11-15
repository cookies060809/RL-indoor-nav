#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ID="${1:-split_empty_pretrain_v1}"
if [[ $# -gt 0 ]]; then shift; fi
MODEL_DIR="$ROOT/models/$MODEL_ID"
if [[ ! -f "$MODEL_DIR/policy.zip" ]]; then
  echo "Model bundle not found: $MODEL_DIR/policy.zip" >&2
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
  --ros-args -p use_sim_time:=true -p "model_path:=$MODEL_DIR/policy.zip" "$@"
