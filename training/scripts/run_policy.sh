#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ID="${1:-split_empty_pretrain_v1}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$MODEL_ID" != split_empty_pretrain_v1 && "$MODEL_ID" != split_single_pretrain_v1 && "$MODEL_ID" != split_mixed_pretrain_v1 ]]; then
  echo 'Choose one of the three shipped model IDs, or edit this launcher for a new model.' >&2
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
