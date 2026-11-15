#!/usr/bin/env bash
# Native ROS build checks, including the exact selected model's BEV fixtures.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
PYTHON="$ROOT/.venv/bin/python"
"$PYTHON" -c 'import rclpy, scipy, cv2, torch, stable_baselines3; from rl_nav_pkg.path_tracking import PathTracker; from project_msgs.msg import PolicyObservation; print("PASS: ROS/ML imports")'
"$PYTHON" "$ROOT/models/ppo_v2_recommended/deployment/verify_bundle.py"
"$PYTHON" "$ROOT/ros2_ws/src/bev_pkg/test/export_fixtures.py" \
  "$ROOT/models/ppo_v2_recommended/fixtures" "$ROOT/.tmp/bev_release_v2"
ros2 run bev_pkg bev_fixture_check "$ROOT/.tmp/bev_release_v2"
ros2 run safety_pkg safety_core_check
SAFETY_PREFIX="$(ros2 pkg prefix safety_pkg)"
PYTHONPATH="$ROOT/training/envs:${PYTHONPATH:-}" "$PYTHON" \
  "$ROOT/ros2_ws/src/safety_pkg/test/check_python_parity.py" \
  --executable "$SAFETY_PREFIX/lib/safety_pkg/safety_core_check" \
  --config "$ROOT/models/ppo_v2_recommended/config.yaml" \
  --report "$ROOT/.tmp/safety_cpp_python_parity_linux.json"
echo 'PASS: native cores and imports. Continue with live Gazebo/TF/topic checks.'
