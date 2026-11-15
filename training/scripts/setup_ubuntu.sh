#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PIP_CACHE_DIR="$ROOT/.cache/pip"
export TMPDIR="$ROOT/.tmp"
mkdir -p "$PIP_CACHE_DIR" "$TMPDIR"
if ! command -v python3.10 >/dev/null 2>&1; then
  echo 'Python 3.10 is required. Use the Ubuntu 22.04 interpreter.' >&2
  exit 1
fi
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
  python3.10 -m venv --system-site-packages "$ROOT/.venv"
fi
PYTHON="$ROOT/.venv/bin/python"
"$PYTHON" -m pip install --upgrade pip
"$PYTHON" -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu
"$PYTHON" -m pip install -r "$ROOT/config/requirements-core-lock.txt"
# System ROS OpenCV/coverage may target an older NumPy API; isolate compatible
# replacements in this venv rather than modifying Ubuntu's system Python.
"$PYTHON" -m pip install 'opencv-python-headless>=4.10' 'coverage>=7.4'
"$PYTHON" -m pip check
"$PYTHON" -m pip freeze > "$ROOT/config/requirements-linux-observed.txt"
"$PYTHON" "$ROOT/training/scripts/verify_project.py" --models-only
echo 'Linux dependency setup finished. ROS C++ build and Gazebo integration remain separate.'
