#!/usr/bin/env bash
# Use the same pinned Python/NumPy/SciPy/Numba stack as training and deployment.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT/training/scripts/setup_ubuntu.sh" "$@"
