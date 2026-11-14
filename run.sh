#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$ROOT/.cache" "$ROOT/.tmp"
export MPLCONFIGDIR="$ROOT/.cache/matplotlib"
export NUMBA_CACHE_DIR="$ROOT/.cache/numba"
export TMPDIR="$ROOT/.tmp"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-2}"
export PYTHONPATH="$ROOT/training/envs${PYTHONPATH:+:$PYTHONPATH}"
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
  echo 'Missing Linux environment. Run: bash training/scripts/setup_ubuntu.sh' >&2
  exit 1
fi
cd "$ROOT"
exec "$ROOT/.venv/bin/python" "$ROOT/training/scripts/run.py" "$@"
