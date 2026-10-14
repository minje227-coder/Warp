#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON="${WARP_RETARGET_PYTHON:-$ROOT/.venv-retarget/bin/python}"
export PYTHONPATH="$ROOT/pipeline/01_dataset${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec "$PYTHON" -m retarget.preview "$@"
