#!/usr/bin/env bash
set -euo pipefail
# Keep recording data and generated videos outside the source checkout.
standard_runtime="${KUAVO_STANDARD_RUNTIME:-/home/lab6/Warp-review/wilor-kuavo-standard}"
standard_python="${KUAVO_STANDARD_PYTHON:-/home/lab6/Warp/.venv-retarget/bin/python}"
exec "$standard_python" "$standard_runtime/standard.py" "$@"
