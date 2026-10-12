#!/usr/bin/env bash
# Pure-standard-library skeleton CSV validation; works with Git Bash on Windows.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -x "$ROOT/.venv/bin/python" ]]; then
  PY="$ROOT/.venv/bin/python"
elif [[ -x "$ROOT/.venv/Scripts/python.exe" ]]; then
  PY="$ROOT/.venv/Scripts/python.exe"
elif command -v python3 >/dev/null 2>&1; then
  PY=python3
elif command -v python >/dev/null 2>&1; then
  PY=python
else
  echo 'ERROR: Python 3.10+ not found.' >&2
  exit 3
fi
cd "$ROOT"
exec "$PY" pipeline/01_dataset/validate_episode.py "$@"
