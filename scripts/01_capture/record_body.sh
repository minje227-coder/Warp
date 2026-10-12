#!/usr/bin/env bash
# One-app shared-timestamp RGB + Skeleton + AprilTag. --body-only keeps legacy support.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -x "$ROOT/.venv/bin/python" ]]; then
  PY="$ROOT/.venv/bin/python"
elif [[ -x "$ROOT/.venv/Scripts/python.exe" ]]; then
  PY="$ROOT/.venv/Scripts/python.exe"
else
  echo 'ERROR: XRT receiver venv not found. Run:' >&2
  echo '  bash scripts/bootstrap.sh capture' >&2
  echo '  bash scripts/01_capture/install_receiver.sh' >&2
  exit 3
fi
cd "$ROOT"
for arg in "$@"; do
  if [[ "$arg" == "--body-only" ]]; then
    exec "$PY" pipeline/01_dataset/record_quest_body.py "$@"
  fi
done
exec "$PY" pipeline/01_dataset/record_unified_quest.py "$@"
