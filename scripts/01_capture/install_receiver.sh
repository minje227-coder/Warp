#!/usr/bin/env bash
# Create a dedicated Python venv and install the pinned upstream receiver.
# Does not install the Quest APK or implement an RGB recorder.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
SRC="$ROOT/third_party/capture/xrt-devices"
if [[ ! -f "$SRC/pyproject.toml" ]]; then
  echo 'ERROR: XRT_devices source not downloaded yet.' >&2
  echo '  bash scripts/bootstrap.sh capture' >&2
  exit 3
fi

if [[ -x "$ROOT/.venv/bin/python" ]]; then
  PY="$ROOT/.venv/bin/python"
elif [[ -x "$ROOT/.venv/Scripts/python.exe" ]]; then
  PY="$ROOT/.venv/Scripts/python.exe"
else
  if command -v python3 >/dev/null 2>&1; then
    SYSTEM_PY=python3
  elif command -v python >/dev/null 2>&1; then
    SYSTEM_PY=python
  else
    echo 'ERROR: Python 3.10+ not found.' >&2
    exit 3
  fi
  "$SYSTEM_PY" -c 'import sys; assert sys.version_info >= (3,10), "Python >=3.10 required"'
  "$SYSTEM_PY" -m venv "$ROOT/.venv"
  if [[ -x "$ROOT/.venv/bin/python" ]]; then PY="$ROOT/.venv/bin/python"; else PY="$ROOT/.venv/Scripts/python.exe"; fi
fi

"$PY" -c 'import sys; assert sys.version_info >= (3,10), "Python >=3.10 required"'
"$PY" -m pip install -e "${SRC}[xr,recording]"
echo 'Receiver installed into .venv. Check with:'
echo '  bash scripts/01_capture/record_body.sh --check'
