#!/usr/bin/env bash
# Hardware-free smoke checks, not a Meta Quest integration test.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if command -v python3 >/dev/null 2>&1; then
  PY=python3
elif command -v python >/dev/null 2>&1; then
  PY=python
else
  echo 'ERROR: Python 3.10+ required to run tests.' >&2
  exit 3
fi
"$PY" -c 'import sys; assert sys.version_info >= (3,10), "Python >= 3.10 required"'
echo '=== Bash syntax check ==='
bash -n scripts/*.sh scripts/01_capture/*.sh
echo '=== Setup options check ==='
bash scripts/test_setup.sh
echo '=== Offline recorder and CSV validator tests ==='
"$PY" -m unittest discover -s pipeline/01_dataset/tests -p 'test_*.py' -v
echo '=== Source download preview ==='
bash scripts/bootstrap.sh --dry-run capture video retarget gripper
echo 'PASS: software dry-run checks. Real Meta Quest, WebRTC, camera and tracking NOT tested.'
