#!/usr/bin/env bash
# Read-only readiness report. No hardware test, downloads, or installation.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
echo '=== saved setup choices (not activated features) ==='
if [[ -f configs/local.env ]]; then
  grep -E '^(HOST_OS|QUEST_MODEL|CAPTURE_STREAMS|UPSTREAM_MODULES)=' configs/local.env || true
else
  echo '[unconfigured] bash scripts/setup.sh'
fi

echo '=== host tools ==='
for prog in git python3 python uv adb gh; do
  if command -v "$prog" >/dev/null 2>&1; then
    echo "[found]   $prog: $(command -v "$prog")"
  else
    echo "[missing] $prog"
  fi
done

echo '=== upstream sources ==='
if [[ -f third_party/capture/xrt-devices/pyproject.toml ]]; then
  echo '[downloaded] XRT_devices receiver source'
else
  echo '[missing] XRT_devices receiver source. bash scripts/bootstrap.sh capture'
fi
if command -v git >/dev/null 2>&1; then
  git submodule status || true
  echo '(Leading -: pinned upstream source not downloaded in this checkout.)'
fi

echo '=== installed receiver ==='
if [[ -x .venv/bin/python ]]; then
  PY=.venv/bin/python
elif [[ -x .venv/Scripts/python.exe ]]; then
  PY=.venv/Scripts/python.exe
else
  PY=""
fi
if [[ -z "$PY" ]]; then
  echo '[missing] project .venv. Run bash scripts/01_capture/install_receiver.sh after bootstrap.'
elif "$PY" pipeline/01_dataset/record_quest_body.py --check; then
  echo '[ready] XRT Python API and fresh physical Quest RGB source; skeleton connection not tested.'
else
  echo '[not ready] Check XR dependencies and the configured Warp Quest RGB sender.'
fi

echo '=== runnable vs not-yet-implemented ==='
echo '[implemented] Skeleton CSV + physical Quest RGB JPEG/MP4, original clock sidecars and episode validation'
echo '[not implemented] Hardware synchronization of camera exposure and skeleton acquisition'
echo '[not implemented] Quest/Rack/Box camera calibration and object pose'
echo '[not implemented] 4-DoF waist + gripper retargeting'
echo '[not implemented] Collision filter, policy training, RL fine-tuning'
echo
cat <<'HELP'
Suggested first real capture commands (requires Quest, same LAN, pinned upstream + venv):
  bash scripts/bootstrap.sh capture
  bash scripts/01_capture/install_receiver.sh
  bash scripts/01_capture/record_body.sh --session smoke_0001 --duration 20
  bash scripts/01_capture/validate_body.sh local_data/raw_human/smoke_0001 --strict-sew
See docs/CAPTURE_GUIDE.md for developer mode, APK installation, and troubleshooting.
HELP
