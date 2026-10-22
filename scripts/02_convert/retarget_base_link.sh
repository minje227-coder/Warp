#!/usr/bin/env bash
# Offline preparation + adopted IK. No ROS control publishers/services.
set -euo pipefail
if [[ $# -lt 4 ]]; then
  echo "Usage: $0 EPISODE PREPARED_JSON CALIBRATION_JSON NEW_INPUT_DIR [standard converter options]" >&2
  exit 2
fi
scripts_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
episode="$1"; prepared="$2"; calibration="$3"; inputs="$4"
shift 4
standard_python="${KUAVO_STANDARD_PYTHON:-/home/lab6/Warp/.venv-retarget/bin/python}"
"$standard_python" "$scripts_dir/prepare_base_link.py" --prepared "$prepared" --calibration "$calibration" --output "$inputs"
# The adopted renderer reads coordinate_preparation and transforms the original
# scene into base_link. Four-window rendering is enabled unless --no-render is supplied.
bash "$scripts_dir/convert_episode_standard.sh" "$episode" --prepared "$inputs/prepared.json" "$@"
