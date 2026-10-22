#!/usr/bin/env bash
set -euo pipefail
standard_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$standard_script_dir/convert_episode_standard.py" "$@"
