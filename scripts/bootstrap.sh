#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

usage() {
  cat <<'HELP'
Usage: bash scripts/bootstrap.sh [--dry-run] {capture|video|retarget|gripper|all|status} [...]
  capture   Quest streaming/client and Python receiver
  video     Meta Passthrough Camera Unity examples
  retarget  SEW references (WARP analytic solver requires separate licensing)
  gripper   RoboWheel public toolkit (not full hand-to-gripper mapping)
  all       all submodules
  status    list module states (no downloads)

Multiple groups can be selected, e.g. 'capture video gripper'.
This command ONLY clones pinned public source; it does not install dependencies,
build a Quest app, implement the recorder, or initialize any hardware.
HELP
}

if [[ ! -d .git ]]; then
  echo 'ERROR: Git repository metadata missing.' >&2; exit 2
fi

dry_run=0
all=0
status=0
declare -a selected=( ) paths=( )
for mode in "${@:-capture}"; do
  case "$mode" in
    --dry-run) dry_run=1;;
    capture|video|retarget|gripper) selected+=("$mode");;
    all) all=1;;
    status) status=1;;
    -h|--help|help) usage; exit 0;;
    *) echo "Unknown selection: $mode" >&2; usage >&2; exit 2;;
  esac
done
if ((status == 1)); then git submodule status || true; exit 0; fi
if ((all == 1)); then selected=(capture video retarget gripper); fi
if ((${#selected[@]} == 0)); then selected=(capture); fi

add_unique() {
  local new="$1" p
  for p in "${paths[@]}"; do [[ "$p" == "$new" ]] && return 0; done
  paths+=("$new")
}

for mode in "${selected[@]}"; do
  case "$mode" in
    capture)
      add_unique third_party/capture/xrt-client
      add_unique third_party/capture/xrt-devices;;
    video) add_unique third_party/capture/meta-passthrough-samples;;
    retarget)
      add_unique third_party/retarget/rby1-teleop
      add_unique third_party/retarget/stereo-sew
      add_unique third_party/retarget/ik-geo;;
    gripper) add_unique third_party/gripper/robowheel-toolkits;;
  esac
done

printf '%s (%s pinned git submodules):\n' "$([[ $dry_run -eq 1 ]] && echo 'DRY RUN' || echo 'Fetching')" "${#paths[@]}"
for p in "${paths[@]}"; do
  echo "--> $p"
  if ((dry_run == 0)); then
    git -c submodule.recurse=false submodule update --init -- "$p"
  fi
done

if ((dry_run == 1)); then
  echo 'No downloads performed.'
  exit 0
fi
for mode in "${selected[@]}"; do
  if [[ "$mode" == retarget ]]; then
    echo 'rby1_teleop nested submodules can be added later via its own README.'
    echo 'Its licensed geometric solver is NOT included in the public fallback.'
  fi
done
echo 'Source references downloaded at pinned revisions; no packages/apps installed.'
git submodule status
