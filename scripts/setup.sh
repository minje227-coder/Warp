#!/usr/bin/env bash
# Interactive cross-platform configuration; does not implement or run capture.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

usage() {
  cat <<'HELP'
Usage:
  bash scripts/setup.sh                 # interactive terminal menu
  bash scripts/setup.sh --interactive   # force menu
  bash scripts/setup.sh --os linux --quest quest3 --streams all --modules all
  bash scripts/setup.sh --os macos --quest quest3s --streams body,hand --modules capture
  bash scripts/setup.sh --os windows --quest quest3 --streams rgb_left,rgb_right --modules capture,video

Options:
  --os        auto | linux | macos | windows
  --quest     quest3 | quest3s | questpro | quest2 | other | none
  --streams   comma-separated: body,hand,rgb_left,rgb_right | all | none
  --modules   comma-separated: capture,video,retarget,gripper | all | none
  --download  additionally fetch the selected pinned Git submodules (needs internet)
  --dry-run   show selected settings; do not save or download
  --interactive   show terminal menu even if CLI values were provided
  -h, --help     display help

The setting records *planned* capture streams; there is no unified Quest RGB+pose
recorder yet. Selecting unsupported hardware/options does not make them work.
Windows: run Bash scripts through Git Bash or WSL. In WSL choose OS=linux.
HELP
}

detect_os() {
  case "$(uname -s)" in
    Linux*) echo linux ;;
    Darwin*) echo macos ;;
    MINGW*|MSYS*|CYGWIN*) echo windows ;;
    *) echo unknown ;;
  esac
}

# Accept either numbers from the interactive menu or symbolic names from CLI.
map_menu_numbers() {
  local input="$1" kind="$2" item
  local -a parts=() out=()
  IFS=',' read -r -a parts <<< "$input"
  for item in "${parts[@]}"; do
    item="${item//[[:space:]]/}"
    case "$kind:$item" in
      stream:1) out+=(body);; stream:2) out+=(hand);;
      stream:3) out+=(rgb_left);; stream:4) out+=(rgb_right);;
      module:1) out+=(capture);; module:2) out+=(video);;
      module:3) out+=(retarget);; module:4) out+=(gripper);;
      *) echo "Invalid $kind choice: $item" >&2; return 2;;
    esac
  done
  ((${#out[@]} > 0)) || return 2
  local joined
  joined="$(IFS=,; echo "${out[*]}")"
  echo "$joined"
}

# Validate and canonically order multi-select options; reject invalid keys.
normalize_multi() {
  local raw="$1" kind="$2" item allowed="" result="" name
  local -a parts=()
  case "$kind" in
    streams) allowed='body hand rgb_left rgb_right';;
    modules) allowed='capture video retarget gripper';;
    *) echo "Invalid kind" >&2; return 2;;
  esac
  if [[ "$raw" == all ]]; then raw="${allowed// /,}"; fi
  if [[ "$raw" == none ]]; then echo none; return 0; fi
  IFS=',' read -r -a parts <<< "$raw"
  ((${#parts[@]} > 0)) || { echo "Empty $kind selection" >&2; return 2; }
  for item in "${parts[@]}"; do
    item="${item//[[:space:]]/}"
    [[ -n "$item" ]] || { echo "Empty $kind item" >&2; return 2; }
    case " $allowed " in *" $item "*) : ;; *) echo "Invalid $kind: $item" >&2; return 2;; esac
  done
  for name in $allowed; do
    for item in "${parts[@]}"; do
      if [[ "${item//[[:space:]]/}" == "$name" ]]; then
        [[ -z "$result" ]] && result="$name" || result+=",$name"
        break
      fi
    done
  done
  [[ -n "$result" ]] || return 2
  echo "$result"
}

os=auto
quest=quest3
streams=body,hand,rgb_right
modules=capture
interactive=0
download=0
dry_run=0
explicit=0

while (($#)); do
  case "$1" in
    --os|--quest|--streams|--modules)
      if (($# < 2)); then echo "Missing value for $1" >&2; exit 2; fi
      case "$1" in
        --os) os="$2";; --quest) quest="$2";;
        --streams) streams="$2";; --modules) modules="$2";;
      esac
      explicit=1; shift 2;;
    --download) download=1; shift;;
    --dry-run) dry_run=1; shift;;
    --interactive) interactive=1; shift;;
    -h|--help|help) usage; exit 0;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2;;
  esac
done

if ((interactive == 1)) || { ((explicit == 0)) && [[ -t 0 ]]; }; then
  echo '=== Humanoid Box Retrieval: configure ==='
  echo 'Choose one OS; WSL users should choose linux.'
  printf '1) auto [detected: %s]  2) linux  3) macos  4) windows\n' "$(detect_os)"
  read -r -p 'Host OS [1]: ' answer
  case "${answer:-1}" in 1) os=auto;; 2) os=linux;; 3) os=macos;; 4) os=windows;; *) echo 'Invalid OS choice' >&2; exit 2;; esac
  echo 'Quest model: 1) Quest 3  2) Quest 3S  3) Quest Pro  4) Quest 2  5) other  6) none'
  read -r -p 'Quest [1]: ' answer
  case "${answer:-1}" in 1) quest=quest3;; 2) quest=quest3s;; 3) quest=questpro;; 4) quest=quest2;; 5) quest=other;; 6) quest=none;; *) echo 'Invalid Quest choice' >&2; exit 2;; esac
  echo 'Planned streams (multi-select with comma-separated numbers):'
  echo '1) body  2) hand  3) left RGB  4) right RGB  5) all  6) none'
  read -r -p 'Streams [1,2,4]: ' answer
  case "${answer:-1,2,4}" in 5) streams=all;; 6) streams=none;; *) streams="$(map_menu_numbers "${answer:-1,2,4}" stream)";; esac
  echo 'Upstream code to select (multi-select with comma-separated numbers):'
  echo '1) capture  2) video  3) retarget  4) gripper  5) all  6) none'
  read -r -p 'Modules [1]: ' answer
  case "${answer:-1}" in 5) modules=all;; 6) modules=none;; *) modules="$(map_menu_numbers "${answer:-1}" module)";; esac
  if ((dry_run == 0)) && ((download == 0)); then
    read -r -p 'Download selected sources now (requires GitHub access)? [y/N]: ' answer
    case "$answer" in y|Y|yes|YES) download=1;; esac
  fi
fi

case "$os" in auto|linux|macos|windows) : ;; *) echo "Invalid OS: $os" >&2; exit 2;; esac
case "$quest" in quest3|quest3s|questpro|quest2|other|none) : ;; *) echo "Invalid Quest: $quest" >&2; exit 2;; esac
streams="$(normalize_multi "$streams" streams)"
modules="$(normalize_multi "$modules" modules)"
resolved_os="$os"
if [[ "$os" == auto ]]; then resolved_os="$(detect_os)"; fi
if [[ "$resolved_os" == unknown ]]; then
  echo 'Unsupported current host; choose --os explicitly' >&2
  exit 2
fi

cat <<EOF2
=== Selected configuration ===
Host OS:          $resolved_os (selection: $os)
Quest:            $quest
Capture streams:  $streams
Upstream modules: $modules
Downloads now:    $([[ $download -eq 1 ]] && echo yes || echo no)
EOF2
if [[ "$quest" == quest2 || "$quest" == questpro || "$quest" == other ]]; then
  echo 'NOTE: Quest body tracking/XRT-Client compatibility must be verified for this model.'
fi
if [[ "$streams" == *rgb* ]]; then
  echo 'NOTE: Recorder requires one physical Quest RGB camera by default. Configure Warp Quest RGB; --body-only explicitly disables RGB.'
  echo 'NOTE: One eye per episode; setup selections do not enable a second RGB camera. Hardware clock synchronization is not implemented.'
fi
if [[ "$resolved_os" == windows ]]; then
  echo 'NOTE: Windows scripts require Git Bash; alternatively use WSL with --os linux.'
fi
if [[ "$resolved_os" != "$(detect_os)" ]]; then
  echo 'NOTE: The selected OS differs from this shell host; configuration is a target, not installation.'
fi

if ((dry_run == 1)); then
  echo 'DRY RUN: no files updated, no downloads.'
  if ((download == 1)) && [[ "$modules" != none ]]; then
    IFS=',' read -r -a selected <<< "$modules"
    bash scripts/bootstrap.sh --dry-run "${selected[@]}"
  fi
  exit 0
fi

mkdir -p configs
# Only predefined tokens reach this file (no arbitrary shell input).
cat > configs/local.env <<EOF2
# Generated by scripts/setup.sh; gitignored local preferences.
HOST_OS=$resolved_os
QUEST_MODEL=$quest
CAPTURE_STREAMS=$streams
UPSTREAM_MODULES=$modules
EOF2
echo 'Saved local preferences to configs/local.env (gitignored).'

if ((download == 1)); then
  if [[ "$modules" == none ]]; then
    echo 'No upstream modules selected; skipping downloads.'
  else
    IFS=',' read -r -a selected <<< "$modules"
    bash scripts/bootstrap.sh "${selected[@]}"
  fi
else
  echo 'No sources downloaded. To fetch selected upstream modules later:'
  if [[ "$modules" != none ]]; then
    echo "  bash scripts/bootstrap.sh ${modules//,/ }"
  fi
fi

echo 'Next: bash scripts/doctor.sh'
