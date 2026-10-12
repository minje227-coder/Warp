#!/usr/bin/env bash
# Lightweight no-network validation of CLI setup options on an available host.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
bash -n scripts/setup.sh scripts/bootstrap.sh
for os in linux macos windows; do
  bash scripts/setup.sh --os "$os" --quest quest3 --streams all --modules all --dry-run >/dev/null
  bash scripts/setup.sh --os "$os" --quest quest3s --streams body,hand --modules capture,video --dry-run >/dev/null
done
bash scripts/bootstrap.sh --dry-run capture video retarget gripper >/dev/null
bash scripts/bootstrap.sh --dry-run all >/dev/null
if bash scripts/setup.sh --os unsupported --dry-run >/dev/null 2>&1; then echo 'Invalid OS accepted' >&2; exit 1; fi
if bash scripts/setup.sh --os linux --streams unknown --dry-run >/dev/null 2>&1; then echo 'Invalid stream accepted' >&2; exit 1; fi
if bash scripts/setup.sh --os linux --modules none,retarget --dry-run >/dev/null 2>&1; then echo 'Invalid selection accepted' >&2; exit 1; fi
menu_out="$(printf '4\n2\n1,2,3\n1,2\nn\n' | bash scripts/setup.sh --interactive --dry-run)"
grep -q 'Host OS:          windows' <<< "$menu_out"
grep -q 'Capture streams:  body,hand,rgb_left' <<< "$menu_out"

echo 'OK: OS/Quest/stream/module selections and menu preview are valid (no hardware/network tests)'
