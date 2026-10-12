#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git is required." >&2
  exit 2
fi
if ! command -v gh >/dev/null 2>&1; then
  echo "ERROR: GitHub CLI (gh) is required. Install from https://cli.github.com/" >&2
  exit 2
fi
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "ERROR: Project is not initialized as a Git repository." >&2
  exit 2
fi
if [[ "$(git branch --show-current)" != "main" ]]; then
  echo "ERROR: Expected the main branch before publishing." >&2
  exit 2
fi
if [[ -n "$(git status --porcelain)" ]]; then
  echo "ERROR: Uncommitted or untracked files detected; review them before public publishing." >&2
  git status --short
  exit 2
fi
if git remote get-url origin >/dev/null 2>&1; then
  echo "ERROR: An origin remote already exists. Refusing to push to an unverified target." >&2
  exit 2
fi
if ! gh auth status >/dev/null 2>&1; then
  echo "ERROR: Log in first: gh auth login" >&2
  exit 2
fi
OWNER="$(gh api user --jq '.login')"
[[ -n "$OWNER" ]] || { echo "ERROR: Cannot determine your GitHub account." >&2; exit 2; }
if gh repo view "$OWNER/Warp" >/dev/null 2>&1; then
  echo "ERROR: $OWNER/Warp already exists. Refusing to overwrite it." >&2
  exit 2
fi
printf 'Create and publicly publish this checked-in source to GitHub: %s/Warp? [y/N] ' "$OWNER"
read -r reply
case "$reply" in y|Y|yes|YES) : ;; *) echo 'Cancelled.'; exit 1;; esac

gh repo create "$OWNER/Warp" --public --source=. --remote=origin --push

echo 'Checking remote visibility...'
gh repo view "$OWNER/Warp" --json nameWithOwner,url,isPrivate --jq '"Repo: " + .nameWithOwner + " URL: " + .url + " public=" + (if .isPrivate then "false" else "true" end)'
