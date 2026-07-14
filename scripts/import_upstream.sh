#!/usr/bin/env bash
set -euo pipefail

WS="${SANITATION_WS:-$HOME/sanitation_ws}"
mkdir -p "$WS/src"

clone_or_update() {
  local url="$1"
  local branch="$2"
  local dst="$3"
  if [[ -d "$dst/.git" ]]; then
    git -C "$dst" fetch --all --tags --prune
    git -C "$dst" checkout "$branch"
    git -C "$dst" pull --ff-only
  else
    git clone --depth 1 --branch "$branch" "$url" "$dst"
  fi
}

clone_or_update \
  https://github.com/linorobot/linorobot2.git \
  jazzy \
  "$WS/src/linorobot2"

COV_URL=https://github.com/open-navigation/opennav_coverage.git
COV_DST="$WS/src/opennav_coverage"

choose_branch() {
  for b in jazzy-v2 v1.2.1-devel main; do
    if git ls-remote --exit-code --heads "$COV_URL" "$b" >/dev/null 2>&1; then
      echo "$b"
      return 0
    fi
  done
  return 1
}

COV_BRANCH="$(choose_branch)"
echo "OpenNav Coverage branch: $COV_BRANCH"
clone_or_update "$COV_URL" "$COV_BRANCH" "$COV_DST"

git -C "$WS/src/linorobot2" rev-parse HEAD
git -C "$COV_DST" rev-parse HEAD
