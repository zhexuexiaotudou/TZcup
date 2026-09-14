#!/usr/bin/env bash
set -euo pipefail

WS="${SANITATION_WS:-$HOME/sanitation_ws}"
mkdir -p "$WS/src"

clone_pinned() {
  local url="$1"
  local revision="$2"
  local dst="$3"

  if [[ -d "$dst/.git" ]]; then
    if [[ -n "$(git -C "$dst" status --porcelain)" ]]; then
      echo "ERROR: refusing to alter dirty third-party checkout: $dst" >&2
      return 3
    fi
  elif [[ -e "$dst" ]]; then
    echo "ERROR: destination exists but is not a git repository: $dst" >&2
    return 3
  else
    git clone --filter=blob:none --no-checkout "$url" "$dst"
  fi

  git -C "$dst" fetch --depth 1 origin "$revision"
  git -C "$dst" checkout --detach FETCH_HEAD
  local actual
  actual="$(git -C "$dst" rev-parse HEAD)"
  if [[ "$actual" != "$revision" ]]; then
    echo "ERROR: revision mismatch for $dst: expected $revision, got $actual" >&2
    return 4
  fi
  printf '%s %s\n' "$actual" "$dst"
}

clone_pinned \
  https://github.com/linorobot/linorobot2.git \
  b96aa42fbfa4390a77e0aab90935fe55d66d04ba \
  "$WS/src/linorobot2"

clone_pinned \
  https://github.com/open-navigation/opennav_coverage.git \
  224118081c4c8de651f1db621053ab873b08f13d \
  "$WS/src/opennav_coverage"
