#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${SANITATION_WS:-$HOME/sanitation_ws}"
mkdir -p "$WS/src"

OPENNAV_COVERAGE_REVISION="224118081c4c8de651f1db621053ab873b08f13d"
OPENNAV_COVERAGE_PATCH="$PACK_ROOT/patches/upstream/opennav_coverage/2241180-test-path-fixed-seed.patch"
OPENNAV_COVERAGE_PATCH_PATH="patches/upstream/opennav_coverage/2241180-test-path-fixed-seed.patch"
OPENNAV_COVERAGE_PATCH_SHA256="c101a9bfa3078139566fe8577f63a4cc525bde71d8fb3f244fdc2beb846af0b1"
OPENNAV_COVERAGE_PATCH_TARGET="opennav_coverage/test/test_path.cpp"
OPENNAV_COVERAGE_PATCH_COMMIT_MESSAGE="Apply deterministic seed to upstream path test"
OPENNAV_COVERAGE_PATCH_IDENTITY="TZcup Upstream Patch Bot"
OPENNAV_COVERAGE_PATCH_EMAIL="upstream-patches@tzcup.invalid"
OPENNAV_COVERAGE_PATCH_DATE="2000-01-01T00:00:00+00:00"
UPSTREAM_PATCH_REPORT="${UPSTREAM_PATCH_REPORT:-$WS/opennav_coverage_patch_status.json}"

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

apply_pinned_patch() {
  local repo="$1"
  local base_revision="$2"
  local patch_file="$3"
  local expected_target="$4"
  local report_path="$5"

  if [[ ! -f "$patch_file" ]]; then
    echo "ERROR: required upstream patch does not exist: $patch_file" >&2
    return 5
  fi

  local patch_sha256
  patch_sha256="$(sha256sum "$patch_file" | awk '{print $1}')"
  if [[ "$patch_sha256" != "$OPENNAV_COVERAGE_PATCH_SHA256" ]]; then
    echo "ERROR: upstream patch SHA256 mismatch: expected $OPENNAV_COVERAGE_PATCH_SHA256, got $patch_sha256" >&2
    return 5
  fi

  local actual_base
  actual_base="$(git -C "$repo" rev-parse HEAD)"
  if [[ "$actual_base" != "$base_revision" ]]; then
    echo "ERROR: refusing to patch unexpected base: expected $base_revision, got $actual_base" >&2
    return 5
  fi

  git -C "$repo" apply --check "$patch_file"
  git -C "$repo" apply "$patch_file"
  git -C "$repo" diff --check

  local status
  status="$(git -C "$repo" status --porcelain=v1)"
  if [[ "$status" != " M $expected_target" ]]; then
    echo "ERROR: patched checkout has unexpected changes:" >&2
    printf '%s\n' "$status" >&2
    return 5
  fi

  local base_tree patched_commit patched_tree patched_diff_sha256
  base_tree="$(git -C "$repo" rev-parse 'HEAD^{tree}')"
  git -C "$repo" add -- "$expected_target"
  GIT_AUTHOR_NAME="$OPENNAV_COVERAGE_PATCH_IDENTITY" \
  GIT_AUTHOR_EMAIL="$OPENNAV_COVERAGE_PATCH_EMAIL" \
  GIT_AUTHOR_DATE="$OPENNAV_COVERAGE_PATCH_DATE" \
  GIT_COMMITTER_NAME="$OPENNAV_COVERAGE_PATCH_IDENTITY" \
  GIT_COMMITTER_EMAIL="$OPENNAV_COVERAGE_PATCH_EMAIL" \
  GIT_COMMITTER_DATE="$OPENNAV_COVERAGE_PATCH_DATE" \
    git -C "$repo" -c commit.gpgSign=false commit --no-gpg-sign --no-verify \
      -m "$OPENNAV_COVERAGE_PATCH_COMMIT_MESSAGE"

  patched_commit="$(git -C "$repo" rev-parse HEAD)"
  patched_tree="$(git -C "$repo" rev-parse 'HEAD^{tree}')"
  patched_diff_sha256="$(git -C "$repo" diff --binary --no-ext-diff \
    "$base_revision..$patched_commit" | sha256sum | awk '{print $1}')"
  status="$(git -C "$repo" status --porcelain=v1)"
  if [[ -n "$status" ]]; then
    echo "ERROR: deterministic patched checkout is not clean:" >&2
    printf '%s\n' "$status" >&2
    return 5
  fi

  OPENNAV_PATCH_BASE_COMMIT="$actual_base" \
  OPENNAV_PATCH_BASE_TREE="$base_tree" \
  OPENNAV_PATCH_PATH="$OPENNAV_COVERAGE_PATCH_PATH" \
  OPENNAV_PATCH_SHA256="$patch_sha256" \
  OPENNAV_PATCH_PATCHED_COMMIT="$patched_commit" \
  OPENNAV_PATCH_PATCHED_TREE="$patched_tree" \
  OPENNAV_PATCH_DIFF_SHA256="$patched_diff_sha256" \
  OPENNAV_PATCH_TARGET="$expected_target" \
  OPENNAV_PATCH_COMMIT_MESSAGE="$OPENNAV_COVERAGE_PATCH_COMMIT_MESSAGE" \
  OPENNAV_PATCH_COMMIT_IDENTITY="$OPENNAV_COVERAGE_PATCH_IDENTITY" \
  OPENNAV_PATCH_COMMIT_EMAIL="$OPENNAV_COVERAGE_PATCH_EMAIL" \
  OPENNAV_PATCH_COMMIT_DATE="$OPENNAV_COVERAGE_PATCH_DATE" \
  OPENNAV_PATCH_REPORT="$report_path" \
  python3 - <<'PY'
import json
import os
from pathlib import Path

report = {
    "schema_version": 1,
    "repository": "opennav_coverage",
    "base_commit": os.environ["OPENNAV_PATCH_BASE_COMMIT"],
    "base_tree": os.environ["OPENNAV_PATCH_BASE_TREE"],
    "patch_path": os.environ["OPENNAV_PATCH_PATH"],
    "patch_sha256": os.environ["OPENNAV_PATCH_SHA256"],
    "patched_commit": os.environ["OPENNAV_PATCH_PATCHED_COMMIT"],
    "patched_tree": os.environ["OPENNAV_PATCH_PATCHED_TREE"],
    "patched_diff_sha256": os.environ["OPENNAV_PATCH_DIFF_SHA256"],
    "patched_files": [os.environ["OPENNAV_PATCH_TARGET"]],
    "commit_metadata": {
        "message": os.environ["OPENNAV_PATCH_COMMIT_MESSAGE"],
        "name": os.environ["OPENNAV_PATCH_COMMIT_IDENTITY"],
        "email": os.environ["OPENNAV_PATCH_COMMIT_EMAIL"],
        "author_date": os.environ["OPENNAV_PATCH_COMMIT_DATE"],
        "committer_date": os.environ["OPENNAV_PATCH_COMMIT_DATE"],
    },
    "working_tree_clean": True,
    "status_porcelain": [],
}
output = Path(os.environ["OPENNAV_PATCH_REPORT"])
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
PY

  printf 'opennav_coverage base_commit=%s patch_sha256=%s patched_commit=%s patched_tree=%s patched_diff_sha256=%s working_tree_clean=true\n' \
    "$actual_base" "$patch_sha256" "$patched_commit" "$patched_tree" "$patched_diff_sha256"
}

clone_pinned \
  https://github.com/linorobot/linorobot2.git \
  b96aa42fbfa4390a77e0aab90935fe55d66d04ba \
  "$WS/src/linorobot2"

clone_pinned \
  https://github.com/open-navigation/opennav_coverage.git \
  "$OPENNAV_COVERAGE_REVISION" \
  "$WS/src/opennav_coverage"

apply_pinned_patch \
  "$WS/src/opennav_coverage" \
  "$OPENNAV_COVERAGE_REVISION" \
  "$OPENNAV_COVERAGE_PATCH" \
  "$OPENNAV_COVERAGE_PATCH_TARGET" \
  "$UPSTREAM_PATCH_REPORT"
