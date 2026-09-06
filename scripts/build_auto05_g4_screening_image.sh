#!/usr/bin/env bash
# Build the Stage5B/G4 screening image from a fresh, retained context below
# TZcup/.work.  The context is deliberately not a sibling of TZcup.
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# A cross-host importer owns AUTO05_G4_ROOT atomically.  Build its OCI receipt
# elsewhere first when that root must remain absent for fail-closed import.
root="${AUTO05_G4_IMAGE_ROOT:-${AUTO05_G4_ROOT:-$repo/.work/auto05-g4}}"
tag="${AUTO05_G4_SCREENING_IMAGE_TAG:-tzcup/auto05-g4-screening:local}"
case "$root" in "$repo/.work"/*) ;; *) echo "AUTO05_G4_ROOT must be under $repo/.work" >&2; exit 64;; esac
context="$root/image-context"
receipt="$root/evidence/screening_image.json"
test ! -e "$context" && test ! -e "$receipt" || {
  echo "refusing to overwrite G4 image context or receipt" >&2; exit 64;
}
mkdir -p "$context" "$root/evidence"
cp "$repo/docker/Dockerfile.stage5b" "$context/Dockerfile"
docker build --tag "$tag" "$context"
docker run --rm -v "$repo:/repo:ro" "$tag" \
  python3 /repo/scripts/test_auto05_g4_torch_runtime.py
python3 - "$tag" "$context" "$receipt" "$repo" <<'PY'
import json, subprocess, sys, time
tag, context, receipt, repo = sys.argv[1:]
image = subprocess.check_output(
    ["docker", "image", "inspect", "--format", "{{.Id}}", tag], text=True
).strip()
git = lambda *args: subprocess.check_output(["git", "-C", repo, *args], text=True).strip()
open(receipt, "x", encoding="utf-8").write(json.dumps({
    "schema_version": 1, "status": "AUTO05_G4_SCREENING_IMAGE_BUILT",
    "image_tag": tag, "image_id": image,
    "build_context_repository_relative": __import__("pathlib").Path(context).resolve().relative_to(__import__("pathlib").Path(repo).resolve()).as_posix(),
    "runtime_parity_test": "scripts/test_auto05_g4_torch_runtime.py",
    "runtime_parity_test_passed": True,
    "git": {"head": git("rev-parse", "HEAD"), "tree": git("rev-parse", "HEAD^{tree}")},
    "epoch_ns": time.time_ns(),
}, indent=2, sort_keys=True) + "\n")
PY
