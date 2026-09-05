#!/usr/bin/env bash
# Capture G3 only from a caller-provided fresh combined runtime.  This wrapper
# deliberately refuses the historical Stage1/Docker fallback.
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
root="${AUTO05_G4_ROOT:-$repo/.work/auto05-g4}"
runtime_setup="${AUTO05_COMBINED_RUNTIME_SETUP:?set AUTO05_COMBINED_RUNTIME_SETUP}"

case "$root" in "$repo/.work"/*) ;; *) echo "AUTO05_G4_ROOT must be under $repo/.work" >&2; exit 64;; esac
case "$runtime_setup" in "$repo/.work"/*) ;; *) echo "runtime setup must be in a fresh TZcup .work runtime" >&2; exit 64;; esac
test -f "$runtime_setup"
mkdir -p "$root/data/g3_screening_native" "$root/runtime_ws" "$root/evidence"

export AUTO05_G4_RUNTIME_BOUND=1
export AUTO05_DATA_ROOT="$root/data/g3_screening_native"
export AUTO05_RUNTIME_WS="$root/runtime_ws"
export AUTO05_COMBINED_RUNTIME_SETUP="$runtime_setup"
exec "$repo/scripts/auto05_capture_all.sh"
