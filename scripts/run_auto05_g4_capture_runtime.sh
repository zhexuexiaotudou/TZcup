#!/usr/bin/env bash
# Capture G3 only from a caller-provided fresh combined runtime.  This wrapper
# deliberately refuses the historical Stage1/Docker fallback.
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
root="${AUTO05_G4_ROOT:-$repo/.work/auto05-g4}"
runtime_setup="${AUTO05_COMBINED_RUNTIME_SETUP:?set AUTO05_COMBINED_RUNTIME_SETUP}"
install_root="${AUTO05_G4_INSTALL_ROOT:?set AUTO05_G4_INSTALL_ROOT}"
closure="${AUTO05_G4_CLOSURE_MANIFEST:?set AUTO05_G4_CLOSURE_MANIFEST}"
session="${AUTO05_G4_SESSION:?set AUTO05_G4_SESSION}"
snapshot="${AUTO05_G4_SNAPSHOT:?set AUTO05_G4_SNAPSHOT}"
domain="${AUTO05_G4_ROS_DOMAIN_ID:?set AUTO05_G4_ROS_DOMAIN_ID}"
partition="${AUTO05_G4_GZ_PARTITION:?set AUTO05_G4_GZ_PARTITION}"

case "$root" in "$repo/.work"/*) ;; *) echo "AUTO05_G4_ROOT must be under $repo/.work" >&2; exit 64;; esac
case "$runtime_setup" in "$repo/.work"/*) ;; *) echo "runtime setup must be in a fresh TZcup .work runtime" >&2; exit 64;; esac
test -f "$runtime_setup"
test ! -e "$root/data/g3_screening_native" || {
  echo "G4 capture requires a fresh raw data root" >&2; exit 64;
}
mkdir -p "$root/data/g3_screening_native" "$root/runtime_ws" "$root/evidence"

binding="$root/evidence/runtime_gate_binding.json"
test ! -e "$binding" || { echo "refusing to overwrite $binding" >&2; exit 64; }
source "$repo/scripts/run_formal_runtime_isolation.sh"
formal_runtime_configure "$domain"
export GZ_PARTITION="$partition"
trap 'formal_runtime_cleanup_partition "$GZ_PARTITION" || true' EXIT
python3 "$repo/scripts/bind_auto05_g4_runtime.py" \
  --repository-root "$repo" --install-root "$install_root" \
  --closure-manifest "$closure" --session "$session" --snapshot "$snapshot" \
  --contract "$repo/starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml" \
  --data-root "$root/data/g3_screening_native" --ros-domain-id "$domain" \
  --gz-partition "$partition" --gazebo-lock "$FORMAL_RUNTIME_LOCK_FILE" \
  --output "$binding"

export AUTO05_G4_RUNTIME_BOUND=1
export AUTO05_REPO_ROOT="$repo"
export AUTO05_DATA_ROOT="$root/data/g3_screening_native"
export AUTO05_RUNTIME_WS="$root/runtime_ws"
export AUTO05_COMBINED_RUNTIME_SETUP="$runtime_setup"
bash "$repo/scripts/auto05_capture_all.sh"
python3 - "$binding" "$root/data/g3_screening_native" "$root/evidence/capture_complete.json" <<'PY'
import hashlib, json, sys, time
binding, data, output = map(__import__("pathlib").Path, sys.argv[1:])
if output.exists():
    raise SystemExit("refusing to overwrite capture receipt")
world_generation = data / "world_generation.json"
reports = sorted(data.glob("scenes/*/capture_report.json"))
if not world_generation.is_file() or len(reports) != 120:
    raise SystemExit("G4 capture is incomplete: expected world generation and 120 scene reports")
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
output.write_text(json.dumps({
    "schema_version": 1, "status": "AUTO05_G4_CAPTURE_COMPLETE",
    "runtime_binding_sha256": sha(binding), "raw_data_root": str(data),
    "world_generation_sha256": sha(world_generation),
    "capture_report_count": len(reports),
    "capture_report_sha256": {str(path.relative_to(data)): sha(path) for path in reports},
    "epoch_ns": time.time_ns(),
}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
