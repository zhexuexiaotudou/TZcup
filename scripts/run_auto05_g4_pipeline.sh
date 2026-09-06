#!/usr/bin/env bash
# One fail-closed G4 entrypoint: either formal-bound local capture or a verified
# native cross-host import, then QA, frozen screening, and review finalization.
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
root="${AUTO05_G4_ROOT:-$repo/.work/auto05-g4}"
image="${AUTO05_G4_SCREENING_IMAGE:?set AUTO05_G4_SCREENING_IMAGE}"
case "$root" in "$repo/.work"/*) ;; *) echo "AUTO05_G4_ROOT must be under $repo/.work" >&2; exit 64;; esac
image_receipt="$root/evidence/screening_image.json"
test -f "$image_receipt" || { echo "G4 requires a retained screening-image receipt" >&2; exit 64; }
python3 - "$image_receipt" "$image" "$repo" <<'PY'
import json, subprocess, sys
receipt = json.load(open(sys.argv[1], encoding="utf-8"))
actual = subprocess.check_output(
    ["docker", "image", "inspect", "--format", "{{.Id}}", sys.argv[2]], text=True
).strip()
head = subprocess.check_output(["git", "-C", sys.argv[3], "rev-parse", "HEAD"], text=True).strip()
if receipt.get("status") != "AUTO05_G4_SCREENING_IMAGE_BUILT" or receipt.get("image_tag") != sys.argv[2] or receipt.get("image_id") != actual or receipt.get("runtime_parity_test_passed") is not True or receipt.get("git", {}).get("head") != head:
    raise SystemExit("screening image is not the retained G4 build receipt")
PY

if [[ "${AUTO05_G4_IMPORTED_HANDOFF:-0}" == 1 ]]; then
  python3 "$repo/scripts/auto05_g4_cross_host_handoff.py" verify-import \
    --repo "$repo" --oci-receipt "$image_receipt"
else
  bash "$repo/scripts/run_auto05_g4_capture_runtime.sh"
fi
dataset="$root/evidence/dataset"
test ! -e "$dataset" || { echo "G4 dataset finalization output already exists" >&2; exit 64; }
python3 "$repo/scripts/auto05_finalize_dataset.py" \
  --data-root "$root/data/g3_screening_native" --output-dir "$dataset"
if [[ "${AUTO05_G4_IMPORTED_HANDOFF:-0}" == 1 ]]; then
  python3 "$repo/scripts/auto05_g4_cross_host_handoff.py" verify-import \
    --repo "$repo" --oci-receipt "$image_receipt"
fi

screening="$root/evidence/screening"
attempt_ledger="$root/evidence/g4_attempt_ledger.json"
test_lock="$root/evidence/g4_test_consumed_lock.json"
test ! -e "$screening" && test ! -e "$attempt_ledger" && test ! -e "$test_lock" || {
  echo "G4 screening output, attempt ledger, or frozen test lock already exists" >&2; exit 64;
}
head=$(git -C "$repo" rev-parse HEAD)
cross_host_args=()
if [[ "${AUTO05_G4_IMPORTED_HANDOFF:-0}" == 1 ]]; then
  cross_host_args+=(--g4-cross-host-import /repo/.work/auto05-g4/evidence/cross_host_import.json)
fi
set +e
docker run --rm --gpus all --shm-size 4g \
  -v "$repo:/repo:ro" -v "$root:/repo/.work/auto05-g4" \
  "$image" python3 /repo/scripts/auto05_screening.py \
    --data-root /repo/.work/auto05-g4/data/g3_screening_native \
    --dataset-evidence /repo/.work/auto05-g4/evidence/dataset \
    --output /repo/.work/auto05-g4/evidence/screening \
    --implementation-commit "$head" --attempt 4 \
    --g4-contract /repo/starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml \
  --g4-runtime-binding /repo/.work/auto05-g4/evidence/runtime_gate_binding.json \
  --g4-attempt-ledger /repo/.work/auto05-g4/evidence/g4_attempt_ledger.json \
  --g4-test-lock /repo/.work/auto05-g4/evidence/g4_test_consumed_lock.json \
  "${cross_host_args[@]}"
screening_rc=$?
set -e
if [[ "$screening_rc" != 0 && "$screening_rc" != 2 ]]; then
  exit "$screening_rc"
fi
if [[ "${AUTO05_G4_IMPORTED_HANDOFF:-0}" == 1 ]]; then
  python3 "$repo/scripts/auto05_g4_cross_host_handoff.py" verify-import \
    --repo "$repo" --oci-receipt "$image_receipt"
fi

finalizer_import=()
if [[ "${AUTO05_G4_IMPORTED_HANDOFF:-0}" == 1 ]]; then
  finalizer_import+=(--cross-host-import "$root/evidence/cross_host_import.json")
fi
python3 "$repo/scripts/finalize_auto05_g4.py" \
  --repo "$repo" --raw-root "$screening" --dataset-evidence "$dataset" \
  --g4-contract "$repo/starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml" \
  --runtime-binding "$root/evidence/runtime_gate_binding.json" \
  --capture-receipt "$root/evidence/capture_complete.json" \
  --attempt-ledger "$attempt_ledger" \
  --test-lock "$test_lock" --output "$root/evidence/finalization" \
  "${finalizer_import[@]}"
exit "$screening_rc"
