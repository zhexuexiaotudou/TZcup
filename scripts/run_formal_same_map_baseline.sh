#!/usr/bin/env bash
# Seal an already-completed live FullCoverage run as the same-map E2E baseline.
# This wrapper never launches Gazebo; coverage-runtime must come from the
# completed coverage_probe in the saved-map cleaning process.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EPISODE_MANIFEST=""
MAP_ROOT=""
MAPPING_RUNTIME=""
CLEANING_RUNTIME=""
LIFECYCLE_ACCEPTANCE=""
COVERAGE_RUNTIME=""
SESSION=""
SNAPSHOT="${ROOT}/reports/engineering/formal_vehicle_snapshot_manifest.json"
OUTPUT=""
SAFETY_MANAGER_READBACK=""
RUNTIME_BINDING=""
RUNTIME_CLOSURE=""
RUNTIME_INSTALL=""
EXPECTED_SAFETY_CAP=""

while (($#)); do
  case "$1" in
    --episode-manifest) EPISODE_MANIFEST="$2"; shift 2 ;;
    --map-root) MAP_ROOT="$2"; shift 2 ;;
    --mapping-runtime) MAPPING_RUNTIME="$2"; shift 2 ;;
    --cleaning-runtime) CLEANING_RUNTIME="$2"; shift 2 ;;
    --lifecycle-acceptance) LIFECYCLE_ACCEPTANCE="$2"; shift 2 ;;
    --coverage-runtime) COVERAGE_RUNTIME="$2"; shift 2 ;;
    --session) SESSION="$2"; shift 2 ;;
    --snapshot) SNAPSHOT="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    --safety-manager-readback) SAFETY_MANAGER_READBACK="$2"; shift 2 ;;
    --runtime-binding) RUNTIME_BINDING="$2"; shift 2 ;;
    --runtime-closure) RUNTIME_CLOSURE="$2"; shift 2 ;;
    --runtime-install) RUNTIME_INSTALL="$2"; shift 2 ;;
    --expected-safety-cap) EXPECTED_SAFETY_CAP="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

for value in EPISODE_MANIFEST MAP_ROOT MAPPING_RUNTIME CLEANING_RUNTIME LIFECYCLE_ACCEPTANCE COVERAGE_RUNTIME SESSION SNAPSHOT OUTPUT SAFETY_MANAGER_READBACK RUNTIME_BINDING RUNTIME_CLOSURE RUNTIME_INSTALL EXPECTED_SAFETY_CAP; do
  [[ -n "${!value}" ]] || { echo "missing required argument for ${value}" >&2; exit 2; }
done
[[ ! -e "${OUTPUT}" ]] || { echo "refusing to overwrite retained baseline: ${OUTPUT}" >&2; exit 3; }

SAFETY_ARGS=(
  --safety-manager-readback "${SAFETY_MANAGER_READBACK}"
  --runtime-binding "${RUNTIME_BINDING}"
  --runtime-closure "${RUNTIME_CLOSURE}"
  --runtime-install "${RUNTIME_INSTALL}"
  --expected-safety-cap "${EXPECTED_SAFETY_CAP}"
)

python3 "${ROOT}/scripts/generate_formal_same_map_baseline.py" generate \
  --episode-manifest "${EPISODE_MANIFEST}" \
  --map-root "${MAP_ROOT}" \
  --mapping-runtime "${MAPPING_RUNTIME}" \
  --cleaning-runtime "${CLEANING_RUNTIME}" \
  --lifecycle-acceptance "${LIFECYCLE_ACCEPTANCE}" \
  --coverage-runtime "${COVERAGE_RUNTIME}" \
  --session "${SESSION}" --snapshot "${SNAPSHOT}" --output "${OUTPUT}" \
  "${SAFETY_ARGS[@]}"

python3 "${ROOT}/scripts/generate_formal_same_map_baseline.py" validate \
  --input "${OUTPUT}" --session "${SESSION}" --snapshot "${SNAPSHOT}"
echo "sealed formal same-map FullCoverage baseline: ${OUTPUT}"
