#!/usr/bin/env bash
# Serial simulation-only 0.25 -> 0.45 -> 0.70 -> 1.00 m/s requalification.
# It never changes the checked-in product 0.45 m/s envelope.
set -euo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
[[ "${FORMAL_DRY_SPEED_REQUALIFICATION:-}" == "1" ]] || {
  echo "Refusing speed qualification: set FORMAL_DRY_SPEED_REQUALIFICATION=1" >&2; exit 2;
}
run_root="${FORMAL_DRY_SPEED_REQUALIFICATION_ROOT:-${repo_root}/.work/formal_dry_speed_requalification}"
[[ ! -e "${run_root}" ]] || { echo "Refusing stale requalification evidence: ${run_root}" >&2; exit 2; }
mkdir -p "${run_root}"
profile="${repo_root}/config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml"
stages=(speed_0_25_mps speed_0_45_mps speed_0_70_mps speed_1_00_mps)
predecessor=""

for stage in "${stages[@]}"; do
  stage_root="${run_root}/${stage}"
  marker="${stage_root}/requalification.opt_in.marker.json"
  mkdir -p "${stage_root}"
  speed="$(python3 - "${profile}" "${stage}" <<'PY'
import sys, yaml
profile = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
for item in profile["qualification_stages"]:
    if item["id"] == sys.argv[2]:
        print(item["target_linear_speed_mps"])
        break
else:
    raise SystemExit("qualification stage missing from profile")
PY
)"
  python3 "${repo_root}/scripts/formal_dry_speed_requalification_token.py" --create \
    --profile "${profile}" --run-root "${stage_root}" --token "${marker}" --stage "${stage}"

  common=(FORMAL_DRY_SPEED_REQUALIFICATION=1 FORMAL_DRY_SPEED_REQUALIFICATION_ROOT="${stage_root}" FORMAL_DRY_SPEED_REQUALIFICATION_MARKER="${marker}")
  env "${common[@]}" FORMAL_VEHICLE_MOBILITY_OUTPUT="${stage_root}/mobility.json" \
    FORMAL_VEHICLE_MOBILITY_FORWARD_SPEED_MPS="${speed}" FORMAL_VEHICLE_MOBILITY_FORWARD_DURATION_S=4.0 \
    FORMAL_VEHICLE_MOBILITY_SAFETY_MAX_LINEAR_VELOCITY="${speed}" FORMAL_VEHICLE_MOBILITY_EXERCISE_ESTOP=1 \
    "${repo_root}/scripts/run_formal_vehicle_mobility_runtime.sh"
  env "${common[@]}" WHOLE_VEHICLE_INTERLOCK_SAFETY_MAX_LINEAR_VELOCITY="${speed}" \
    WHOLE_VEHICLE_INTERLOCK_BASE_LINEAR_SPEED="${speed}" \
    "${repo_root}/scripts/run_whole_vehicle_actuator_interlock_runtime.sh" "${stage_root}/interlock.json"
  env "${common[@]}" FORMAL_DYNAMIC_OUTPUT="${stage_root}/dynamic.json" \
    FORMAL_DYNAMIC_TELEMETRY="${stage_root}/dynamic/runtime_telemetry.json" \
    FORMAL_DYNAMIC_OPERATION_SPEED_PROFILE=dry_cleaning_competition_candidate \
    FORMAL_DYNAMIC_SAFETY_MAX_LINEAR_VELOCITY="${speed}" \
    "${repo_root}/scripts/run_formal_dynamic_obstacle_avoidance.sh"
  env "${common[@]}" FORMAL_DIRT_OUTPUT_DIR="${stage_root}/ground_dirt" \
    FORMAL_DIRT_DRIVE_SPEED_MPS="${speed}" FORMAL_DIRT_SAFETY_MAX_LINEAR_VELOCITY="${speed}" \
    "${repo_root}/scripts/run_formal_ground_dirt_cleaning_runtime.sh"

  runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime}"
  session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
  snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
  closure="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
  binding="${stage_root}/aggregation.runtime_binding.json"
  python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
    --repository-root "${repo_root}" --install-root "${runtime_ws}/install" \
    --closure-manifest "${closure}" --session "${session}" --snapshot "${snapshot}" --output "${binding}"
  aggregate=(python3 "${repo_root}/scripts/validate_formal_dry_speed_requalification.py" \
    --mobility "${stage_root}/mobility.json" --interlock "${stage_root}/interlock.json" \
    --dynamic "${stage_root}/dynamic.json" --ground-dirt "${stage_root}/ground_dirt/ground_dirt_acceptance.json" \
    --current-runtime-binding "${binding}" --token "${marker}" --stage "${stage}" \
    --output "${stage_root}/dry_speed_requalification.json")
  if [[ -n "${predecessor}" ]]; then
    aggregate+=(--predecessor "${predecessor}")
  fi
  "${aggregate[@]}"
  predecessor="${stage_root}/dry_speed_requalification.json"
done
