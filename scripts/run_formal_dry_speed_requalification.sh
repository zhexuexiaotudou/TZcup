#!/usr/bin/env bash
# Isolated, serial 1.0 m/s dry-cleaning safety requalification. Never changes
# the checked-in product 0.45 m/s envelope.
set -euo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
[[ "${FORMAL_DRY_SPEED_REQUALIFICATION:-}" == "1" ]] || {
  echo "Refusing 1.0 m/s test lane: set FORMAL_DRY_SPEED_REQUALIFICATION=1" >&2; exit 2;
}
run_root="${FORMAL_DRY_SPEED_REQUALIFICATION_ROOT:-${repo_root}/artifacts/formal_dry_speed_requalification}"
[[ ! -e "${run_root}" ]] || { echo "Refusing stale requalification evidence: ${run_root}" >&2; exit 2; }
mkdir -p "${run_root}"
profile="${repo_root}/config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml"
marker="${run_root}/requalification.opt_in.marker.json"
python3 "${repo_root}/scripts/formal_dry_speed_requalification_token.py" --create \
  --profile "${profile}" --run-root "${run_root}" --token "${marker}"

# One Gazebo at a time. Existing runners retain their source/session/runtime
# binding and process-group cleanup; this wrapper only supplies the isolated cap.
FORMAL_DRY_SPEED_REQUALIFICATION=1 FORMAL_DRY_SPEED_REQUALIFICATION_ROOT="${run_root}" FORMAL_DRY_SPEED_REQUALIFICATION_MARKER="${marker}" \
FORMAL_VEHICLE_MOBILITY_OUTPUT="${run_root}/mobility.json" \
FORMAL_VEHICLE_MOBILITY_FORWARD_SPEED_MPS=1.0 \
FORMAL_VEHICLE_MOBILITY_FORWARD_DURATION_S=1.0 \
FORMAL_VEHICLE_MOBILITY_SAFETY_MAX_LINEAR_VELOCITY=1.0 \
FORMAL_VEHICLE_MOBILITY_EXERCISE_ESTOP=1 \
"${repo_root}/scripts/run_formal_vehicle_mobility_runtime.sh"
FORMAL_DRY_SPEED_REQUALIFICATION=1 FORMAL_DRY_SPEED_REQUALIFICATION_ROOT="${run_root}" FORMAL_DRY_SPEED_REQUALIFICATION_MARKER="${marker}" \
WHOLE_VEHICLE_INTERLOCK_SAFETY_MAX_LINEAR_VELOCITY=1.0 \
WHOLE_VEHICLE_INTERLOCK_BASE_LINEAR_SPEED=1.0 \
"${repo_root}/scripts/run_whole_vehicle_actuator_interlock_runtime.sh" "${run_root}/interlock.json"
FORMAL_DRY_SPEED_REQUALIFICATION=1 FORMAL_DRY_SPEED_REQUALIFICATION_ROOT="${run_root}" FORMAL_DRY_SPEED_REQUALIFICATION_MARKER="${marker}" \
FORMAL_DYNAMIC_OUTPUT="${run_root}/dynamic.json" \
FORMAL_DYNAMIC_TELEMETRY="${run_root}/dynamic.telemetry.json" \
FORMAL_DYNAMIC_OPERATION_SPEED_PROFILE=dry_cleaning_competition_candidate \
FORMAL_DYNAMIC_SAFETY_MAX_LINEAR_VELOCITY=1.0 \
"${repo_root}/scripts/run_formal_dynamic_obstacle_avoidance.sh"
FORMAL_DRY_SPEED_REQUALIFICATION=1 FORMAL_DRY_SPEED_REQUALIFICATION_ROOT="${run_root}" FORMAL_DRY_SPEED_REQUALIFICATION_MARKER="${marker}" \
FORMAL_DIRT_OUTPUT_DIR="${run_root}/ground_dirt" \
FORMAL_DIRT_DRIVE_SPEED_MPS=1.0 \
FORMAL_DIRT_SAFETY_MAX_LINEAR_VELOCITY=1.0 \
"${repo_root}/scripts/run_formal_ground_dirt_cleaning_runtime.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
closure="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
current_binding="${run_root}/aggregation.runtime_binding.json"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${runtime_ws}/install" \
  --closure-manifest "${closure}" --session "${session}" --snapshot "${snapshot}" \
  --output "${current_binding}"
python3 "${repo_root}/scripts/validate_formal_dry_speed_requalification.py" \
  --mobility "${run_root}/mobility.json" --interlock "${run_root}/interlock.json" \
  --dynamic "${run_root}/dynamic.json" --ground-dirt "${run_root}/ground_dirt/ground_dirt_acceptance.json" \
  --current-runtime-binding "${current_binding}" --token "${marker}" \
  --output "${run_root}/dry_speed_requalification.json"
