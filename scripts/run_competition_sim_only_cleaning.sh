#!/usr/bin/env bash
# Fresh component-evidence collection. It cannot claim a bundled mission.
set -euo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
run_root="${FORMAL_COMPETITION_SIM_ONLY_RUN_ROOT:?set a fresh FORMAL_COMPETITION_SIM_ONLY_RUN_ROOT}"
base_domain="${FORMAL_COMPETITION_SIM_ONLY_BASE_DOMAIN:-160}"
runtime_ws="${FORMAL_COMPETITION_SIM_ONLY_RUNTIME_WS:?set the fresh frozen runtime workspace}"
closure="${FORMAL_COMPETITION_SIM_ONLY_CLOSURE_MANIFEST:?set the frozen runtime closure manifest}"
public_split="${FORMAL_COMPETITION_SIM_ONLY_PUBLIC_SPLIT:-train}"
map_index="${FORMAL_COMPETITION_SIM_ONLY_MAP_INDEX:-0}"
mission_index="${FORMAL_COMPETITION_SIM_ONLY_MISSION_INDEX:-0}"

for forbidden in FORMAL_A12_SCENARIO FORMAL_A12_SEED FORMAL_CUBE_TARGET_COUNT FORMAL_SINGLE_EPISODE_CUBE_COUNT; do
  if [[ -n "${!forbidden:-}" ]]; then
    echo "competition_sim_only cleaning refuses ${forbidden}; use the separate A12 route" >&2
    exit 2
  fi
done
if [[ -e "${run_root}" ]]; then
  echo "refusing stale competition_sim_only run root: ${run_root}" >&2
  exit 2
fi
mkdir -p "${run_root}"

snapshot="${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json"
session="${run_root}/formal_acceptance_session.json"
runtime_install="${runtime_ws}/install"
episode="${run_root}/episode"
[[ "${public_split}" == train || "${public_split}" == val ]] || { echo "public split must be train or val" >&2; exit 2; }
for value in "${public_split}" "${map_index}" "${mission_index}"; do
  [[ "${value,,}" != *hidden* ]] || { echo "hidden input is forbidden" >&2; exit 2; }
done
python3 "${repo_root}/scripts/formal_acceptance_session.py" start \
  --repository-root "${repo_root}" --snapshot "${snapshot}" --output "${session}" \
  --runtime-closure-manifest "${closure}" --runtime-install-root "${runtime_install}"
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_install}/setup.bash"
set -u
ros2 run sanitation_campus_scenario sanitation-campus-scenario generate \
  --config "${repo_root}/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" \
  --profile formal --split "${public_split}" --map-index "${map_index}" \
  --mission-index "${mission_index}" --output "${episode}"
export FORMAL_DYNAMIC_EPISODE_ROOT="${episode}"
export FORMAL_ACCEPTANCE_SESSION="${session}"
export FORMAL_ACCEPTANCE_SESSION_STATUS="${session}"
export FORMAL_VEHICLE_RUNTIME_WS="${runtime_ws}"
export FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST="${closure}"
export FORMAL_VEHICLE_SNAPSHOT_MANIFEST="${snapshot}"

map_report="${run_root}/map_lifecycle_acceptance.json"
map_root="${run_root}/first_map"
ground_root="${run_root}/ground_dirt"
ground_report="${ground_root}/ground_dirt_acceptance.json"
dynamic_report="${run_root}/dynamic_obstacle_acceptance.json"

# A fresh first-map prerequisite is mandatory and stays within this run root;
# never silently reuse the stale default map artifact.
ROS_DOMAIN_ID="${base_domain}" \
FORMAL_DYNAMIC_SAVED_MAP_ROOT="${map_root}" \
bash "${repo_root}/scripts/run_formal_first_map_dynamic_prerequisite.sh"

ROS_DOMAIN_ID="$((base_domain + 1))" \
FORMAL_DYNAMIC_SAVED_MAP_ROOT="${map_root}" \
FORMAL_MAP_LIFECYCLE_OUTPUT="${map_report}" \
FORMAL_MAP_CLEANING_RUNTIME_ROOT="${run_root}/saved_map_runtime" \
FORMAL_CLEANING_PLANNER="full_coverage" \
bash "${repo_root}/scripts/run_formal_saved_map_cleaning_lifecycle.sh"

ROS_DOMAIN_ID="$((base_domain + 2))" \
FORMAL_DIRT_OUTPUT_DIR="${ground_root}" \
bash "${repo_root}/scripts/run_formal_ground_dirt_cleaning_runtime.sh"

ROS_DOMAIN_ID="$((base_domain + 3))" \
FORMAL_DYNAMIC_SAVED_MAP_ROOT="${map_root}" \
FORMAL_DYNAMIC_OUTPUT="${dynamic_report}" \
FORMAL_DYNAMIC_TELEMETRY="${run_root}/dynamic_runtime/runtime_telemetry.json" \
bash "${repo_root}/scripts/run_formal_dynamic_obstacle_avoidance.sh"

contract_args=(
  --map-lifecycle-report "${map_report}"
  --ground-dirt-report "${ground_report}"
  --dynamic-obstacle-report "${dynamic_report}"
  --output "${run_root}/competition_sim_only_cleaning_contract.json"
)
if [[ -n "${FORMAL_COMPETITION_SIM_ONLY_PERCEPTION_REPORT:-}" ]]; then
  contract_args+=(--perception-report "${FORMAL_COMPETITION_SIM_ONLY_PERCEPTION_REPORT}")
fi
if [[ -n "${FORMAL_COMPETITION_SIM_ONLY_EMERGENCY_BRAKING_REPORT:-}" ]]; then
  contract_args+=(--emergency-braking-report "${FORMAL_COMPETITION_SIM_ONLY_EMERGENCY_BRAKING_REPORT}")
fi
if [[ -n "${FORMAL_COMPETITION_SIM_ONLY_POST_CLEAN_REPORT:-}" ]]; then
  contract_args+=(--post-clean-report "${FORMAL_COMPETITION_SIM_ONLY_POST_CLEAN_REPORT}")
fi
python3 "${repo_root}/scripts/validate_competition_sim_only_cleaning.py" "${contract_args[@]}"
