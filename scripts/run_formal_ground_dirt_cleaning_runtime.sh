#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_ws="${FORMAL_DIRT_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime}"
output_dir="${FORMAL_DIRT_OUTPUT_DIR:-${repo_root}/artifacts/formal_ground_dirt_cleaning_final_retry}"
output="${output_dir}/ground_dirt_acceptance.json"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
runtime_binding="${output}.runtime_binding.json"
drive_speed_mps="${FORMAL_DIRT_DRIVE_SPEED_MPS:-0.06}"
safety_max_linear_velocity="${FORMAL_DIRT_SAFETY_MAX_LINEAR_VELOCITY:-0.45}"
if [[ "${safety_max_linear_velocity}" != "0.45" ]]; then
  [[ "${FORMAL_DRY_SPEED_REQUALIFICATION:-}" == "1" && -n "${FORMAL_DRY_SPEED_REQUALIFICATION_MARKER:-}" && -n "${FORMAL_DRY_SPEED_REQUALIFICATION_ROOT:-}" ]] || {
    echo "non-default safety cap requires the requalification wrapper opt-in marker" >&2; exit 2;
  }
  python3 "${repo_root}/scripts/formal_dry_speed_requalification_token.py" --validate \
    --profile "${repo_root}/config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml" \
    --run-root "${FORMAL_DRY_SPEED_REQUALIFICATION_ROOT}" --token "${FORMAL_DRY_SPEED_REQUALIFICATION_MARKER}" \
    --requested-cap "${safety_max_linear_velocity}"
fi
formal_runtime_register_evidence_paths "${output_dir}" "${output}" "${runtime_binding}"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing built ROS workspace: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
for required in "${session}" "${snapshot}" "${closure_manifest}"; do
  if [[ ! -f "${required}" ]]; then
    echo "Missing frozen ground-dirt runtime input: ${required}" >&2
    exit 2
  fi
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-95}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_ground_dirt_${ROS_DOMAIN_ID}_$$}"
if [[ -e "${output_dir}" || -e "${output}" || -e "${runtime_binding}" ]]; then
  echo "Refusing stale ground-dirt evidence directory: ${output_dir}" >&2
  exit 2
fi
mkdir -p "$(dirname "${output_dir}")"
mkdir "${output_dir}"

python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${runtime_ws}/install" \
  --closure-manifest "${closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${runtime_ws}/install/setup.bash"
set -u

python3 "${repo_root}/scripts/prepare_formal_ground_dirt_runtime.py" \
  --output-dir "${output_dir}/episode" \
  > "${output_dir}/prepare.log"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true start_controllers:=true \
  enable_safety_manager:=true start_simulation_safety_inputs:=true \
  start_power_system_simulators:=true simulation_initial_estop_active:=false \
  high_bandwidth_sensor_runtime:=false start_localization:=false \
  world:="${output_dir}/episode/public/world.sdf" \
  > "${output_dir}/launch.log" 2>&1 &
launch_pid=$!

# Ground-dirt truth and initialization pose are evaluator-only.  The normal
# product launch exposes neither the cell map nor SetEntityPose to ROS.
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  "/world/campus_formal/set_pose@ros_gz_interfaces/srv/SetEntityPose" \
  "/model/tzcup_formal_sanitation_vehicle/ground_dirt/command/enable@std_msgs/msg/Bool]gz.msgs.Boolean" \
  "/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json@std_msgs/msg/String[gz.msgs.StringMsg" \
  >> "${output_dir}/launch.log" 2>&1 &
bridge_pid=$!

cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${bridge_pid}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

if [[ "${safety_max_linear_velocity}" != "0.45" ]]; then
  for _ in $(seq 1 90); do
    timeout 10s ros2 param set /whole_vehicle_safety_manager max_linear_velocity "${safety_max_linear_velocity}" && break
    sleep 1
  done
  timeout 10s ros2 param get /whole_vehicle_safety_manager max_linear_velocity | grep -Fq "${safety_max_linear_velocity}" || {
    echo "requalification safety-cap override was not applied" >&2; exit 3;
  }
fi

python3 "${repo_root}/scripts/validate_formal_ground_dirt_cleaning_runtime.py" \
  --setup "${output_dir}/episode/evaluator/runtime_setup.json" \
  --output "${output}" --snapshot "${snapshot}" --session "${session}" \
  --runtime-binding "${runtime_binding}" \
  --drive-speed "${drive_speed_mps}" --safety-max-linear-velocity "${safety_max_linear_velocity}" \
  > "${output_dir}/probe.log" 2>&1
