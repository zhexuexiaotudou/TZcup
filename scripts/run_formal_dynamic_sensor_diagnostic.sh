#!/usr/bin/env bash
# Diagnostic-only dynamic-spawn sensor transport probe. It is intentionally
# separate from the formal session-bound preembedded sensor acceptance.
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"

runtime_setup="${FORMAL_DIAGNOSTIC_RUNTIME_SETUP:-/home/zhexu/tzcup_final_runtime_r53_attempt_001_ws/install/setup.bash}"
output_dir="${FORMAL_DIAGNOSTIC_OUTPUT_DIR:-}"
timeout_s="${FORMAL_DIAGNOSTIC_TIMEOUT_S:-90}"
domain_id="${ROS_DOMAIN_ID:-96}"

[[ -n "${output_dir}" && "${output_dir}" = /* ]] || {
  echo "FORMAL_DIAGNOSTIC_OUTPUT_DIR must be a new absolute directory" >&2
  exit 2
}
[[ ! -e "${output_dir}" ]] || {
  echo "refusing existing diagnostic output directory: ${output_dir}" >&2
  exit 2
}
[[ "${timeout_s}" =~ ^[1-9][0-9]*$ ]] && (( timeout_s <= 180 )) || {
  echo "FORMAL_DIAGNOSTIC_TIMEOUT_S must be an integer in 1..180" >&2
  exit 2
}
[[ -f "${runtime_setup}" ]] || {
  echo "missing frozen diagnostic runtime: ${runtime_setup}" >&2
  exit 2
}

mkdir -p "${output_dir}"
install_root="$(cd "$(dirname "${runtime_setup}")" && pwd -P)"
source "${runtime_setup}"
package_share="$(ros2 pkg prefix --share sanitation_vehicle_description)"
expected_share="${install_root}/share/sanitation_vehicle_description"
[[ "$(cd "${package_share}" && pwd -P)" == "$(cd "${expected_share}" && pwd -P)" ]] || {
  echo "vehicle package resolves outside frozen runtime: ${package_share}" >&2
  exit 2
}

export ROS_DOMAIN_ID="${domain_id}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_dynamic_sensor_diag_${ROS_DOMAIN_ID}_$$}"
launch_log="${output_dir}/launch.log"
collector_log="${output_dir}/collector.log"
report="${output_dir}/dynamic_sensor_diagnostic.json"
launch_pid=""

cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup
formal_runtime_memory_preflight "${output_dir}/memory_preflight"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch \
  sanitation_vehicle_description formal_vehicle_sim.launch.py \
  world:="${package_share}/worlds/formal_vehicle_validation.sdf" \
  model:="${package_share}/urdf/formal_competition_vehicle.urdf.xacro" \
  spawn_robot:=true gui:=false headless_rendering:=true bodywork_visible:=true \
  start_controllers:=true enable_safety_manager:=true \
  simulation_initial_estop_active:=true high_bandwidth_sensor_runtime:=true \
  >"${launch_log}" 2>&1 &
launch_pid=$!
formal_runtime_start_memory_watchdog "${launch_pid}" "${output_dir}/memory_watchdog"

set +e
python3 "${repo_root}/scripts/collect_formal_dynamic_sensor_diagnostic.py" \
  --output "${report}" --timeout "${timeout_s}" \
  >"${collector_log}" 2>&1
collector_status=$?
set -e

write_markers="$(grep -c 'GazeboSimSystem::write' "${launch_log}" || true)"
segfault_markers="$(grep -Eic 'Segmentation fault|exit code 139' "${launch_log}" || true)"
switch_markers="$(grep -c 'Successfully switched controllers' "${launch_log}" || true)"
printf 'collector_status=%s\n' "${collector_status}"
printf 'switch_success_markers=%s\n' "${switch_markers}"
printf 'write_stack_markers=%s\n' "${write_markers}"
printf 'segfault_markers=%s\n' "${segfault_markers}"

if (( write_markers > 0 && segfault_markers > 0 )); then
  echo "DYNAMIC_SENSOR_DIAGNOSTIC_CONTROL_CRASHED"
  exit 139
fi
exit "${collector_status}"
