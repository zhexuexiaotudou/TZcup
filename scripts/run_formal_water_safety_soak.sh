#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS}"
output_dir="${FORMAL_WATER_SOAK_OUTPUT_DIR:?set FORMAL_WATER_SOAK_OUTPUT_DIR}"
stable_duration_s="${FORMAL_WATER_SOAK_DURATION_S:-65}"

[[ ! -e "${output_dir}" ]] || {
  echo "Refusing stale water safety soak output: ${output_dir}" >&2
  exit 2
}
[[ -f "${runtime_ws}/install/setup.bash" ]] || {
  echo "Missing built ROS workspace setup: ${runtime_ws}" >&2
  exit 2
}
source "${runtime_ws}/install/setup.bash"
set -u

vehicle_xacro="$(ros2 pkg prefix --share sanitation_vehicle_description)/urdf/formal_competition_vehicle.urdf.xacro"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
mkdir -p "${output_dir}"
export GZ_PARTITION="tzcup_formal_water_soak_${ROS_DOMAIN_ID}_$$"
launch_pid=""

cleanup_launch() {
  local status=0
  if [[ -n "${launch_pid}" ]]; then
    formal_runtime_kill_group "${launch_pid}" || status=1
    launch_pid=""
  fi
  formal_runtime_stop_memory_watchdog || status=1
  formal_runtime_cleanup_partition "${GZ_PARTITION}" || status=1
  return "${status}"
}
formal_runtime_install_traps cleanup_launch

python3 "${repo_root}/scripts/validate_formal_side_brush_sdf_surface.py" \
  --vehicle-xacro "${vehicle_xacro}" \
  --output "${output_dir}/side_brush_sdf_surface.json" \
  >"${output_dir}/side_brush_sdf_surface.log" 2>&1

formal_runtime_memory_preflight "${output_dir}/windows_memory_preflight"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true high_bandwidth_sensor_runtime:=false \
  start_controllers:=true \
  enable_safety_manager:=true simulation_initial_estop_active:=true \
  start_simulation_safety_inputs:=true start_power_system_simulators:=true \
  water_evaluation_interfaces:=true \
  >"${output_dir}/soak_launch.log" 2>&1 &
launch_pid=$!
formal_runtime_start_memory_watchdog "${launch_pid}" \
  "${output_dir}/memory_watchdog"

python3 "${repo_root}/scripts/check_formal_water_preoperational_readiness.py" \
  --output "${output_dir}/preoperational_readiness.json" \
  >"${output_dir}/preoperational_readiness.log" 2>&1

python3 "${repo_root}/scripts/collect_formal_water_safety_preflight.py" \
  --stable-duration-s "${stable_duration_s}" \
  --timeout-s 240 \
  --inject-estop-edge \
  --output "${output_dir}/safety_soak.json" \
  >"${output_dir}/safety_soak.log" 2>&1

cleanup_launch
python3 "${repo_root}/scripts/audit_formal_water_launch_log.py" \
  --log "${output_dir}/soak_launch.log" \
  --output "${output_dir}/soak_launch_audit.json"
sha256sum "${output_dir}/safety_soak.json" \
  "${output_dir}/safety_soak.log" \
  "${output_dir}/side_brush_sdf_surface.json" \
  "${output_dir}/side_brush_sdf_surface.log" \
  "${output_dir}/soak_launch.log" \
  "${output_dir}/soak_launch_audit.json" \
  "${output_dir}/preoperational_readiness.json" \
  "${output_dir}/preoperational_readiness.log" \
  "${output_dir}/memory_watchdog.json" \
  "${output_dir}/memory_watchdog.log" \
  "${output_dir}/windows_memory_preflight.json" \
  "${output_dir}/windows_memory_preflight.log" \
  >"${output_dir}/SHA256SUMS"
