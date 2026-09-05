#!/usr/bin/env bash
# Low-memory diagnostic: keep dynamic sensors enabled, disable their ROS
# high-bandwidth bridges, and request one native Gazebo message per plane.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"

runtime_setup="${FORMAL_DIAGNOSTIC_RUNTIME_SETUP:-}"
output_root="${FORMAL_DIAGNOSTIC_OUTPUT_DIR:-}"
domain_id="${ROS_DOMAIN_ID:-97}"
probe_timeout_s="${FORMAL_DIAGNOSTIC_PROBE_TIMEOUT_S:-20}"

[[ -f "${runtime_setup}" ]] || { echo "missing frozen runtime setup" >&2; exit 2; }
[[ "${output_root}" = /* && "${output_root}" != / && ! -e "${output_root}" ]] || {
  echo "FORMAL_DIAGNOSTIC_OUTPUT_DIR must be a new absolute directory" >&2
  exit 2
}
[[ "${probe_timeout_s}" =~ ^[1-9][0-9]*$ ]] && (( probe_timeout_s <= 30 )) || {
  echo "FORMAL_DIAGNOSTIC_PROBE_TIMEOUT_S must be in 1..30" >&2
  exit 2
}
mkdir -p "${output_root}"
set +u
source "${runtime_setup}"
set -u
package_share="$(ros2 pkg prefix --share sanitation_vehicle_description)"
install_root="$(cd "$(dirname "${runtime_setup}")" && pwd -P)"
expected_package_share="${install_root}/share/sanitation_vehicle_description"
[[ "$(cd "${package_share}" && pwd -P)" == "$(cd "${expected_package_share}" && pwd -P)" ]] || {
  echo "vehicle package resolves outside the requested frozen runtime" >&2
  exit 2
}
for relative in \
  launch/formal_vehicle_sim.launch.py \
  urdf/formal_competition_vehicle.urdf.xacro \
  urdf/high_fidelity/manipulator_stack.xacro \
  urdf/high_fidelity/sensor_suite.xacro \
  worlds/formal_vehicle_validation.sdf; do
  installed="${package_share}/${relative}"
  source_path="${repo_root}/starter_ws/src/sanitation_vehicle_description/${relative}"
  [[ -f "${installed}" && -f "${source_path}" && "$(sha256sum "${installed}" | cut -d' ' -f1)" == "$(sha256sum "${source_path}" | cut -d' ' -f1)" ]] || {
    echo "frozen diagnostic runtime is stale for ${relative}" >&2
    exit 2
  }
done

export ROS_DOMAIN_ID="${domain_id}"
formal_runtime_configure "${ROS_DOMAIN_ID}" 1
export GZ_PARTITION="${GZ_PARTITION:-tzcup_gz_native_sensor_smoke_${ROS_DOMAIN_ID}_$$}"
launch_pid=""
cleanup() { formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"; }
formal_runtime_install_traps cleanup
formal_runtime_register_evidence_paths "${output_root}"
formal_runtime_memory_preflight "${output_root}/memory_preflight"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch \
  sanitation_vehicle_description formal_vehicle_sim.launch.py \
  world:="${package_share}/worlds/formal_vehicle_validation.sdf" \
  model:="${package_share}/urdf/formal_competition_vehicle.urdf.xacro" \
  spawn_robot:=true gui:=false headless_rendering:=true bodywork_visible:=true \
  high_bandwidth_sensor_runtime:=true start_high_bandwidth_sensor_bridges:=false \
  visual_acceptance_runtime:=false start_controllers:=false \
  enable_safety_manager:=false start_simulation_safety_inputs:=false \
  start_power_system_simulators:=false start_localization:=false \
  cleaning_realtime_telemetry_enabled:=false cleaning_status_json_enabled:=false \
  >"${output_root}/launch.log" 2>&1 &
launch_pid=$!
formal_runtime_start_memory_watchdog "${launch_pid}" "${output_root}/memory_watchdog"

sleep 15
launch_alive=1
kill -0 "${launch_pid}" 2>/dev/null || launch_alive=0

probe() {
  local name="$1"
  local topic="$2"
  local sample="${output_root}/${name}.sample"
  gz topic -i -t "${topic}" >"${output_root}/${name}.info" 2>&1 || true
  set +e
  timeout -k 3 "${probe_timeout_s}" gz topic -e -t "${topic}" -n 1 \
    >"${sample}" 2>"${output_root}/${name}.stderr"
  local status=$?
  set -e
  printf '%s\n' "${status}" >"${output_root}/${name}.status"
  wc -c <"${sample}" | tr -d ' ' >"${output_root}/${name}.bytes"
}

probe imu /sensors/imu/data
probe utm30_gpu_lidar /sensors/lidar_2d/scan
probe front_rgbd_depth /sensors/front_rgbd/depth/image_rect_raw/image
kill -0 "${launch_pid}" 2>/dev/null || launch_alive=0

formal_runtime_stop_memory_watchdog
watchdog_result="${FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT}"
set +e
python3 "${repo_root}/scripts/finalize_formal_gz_native_sensor_smoke.py" \
  --output-root "${output_root}" --launch-alive "${launch_alive}" \
  --memory-watchdog-result "${watchdog_result}"
summary_status=$?
set -e
if (( watchdog_result != 0 )); then exit "${watchdog_result}"; fi
exit "${summary_status}"
