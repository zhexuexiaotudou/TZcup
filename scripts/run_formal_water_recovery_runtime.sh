#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-${repo_root}/.water_recovery_build}"
output_dir="${FORMAL_WATER_OUTPUT_DIR:-${repo_root}/artifacts/formal_water_recovery}"
scenario="all"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      scenario="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ "${scenario}" != "normal" && "${scenario}" != "full" && "${scenario}" != "all" ]]; then
  echo "--scenario must be normal, full, or all" >&2
  exit 2
fi
if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing built ROS workspace: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
source "${runtime_ws}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-176}"
mkdir -p "${output_dir}"

launch_pid=""
cleanup_launch() {
  if [[ -n "${launch_pid}" ]]; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    sleep 1
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    launch_pid=""
  fi
}
trap cleanup_launch EXIT INT TERM

run_scenario() {
  local selected="$1"
  export GZ_PARTITION="tzcup_formal_water_${selected}_${ROS_DOMAIN_ID}_$$"
  setsid ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
    gui:=false bodywork_visible:=true start_controllers:=true \
    water_evaluation_interfaces:=true \
    >"${output_dir}/water_${selected}_launch.log" 2>&1 &
  launch_pid=$!

  python3 "${repo_root}/scripts/validate_formal_water_recovery_runtime.py" \
    --scenario "${selected}" \
    --output "${output_dir}/water_${selected}.json" \
    >"${output_dir}/water_${selected}_probe.log" 2>&1
  cleanup_launch
}

if [[ "${scenario}" == "normal" || "${scenario}" == "all" ]]; then
  run_scenario normal
fi
if [[ "${scenario}" == "full" || "${scenario}" == "all" ]]; then
  run_scenario full
fi
if [[ "${scenario}" == "all" ]]; then
  python3 "${repo_root}/scripts/finalize_formal_water_recovery_acceptance.py" \
    --normal "${output_dir}/water_normal.json" \
    --full "${output_dir}/water_full.json" \
    --output "${output_dir}/water_recovery_acceptance.json"
fi
