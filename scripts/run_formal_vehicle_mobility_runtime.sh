#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-${repo_root}/.mobility_build}"
output="${FORMAL_VEHICLE_MOBILITY_OUTPUT:-${repo_root}/artifacts/formal_vehicle_mobility_runtime.json}"
launch_log="${FORMAL_VEHICLE_MOBILITY_LOG:-${output%.json}.launch.log}"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing built ROS workspace: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
source "${runtime_ws}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_mobility_${ROS_DOMAIN_ID}}"
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

setsid ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true start_controllers:=true \
  >"${launch_log}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 1
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

python3 "${repo_root}/scripts/validate_formal_vehicle_mobility_runtime.py" \
  --output "${output}" --timeout 150 --forward-speed 0.25 --forward-duration 4.0
