#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${FORMAL_MANIPULATION_RUNTIME_WS:-${repo_root}/.manipulation_build}"
output="${FORMAL_MANIPULATION_OUTPUT:-${repo_root}/artifacts/formal_cube_pick_place_runtime.json}"
launch_log="${FORMAL_MANIPULATION_LOG:-${output%.json}.launch.log}"
material="${FORMAL_MANIPULATION_MATERIAL:-PET}"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing built ROS workspace: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
source "${runtime_ws}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_manipulation_${ROS_DOMAIN_ID}}"
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

setsid ros2 launch sanitation_manipulation formal_cube_pick_place.launch.py \
  gui:=false material:="${material}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 1
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

python3 "${repo_root}/scripts/validate_formal_cube_pick_place_runtime.py" \
  --output "${output}" --material "${material}" --timeout 180
