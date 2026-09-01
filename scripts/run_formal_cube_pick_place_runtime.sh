#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${FORMAL_MANIPULATION_RUNTIME_WS:-${repo_root}/.manipulation_build}"
output="${FORMAL_MANIPULATION_OUTPUT:-${repo_root}/artifacts/formal_cube_pick_place_runtime.json}"
launch_log="${FORMAL_MANIPULATION_LOG:-${output%.json}.launch.log}"
material="${FORMAL_MANIPULATION_MATERIAL:-PET}"
timeout_s="${FORMAL_MANIPULATION_TIMEOUT_S:-180}"

# Do not let a missing ROS setup or runtime overlay leave an older canonical
# PASS artifact at the path consumed by later acceptance aggregation.
if [[ -e "${output}" ]]; then
  mv -- "${output}" "${output}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"
fi

source /opt/ros/jazzy/setup.bash
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
formal_runtime_register_evidence_paths "${output}"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing built ROS workspace: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
source "${runtime_ws}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_manipulation_${ROS_DOMAIN_ID}_$$}"
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_manipulation formal_cube_pick_place.launch.py \
  gui:=false material:="${material}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

# The dry-bin monitor publishes Gazebo transport truth only.  Keep its ROS
# evaluator surface observation-only and scoped to this formal acceptance run.
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json@std_msgs/msg/String[gz.msgs.StringMsg" \
  >>"${launch_log}" 2>&1 &
dry_bin_bridge_pid=$!

cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" \
    "${dry_bin_bridge_pid}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

python3 "${repo_root}/scripts/validate_formal_cube_pick_place_runtime.py" \
  --output "${output}" --material "${material}" --timeout "${timeout_s}"
