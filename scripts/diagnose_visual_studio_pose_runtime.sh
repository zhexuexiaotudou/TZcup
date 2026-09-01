#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_setup="${FORMAL_VEHICLE_VISUAL_RUNTIME_SETUP:-${repo_root}/.work/final_functional_build/install/setup.bash}"
source "${runtime_setup}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-57}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_visual_pose_diag_${ROS_DOMAIN_ID}_$$}"
log_path="${repo_root}/.work/visual_studio_pose_diag.log"

setsid ros2 launch sanitation_vehicle_description formal_vehicle_visual_acceptance.launch.py \
  bodywork_visible:=true >"${log_path}" 2>&1 &
launch_pid=$!

cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

sleep "${FORMAL_VISUAL_POSE_SETTLE_SECONDS:-45}"
pose_path="${repo_root}/.work/visual_studio_dynamic_pose.txt"
odom_topic="/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom"
status_topic="/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status"
timeout 20 gz topic -e -t "${odom_topic}" -n 1 >"${pose_path}"
timeout 20 gz topic -e -t "${status_topic}" -n 1 \
  >"${repo_root}/.work/visual_studio_drivetrain_status.txt"
grep -F "position" "${pose_path}" >/dev/null || {
  echo "Gazebo did not return the sanitation vehicle raw drivetrain odometry" >&2
  exit 3
}
echo "${pose_path}"
