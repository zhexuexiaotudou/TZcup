#!/usr/bin/env bash
set -euo pipefail

bag="${1:?bag path required}"
out="${2:?output directory required}"
mkdir -p "${out}"

set +u
source /opt/ros/jazzy/setup.bash
source "${SANITATION_BASE_WS:?SANITATION_BASE_WS required}/install/setup.bash"
set -u

timeout 35 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --field angular.z \
  > "${out}/replay_cmd_vel_angular.txt" & requested_pid=$!
timeout 35 ros2 topic echo /cmd_vel_gate geometry_msgs/msg/Twist --field angular.z \
  > "${out}/replay_cmd_gate_angular.txt" & gated_pid=$!
sleep 2
ros2 bag play "${bag}" --rate 20 --delay 5 --topics /cmd_vel /cmd_vel_gate \
  > "${out}/replay_cmd_diag.log" 2>&1
wait "${requested_pid}" || true
wait "${gated_pid}" || true
