#!/usr/bin/env bash
set -euo pipefail

bag="${1:?bag path required}"
out="${2:?output directory required}"
offset="${3:-55}"
mkdir -p "${out}"

set +u
source /opt/ros/jazzy/setup.bash
source "${SANITATION_BASE_WS:?SANITATION_BASE_WS required}/install/setup.bash"
set -u

timeout 25 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once \
  > "${out}/replay_scan_snapshot.yaml" & scan_pid=$!
timeout 25 ros2 topic echo /ground_truth/odom nav_msgs/msg/Odometry --once \
  > "${out}/replay_gt_snapshot.yaml" & odom_pid=$!
sleep 2
ros2 bag play "${bag}" --start-offset "${offset}" --rate 1.0 --delay 5 \
  --topics /scan /ground_truth/odom > "${out}/replay_snapshot.log" 2>&1
wait "${scan_pid}"
wait "${odom_pid}"
