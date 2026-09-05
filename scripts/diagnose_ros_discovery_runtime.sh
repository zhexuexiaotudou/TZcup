#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}"
export ROS2CLI_DISABLE_DAEMON=1
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

log_path="${ROS_DISCOVERY_DIAGNOSTIC_LOG:-/tmp/tzcup_ros_discovery_talker.log}"
ros2 run demo_nodes_cpp talker >"${log_path}" 2>&1 &
talker_pid=$!
cleanup() {
  kill -INT "${talker_pid}" 2>/dev/null || true
  wait "${talker_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3
timeout 10 ros2 topic echo /chatter --once
