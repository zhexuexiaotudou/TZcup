#!/usr/bin/env bash
# Wait for the physical UTM-30LX's first Gazebo frame before creating the sole
# ROS raw-scan bridge. Pinned Gazebo Harmonic transport can advertise this
# sensor well before it emits; delay the independent native bridge until a
# physical scan arrives. This wrapper never writes /scan/navigation.
set -euo pipefail

readonly LIDAR_GZ_TOPIC='/sensors/lidar_2d/scan'

timeout_sec=600
if [[ "${1:-}" == '--timeout-sec' ]]; then
  [[ $# -ge 2 ]] || { echo 'formal lidar bridge: --timeout-sec requires a value' >&2; exit 64; }
  timeout_sec="$2"
  shift 2
fi

if [[ ! "$timeout_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "formal lidar bridge: timeout must be a positive integer, got '$timeout_sec'" >&2
  exit 64
fi

echo "formal lidar bridge: waiting up to ${timeout_sec}s for ${LIDAR_GZ_TOPIC}" >&2
set +e
timeout --foreground --signal=INT "${timeout_sec}s" \
  gz topic -e -t "$LIDAR_GZ_TOPIC" -n 1 >/dev/null
probe_status=$?
set -e
if [[ $probe_status -ne 0 ]]; then
  echo "formal lidar bridge: first Gazebo lidar frame was not observed (status=${probe_status})" >&2
  exit 70
fi

echo 'formal lidar bridge: first Gazebo lidar frame observed; starting sole native /scan bridge' >&2
exec ros2 run sanitation_gazebo_control formal_lidar_native_bridge
