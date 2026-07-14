#!/usr/bin/env bash
set -euo pipefail

WS="${SANITATION_WS:-$HOME/sanitation_ws}"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$WS/artifacts/snapshot_$STAMP"
mkdir -p "$OUT"

ros2 topic list -t > "$OUT/topics.txt"
ros2 node list > "$OUT/nodes.txt"
ros2 service list -t > "$OUT/services.txt"
ros2 action list -t > "$OUT/actions.txt"
ros2 param list > "$OUT/params.txt" || true
timeout 10s ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true \
  > "$OUT/tf_view_frames.log" 2>&1 || true
find . -maxdepth 1 -name 'frames*' -exec mv {} "$OUT/" \; 2>/dev/null || true

ros2 run sanitation_tasks sanitation_smoke_check --ros-args \
  -p timeout_sec:=30.0 \
  -p output_path:="$OUT/smoke_check.json"

echo "$OUT"
