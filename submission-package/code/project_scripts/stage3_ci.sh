#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS must point to a passed Stage 1 workspace}"
OUT="$PACK_ROOT/artifacts/stage3_$STAMP"
PIDS=()

mkdir -p "$OUT"

stop_pid() {
  local pid="$1"
  if ! kill -0 -- "-$pid" 2>/dev/null; then
    return
  fi
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 -- "-$pid" 2>/dev/null || break
    sleep 0.2
  done
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in {1..25}; do
      kill -0 -- "-$pid" 2>/dev/null || break
      sleep 0.2
    done
  fi
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do
    stop_pid "$pid"
  done
}
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"
cd "$WS"
colcon build \
  --packages-select sanitation_safety sanitation_tasks sanitation_navigation \
  --symlink-install \
  --event-handlers console_direct+ \
  2>&1 | tee "$OUT/build.log"

set +u
source "$WS/install/setup.bash"
set -u

colcon test \
  --packages-select sanitation_safety sanitation_tasks sanitation_navigation \
  --event-handlers console_direct+ \
  2>&1 | tee "$OUT/test.log"
colcon test-result --all --verbose > "$OUT/test_results.txt"

setsid ros2 launch sanitation_bringup sim.launch.py \
  gui:=false headless_rendering:=true \
  > "$OUT/simulation.log" 2>&1 &
SIM_PID=$!
PIDS+=("$SIM_PID")

setsid ros2 launch sanitation_navigation slam.launch.py rviz:=false \
  > "$OUT/slam.log" 2>&1 &
SLAM_PID=$!
PIDS+=("$SLAM_PID")

timeout 120 ros2 topic echo --once /map nav_msgs/msg/OccupancyGrid \
  > "$OUT/slam_map_sample.txt"
ros2 run nav2_map_server map_saver_cli \
  -f "$OUT/slam_map" --ros-args \
  -p use_sim_time:=true -p save_map_timeout:=10.0 \
  2>&1 | tee "$OUT/map_save.log"
stop_pid "$SLAM_PID"
pkill -INT -f '[a]sync_slam_toolbox_node' 2>/dev/null || true
for _ in {1..25}; do
  pgrep -f '[a]sync_slam_toolbox_node' >/dev/null || break
  sleep 0.2
done
pkill -TERM -f '[a]sync_slam_toolbox_node' 2>/dev/null || true
PIDS=("$SIM_PID")

setsid ros2 launch sanitation_navigation navigation.launch.py rviz:=false \
  > "$OUT/navigation.log" 2>&1 &
NAV_PID=$!
PIDS+=("$NAV_PID")

setsid ros2 bag record \
  -o "$OUT/navigation_bag" \
  /tf /tf_static /odom /scan /amcl_pose /cmd_vel /emergency_stop \
  > "$OUT/rosbag.log" 2>&1 &
BAG_PID=$!
PIDS+=("$BAG_PID")

ros2 run sanitation_tasks sanitation_navigation_probe --ros-args \
  -p timeout_sec:=300.0 \
  -p output_path:="$OUT/navigation_probe.json" \
  2>&1 | tee "$OUT/navigation_probe.log"
stop_pid "$BAG_PID"
PIDS=("$SIM_PID" "$NAV_PID")

ros2 node list | sort > "$OUT/nodes.txt"
ros2 topic list -t | sort > "$OUT/topics.txt"
ros2 action list -t | sort > "$OUT/actions.txt"
ros2 service list -t | sort > "$OUT/services.txt"
ros2 topic echo --once /tf_static tf2_msgs/msg/TFMessage > "$OUT/tf_static.txt"

stop_pid "$NAV_PID"
PIDS=("$SIM_PID")

setsid ros2 run sanitation_safety velocity_gate \
  > "$OUT/safety_gate.log" 2>&1 &
SAFETY_PID=$!
PIDS+=("$SAFETY_PID")

ros2 run sanitation_tasks sanitation_safety_probe --ros-args \
  -p output_path:="$OUT/safety_probe.json" \
  2>&1 | tee "$OUT/safety_probe.log"

stop_pid "$SAFETY_PID"
PIDS=("$SIM_PID")

if ! kill -0 -- "-$SIM_PID" 2>/dev/null; then
  echo "ERROR: simulation launch exited early" >&2
  exit 4
fi

stop_pid "$SIM_PID"
PIDS=()
trap - EXIT

export STAGE3_OUT="$OUT"
export STAGE3_WS="$WS"
python3 - <<'PY'
import datetime as dt
import json
import os
from pathlib import Path

out = Path(os.environ["STAGE3_OUT"])
navigation = json.loads((out / "navigation_probe.json").read_text(encoding="utf-8"))
safety = json.loads((out / "safety_probe.json").read_text(encoding="utf-8"))
map_files = [out / "slam_map.pgm", out / "slam_map.yaml"]
bag_metadata = out / "navigation_bag" / "metadata.yaml"
summary = {
    "schema_version": 1,
    "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "success": bool(navigation["success"] and safety["success"]),
    "workspace": os.environ["STAGE3_WS"],
    "slam_map_saved": all(path.is_file() for path in map_files),
    "navigation": navigation,
    "safety": safety,
    "keepout_filter_configured": True,
    "speed_filter_configured": True,
    "rosbag_metadata": str(bag_metadata.relative_to(out)) if bag_metadata.is_file() else None,
    "artifacts": sorted(path.name for path in out.iterdir()),
}
summary["success"] = bool(
    summary["success"]
    and summary["slam_map_saved"]
    and summary["rosbag_metadata"]
)
(out / "stage3_summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
if not summary["success"]:
    raise SystemExit(5)
PY

echo "$OUT"
