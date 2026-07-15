#!/usr/bin/env bash
set -uo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS must point to the reusable Stage 1 workspace}"
OUT="$PACK_ROOT/artifacts/stage4s_gt_$STAMP"
LAUNCH_PID=""
BAG_PID=""

mkdir -p "$OUT"

cleanup() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT -- "-$BAG_PID" 2>/dev/null || kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in {1..100}; do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-$BAG_PID" 2>/dev/null || kill -TERM "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"
cd "$WS"
colcon build \
  --packages-select-regex '^sanitation_' \
  --symlink-install \
  --event-handlers console_direct+ \
  2>&1 | tee "$OUT/build.log"
BUILD_CODE=${PIPESTATUS[0]}
if [[ "$BUILD_CODE" -ne 0 ]]; then
  exit "$BUILD_CODE"
fi

set +u
source "$WS/install/setup.bash"
set -u

colcon test \
  --packages-select sanitation_tasks \
  --event-handlers console_direct+ \
  2>&1 | tee "$OUT/test.log"
TEST_CODE=${PIPESTATUS[0]}
colcon test-result --all --verbose > "$OUT/test_results.txt"
if [[ "$TEST_CODE" -ne 0 ]]; then
  exit "$TEST_CODE"
fi

setsid ros2 launch sanitation_bringup sim.launch.py \
  gui:=false headless_rendering:=true \
  > "$OUT/simulation.log" 2>&1 &
LAUNCH_PID=$!

timeout 75s ros2 topic echo --once \
  /ground_truth/model_odom_raw nav_msgs/msg/Odometry \
  > "$OUT/model_odom_first.txt"
READY_CODE=$?
if [[ "$READY_CODE" -ne 0 ]]; then
  echo "ERROR: dedicated model ground truth did not become ready" >&2
  exit "$READY_CODE"
fi

setsid ros2 bag record --storage mcap \
  --output "$OUT/ground_truth_identity_bag" \
  /clock /cmd_vel /ground_truth/model_odom_raw /ground_truth/odom \
  /ground_truth/identity_valid /ground_truth/dynamic_pose \
  /odom/unfiltered /imu/data /joint_states /tf /tf_static \
  > "$OUT/rosbag_record.log" 2>&1 &
BAG_PID=$!

timeout 180s ros2 run sanitation_tasks sanitation_ground_truth_identity_probe --ros-args \
  -p output_path:="$OUT/ground_truth_identity_report.json" \
  -p inventory_path:="$OUT/ground_truth_transform_inventory.json" \
  -p trajectory_path:="$OUT/ground_truth_identity_trajectory.csv" \
  -p timeout_sec:=120.0 \
  2>&1 | tee "$OUT/ground_truth_identity_probe.log"
PROBE_CODE=${PIPESTATUS[0]}

cleanup
BAG_PID=""
LAUNCH_PID=""
trap - EXIT

if [[ -d "$OUT/ground_truth_identity_bag" ]]; then
  ros2 bag info "$OUT/ground_truth_identity_bag" > "$OUT/rosbag_info.txt" 2>&1 || true
fi

ros2 node list | sort > "$OUT/nodes_after_shutdown.txt" 2>&1 || true
export STAGE4S_GT_OUT="$OUT"
export STAGE4S_GT_PROBE_CODE="$PROBE_CODE"
python3 - <<'PY'
import datetime as dt
import json
import os
from pathlib import Path

out = Path(os.environ["STAGE4S_GT_OUT"])
report_path = out / "ground_truth_identity_report.json"
report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else None
summary = {
    "schema_version": 1,
    "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "baseline_commit": "413b6ebfb16d40e00a820c1dcf8cb5c87c90e566",
    "probe_exit_code": int(os.environ["STAGE4S_GT_PROBE_CODE"]),
    "ground_truth_identity": report,
    "success": bool(report and report.get("success")) and int(os.environ["STAGE4S_GT_PROBE_CODE"]) == 0,
    "stop_if_failed": "Do not tune wheel, friction, SLAM, or AMCL when this gate fails.",
    "artifacts": sorted(path.name for path in out.iterdir()),
}
(out / "stage4s_ground_truth_summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY

echo "$OUT"
exit "$PROBE_CODE"
