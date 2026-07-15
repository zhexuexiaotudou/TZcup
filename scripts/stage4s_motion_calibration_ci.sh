#!/usr/bin/env bash
set -uo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS must point to the reusable Stage 1 workspace}"
OUT="$PACK_ROOT/artifacts/stage4s_motion_$STAMP"
DRIVE_WHEEL_RADIUS="${TZCUP_DRIVE_WHEEL_RADIUS:-0.14}"
DRIVE_WHEEL_SEPARATION="${TZCUP_DRIVE_WHEEL_SEPARATION:-0.80}"
CALIBRATION_LABEL="${TZCUP_CALIBRATION_LABEL:-baseline}"
LAUNCH_PID=""
BAG_PID=""

mkdir -p "$OUT"

stop_group() {
  local pid="$1"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    return
  fi
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in {1..100}; do
    kill -0 "$pid" 2>/dev/null || return
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$pid" 2>/dev/null || return
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

cleanup() {
  stop_group "$BAG_PID"
  stop_group "$LAUNCH_PID"
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

setsid ros2 launch sanitation_bringup motion_calibration.launch.py \
  gui:=false headless_rendering:=true \
  drive_wheel_radius:="$DRIVE_WHEEL_RADIUS" \
  drive_wheel_separation:="$DRIVE_WHEEL_SEPARATION" \
  > "$OUT/simulation.log" 2>&1 &
LAUNCH_PID=$!

timeout 75s ros2 topic echo --once \
  /ground_truth/model_odom_raw nav_msgs/msg/Odometry \
  > "$OUT/model_odom_first.txt"
READY_CODE=$?
if [[ "$READY_CODE" -ne 0 ]]; then
  echo "ERROR: calibration launch did not become ready" >&2
  exit "$READY_CODE"
fi

setsid ros2 bag record --storage mcap \
  --output "$OUT/motion_calibration_bag" \
  /clock /cmd_vel_gate /cmd_vel /calibration/segment_marker \
  /joint_states /odom/unfiltered /odom /imu/data \
  /ground_truth/model_odom_raw /ground_truth/odom \
  /ground_truth/identity_valid /tf /tf_static \
  > "$OUT/rosbag_record.log" 2>&1 &
BAG_PID=$!

timeout 780s ros2 run sanitation_tasks sanitation_motion_calibration_runner --ros-args \
  -p use_sim_time:=true \
  -p output_dir:="$OUT" \
  -p timeout_margin_sec:=90.0 \
  -p drive_wheel_radius:="$DRIVE_WHEEL_RADIUS" \
  -p drive_wheel_separation:="$DRIVE_WHEEL_SEPARATION" \
  -p calibration_label:="$CALIBRATION_LABEL" \
  2>&1 | tee "$OUT/motion_calibration_runner.log"
RUNNER_CODE=${PIPESTATUS[0]}

stop_group "$BAG_PID"
BAG_PID=""
stop_group "$LAUNCH_PID"
LAUNCH_PID=""
trap - EXIT

if [[ -d "$OUT/motion_calibration_bag" ]]; then
  ros2 bag info "$OUT/motion_calibration_bag" > "$OUT/rosbag_info.txt" 2>&1 || true
fi

export STAGE4S_MOTION_OUT="$OUT"
export STAGE4S_MOTION_CODE="$RUNNER_CODE"
export STAGE4S_DRIVE_WHEEL_RADIUS="$DRIVE_WHEEL_RADIUS"
export STAGE4S_DRIVE_WHEEL_SEPARATION="$DRIVE_WHEEL_SEPARATION"
export STAGE4S_CALIBRATION_LABEL="$CALIBRATION_LABEL"
python3 - <<'PY'
import datetime as dt
import json
import os
from pathlib import Path

out = Path(os.environ["STAGE4S_MOTION_OUT"])
report_path = out / "motion_calibration_report.json"
fault_path = out / "fault_isolation_report.json"
report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else None
fault = json.loads(fault_path.read_text(encoding="utf-8")) if fault_path.exists() else None
summary = {
    "schema_version": 1,
    "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "baseline_commit": "413b6ebfb16d40e00a820c1dcf8cb5c87c90e566",
    "runner_exit_code": int(os.environ["STAGE4S_MOTION_CODE"]),
    "timing_basis": "simulation_clock",
    "calibration_label": os.environ["STAGE4S_CALIBRATION_LABEL"],
    "drive_wheel_radius": float(os.environ["STAGE4S_DRIVE_WHEEL_RADIUS"]),
    "drive_wheel_separation": float(os.environ["STAGE4S_DRIVE_WHEEL_SEPARATION"]),
    "experiment_completed": bool(report and report.get("experiment_completed")),
    "realistic_motion_calibration_pass": bool(report and report.get("realistic_motion_calibration_pass")),
    "first_failed_layer": fault.get("first_failed_layer") if fault else "missing_report",
    "artifacts": sorted(path.name for path in out.iterdir()),
}
(out / "stage4s_motion_summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY

echo "$OUT"
exit "$RUNNER_CODE"
