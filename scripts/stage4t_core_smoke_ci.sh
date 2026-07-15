#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS must point to the reusable workspace}"
OUT="${STAGE4T_OUT:-$PACK_ROOT/artifacts/stage4t_core_smoke_$STAMP}"
PIDS=()
mkdir -p "$OUT/angular_rate_trials"

stop_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  if ! kill -0 "$pid" 2>/dev/null; then return 0; fi
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in {1..100}; do
    if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return 0; fi
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in {1..30}; do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.1; done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() { for pid in "${PIDS[@]:-}"; do stop_group "$pid"; done; }
stop_bag() { local pid="$1" log="$2"; if timeout 60s ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop '{}' > "$log" 2>&1; then kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; else stop_group "$pid"; fi; }
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"
cd "$WS"
colcon build --packages-select-regex '^sanitation_' --symlink-install --event-handlers console_direct+ > "$OUT/build.log" 2>&1
set +u
source "$WS/install/setup.bash"
set -u

start_sim() {
  local profile="$1" linear="$2" angular="$3" log="$4"
  setsid ros2 launch sanitation_bringup motion_calibration.launch.py \
    gui:=false headless_rendering:=true \
    drive_wheel_radius:=0.14 drive_wheel_separation:=1.22 \
    operational_profile:="$profile" max_linear_velocity:="$linear" max_angular_velocity:="$angular" \
    > "$log" 2>&1 &
  SIM_PID=$!; PIDS+=("$SIM_PID")
  timeout 75s ros2 topic echo --once /ground_truth/odom nav_msgs/msg/Odometry > "$OUT/${profile}_first_truth.txt"
}

run_trial() {
  local id="$1" type="$2" thermal="$3" rate="$4" target="$5"
  timeout 120s ros2 run sanitation_tasks sanitation_transient_response_runner --ros-args \
    -p use_sim_time:=true \
    -p trial_id:="$id" -p trial_type:="$type" -p thermal_state:="$thermal" \
    -p angular_rate:="$rate" -p target_heading_rad:="$target" \
    -p output_path:="$OUT/angular_rate_trials/$id.json" \
    -p csv_path:="$OUT/angular_rate_trials/$id.csv" \
    > "$OUT/angular_rate_trials/$id.log" 2>&1
}

start_sim localization_coverage 0.45 0.35 "$OUT/coverage_simulation.log"
setsid ros2 bag record --storage mcap --output "$OUT/stage4t_core_bag" \
  /clock /cmd_vel_gate /cmd_vel /odom/unfiltered /measurements/wheel_odom \
  /imu/data /measurements/imu /odom /ground_truth/odom /tf /tf_static \
  > "$OUT/rosbag_record.log" 2>&1 &
BAG_PID=$!; PIDS+=("$BAG_PID")

ros2 run sanitation_tasks sanitation_covariance_audit --ros-args \
  -p use_sim_time:=true -p duration_sec:=6.0 \
  -p output_path:="$OUT/measurement_covariance_report.json" \
  > "$OUT/covariance_audit.log" 2>&1

setsid ros2 run sanitation_tasks sanitation_operational_envelope_audit --ros-args \
  -p use_sim_time:=true -p profile_name:=localization_coverage \
  -p max_linear_velocity:=0.45 -p max_angular_velocity:=0.35 -p duration_sec:=25.0 \
  -p output_path:="$OUT/coverage_envelope_trial.json" \
  > "$OUT/coverage_envelope.log" 2>&1 &
AUDIT_PID=$!; PIDS+=("$AUDIT_PID")
run_trial fixed_cold_p035 fixed_time cold 0.35 1.5707963267948966
run_trial heading_cold_p090 closed_loop_heading cold 0.35 1.5707963267948966
wait "$AUDIT_PID"; PIDS=("$SIM_PID" "$BAG_PID")
stop_bag "$BAG_PID" "$OUT/rosbag_stop.log"; PIDS=("$SIM_PID")
stop_group "$SIM_PID"; PIDS=()

start_sim precision_mapping 0.30 0.25 "$OUT/precision_simulation.log"
setsid ros2 run sanitation_tasks sanitation_operational_envelope_audit --ros-args \
  -p use_sim_time:=true -p profile_name:=precision_mapping \
  -p max_linear_velocity:=0.30 -p max_angular_velocity:=0.25 -p duration_sec:=15.0 \
  -p output_path:="$OUT/precision_envelope_trial.json" \
  > "$OUT/precision_envelope.log" 2>&1 &
AUDIT_PID=$!; PIDS+=("$AUDIT_PID")
run_trial fixed_cold_p025 fixed_time cold 0.25 1.5707963267948966
wait "$AUDIT_PID"; PIDS=("$SIM_PID")
stop_group "$SIM_PID"; PIDS=()

ros2 bag info "$OUT/stage4t_core_bag" > "$OUT/rosbag_info.txt"
python3 "$PACK_ROOT/scripts/stage4t_aggregate.py" "$OUT" --required-repeats 10
python3 - "$OUT" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
load = lambda name: json.loads((out / name).read_text(encoding="utf-8"))
summary = {
    "schema_version": 1,
    "real_gazebo_executed": True,
    "covariance_pass": load("measurement_covariance_report.json")["pass"],
    "operational_envelope_pass": load("operational_envelope_report.json")["pass"],
    "transient_matrix_complete": load("transient_response_report.json")["matrix_complete"],
    "closed_loop_matrix_complete": load("closed_loop_heading_report.json")["matrix_complete"],
    "first_incomplete_gate": "stage4t_1_full_repeatability_matrix",
}
(out / "stage4t_core_smoke_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
if not summary["covariance_pass"] or not summary["operational_envelope_pass"]:
    raise SystemExit(5)
PY

echo "$OUT"
