#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORIGINAL_ARGS=("$@")
RUN_COMMAND="$(printf '%q ' "$0" "${ORIGINAL_ARGS[@]}")"
BASE_WS="${SANITATION_BASE_WS:-$HOME/sanitation_ws}"
PRODUCT_WS="${TZCUP_PRODUCT_MAPPING_WS:-$HOME/tzcup_product_mapping_ws}"
OUTPUT_DIR=""
BUILD=1
SMOKE=0
SEED=2028
MAPPING_TIMEOUT_SEC=7200
NAVIGATION_TIMEOUT_SEC=900
SPAWN_X=0.0
SPAWN_Y=0.0
SPAWN_YAW=0.0
INITIAL_SWEEP_TARGET_INDEX=0
DIAGNOSTIC_OVERRIDE=0

usage() {
  cat <<'EOF'
Usage: bash scripts/run_product_mapping_acceptance.sh [options]

Runs the fail-closed map -> save -> stop -> restart -> load -> relocalize ->
NavigateThroughPoses acceptance chain. Gazebo truth drives the simulated RTK
sensor and post-run evaluator, but no oracle pose topic enters a controller.

Options:
  --output DIR          Evidence directory
  --workspace DIR       Dedicated ROS overlay workspace
  --base-workspace DIR  Existing ROS 2 Jazzy dependency workspace
  --seed N              Gazebo random seed (default: 2028)
  --mapping-timeout N   Frontier exploration budget (default: 7200 seconds)
  --navigation-timeout N Reload navigation budget (default: 900 seconds)
  --spawn-x X           Diagnostic initial X in map/world metres (default: 0.0)
  --spawn-y Y           Diagnostic initial Y in map/world metres (default: 0.0)
  --spawn-yaw RAD       Diagnostic initial yaw in radians (default: 0.0)
  --initial-sweep-target-index N
                        Diagnostic sweep target index (default: 0)
  --skip-build          Reuse an existing overlay install
  --smoke               40 x 20 m wiring check; can never pass the formal gate
  -h, --help            Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) OUTPUT_DIR="$2"; shift 2 ;;
    --workspace) PRODUCT_WS="$2"; shift 2 ;;
    --base-workspace) BASE_WS="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --mapping-timeout) MAPPING_TIMEOUT_SEC="$2"; shift 2 ;;
    --navigation-timeout) NAVIGATION_TIMEOUT_SEC="$2"; shift 2 ;;
    --spawn-x) SPAWN_X="$2"; DIAGNOSTIC_OVERRIDE=1; shift 2 ;;
    --spawn-y) SPAWN_Y="$2"; DIAGNOSTIC_OVERRIDE=1; shift 2 ;;
    --spawn-yaw) SPAWN_YAW="$2"; DIAGNOSTIC_OVERRIDE=1; shift 2 ;;
    --initial-sweep-target-index)
      INITIAL_SWEEP_TARGET_INDEX="$2"; DIAGNOSTIC_OVERRIDE=1; shift 2 ;;
    --skip-build) BUILD=0; shift ;;
    --smoke) SMOKE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$SEED" =~ ^[0-9]+$ ]] || { echo "--seed must be a non-negative integer" >&2; exit 2; }
[[ "$MAPPING_TIMEOUT_SEC" =~ ^[0-9]+$ ]] || { echo "--mapping-timeout must be an integer" >&2; exit 2; }
[[ "$NAVIGATION_TIMEOUT_SEC" =~ ^[0-9]+$ ]] || { echo "--navigation-timeout must be an integer" >&2; exit 2; }
numeric_re='^-?([0-9]+([.][0-9]*)?|[.][0-9]+)$'
[[ "$SPAWN_X" =~ $numeric_re ]] || { echo "--spawn-x must be numeric" >&2; exit 2; }
[[ "$SPAWN_Y" =~ $numeric_re ]] || { echo "--spawn-y must be numeric" >&2; exit 2; }
[[ "$SPAWN_YAW" =~ $numeric_re ]] || { echo "--spawn-yaw must be numeric" >&2; exit 2; }
[[ "$INITIAL_SWEEP_TARGET_INDEX" =~ ^[0-9]+$ ]] || { echo "--initial-sweep-target-index must be a non-negative integer" >&2; exit 2; }
if [[ -z "$OUTPUT_DIR" ]]; then
  scope="formal"
  [[ "$SMOKE" -eq 1 ]] && scope="smoke"
  [[ "$DIAGNOSTIC_OVERRIDE" -eq 1 ]] && scope="diagnostic"
  OUTPUT_DIR="$ROOT/artifacts/product_mapping_${scope}_$(date -u +%Y%m%dT%H%M%SZ)"
fi
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 3
fi
if [[ ! -f "$BASE_WS/install/setup.bash" ]]; then
  echo "Base workspace is missing: $BASE_WS/install/setup.bash" >&2
  exit 3
fi

pids=()
failure_file="$OUTPUT_DIR/run_failure.json"
pid_dir="$OUTPUT_DIR/runtime_pids"
mkdir -p "$pid_dir"

record_failure() {
  local code="$1" line="$2" command="$3"
  FAILURE_CODE="$code" FAILURE_LINE="$line" FAILURE_COMMAND="$command" \
    python3 - "$failure_file" <<'PY'
import json
import os
from pathlib import Path
import time

path = Path(__import__("sys").argv[1])
payload = {
    "status": "failed",
    "exit_code": int(os.environ["FAILURE_CODE"]),
    "line": int(os.environ["FAILURE_LINE"]),
    "command": os.environ["FAILURE_COMMAND"],
    "recorded_unix_time": time.time(),
}
path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
PY
}

on_error() {
  local code="$?"
  record_failure "$code" "${BASH_LINENO[0]:-0}" "${BASH_COMMAND:-unknown}"
  return "$code"
}

stop_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return
  pkill -INT -s "$pid" 2>/dev/null || true
  kill -INT -- "-$pid" 2>/dev/null || true
  kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 100); do
    if ! kill -0 "$pid" 2>/dev/null && \
       ! pgrep -g "$pid" >/dev/null 2>&1 && \
       ! pgrep -s "$pid" >/dev/null 2>&1; then
      wait "$pid" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  pkill -TERM -s "$pid" 2>/dev/null || true
  kill -TERM -- "-$pid" 2>/dev/null || true
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! kill -0 "$pid" 2>/dev/null && \
       ! pgrep -g "$pid" >/dev/null 2>&1 && \
       ! pgrep -s "$pid" >/dev/null 2>&1; then
      wait "$pid" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  pkill -KILL -s "$pid" 2>/dev/null || true
  kill -KILL -- "-$pid" 2>/dev/null || true
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  local sessions=("${pids[@]}")
  local file pid any
  shopt -s nullglob
  for file in "$pid_dir"/*.pid; do
    pid="$(<"$file")"
    [[ "$pid" =~ ^[0-9]+$ ]] && sessions+=("$pid")
  done
  shopt -u nullglob
  for pid in "${sessions[@]}"; do
    pkill -INT -s "$pid" 2>/dev/null || true
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 100); do
    any=0
    for pid in "${sessions[@]}"; do
      if kill -0 "$pid" 2>/dev/null || pgrep -s "$pid" >/dev/null 2>&1; then
        any=1
        break
      fi
    done
    [[ "$any" -eq 0 ]] && return
    sleep 0.1
  done
  for pid in "${sessions[@]}"; do
    pkill -TERM -s "$pid" 2>/dev/null || true
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${sessions[@]}"; do
    pkill -KILL -s "$pid" 2>/dev/null || true
    kill -KILL -- "-$pid" 2>/dev/null || true
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap on_error ERR
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

set +u
source /opt/ros/jazzy/setup.bash
source "$BASE_WS/install/setup.bash"
set -u

mkdir -p "$PRODUCT_WS/src"
rsync -a --delete "$ROOT/starter_ws/src/" "$PRODUCT_WS/src/"
if [[ "$BUILD" -eq 1 ]]; then
  (
    cd "$PRODUCT_WS"
    colcon build --packages-select-regex '^sanitation_' --symlink-install \
      --event-handlers console_direct+
  ) > "$OUTPUT_DIR/build.log" 2>&1
fi
set +u
source "$PRODUCT_WS/install/setup.bash"
set -u

runtime="$OUTPUT_DIR/runtime"
mapping="$OUTPUT_DIR/mapping"
reload="$OUTPUT_DIR/reload"
mkdir -p "$runtime" "$mapping" "$reload"
navigation_share="$(ros2 pkg prefix sanitation_navigation)/share/sanitation_navigation"
bringup_share="$(ros2 pkg prefix sanitation_bringup)/share/sanitation_bringup"
refiner_share="$(ros2 pkg prefix sanitation_scan_refiner)/share/sanitation_scan_refiner"
worlds_share="$(ros2 pkg prefix sanitation_worlds)/share/sanitation_worlds"
source_world="$worlds_share/worlds/sanitation_campus_large.sdf"
runtime_world="$runtime/sanitation_campus_large_product.sdf"
nav_params="$runtime/nav2_mapping_no_prior_filters.yaml"
slam_params="$runtime/slam_product_20000_runtime.yaml"
mapping_ekf_params="$runtime/ekf_mapping_local.yaml"
hybrid_params="$runtime/hybrid_mapping_rtk.yaml"

# Keep smoke at real-time rate too. A 3x rate overloaded SLAM/Nav2 while the
# vehicle continued advancing, creating stale maps and severe goal overshoot.
simulation_rtf="1.0"
python3 - "$source_world" "$runtime_world" "$simulation_rtf" <<'PY'
from pathlib import Path
import re
import sys
source, target = map(Path, sys.argv[1:3])
factor = sys.argv[3]
text = source.read_text(encoding="utf-8")
text, count = re.subn(
    r"<real_time_factor>[^<]+</real_time_factor>",
    f"<real_time_factor>{factor}</real_time_factor>", text, count=1,
)
if count != 1:
    raise SystemExit("large world must define exactly one real_time_factor")
target.write_text(text, encoding="utf-8")
PY

# First-principles mapping must not consume keepout/speed masks derived from a
# prior map. Remove the filter plugins as well as disabling their servers.
python3 - "$navigation_share/config/nav2_ackermann.yaml" "$nav_params" <<'PY'
from pathlib import Path
import sys
import yaml
source, target = map(Path, sys.argv[1:])
config = yaml.safe_load(source.read_text(encoding="utf-8"))
global_params = config["global_costmap"]["global_costmap"]["ros__parameters"]
global_params["plugins"] = [
    item for item in global_params["plugins"]
    if item not in {"keepout_filter", "speed_filter"}
]
global_params.pop("keepout_filter", None)
global_params.pop("speed_filter", None)
config.pop("keepout_filter_mask_server", None)
config.pop("keepout_costmap_filter_info_server", None)
config.pop("speed_filter_mask_server", None)
config.pop("speed_costmap_filter_info_server", None)
manager = config.get("filter_lifecycle_manager", {}).get("ros__parameters", {})
manager["node_names"] = []
for name in (
    "FollowPath", "DubinsPath", "ReversePath", "ConnectorPath",
    "CleanPath", "RepairPath",
):
    config["controller_server"]["ros__parameters"][name]["transform_tolerance"] = 0.5
target.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
PY

python3 - "$navigation_share/config/slam_product_20000.yaml" "$slam_params" <<'PY'
from pathlib import Path
import sys
import yaml
source, target = map(Path, sys.argv[1:])
config = yaml.safe_load(source.read_text(encoding="utf-8"))
config["slam_toolbox"]["ros__parameters"]["scan_topic"] = "/scan/mapping"
target.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
PY

# Build a two-level odometry tree for RTK-aided mapping:
# map(SLAM) -> odom(RTK global) -> wheel_odom(local EKF) -> base_footprint.
# The GNSS simulator is a noisy sensor model; its internal oracle input is
# never remapped or exposed as an odometry/control input.
python3 - "$bringup_share/config/ekf_ackermann.yaml" "$mapping_ekf_params" <<'PY'
from pathlib import Path
import sys
import yaml
source, target = map(Path, sys.argv[1:])
config = yaml.safe_load(source.read_text(encoding="utf-8"))
params = config["ekf_filter_node"]["ros__parameters"]
params["map_frame"] = "odom"
params["odom_frame"] = "wheel_odom"
params["world_frame"] = "wheel_odom"
target.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
PY
python3 - "$refiner_share/config/stage4v_hybrid.yaml" "$hybrid_params" <<'PY'
from pathlib import Path
import sys
import yaml
source, target = map(Path, sys.argv[1:])
config = yaml.safe_load(source.read_text(encoding="utf-8"))
params = config["hybrid_global_fuser"]["ros__parameters"]
params["map_frame"] = "odom"
params["odom_frame"] = "wheel_odom"
params["base_frame"] = "base_footprint"
params["local_odom_topic"] = "/odom"
params["mode"] = "rtk_imu_wheel"
params["publish_map_to_odom"] = True
target.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
PY

if [[ "$SMOKE" -eq 1 ]]; then
  # Larger than one lidar footprint so the smoke path exercises multiple
  # successful frontier goals and can derive a non-trivial reload route.
  BOUNDS=(-20.0 -10.0 20.0 10.0)
  MIN_SPAN_X=40.0; MIN_SPAN_Y=20.0; MIN_AREA=800.0
  ROUTE_SEPARATION=3.0; MAX_GOALS=160; MAX_FRONTIER_GOAL_DISTANCE=2.0
  MAX_LINEAR_VELOCITY=0.30; FORMAL_SCOPE=false; SWEEP_ENABLED=false
else
  BOUNDS=(-100.0 -50.0 100.0 50.0)
  MIN_SPAN_X=200.0; MIN_SPAN_Y=100.0; MIN_AREA=20000.0
  ROUTE_SEPARATION=15.0; MAX_GOALS=800; MAX_FRONTIER_GOAL_DISTANCE=3.0
  MAX_LINEAR_VELOCITY=0.45; FORMAL_SCOPE=true; SWEEP_ENABLED=true
fi
if [[ "$DIAGNOSTIC_OVERRIDE" -eq 1 ]]; then
  FORMAL_SCOPE=false
fi

wait_for_topic() {
  local topic="$1" type="$2" timeout_sec="$3" output="$4"
  timeout "${timeout_sec}s" ros2 topic echo --once "$topic" "$type" > "$output"
}

wait_for_lifecycle_active() {
  local node="$1" timeout_sec="$2" output="$3"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    state="$(timeout 10s ros2 lifecycle get "$node" 2>&1 || true)"
    printf '%s\n' "$state" > "$output"
    if grep -Eq '^active( |$)' <<< "$state"; then
      return 0
    fi
    sleep 1
  done
  echo "Lifecycle node did not become active: $node" >&2
  return 1
}

activate_slam_toolbox() {
  local output="$1"
  local state=""
  : > "$output"
  for _ in $(seq 1 60); do
    state="$(timeout 5s ros2 lifecycle get /slam_toolbox 2>&1 || true)"
    printf 'state: %s\n' "$state" >> "$output"
    if grep -Eq '^unconfigured( |$)' <<< "$state"; then
      timeout 30s ros2 service call /slam_toolbox/change_state \
        lifecycle_msgs/srv/ChangeState '{transition: {id: 1}}' >> "$output" 2>&1
    elif grep -Eq '^inactive( |$)' <<< "$state"; then
      timeout 30s ros2 service call /slam_toolbox/change_state \
        lifecycle_msgs/srv/ChangeState '{transition: {id: 3}}' >> "$output" 2>&1
    elif grep -Eq '^active( |$)' <<< "$state"; then
      return 0
    fi
    sleep 1
  done
  echo "slam_toolbox did not reach active state" >&2
  return 1
}

# GNU setsid can fork when invoked from a background shell. In WSL, $! can
# therefore identify a short-lived wrapper rather than the real ROS session.
# The inner shell records its own PID after setsid; that PID is also the process
# group/session leader used by stop_group.
start_group() {
  local name="$1" log="$2"
  shift 2
  local pid_file="$pid_dir/${name}.pid"
  local exit_file="$pid_dir/${name}.exit"
  rm -f -- "$pid_file" "$exit_file"
  setsid bash -c '
    pid_file="$1"; exit_file="$2"; shift 2
    printf "%s\n" "$$" > "$pid_file"
    "$@"
    code=$?
    printf "%s\n" "$code" > "$exit_file"
    exit "$code"
  ' _ "$pid_file" "$exit_file" "$@" > "$log" 2>&1 &
  local launcher_pid=$!
  for _ in $(seq 1 100); do
    if [[ -s "$pid_file" ]]; then
      STARTED_PID="$(<"$pid_file")"
      [[ "$STARTED_PID" =~ ^[0-9]+$ ]] || break
      pids+=("$STARTED_PID")
      return 0
    fi
    if ! kill -0 "$launcher_pid" 2>/dev/null; then
      wait "$launcher_pid" 2>/dev/null || true
      break
    fi
    sleep 0.05
  done
  echo "Failed to capture process-group leader for $name" >&2
  return 1
}

wait_group() {
  local name="$1" pid="$2"
  local exit_file="$pid_dir/${name}.exit"
  while kill -0 "$pid" 2>/dev/null || \
        pgrep -g "$pid" >/dev/null 2>&1 || \
        pgrep -s "$pid" >/dev/null 2>&1; do
    sleep 0.2
  done
  if [[ -s "$exit_file" ]]; then
    GROUP_EXIT_CODE="$(<"$exit_file")"
  else
    GROUP_EXIT_CODE=125
  fi
  [[ "$GROUP_EXIT_CODE" =~ ^[0-9]+$ ]] || GROUP_EXIT_CODE=125
}

start_simulation() {
  local name="$1" log="$2"
  start_group "$name" "$log" ros2 launch sanitation_bringup sim.launch.py \
    gui:=false headless_rendering:=true drive_model:=ackermann \
    random_seed:="$SEED" world_file:="$runtime_world" \
    world_name:=sanitation_campus_large spawn_x:="$SPAWN_X" spawn_y:="$SPAWN_Y" spawn_yaw:="$SPAWN_YAW" \
    world_to_map_x:=0.0 world_to_map_y:=0.0 world_to_map_yaw:=0.0 \
    ekf_config:="$mapping_ekf_params"
}

start_positioning_chain() {
  local directory="$1"
  local prefix="$2"
  start_group "${prefix}_gnss" "$directory/gnss_sensor.log" \
    ros2 launch sanitation_gnss_sim gnss_sim.launch.py \
    profile:=rtk_fixed random_seed:="$SEED"
  GNSS_PID="$STARTED_PID"
  start_group "${prefix}_fusion" "$directory/rtk_fusion.log" \
    ros2 launch sanitation_scan_refiner hybrid_localization.launch.py \
    hybrid_config_file:="$hybrid_params" fusion_mode:=rtk_imu_wheel \
    enable_scan_refiner:=false publish_map_to_odom:=true \
    initial_pose_x:="$SPAWN_X" initial_pose_y:="$SPAWN_Y" initial_pose_yaw:="$SPAWN_YAW"
  FUSION_PID="$STARTED_PID"
}

start_navigation() {
  local name="$1" backend="$2" map_file="$3" log="$4"
  start_group "$name" "$log" ros2 launch sanitation_navigation navigation.launch.py \
    rviz:=false localization_backend:="$backend" enable_filters:=false \
    params_file:="$nav_params" map_file:="$map_file" \
    operational_profile:=precision_mapping max_linear_velocity:="$MAX_LINEAR_VELOCITY" \
    max_angular_velocity:=0.25 initial_pose_x:="$SPAWN_X" initial_pose_y:="$SPAWN_Y" \
    initial_pose_yaw:="$SPAWN_YAW"
}

echo "[PRODUCT-MAPPING] phase 1: continuous first-principles mapping"
start_simulation mapping_sim "$mapping/simulation.log"; sim_pid="$STARTED_PID"
start_positioning_chain "$mapping" mapping; gnss_pid="$GNSS_PID"; fusion_pid="$FUSION_PID"
wait_for_topic /localization/fused_pose geometry_msgs/msg/PoseWithCovarianceStamped 180 "$mapping/first_fused_pose.txt"
start_group mapping_scan "$mapping/scan_normalizer.log" \
  ros2 run sanitation_navigation scan_self_filter --ros-args \
  -p use_sim_time:=true -p input_topic:=/scan -p output_topic:=/scan/mapping \
  -p replace_infinite_ranges_with_max:=true -p maximum_range_margin_m:=0.01
scan_pid="$STARTED_PID"
start_group mapping_slam "$mapping/slam.log" ros2 launch slam_toolbox \
  online_async_launch.py use_sim_time:=true autostart:=false \
  slam_params_file:="$slam_params"
slam_pid="$STARTED_PID"
activate_slam_toolbox "$mapping/slam_lifecycle.txt"
start_navigation mapping_nav external "$mapping/unused_map.yaml" "$mapping/navigation.log"; nav_pid="$STARTED_PID"
wait_for_topic /map nav_msgs/msg/OccupancyGrid 180 "$mapping/first_map.txt"
if ! wait_for_lifecycle_active /bt_navigator 90 "$mapping/bt_navigator_active.txt"; then
  stop_group "$nav_pid"
  start_navigation mapping_nav_retry external "$mapping/unused_map.yaml" "$mapping/navigation_retry.log"
  nav_pid="$STARTED_PID"
  wait_for_lifecycle_active /bt_navigator 180 "$mapping/bt_navigator_active_retry.txt"
fi
start_group mapping_tf "$mapping/tf_continuity.log" \
  ros2 run sanitation_tasks sanitation_tf_continuity_probe --ros-args \
  -p use_sim_time:=true -p output_path:="$mapping/tf_continuity.json"
mapping_tf_pid="$STARTED_PID"

start_group mapping_explorer "$mapping/frontier_exploration.log" \
  timeout "$((MAPPING_TIMEOUT_SEC + 180))s" \
  ros2 run sanitation_tasks sanitation_frontier_explorer --ros-args \
  -p use_sim_time:=true \
  -p output_path:="$mapping/frontier_exploration.json" \
  -p required_bounds_xyxy_m:="[${BOUNDS[0]},${BOUNDS[1]},${BOUNDS[2]},${BOUNDS[3]}]" \
  -p required_bounds_coverage_ratio:=1.0 -p required_bounds_goal_margin_m:=1.5 \
  -p minimum_goal_distance_m:=0.80 -p minimum_turning_radius_m:=1.429 \
  -p maximum_frontier_goal_yaw_change_rad:=0.70 \
  -p minimum_frontier_arc_yaw_change_rad:=0.15 \
  -p boundary_turn_buffer_m:=1.429 \
  -p maximum_frontier_goal_distance_m:="$MAX_FRONTIER_GOAL_DISTANCE" \
  -p initial_frontier_goal_distance_m:=2.0 \
  -p goal_distance_growth_success_count:=5 -p goal_distance_growth_step_m:=0.5 \
  -p failed_goal_exclusion_radius_m:=1.0 -p timed_out_goal_exclusion_radius_m:=1.5 \
  -p maximum_goal_count:="$MAX_GOALS" -p timeout_sec:="${MAPPING_TIMEOUT_SEC}.0" \
  -p goal_timeout_sec:=60.0 -p failed_goal_cooldown_sec:=10.0 \
  -p failed_goal_exclusion_ttl_sec:=180.0 \
  -p minimum_frontier_map_gain_m2:=2.0 \
  -p no_progress_staging_success_limit:=3 \
  -p no_progress_raw_frontier_success_limit:=12 \
  -p no_progress_raw_exclusion_ttl_sec:=900.0 \
  -p horizontal_sweep_staging_distances_m:="[8.0, 6.0, 4.0]" \
  -p reverse_escape_distance_m:=2.0 \
  -p reverse_escape_speed_mps:=0.15 \
  -p frontier_sweep_enabled:="$SWEEP_ENABLED" \
  -p frontier_sweep_initial_target_index:="$INITIAL_SWEEP_TARGET_INDEX" \
  -p frontier_sweep_reference_pose_xyyaw_m_rad:="[0.0, 0.001, 0.0]" \
  -p mapping_sensor_range_m:=12.0 \
  -p frontier_sweep_lane_overlap_m:=2.0 \
  -p frontier_sweep_target_tolerance_m:=2.0 \
  -p frontier_sweep_mapped_target_radius_m:=5.0 \
  -p frontier_sweep_lane_shift_backup_distance_m:=4.0 \
  -p frontier_sweep_lane_shift_backup_max_attempts:=2 \
  -p frontier_sweep_lane_shift_connector_distances_m:="[6.0, 4.0, 2.0]" \
  -p lane_shift_connector_timeout_sec:=180.0 \
  -p behavior_tree:="$navigation_share/behavior_trees/navigate_to_pose_ackermann_frontier.xml" \
  -p positioning_source:=rtk_gnss_sensor_wheel_imu_scan_matching
mapping_explorer_pid="$STARTED_PID"
set +e
wait_group mapping_explorer "$mapping_explorer_pid"
EXPLORATION_CODE="$GROUP_EXIT_CODE"
ros2 run nav2_map_server map_saver_cli -f "$mapping/product_map" --ros-args \
  -p use_sim_time:=true -p save_map_timeout:=60.0 > "$mapping/map_save.log" 2>&1
MAP_SAVE_CODE=$?
timeout 120s ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$mapping/product_posegraph'}" > "$mapping/posegraph_serialize.log" 2>&1
POSEGRAPH_CODE=$?
set -e

# This is the required hard process restart, not a lifecycle transition.
stop_group "$mapping_tf_pid"; stop_group "$nav_pid"; stop_group "$slam_pid"; stop_group "$scan_pid"; stop_group "$fusion_pid"; stop_group "$gnss_pid"; stop_group "$sim_pid"
pids=()
sleep 2
RESTART_COMPLETED=true

set +e
ros2 run sanitation_tasks sanitation_map_quality \
  --map-yaml "$mapping/product_map.yaml" --output "$mapping/map_quality.json" \
  --preview "$mapping/map_preview.png" --maximum-resolution-m 0.10 \
  --minimum-span-x-m "$MIN_SPAN_X" --minimum-span-y-m "$MIN_SPAN_Y" \
  --minimum-known-area-m2 "$MIN_AREA"
MAP_QUALITY_CODE=$?
python3 "$ROOT/scripts/stage4t_map_geometry.py" \
  --map-yaml "$mapping/product_map.yaml" --world-sdf "$source_world" \
  --output "$mapping/map_geometry.json" --overlay "$mapping/map_truth_overlay.png" \
  --alignment-x 0.0 --alignment-y 0.0 --alignment-yaw 0.0
MAP_GEOMETRY_CODE=$?
python3 "$ROOT/scripts/product_mapping_acceptance.py" build-route \
  --exploration "$mapping/frontier_exploration.json" --output "$reload/route.json" \
  --minimum-separation-m "$ROUTE_SEPARATION" --minimum-waypoints 3 --maximum-waypoints 5
ROUTE_CODE=$?
set -e

NAVIGATION_CODE=125
if [[ "$EXPLORATION_CODE" -eq 0 && "$MAP_SAVE_CODE" -eq 0 && \
      "$POSEGRAPH_CODE" -eq 0 && "$MAP_QUALITY_CODE" -eq 0 && \
      "$MAP_GEOMETRY_CODE" -eq 0 && "$ROUTE_CODE" -eq 0 ]]; then
  echo "[PRODUCT-MAPPING] phase 2: fresh simulator, saved map, AMCL, Nav2"
  start_simulation reload_sim "$reload/simulation.log"; reload_sim_pid="$STARTED_PID"
  start_positioning_chain "$reload" reload; reload_gnss_pid="$GNSS_PID"; reload_fusion_pid="$FUSION_PID"
  wait_for_topic /localization/fused_pose geometry_msgs/msg/PoseWithCovarianceStamped 180 "$reload/first_fused_pose.txt"
  start_navigation reload_nav amcl "$mapping/product_map.yaml" "$reload/navigation.log"; reload_nav_pid="$STARTED_PID"
  wait_for_topic /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped 180 "$reload/first_amcl.txt"
  start_group reload_tf "$reload/tf_continuity.log" \
    ros2 run sanitation_tasks sanitation_tf_continuity_probe --ros-args \
    -p use_sim_time:=true -p output_path:="$reload/tf_continuity.json"
  reload_tf_pid="$STARTED_PID"
  start_group reload_probe "$reload/navigation_probe.log" \
    timeout "$((NAVIGATION_TIMEOUT_SEC + 180))s" \
    ros2 run sanitation_tasks sanitation_navigation_probe --ros-args \
    -p use_sim_time:=true -p waypoints_file:="$reload/route.json" \
    -p output_path:="$reload/navigation_probe.json" \
    -p timeout_sec:="${NAVIGATION_TIMEOUT_SEC}.0"
  reload_probe_pid="$STARTED_PID"
  set +e
  wait_group reload_probe "$reload_probe_pid"
  NAVIGATION_CODE="$GROUP_EXIT_CODE"
  set -e
  stop_group "$reload_tf_pid"; stop_group "$reload_nav_pid"; stop_group "$reload_fusion_pid"; stop_group "$reload_gnss_pid"; stop_group "$reload_sim_pid"
  pids=()
else
  printf '%s\n' "phase 2 skipped because a phase 1 prerequisite failed" > "$reload/skipped.txt"
fi

PRODUCT_RUN_COMMAND="$RUN_COMMAND" python3 - \
  "$OUTPUT_DIR/processes.json" "$FORMAL_SCOPE" "$RESTART_COMPLETED" \
  "$EXPLORATION_CODE" "$MAP_SAVE_CODE" "$POSEGRAPH_CODE" "$MAP_QUALITY_CODE" \
  "$MAP_GEOMETRY_CODE" "$ROUTE_CODE" "$NAVIGATION_CODE" "$ROOT" "$SEED" \
  "$runtime_world" "$nav_params" "$slam_params" "$mapping_ekf_params" \
  "$hybrid_params" "$DIAGNOSTIC_OVERRIDE" "$SPAWN_X" "$SPAWN_Y" \
  "$SPAWN_YAW" "$INITIAL_SWEEP_TARGET_INDEX" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
path = Path(sys.argv[1])
names = ("exploration", "map_save", "posegraph_serialize", "map_quality",
         "map_geometry", "route_build", "navigation")
root = Path(sys.argv[11])
seed = int(sys.argv[12])
config_paths = [Path(item) for item in sys.argv[13:]]
config_paths = config_paths[:5]

def git(*args):
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), *args], text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (FileNotFoundError, subprocess.CalledProcessError):
        # Git worktrees created by Windows store a Windows gitdir in .git.
        # WSL git interprets that pointer as a Linux-relative path, so use the
        # host Git executable with an explicitly converted worktree path.
        windows_root = subprocess.check_output(
            ["wslpath", "-m", str(root)], text=True
        ).strip()
        return subprocess.check_output(
            ["git.exe", "-C", windows_root, *args], text=True
        ).strip()

def sha256(file_path):
    digest = hashlib.sha256()
    with file_path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()

payload = {
    "formal_scope": sys.argv[2].lower() == "true",
    "diagnostic_override": {
        "enabled": sys.argv[18] == "1",
        "spawn_x": float(sys.argv[19]),
        "spawn_y": float(sys.argv[20]),
        "spawn_yaw": float(sys.argv[21]),
        "initial_sweep_target_index": int(sys.argv[22]),
    },
    "restart_completed": sys.argv[3].lower() == "true",
    "exit_codes": dict(zip(names, map(int, sys.argv[4:]))),
    "sensor_provenance": {
        "positioning": "simulated_rtk_gnss_plus_wheel_imu_plus_scan_matching",
        "gazebo_truth_to_gnss_sensor_model": True,
        "oracle_pose_topic_to_controller": False,
    },
    "reproducibility": {
        "source_commit": git("rev-parse", "HEAD"),
        "source_dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
        "seed": seed,
        "command": os.environ["PRODUCT_RUN_COMMAND"].strip(),
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "model_sha256": "not_applicable_mapping_pipeline_has_no_model",
        "dataset_sha256": "not_applicable_seeded_gazebo_world",
        "container_digest": "not_applicable_host_ros_runtime",
        "config_sha256": {
            str(item): sha256(item) for item in config_paths
        },
    },
}
path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
PY

if python3 "$ROOT/scripts/product_mapping_acceptance.py" evaluate \
  --exploration "$mapping/frontier_exploration.json" \
  --map-quality "$mapping/map_quality.json" --map-geometry "$mapping/map_geometry.json" \
  --mapping-tf "$mapping/tf_continuity.json" --reload-tf "$reload/tf_continuity.json" \
  --navigation "$reload/navigation_probe.json" --processes "$OUTPUT_DIR/processes.json" \
  --map-yaml "$mapping/product_map.yaml" \
  --posegraph "$mapping/product_posegraph.posegraph" \
  --posegraph-data "$mapping/product_posegraph.data" --reload-route "$reload/route.json" \
  --output "$OUTPUT_DIR/product_mapping_acceptance.json"; then
  EVALUATION_CODE=0
else
  EVALUATION_CODE=$?
fi

echo "$OUTPUT_DIR"
if [[ "$SMOKE" -eq 1 ]]; then
  echo "Smoke wiring run finished; inspect smoke_chain_pass. formal_scope=false, so it cannot pass PRODUCT-MAPPING-20000M2."
  exit "$EVALUATION_CODE"
fi
exit "$EVALUATION_CODE"
