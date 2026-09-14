#!/usr/bin/env bash
# Guest-side one-shot bounded saved-map coverage runner.
set -eo pipefail

: "${SOURCE:?}" "${RUNTIME:?}" "${OUTPUT:?}" "${EPISODE:?}" "${MAP_SOURCE:?}"
: "${OVERLAY:?}" "${OPS_DIR:?}" "${COVERAGE_CONFIG:?}" "${CLEANING_BRIDGE:?}" "${SUMMARIZER:?}"

source /opt/ros/jazzy/setup.bash
source "$RUNTIME/install/setup.bash"
source "$OVERLAY/install/local_setup.bash"
source "$SOURCE/scripts/run_formal_runtime_isolation.sh"

export TZCUP_REPOSITORY_ROOT="$SOURCE"
export ROS_DOMAIN_ID="${PROBE_DOMAIN:-94}"
export GZ_PARTITION="${PROBE_PARTITION:-tzcup_day1_bounded_coverage_20260914_01}"
formal_runtime_configure "$ROS_DOMAIN_ID"

mkdir "$OUTPUT"
cp "$MAP_SOURCE"/{occupancy.pgm,occupancy.yaml,geofence_keepout.pgm,geofence_keepout.yaml,neutral_speed.pgm,neutral_speed.yaml,mission_geometry.yaml,materialization_contract.yaml} "$OUTPUT/"
cp "$COVERAGE_CONFIG" "$OUTPUT/coverage_config.yaml"

python3 - "$OUTPUT/coverage_server_params.yaml" <<'PY'
import pathlib
import sys
import yaml

payload = {
    "coverage_server": {
        "ros__parameters": {
            "use_sim_time": True,
            "action_server_result_timeout": 30.0,
            "coordinates_in_cartesian_frame": True,
            "robot_width": 1.32,
            "operation_width": 0.55,
            "min_turning_radius": 0.40,
            "linear_curv_change": 200.0,
            "default_headland_width": 0.0,
            "default_headland_type": "CONSTANT",
            "default_allow_overlap": True,
            "default_swath_type": "COVERAGE",
            "default_swath_angle_type": "SET_ANGLE",
            "default_swath_angle": 0.0,
            "default_route_type": "BOUSTROPHEDON",
            "default_path_type": "DUBIN",
            "default_path_continuity_type": "DISCONTINUOUS",
            "default_turn_point_distance": 0.10,
            "max_turn_angular_velocity": 0.60,
        }
    }
}
pathlib.Path(sys.argv[1]).write_text(
    yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
)
PY

python3 - "$SOURCE" "$OUTPUT" <<'PY'
import os
from pathlib import Path
import yaml

source = Path(os.environ["SOURCE"])
out = Path(os.environ["OUTPUT"])
nav = yaml.safe_load(
    (source / "starter_ws/src/sanitation_navigation/config/nav2.yaml").read_text(
        encoding="utf-8"
    )
)
for name in ("controller_server", "velocity_smoother", "collision_monitor"):
    nav[name]["ros__parameters"]["enable_stamped_cmd_vel"] = False
nav["amcl"]["ros__parameters"]["scan_topic"] = "/scan/navigation"
nav["amcl"]["ros__parameters"]["tf_broadcast"] = False
collision = nav["collision_monitor"]["ros__parameters"]
collision["observation_sources"] = ["scan"]
collision.pop("mid360", None)
collision["scan"]["topic"] = "/scan/navigation"
for name in ("local_costmap", "global_costmap"):
    obstacle = nav[name][name]["ros__parameters"]["obstacle_layer"]
    obstacle["observation_sources"] = "scan"
    obstacle.pop("mid360", None)
    obstacle["scan"]["topic"] = "/scan/navigation"
(out / "nav2.yaml").write_text(
    yaml.safe_dump(nav, sort_keys=False), encoding="utf-8"
)
PY

launch_pid=""
filter_pid=""
nav_pid=""
dirt_bridge_pid=""
coverage_pid=""
cleaning_bridge_pid=""
recorder_pid=""
probe_pid=""

cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -TERM -- "-$recorder_pid" 2>/dev/null || true
    for _ in {1..100}; do kill -0 "$recorder_pid" 2>/dev/null || break; sleep 0.1; done
  fi
  formal_runtime_cleanup_groups "$GZ_PARTITION" \
    "$probe_pid" "$cleaning_bridge_pid" "$coverage_pid" "$dirt_bridge_pid" \
    "$nav_pid" "$filter_pid" "$launch_pid" || true
}
formal_runtime_install_traps cleanup

setsid ros2 launch sanitation_formal_campus_integration formal_campus.launch.py \
  gui:=false world:="$EPISODE/public/world.sdf" world_name:=campus_formal \
  episode_manifest:="$EPISODE/public/episode_manifest.json" \
  pedestrian_schedule:="$EPISODE/environment/pedestrian_schedule.json" \
  start_pedestrians:=false high_bandwidth_sensor_runtime:=false \
  enable_training_gt:=false runtime_artifact_dir:="$OUTPUT" \
  materialize_static_maps:=false start_navigation:=false start_coverage:=false \
  localization_backend:=amcl mission_mode:=cleaning \
  operation_speed_profile:=mapping_safe max_linear_velocity:=0.45 \
  speed_qualification_state:=none \
  >"$OUTPUT/launch.log" 2>&1 &
launch_pid=$!

setsid ros2 run sanitation_formal_campus_integration formal-scan-self-filter \
  --ros-args \
  --params-file "$SOURCE/starter_ws/src/sanitation_formal_campus_integration/config/formal_utm30lx_self_filter.yaml" \
  -p use_sim_time:=true >"$OUTPUT/filter.log" 2>&1 &
filter_pid=$!

setsid ros2 run ros_gz_bridge parameter_bridge \
  '/model/tzcup_formal_sanitation_vehicle/ground_dirt/command/enable@std_msgs/msg/Bool]gz.msgs.Boolean' \
  '/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json@std_msgs/msg/String[gz.msgs.StringMsg' \
  >"$OUTPUT/dirt_bridge.log" 2>&1 &
dirt_bridge_pid=$!

sleep 40
setsid ros2 launch sanitation_navigation navigation.launch.py \
  use_sim_time:=true rviz:=false params_file:="$OUTPUT/nav2.yaml" \
  map_file:="$OUTPUT/occupancy.yaml" keepout_map:="$OUTPUT/geofence_keepout.yaml" \
  speed_map:="$OUTPUT/neutral_speed.yaml" \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0 \
  localization_backend:=amcl start_velocity_gate:=false \
  >"$OUTPUT/navigation.log" 2>&1 &
nav_pid=$!

setsid python3 "$CLEANING_BRIDGE" \
  --output-dir "$OUTPUT" --stop-file "$OUTPUT/cleaning_bridge.stop" \
  >"$OUTPUT/cleaning_bridge.log" 2>&1 &
cleaning_bridge_pid=$!

sleep 20
setsid ros2 launch sanitation_coverage coverage.launch.py \
  params_file:="$OUTPUT/coverage_server_params.yaml" \
  footprint_profile:=auto12_efficiency_v1 \
  >"$OUTPUT/coverage_server.log" 2>&1 &
coverage_pid=$!

ready_deadline=$((SECONDS + 360))
while (( SECONDS < ready_deadline )); do
  if [[ -f "$OUTPUT/cleaning_bridge_ready.json" ]] && \
     ros2 action list 2>/dev/null | grep -q '^/compute_coverage_path$' && \
     ros2 topic list 2>/dev/null | grep -q '^/ground_truth/odom$'; then
    break
  fi
  kill -0 "$launch_pid" 2>/dev/null || { echo "formal launch exited during readiness" >&2; exit 4; }
  kill -0 "$nav_pid" 2>/dev/null || { echo "navigation exited during readiness" >&2; exit 4; }
  kill -0 "$coverage_pid" 2>/dev/null || { echo "coverage launch exited during readiness" >&2; exit 4; }
  sleep 1
done
[[ -f "$OUTPUT/cleaning_bridge_ready.json" ]] || { echo "cleaning actuators never became ready" >&2; exit 4; }
ros2 action list 2>/dev/null | grep -q '^/compute_coverage_path$' || { echo "coverage action unavailable" >&2; exit 4; }
ros2 topic list 2>/dev/null | grep -q '^/ground_truth/odom$' || { echo "evaluation ground truth unavailable" >&2; exit 4; }

setsid ros2 bag record --storage mcap --output "$OUTPUT/bag" \
  /clock /ground_truth/odom /odom /amcl_pose /localization/fused_pose \
  /scan /scan/navigation /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_gate \
  /base_controller/cmd_vel /joint_states /safety/status /brush_enabled \
  /safety/command/brush /brush_controller/commands \
  /cleaning_controller/joint_trajectory /coverage/state \
  /coverage/component_state /coverage/current_path \
  /model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json \
  >"$OUTPUT/bag.log" 2>&1 &
recorder_pid=$!
sleep 5

set +e
setsid timeout --signal=TERM --kill-after=10s 900s \
  python3 -m sanitation_coverage.coverage_probe --ros-args \
  -p config_path:="$OUTPUT/coverage_config.yaml" \
  -p output_path:="$OUTPUT/coverage_report.json" \
  -p path_output_path:="$OUTPUT/coverage_path.json" \
  -p trajectory_output_path:="$OUTPUT/coverage_trajectory.csv" \
  -p component_retry_limit:=1 \
  -p minimum_component_timeout_sec:=45.0 \
  -p rotation_timeout_sec:=45.0 \
  -p translation_timeout_sec:=45.0 \
  >"$OUTPUT/coverage_probe.log" 2>&1 &
probe_pid=$!
wait "$probe_pid"
probe_rc=$?
set -e
printf '%s\n' "$probe_rc" >"$OUTPUT/probe.rc"
probe_pid=""

touch "$OUTPUT/cleaning_bridge.stop"
for _ in {1..200}; do
  kill -0 "$cleaning_bridge_pid" 2>/dev/null || break
  sleep 0.1
done
if kill -0 "$cleaning_bridge_pid" 2>/dev/null; then
  echo "cleaning bridge did not release" >&2
  exit 5
fi
wait "$cleaning_bridge_pid" || true
cleaning_bridge_pid=""

kill -TERM -- "-$recorder_pid" 2>/dev/null || true
for _ in {1..100}; do
  kill -0 "$recorder_pid" 2>/dev/null || break
  sleep 0.1
done
if kill -0 "$recorder_pid" 2>/dev/null; then
  echo "recorder did not stop" >&2
  exit 5
fi
recorder_pid=""
timeout --signal=TERM --kill-after=5s 15s ros2 bag info "$OUTPUT/bag" \
  >"$OUTPUT/bag_info.txt" 2>&1 || true

set +e
python3 "$SUMMARIZER" \
  --coverage-report "$OUTPUT/coverage_report.json" \
  --coverage-trajectory "$OUTPUT/coverage_trajectory.csv" \
  --cleaning-status "$OUTPUT/cleaning_bridge.json" \
  --output "$OUTPUT/bounded_coverage_result.json" \
  >"$OUTPUT/summary.log" 2>&1
summary_rc=$?
set -e
printf '%s\n' "$summary_rc" >"$OUTPUT/summary.rc"

if (( probe_rc != 0 )); then exit "$probe_rc"; fi
exit "$summary_rc"
