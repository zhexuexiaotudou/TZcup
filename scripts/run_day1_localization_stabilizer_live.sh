#!/usr/bin/env bash
# One bounded live replay for the causal map->odom stabilizer.
set -eo pipefail
: "${SOURCE:?}" "${RUNTIME:?}" "${OUTPUT:?}" "${EPISODE:?}" "${MAP_SOURCE:?}" "${DRIVER:?}" "${CANDIDATE_REVISION:?}"

export PROBE_DOMAIN="${PROBE_DOMAIN:-83}"
export PROBE_PARTITION="${PROBE_PARTITION:-tzcup_localization_stabilizer_20260915_01}"
export PROBE_AMCL_TF_BROADCAST=false
PROBE_SECONDS="${PROBE_SECONDS:-120}"
PROBE_PREPARE_SECONDS="${PROBE_PREPARE_SECONDS:-420}"
STABILIZER_TAU_SEC="${STABILIZER_TAU_SEC:-1.5}"
STABILIZER_MAX_DT_SEC="${STABILIZER_MAX_DT_SEC:-0.1}"
STABILIZER_MAX_GAP_SEC="${STABILIZER_MAX_GAP_SEC:-0.5}"
[[ "${PROBE_SECONDS}" =~ ^[1-9][0-9]*$ && "${PROBE_PREPARE_SECONDS}" =~ ^[0-9]+$ ]] || exit 2

source /opt/ros/jazzy/setup.bash
source "$RUNTIME/install/setup.bash"
if [[ -n "${COMPETITION_RUNTIME_OVERLAY:-}" ]]; then
  source "$COMPETITION_RUNTIME_OVERLAY/install/local_setup.bash"
fi
if [[ -n "${LOCALIZATION_COLLECTOR_OVERLAY:-}" ]]; then
  source "$LOCALIZATION_COLLECTOR_OVERLAY/install/local_setup.bash"
fi
LOCALIZATION_SCORE_DEPS_DIR="${LOCALIZATION_SCORE_DEPS_DIR:-$SOURCE/.work/localization-score-deps}"
export PYTHONPATH="$LOCALIZATION_SCORE_DEPS_DIR${PYTHONPATH:+:$PYTHONPATH}"
python3 -c 'import mcap, mcap_ros2' || {
  echo "localization focus scorer dependencies are not available" >&2
  exit 2
}
ros2 pkg executables sanitation_localization | grep -qx "sanitation_localization map_odom_stabilizer" || {
  echo "live stabilizer executable is not installed in the selected overlay" >&2
  exit 2
}
ros2 pkg executables sanitation_localization_acceptance | grep -qx "sanitation_localization_acceptance formal_localization_runtime_collector" || {
  echo "localization runtime collector is not installed in the selected overlay" >&2
  exit 2
}
source "$SOURCE/scripts/run_formal_runtime_isolation.sh"
export TZCUP_REPOSITORY_ROOT="$SOURCE" ROS_DOMAIN_ID="${PROBE_DOMAIN}" GZ_PARTITION="${PROBE_PARTITION}"
FORMAL_GAZEBO_LOCK_FILE="${FORMAL_GAZEBO_LOCK_FILE:-/tmp/tzcup_formal_gazebo.lock}" formal_runtime_configure "$ROS_DOMAIN_ID"

mkdir "$OUTPUT"
python3 - "$OUTPUT/run_contract.json" "$PROBE_SECONDS" "$PROBE_PREPARE_SECONDS" <<'PY'
import json, os, pathlib, sys
payload = {
    "schema_version": 1,
    "runner": "run_day1_localization_stabilizer_live.sh",
    "status": "ADMITTED",
    "ros_domain_id": int(os.environ["PROBE_DOMAIN"]),
    "gazebo_partition": os.environ["PROBE_PARTITION"],
    "motion_seconds": int(sys.argv[2]),
    "prepare_seconds": int(sys.argv[3]),
    "wall_budget_seconds": int(sys.argv[2]) + int(sys.argv[3]) + 180,
    "candidate_revision": os.environ["CANDIDATE_REVISION"],
    "causal_filter_tau_sec": 1.5,
    "uses_ground_truth_for_control": False,
    "rollback": "set map_odom_stabilizer:=false or return to 47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc",
}
pathlib.Path(sys.argv[1]).write_text(json.dumps(payload, indent=2) + "\n")
PY

cp "$MAP_SOURCE"/{occupancy.pgm,occupancy.yaml,geofence_keepout.pgm,geofence_keepout.yaml,neutral_speed.pgm,neutral_speed.yaml,mission_geometry.yaml,materialization_contract.yaml} "$OUTPUT/"
python3 - <<'PY'
import os,pathlib,yaml
source=pathlib.Path(os.environ['SOURCE']);out=pathlib.Path(os.environ['OUTPUT'])
nav=yaml.safe_load((source/'starter_ws/src/sanitation_navigation/config/nav2.yaml').read_text())
for name in ('controller_server','velocity_smoother','collision_monitor'):
    nav[name]['ros__parameters']['enable_stamped_cmd_vel']=False
nav['amcl']['ros__parameters']['scan_topic']='/scan/navigation'
nav['amcl']['ros__parameters']['tf_broadcast']=False
collision=nav['collision_monitor']['ros__parameters']
collision['observation_sources']=['scan'];collision.pop('mid360',None)
collision['scan']['topic']='/scan/navigation'
for name in ('local_costmap','global_costmap'):
    obstacle=nav[name][name]['ros__parameters']['obstacle_layer']
    obstacle['observation_sources']='scan';obstacle.pop('mid360',None)
    obstacle['scan']['topic']='/scan/navigation'
(out/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
PY

launch_pid=''; nav_pid=''; filter_pid=''; bridge_pid=''; recorder_pid=''; authority_pid=''
finalized=0
cleanup() {
  [[ "$finalized" == 0 ]] || return 0
  finalized=1
  touch "$OUTPUT/authority.stop" 2>/dev/null || true
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -TERM -- "-$recorder_pid" 2>/dev/null || true
    for _ in {1..100}; do kill -0 "$recorder_pid" 2>/dev/null || break; sleep 0.1; done
  fi
  if [[ -n "$authority_pid" ]]; then wait "$authority_pid" 2>/dev/null || true; authority_pid=''; fi
  local cleanup_status=0
  formal_runtime_cleanup_groups "$GZ_PARTITION" "$filter_pid" "$bridge_pid" "$nav_pid" "$launch_pid" || cleanup_status=$?
  exec 9>&-
  local release_status=0
  flock -n "$FORMAL_RUNTIME_LOCK_FILE" true || release_status=$?
  python3 - "$OUTPUT/resource_release.json" "$cleanup_status" "$release_status" "$GZ_PARTITION" "$ROS_DOMAIN_ID" <<'PY'
import hashlib,json,os,pathlib,subprocess,sys
cleanup_status=int(sys.argv[2]); release_status=int(sys.argv[3]); partition=sys.argv[4]; domain=sys.argv[5]
needle=("GZ_PARTITION="+partition).encode()
remaining=[]
for raw in pathlib.Path('/proc').iterdir():
    if not raw.name.isdigit() or raw.name==str(os.getpid()): continue
    try: env=(raw/'environ').read_bytes().split(b'\0')
    except OSError: continue
    if needle in env: remaining.append(int(raw.name))
payload={
  "schema_version":1,
  "status":"RELEASED" if cleanup_status==0 and release_status==0 and not remaining else "BLOCKED",
  "cleanup_status":cleanup_status,
  "gazebo_process_remaining":bool(remaining),
  "remaining_pids":remaining,
  "ros_runtime_process_remaining":bool(remaining),
  "formal_lock_available":release_status==0,
  "ros_domain_id":int(domain),
  "gazebo_partition":partition,
}
pathlib.Path(sys.argv[1]).write_text(json.dumps(payload,indent=2)+"\n")
PY
}
trap cleanup EXIT INT TERM

setsid ros2 launch sanitation_formal_campus_integration formal_campus.launch.py \
  gui:=false world:="$EPISODE/public/world.sdf" world_name:=campus_formal \
  episode_manifest:="$EPISODE/public/episode_manifest.json" \
  pedestrian_schedule:="$EPISODE/environment/pedestrian_schedule.json" \
  start_pedestrians:=false high_bandwidth_sensor_runtime:=false enable_training_gt:=false \
  runtime_artifact_dir:="$OUTPUT" materialize_static_maps:=false \
  start_navigation:=false start_coverage:=false localization_backend:=amcl \
  mission_mode:=cleaning operation_speed_profile:=dry_cleaning_competition_candidate \
  map_odom_stabilizer:=true \
  map_odom_stabilizer_tau_sec:="$STABILIZER_TAU_SEC" \
  map_odom_stabilizer_max_dt_sec:="$STABILIZER_MAX_DT_SEC" \
  map_odom_stabilizer_max_gap_sec:="$STABILIZER_MAX_GAP_SEC" \
  >"$OUTPUT/launch.log" 2>&1 & launch_pid=$!
setsid ros2 run sanitation_formal_campus_integration formal-scan-self-filter --ros-args \
  --params-file "$SOURCE/starter_ws/src/sanitation_formal_campus_integration/config/formal_utm30lx_self_filter.yaml" \
  -p use_sim_time:=true >"$OUTPUT/filter.log" 2>&1 & filter_pid=$!
setsid ros2 run ros_gz_bridge parameter_bridge \
  '/model/tzcup_formal_sanitation_vehicle/ground_dirt/command/enable@std_msgs/msg/Bool]gz.msgs.Boolean' \
  '/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json@std_msgs/msg/String[gz.msgs.StringMsg' \
  >"$OUTPUT/dirt_bridge.log" 2>&1 & bridge_pid=$!
setsid ros2 launch sanitation_navigation navigation.launch.py \
  use_sim_time:=true rviz:=false params_file:="$OUTPUT/nav2.yaml" map_file:="$OUTPUT/occupancy.yaml" \
  keepout_map:="$OUTPUT/geofence_keepout.yaml" speed_map:="$OUTPUT/neutral_speed.yaml" \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0 \
  localization_backend:=amcl start_velocity_gate:=false >"$OUTPUT/navigation.log" 2>&1 & nav_pid=$!
sleep 40

timeout 25s ros2 param dump /amcl >"$OUTPUT/amcl.params.yaml"
timeout 25s ros2 param dump /local_ekf >"$OUTPUT/local_ekf.params.yaml"
timeout 25s ros2 param dump /global_ekf >"$OUTPUT/global_ekf.params.yaml"
python3 "$SOURCE/scripts/capture_competition_localization_parameters.py" \
  --map-odom-owner /map_odom_stabilizer --output "$OUTPUT/effective_parameters.json" \
  >"$OUTPUT/effective_parameters.log" 2>&1
timeout 25s ros2 node info /map_odom_stabilizer >"$OUTPUT/map_odom_stabilizer.node.txt"
timeout 25s ros2 topic info /localization/raw_map_odom -v >"$OUTPUT/raw_map_odom.info.txt"
setsid ros2 run sanitation_localization_acceptance formal_localization_runtime_collector \
  --mode cleaning --output "$OUTPUT/tf_authority.json" --stop-file "$OUTPUT/authority.stop" \
  --duration-seconds "$(( PROBE_SECONDS + PROBE_PREPARE_SECONDS + 80 ))" \
  >"$OUTPUT/tf_authority.log" 2>&1 & authority_pid=$!
setsid ros2 bag record --storage mcap --output "$OUTPUT/bag" \
  /clock /ground_truth/model_odom_raw /odom /odometry/gps /scan /scan/navigation \
  /odom/unfiltered /imu/data /gnss/fix /amcl_pose /localization/fused_odom \
  /localization/raw_map_odom /localization/map_odom_stabilizer/status \
  /diagnostics /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_gate /base_controller/cmd_vel \
  /collision_monitor_state /joint_states /safety/status /tf /tf_static \
  >"$OUTPUT/bag.log" 2>&1 & recorder_pid=$!

set +e
timeout --signal=TERM --kill-after=10s "$(( PROBE_SECONDS + PROBE_PREPARE_SECONDS + 60 ))s" \
  python3 "$DRIVER" --output "$OUTPUT" --seconds "$PROBE_SECONDS" \
  --prepare-seconds "$PROBE_PREPARE_SECONDS" --goal-x "${PROBE_GOAL_X:-3}" \
  --estop-distance "${PROBE_ESTOP_DISTANCE:-0}" >"$OUTPUT/driver.log" 2>&1
driver_status=$?
set -e

timeout 15s ros2 topic echo --full-length --once /localization/map_odom_stabilizer/status \
  >"$OUTPUT/map_odom_stabilizer.status.yaml"
kill -TERM -- "-$recorder_pid" 2>/dev/null || true
for _ in {1..100}; do kill -0 "$recorder_pid" 2>/dev/null || break; sleep 0.1; done
if kill -0 "$recorder_pid" 2>/dev/null; then echo 'recorder did not stop' >&2; exit 4; fi
recorder_pid=''
timeout 15s ros2 bag info "$OUTPUT/bag" >"$OUTPUT/bag_info.txt" 2>&1
touch "$OUTPUT/authority.stop"
wait "$authority_pid"
authority_pid=''

set +e
python3 "$SOURCE/scripts/competition_localization_focus.py" \
  --bag-dir "$OUTPUT/bag" --authority "$OUTPUT/tf_authority.json" \
  --effective-parameters "$OUTPUT/effective_parameters.json" \
  --manifest "$EPISODE/public/episode_manifest.json" \
  --map-odom-owner /map_odom_stabilizer \
  --revision "$CANDIDATE_REVISION" --output "$OUTPUT/localization_focus.json"
focus_status=$?
set -e
echo "$driver_status" >"$OUTPUT/driver.rc"
echo "$focus_status" >"$OUTPUT/focus.rc"

cleanup
set +e
python3 "$SOURCE/scripts/validate_day1_localization_stabilizer_live.py" \
  --run-dir "$OUTPUT" --output "$OUTPUT/live_candidate_receipt.json"
validator_status=$?
set -e
echo "$validator_status" >"$OUTPUT/validator.rc"
exit "$driver_status"
