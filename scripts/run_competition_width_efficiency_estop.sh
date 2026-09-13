#!/usr/bin/env bash
# Competition-only diagnostic. Existing built runtime; no formal gate claim.
set -eo pipefail
: "${SOURCE:?}" "${RUNTIME:?}" "${OUTPUT:?}" "${EPISODE:?}" "${MAP_SOURCE:?}" "${DRIVER:?}"
source /opt/ros/jazzy/setup.bash
source "$RUNTIME/install/setup.bash"
source "${OVERLAY:?}/install/local_setup.bash"
source "$SOURCE/scripts/run_formal_runtime_isolation.sh"
export TZCUP_REPOSITORY_ROOT="$SOURCE" ROS_DOMAIN_ID="${PROBE_DOMAIN:-98}" GZ_PARTITION="${PROBE_PARTITION:-tzcup_motion_cleaning_20260913_02}"
formal_runtime_configure "$ROS_DOMAIN_ID"
mkdir "$OUTPUT"
cp "$MAP_SOURCE"/{occupancy.pgm,occupancy.yaml,geofence_keepout.pgm,geofence_keepout.yaml,neutral_speed.pgm,neutral_speed.yaml,mission_geometry.yaml,materialization_contract.yaml} "$OUTPUT/"
python3 - <<'PY'
import os,pathlib,yaml
source=pathlib.Path(os.environ['SOURCE']);out=pathlib.Path(os.environ['OUTPUT'])
nav=yaml.safe_load((source/'starter_ws/src/sanitation_navigation/config/nav2.yaml').read_text())
for name in ('controller_server','velocity_smoother','collision_monitor'):
    nav[name]['ros__parameters']['enable_stamped_cmd_vel']=False
nav['amcl']['ros__parameters']['scan_topic']='/scan/navigation'
collision=nav['collision_monitor']['ros__parameters']
collision['observation_sources']=['scan'];collision.pop('mid360',None)
collision['scan']['topic']='/scan/navigation'
for name in ('local_costmap','global_costmap'):
    obstacle=nav[name][name]['ros__parameters']['obstacle_layer']
    obstacle['observation_sources']='scan';obstacle.pop('mid360',None)
    obstacle['scan']['topic']='/scan/navigation'
(out/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
PY
launch_pid=''; nav_pid=''; filter_pid=''; bridge_pid=''; recorder_pid=''; perception_pid=''
cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -TERM -- "-$recorder_pid" 2>/dev/null || true
    for _ in {1..100}; do kill -0 "$recorder_pid" 2>/dev/null || break; sleep 0.1; done
  fi
  formal_runtime_cleanup_groups "$GZ_PARTITION" "$perception_pid" "$nav_pid" "$filter_pid" "$bridge_pid" "$launch_pid" || true
}
trap cleanup EXIT INT TERM
setsid ros2 launch sanitation_formal_campus_integration formal_campus.launch.py \
  gui:=false world:="$EPISODE/public/world.sdf" world_name:=campus_formal \
  episode_manifest:="$EPISODE/public/episode_manifest.json" \
  pedestrian_schedule:="$EPISODE/environment/pedestrian_schedule.json" start_pedestrians:=false \
  high_bandwidth_sensor_runtime:="${PROBE_CAMERAS:-false}" enable_training_gt:=false \
  runtime_artifact_dir:="$OUTPUT" materialize_static_maps:=false \
  start_navigation:=false start_coverage:=false localization_backend:=amcl \
  mission_mode:=cleaning operation_speed_profile:=dry_cleaning_competition_candidate \
  max_linear_velocity:=1.0 speed_qualification_state:=isolated_same_map_dry_coverage \
  >"$OUTPUT/launch.log" 2>&1 & launch_pid=$!
setsid ros2 run sanitation_formal_campus_integration formal-scan-self-filter --ros-args \
  --params-file "$SOURCE/starter_ws/src/sanitation_formal_campus_integration/config/formal_utm30lx_self_filter.yaml" \
  -p use_sim_time:=true >"$OUTPUT/filter.log" 2>&1 & filter_pid=$!
setsid ros2 run ros_gz_bridge parameter_bridge \
  '/model/tzcup_formal_sanitation_vehicle/ground_dirt/command/enable@std_msgs/msg/Bool]gz.msgs.Boolean' \
  '/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json@std_msgs/msg/String[gz.msgs.StringMsg' \
  '/world/campus_formal/model/tzcup_formal_sanitation_vehicle/link/left_side_brush_link/sensor/left_side_brush_ground_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' \
  '/world/campus_formal/model/tzcup_formal_sanitation_vehicle/link/right_side_brush_link/sensor/right_side_brush_ground_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' \
  '/world/campus_formal/model/tzcup_formal_sanitation_vehicle/link/central_roller_link/sensor/central_roller_ground_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' \
  --ros-args -r /world/campus_formal/model/tzcup_formal_sanitation_vehicle/link/left_side_brush_link/sensor/left_side_brush_ground_contact/contact:=/cleaning/left_side_brush/contact -r /world/campus_formal/model/tzcup_formal_sanitation_vehicle/link/right_side_brush_link/sensor/right_side_brush_ground_contact/contact:=/cleaning/right_side_brush/contact -r /world/campus_formal/model/tzcup_formal_sanitation_vehicle/link/central_roller_link/sensor/central_roller_ground_contact/contact:=/cleaning/central_roller/contact \
  >"$OUTPUT/dirt_bridge.log" 2>&1 & bridge_pid=$!
sleep 40
if [[ -n "${PERCEPTION_LAUNCH:-}" ]]; then
  : "${PERCEPTION_MODEL:?}" "${PERCEPTION_PYTHONPATH:?}"
  setsid env PYTHONPATH="$PERCEPTION_PYTHONPATH:${PYTHONPATH:-}" \
    ros2 launch "$PERCEPTION_LAUNCH" model_path:="$PERCEPTION_MODEL" \
    >"$OUTPUT/perception.log" 2>&1 & perception_pid=$!
fi
setsid ros2 bag record --storage mcap --output "$OUTPUT/bag" \
  /cleaning/left_side_brush/contact /cleaning/right_side_brush/contact /cleaning/central_roller/contact \
  /clock /ground_truth/model_odom_raw /odom /odometry/gps /scan /scan/navigation \
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_gate /base_controller/cmd_vel \
  /collision_monitor_state /joint_states /safety/status /tf /tf_static \
  /safety/command/brush /brush_controller/commands /cleaning_controller/joint_trajectory \
  /formal_vehicle/simulation/command/emergency_stop \
  /perception/garbage/targets /perception/garbage/diagnostics \
  /model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json \
  >"$OUTPUT/bag.log" 2>&1 & recorder_pid=$!
set +e
timeout --signal=TERM --kill-after=10s 930s python3 "$DRIVER" --output "$OUTPUT" --mode "${MODE:?}" >"$OUTPUT/driver.log" 2>&1
result=$?
set -e
kill -TERM -- "-$recorder_pid" 2>/dev/null || true
for _ in {1..100}; do kill -0 "$recorder_pid" 2>/dev/null || break; sleep 0.1; done
if kill -0 "$recorder_pid" 2>/dev/null; then echo 'recorder did not stop' >&2; exit 4; fi
recorder_pid=''
timeout --signal=TERM --kill-after=5s 15s ros2 bag info "$OUTPUT/bag" >"$OUTPUT/bag_info.txt" 2>&1
echo "$result" >"$OUTPUT/probe.rc"
exit "$result"
