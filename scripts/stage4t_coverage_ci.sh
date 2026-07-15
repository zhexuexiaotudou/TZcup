#!/usr/bin/env bash
set -euo pipefail
PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; WS="${SANITATION_WS:?SANITATION_WS required}"; MAP_ROOT="${TZCUP_MAP_ROOT:?map root required}"
STAMP="$(date +%Y%m%d_%H%M%S)"; OUT="${STAGE4T_OUT:-$PACK_ROOT/artifacts/stage4t_coverage_$STAMP}"; PIDS=(); mkdir -p "$OUT"
WORLD_TO_MAP_X=8.0; WORLD_TO_MAP_Y=0.0; WORLD_TO_MAP_YAW=0.0
INITIAL_POSE_X=0.0; INITIAL_POSE_Y=0.0; INITIAL_POSE_YAW=0.0
stop_group(){ local pid="${1:-}"; [[ -n "$pid" ]] || return; kill -INT -- "-$pid" 2>/dev/null || true; for _ in {1..100}; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return; fi; sleep 0.1; done; kill -TERM -- "-$pid" 2>/dev/null || true; for _ in {1..100}; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return; fi; sleep 0.1; done; kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; }
stop_bag(){ local pid="$1" log="$2"; if timeout 60s ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop '{}' > "$log" 2>&1; then kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; else stop_group "$pid"; fi; }
cleanup(){ for pid in "${PIDS[@]:-}"; do stop_group "$pid"; done; }; trap cleanup EXIT
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"; cd "$WS"; colcon build --packages-select-regex '^sanitation_' --symlink-install --event-handlers console_direct+ > "$OUT/build.log" 2>&1
set +u; source "$WS/install/setup.bash"; set -u
setsid ros2 launch sanitation_bringup sim.launch.py gui:=false headless_rendering:=true drive_wheel_radius:=0.14 drive_wheel_separation:=1.22 world_to_map_x:="$WORLD_TO_MAP_X" world_to_map_y:="$WORLD_TO_MAP_Y" world_to_map_yaw:="$WORLD_TO_MAP_YAW" > "$OUT/simulation.log" 2>&1 & SIM_PID=$!; PIDS+=("$SIM_PID")
setsid ros2 launch sanitation_navigation navigation.launch.py rviz:=false map_file:="$MAP_ROOT/selected_map.yaml" keepout_map:="$MAP_ROOT/filters/keepout_mask.yaml" speed_map:="$MAP_ROOT/filters/speed_mask.yaml" operational_profile:=localization_coverage max_linear_velocity:=0.45 max_angular_velocity:=0.35 initial_pose_x:="$INITIAL_POSE_X" initial_pose_y:="$INITIAL_POSE_Y" initial_pose_yaw:="$INITIAL_POSE_YAW" > "$OUT/navigation.log" 2>&1 & NAV_PID=$!; PIDS+=("$NAV_PID")
setsid ros2 launch sanitation_coverage coverage.launch.py > "$OUT/coverage_server.log" 2>&1 & SERVER_PID=$!; PIDS+=("$SERVER_PID")
timeout 120s ros2 topic echo --once /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped > "$OUT/first_amcl.txt" || true
ros2 run sanitation_tasks sanitation_image_capture --ros-args -p output_path:="$OUT/gazebo_initial.png" > "$OUT/image_initial.log" 2>&1
setsid ros2 bag record --storage mcap --output "$OUT/full_mission_bag" /clock /scan /odom /odom/unfiltered /measurements/wheel_odom /measurements/imu /amcl_pose /particle_cloud /ground_truth/odom /tf /tf_static /cmd_vel_gate /cmd_vel /brush_enabled /emergency_stop /collision_monitor_state /speed_limit > "$OUT/rosbag.log" 2>&1 & BAG_PID=$!; PIDS+=("$BAG_PID")
setsid ros2 run sanitation_tasks sanitation_dynamic_obstacle_probe --ros-args -p use_sim_time:=true -p output_path:="$OUT/dynamic_obstacle_report.json" -p set_pose_script:="$PACK_ROOT/scripts/gz_set_dynamic_obstacle.sh" > "$OUT/dynamic_obstacle.log" 2>&1 & DYNAMIC_PID=$!; PIDS+=("$DYNAMIC_PID")
set +e
timeout 1200s ros2 run sanitation_coverage coverage_probe --ros-args -p use_sim_time:=true -p output_path:="$OUT/coverage_report.json" -p path_output_path:="$OUT/coverage_path.json" -p trajectory_output_path:="$OUT/coverage_trajectory.csv" > "$OUT/coverage_probe.log" 2>&1
COVERAGE_CODE=$?
wait "$DYNAMIC_PID"; DYNAMIC_CODE=$?
ros2 run sanitation_tasks sanitation_image_capture --ros-args -p output_path:="$OUT/gazebo_coverage.png" > "$OUT/image_coverage.log" 2>&1
ros2 run sanitation_tasks sanitation_filter_probe --ros-args -p use_sim_time:=true -p output_path:="$OUT/filter_report.json" > "$OUT/filter_probe.log" 2>&1
FILTER_CODE=$?
ros2 run sanitation_tasks sanitation_safety_probe --ros-args -p use_sim_time:=true -p trial_count:=30 -p output_path:="$OUT/safety_latency_report.json" > "$OUT/safety_probe.log" 2>&1
SAFETY_CODE=$?; set -e
stop_bag "$BAG_PID" "$OUT/rosbag_stop.log"; stop_group "$SERVER_PID"; stop_group "$NAV_PID"; stop_group "$SIM_PID"; PIDS=()
ros2 bag info "$OUT/full_mission_bag" > "$OUT/rosbag_info.txt"
setsid ros2 bag play "$OUT/full_mission_bag" --rate 5.0 --topics /ground_truth/odom > "$OUT/rosbag_replay.log" 2>&1 & REPLAY_PID=$!; PIDS+=("$REPLAY_PID")
timeout 45s ros2 topic echo --once /ground_truth/odom nav_msgs/msg/Odometry > "$OUT/replay_ground_truth.txt"
stop_group "$REPLAY_PID"; PIDS=()
python3 "$PACK_ROOT/scripts/stage4t_coverage_aggregate.py" "$OUT"
python3 - "$OUT" "$COVERAGE_CODE" "$DYNAMIC_CODE" "$FILTER_CODE" "$SAFETY_CODE" <<'PY'
import json, sys
from pathlib import Path
root=Path(sys.argv[1]); report=json.loads((root/'stage4t_coverage_report.json').read_text(encoding='utf-8')); report['exit_codes']={'coverage':int(sys.argv[2]),'dynamic':int(sys.argv[3]),'filters':int(sys.argv[4]),'safety':int(sys.argv[5])}; (root/'stage4t_coverage_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
PY
echo "$OUT"
