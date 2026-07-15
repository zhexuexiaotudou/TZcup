#!/usr/bin/env bash
set -euo pipefail
PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; WS="${SANITATION_WS:?SANITATION_WS required}"
STAMP="$(date +%Y%m%d_%H%M%S)"; OUT="${STAGE4T_OUT:-$PACK_ROOT/artifacts/stage4t_mapping_$STAMP}"
PIDS=(); mkdir -p "$OUT"
stop_group(){ local pid="${1:-}"; [[ -n "$pid" ]] || return; kill -INT -- "-$pid" 2>/dev/null || true; for _ in {1..100}; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return; fi; sleep 0.1; done; kill -TERM -- "-$pid" 2>/dev/null || true; for _ in {1..100}; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return; fi; sleep 0.1; done; kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; }
stop_bag(){ local pid="$1" log="$2"; if timeout 60s ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop '{}' > "$log" 2>&1; then kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; else stop_group "$pid"; fi; }
cleanup(){ for pid in "${PIDS[@]:-}"; do stop_group "$pid"; done; }; trap cleanup EXIT
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"; cd "$WS"
colcon build --packages-select-regex '^sanitation_' --symlink-install --event-handlers console_direct+ > "$OUT/build.log" 2>&1
set +u; source "$WS/install/setup.bash"; set -u

for label in ${TZCUP_MAPPING_LABELS:-005 002}; do
  trial="$OUT/map_$label"; mkdir -p "$trial"
  params="$WS/install/sanitation_navigation/share/sanitation_navigation/config/slam.yaml"
  [[ "$label" == 002 ]] && params="$WS/install/sanitation_navigation/share/sanitation_navigation/config/slam_002.yaml"
  if [[ -f "$trial/mapping_probe.json" && -f "$trial/slam_map.yaml" && -f "$trial/mapping_bag/metadata.yaml" ]]; then
    ros2 bag info "$trial/mapping_bag" > "$trial/rosbag_info.txt"
    if [[ ! -f "$trial/map_quality.json" ]]; then
      ros2 run sanitation_tasks sanitation_map_quality \
        --map-yaml "$trial/slam_map.yaml" --output "$trial/map_quality.json" --preview "$trial/map_preview.png"
    fi
    if [[ ! -f "$trial/map_geometry.json" ]]; then
      python3 "$PACK_ROOT/scripts/stage4t_map_geometry.py" --map-yaml "$trial/slam_map.yaml" \
        --world-sdf "$PACK_ROOT/starter_ws/src/sanitation_worlds/worlds/sanitation_test_world.sdf" \
        --output "$trial/map_geometry.json" --overlay "$trial/map_truth_overlay.png"
    fi
    continue
  fi
  setsid ros2 launch sanitation_bringup sim.launch.py gui:=false headless_rendering:=true \
    drive_wheel_radius:=0.14 drive_wheel_separation:=1.22 random_seed:=0 \
    > "$trial/simulation.log" 2>&1 & SIM_PID=$!; PIDS+=("$SIM_PID")
  setsid ros2 launch sanitation_navigation slam.launch.py rviz:=false params_file:="$params" \
    > "$trial/slam.log" 2>&1 & SLAM_PID=$!; PIDS+=("$SLAM_PID")
  timeout 90s ros2 topic echo --once /ground_truth/odom nav_msgs/msg/Odometry > "$trial/first_truth.txt"
  timeout 120s ros2 topic echo --once /map nav_msgs/msg/OccupancyGrid > "$trial/first_map.txt"
  setsid ros2 bag record --storage mcap --output "$trial/mapping_bag" \
    /clock /scan /odom /odom/unfiltered /measurements/wheel_odom /measurements/imu \
    /ground_truth/odom /map /tf /tf_static /cmd_vel_gate /cmd_vel \
    > "$trial/rosbag.log" 2>&1 & BAG_PID=$!; PIDS+=("$BAG_PID")
  set +e
  timeout 900s ros2 run sanitation_tasks sanitation_mapping_probe --ros-args \
    -p use_sim_time:=true -p feedback_topic:=/odom -p command_topic:=/cmd_vel_gate \
    -p max_linear_speed:=0.30 -p max_angular_speed:=0.25 -p timeout_sec:=780.0 \
    -p output_path:="$trial/mapping_probe.json" -p trajectory_path:="$trial/mapping_trajectory.csv" \
    > "$trial/mapping_probe.log" 2>&1
  MAPPING_CODE=$?
  ros2 run nav2_map_server map_saver_cli -f "$trial/slam_map" --ros-args \
    -p use_sim_time:=true -p save_map_timeout:=30.0 > "$trial/map_save.log" 2>&1
  MAP_SAVE_CODE=$?
  set -e
  ros2 run sanitation_tasks sanitation_image_capture --ros-args -p output_path:="$trial/gazebo_mapping.png" > "$trial/image_capture.log" 2>&1 || true
  stop_bag "$BAG_PID" "$trial/rosbag_stop.log"; stop_group "$SLAM_PID"; stop_group "$SIM_PID"; PIDS=()
  ros2 bag info "$trial/mapping_bag" > "$trial/rosbag_info.txt"
  set +e
  ros2 run sanitation_tasks sanitation_map_quality \
    --map-yaml "$trial/slam_map.yaml" --output "$trial/map_quality.json" --preview "$trial/map_preview.png"
  QUALITY_CODE=$?
  set -e
  python3 "$PACK_ROOT/scripts/stage4t_map_geometry.py" --map-yaml "$trial/slam_map.yaml" \
    --world-sdf "$PACK_ROOT/starter_ws/src/sanitation_worlds/worlds/sanitation_test_world.sdf" \
    --output "$trial/map_geometry.json" --overlay "$trial/map_truth_overlay.png"
  python3 - "$trial" "$MAPPING_CODE" "$MAP_SAVE_CODE" "$QUALITY_CODE" <<'PY'
import json, sys
from pathlib import Path
root=Path(sys.argv[1]); report=json.loads((root/'mapping_probe.json').read_text(encoding='utf-8')); report['exit_codes']={'mapping_probe':int(sys.argv[2]),'map_save':int(sys.argv[3]),'map_quality':int(sys.argv[4])}; (root/'mapping_probe.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
PY
done
if [[ "${SKIP_AGGREGATE:-false}" != true ]]; then
  python3 "$PACK_ROOT/scripts/stage4t_map_aggregate.py" "$OUT"
  python3 "$PACK_ROOT/scripts/generate_stage4r_masks.py" --map-yaml "$OUT/selected_map.yaml" --output-dir "$OUT/filters"
fi
echo "$OUT"
