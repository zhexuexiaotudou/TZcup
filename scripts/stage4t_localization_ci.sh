#!/usr/bin/env bash
set -euo pipefail
PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; WS="${SANITATION_WS:?SANITATION_WS required}"
MAP_ROOT="${TZCUP_MAP_ROOT:?TZCUP_MAP_ROOT must contain selected_map.yaml}"
WORLD_TO_MAP_X=8.0; WORLD_TO_MAP_Y=0.0; WORLD_TO_MAP_YAW=0.0
INITIAL_POSE_X=0.0; INITIAL_POSE_Y=0.0; INITIAL_POSE_YAW=0.0
STAMP="$(date +%Y%m%d_%H%M%S)"; OUT="${STAGE4T_OUT:-$PACK_ROOT/artifacts/stage4t_localization_$STAMP}"
SEEDS="${TZCUP_LOCALIZATION_SEEDS:-1}"; LANE="${TZCUP_LOCALIZATION_LANE:-realistic}"
PIDS=(); mkdir -p "$OUT/localization_trials"
SEED_START="${SEED_START:-0}"; SEED_STEP="${SEED_STEP:-1}"
stop_group(){ local pid="${1:-}"; [[ -n "$pid" ]] || return; kill -INT -- "-$pid" 2>/dev/null || true; for _ in {1..100}; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return; fi; sleep 0.1; done; kill -TERM -- "-$pid" 2>/dev/null || true; for _ in {1..100}; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid" 2>/dev/null || true; return; fi; sleep 0.1; done; kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; }
stop_bag(){ local pid="$1" log="$2"; if timeout 60s ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop '{}' > "$log" 2>&1; then kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; else stop_group "$pid"; fi; }
cleanup(){ for pid in "${PIDS[@]:-}"; do stop_group "$pid"; done; }; trap cleanup EXIT
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
if [[ "${SKIP_BUILD:-false}" != true ]]; then
  rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"; cd "$WS"
  colcon build --packages-select-regex '^sanitation_' --symlink-install --event-handlers console_direct+ > "$OUT/build.log" 2>&1
fi
set +u; source "$WS/install/setup.bash"; set -u
for ((seed=SEED_START; seed<SEEDS; seed+=SEED_STEP)); do
  trial="$OUT/localization_trials/seed_$seed"; mkdir -p "$trial"
  enable_ekf=true; [[ "$LANE" == oracle ]] && enable_ekf=false
  setsid ros2 launch sanitation_bringup sim.launch.py gui:=false headless_rendering:=true \
    drive_wheel_radius:=0.14 drive_wheel_separation:=1.22 random_seed:="$seed" enable_ekf:="$enable_ekf" \
    world_to_map_x:="$WORLD_TO_MAP_X" world_to_map_y:="$WORLD_TO_MAP_Y" world_to_map_yaw:="$WORLD_TO_MAP_YAW" \
    > "$trial/simulation.log" 2>&1 & SIM_PID=$!; PIDS+=("$SIM_PID")
  if [[ "$LANE" == oracle ]]; then
    setsid ros2 run sanitation_tasks sanitation_oracle_odom_adapter > "$trial/oracle_adapter.log" 2>&1 & ORACLE_PID=$!; PIDS+=("$ORACLE_PID")
  fi
  setsid ros2 launch sanitation_navigation navigation.launch.py rviz:=false \
    map_file:="$MAP_ROOT/selected_map.yaml" keepout_map:="$MAP_ROOT/filters/keepout_mask.yaml" speed_map:="$MAP_ROOT/filters/speed_mask.yaml" \
    operational_profile:=localization_coverage max_linear_velocity:=0.45 max_angular_velocity:=0.35 \
    initial_pose_x:="$INITIAL_POSE_X" initial_pose_y:="$INITIAL_POSE_Y" initial_pose_yaw:="$INITIAL_POSE_YAW" \
    > "$trial/navigation.log" 2>&1 & NAV_PID=$!; PIDS+=("$NAV_PID")
  timeout 120s ros2 topic echo --once /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped > "$trial/first_amcl.txt" || true
  setsid ros2 bag record --storage mcap --output "$trial/localization_bag" \
    /clock /scan /odom /amcl_pose /particle_cloud /ground_truth/odom /tf /tf_static /cmd_vel_gate /cmd_vel \
    > "$trial/rosbag.log" 2>&1 & BAG_PID=$!; PIDS+=("$BAG_PID")
  setsid ros2 run sanitation_tasks sanitation_localization_evaluator --ros-args \
    -p use_sim_time:=true -p duration_sec:=120.0 -p rmse_threshold_m:=0.05 \
    -p output_path:="$trial/localization_report.json" -p csv_path:="$trial/localization_trajectory.csv" \
    -p plot_path:="$trial/localization_error.png" > "$trial/evaluator.log" 2>&1 & EVAL_PID=$!; PIDS+=("$EVAL_PID")
  set +e
  timeout 360s ros2 run sanitation_tasks sanitation_navigation_probe --ros-args \
    -p use_sim_time:=true -p timeout_sec:=300.0 -p output_path:="$trial/navigation_probe.json" \
    -p initial_pose_x:="$INITIAL_POSE_X" -p initial_pose_y:="$INITIAL_POSE_Y" -p initial_pose_yaw:="$INITIAL_POSE_YAW" \
    > "$trial/navigation_probe.log" 2>&1
  NAV_CODE=$?
  wait "$EVAL_PID"; EVAL_CODE=$?; set -e
  stop_bag "$BAG_PID" "$trial/rosbag_stop.log"; stop_group "$NAV_PID"; [[ "$LANE" != oracle ]] || stop_group "$ORACLE_PID"; stop_group "$SIM_PID"; PIDS=()
  ros2 bag info "$trial/localization_bag" > "$trial/rosbag_info.txt"
  python3 - "$trial" "$NAV_CODE" "$EVAL_CODE" <<'PY'
import json, sys
from pathlib import Path
root = Path(sys.argv[1]); localization = json.loads((root / "localization_report.json").read_text(encoding="utf-8")); navigation = json.loads((root / "navigation_probe.json").read_text(encoding="utf-8"))
localization["navigation"] = navigation; localization["navigation_exit_code"] = int(sys.argv[2]); localization["evaluator_exit_code"] = int(sys.argv[3])
(root / "localization_report.json").write_text(json.dumps(localization, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY
done
if [[ "${SKIP_AGGREGATE:-false}" != true ]]; then
  output_name="realistic_localization_report.json"; [[ "$LANE" != oracle ]] || output_name="oracle_localization_report.json"
  python3 "$PACK_ROOT/scripts/stage4t_localization_aggregate.py" "$OUT/localization_trials" "$OUT/$output_name" --lane "$LANE" --required-seeds 10
fi
echo "$OUT"
