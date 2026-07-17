#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_WS="${SANITATION_BASE_WS:?SANITATION_BASE_WS required}"
STAGE4V_WS="${SANITATION_STAGE4V_WS:?SANITATION_STAGE4V_WS required}"
WS="${SANITATION_WS:?SANITATION_WS required}"
OUT="${STAGE4W_OUT:?STAGE4W_OUT required}"
SEED="${STAGE4W_SEED:-10}"
mkdir -p "${OUT}"
pids=()

stop_group() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return
  kill -INT -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 100); do
    if ! kill -0 "${pid}" 2>/dev/null; then wait "${pid}" 2>/dev/null || true; return; fi
    sleep 0.1
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
cleanup() { for pid in "${pids[@]:-}"; do stop_group "${pid}"; done; }
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "${BASE_WS}/install/setup.bash"
source "${STAGE4V_WS}/install/setup.bash"
source "${WS}/install/setup.bash"
set -u

map_root="${WS}/install/sanitation_navigation/share/sanitation_navigation/maps"
nav_params="${WS}/install/sanitation_navigation/share/sanitation_navigation/config/nav2.yaml"
setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:="${SEED}" gnss_profile:=rtk_fixed \
  fusion_mode:=hybrid_rtk_scan_imu_wheel enable_scan_refiner:=true \
  > "${OUT}/localization.log" 2>&1 & pids+=("$!")
setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  map_file:="${map_root}/stage4v_surveyed_reference.yaml" \
  keepout_map:="${map_root}/stage4v_filters/keepout_mask.yaml" \
  speed_map:="${map_root}/stage4v_filters/speed_mask.yaml" \
  operational_profile:=localization_coverage max_linear_velocity:=0.45 \
  max_angular_velocity:=0.35 > "${OUT}/navigation.log" 2>&1 & pids+=("$!")
setsid ros2 launch sanitation_coverage coverage.launch.py \
  > "${OUT}/coverage_server.log" 2>&1 & pids+=("$!")
setsid ros2 run ros_gz_bridge parameter_bridge \
  /world/sanitation_structured_world/set_pose@ros_gz_interfaces/srv/SetEntityPose \
  > "${OUT}/set_pose_bridge.log" 2>&1 & pids+=("$!")

ready=0
for _ in $(seq 1 240); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  actions="$(ros2 action list 2>/dev/null || true)"
  services="$(ros2 service list 2>/dev/null || true)"
  gz_services="$(gz service --list 2>/dev/null || true)"
  if grep -q '^/localization/fused_pose$' <<< "${topics}" && \
    grep -q '^/compute_coverage_path$' <<< "${actions}" && \
    grep -q '^/compute_path_to_pose$' <<< "${actions}" && \
    grep -q '^/navigate_to_pose$' <<< "${actions}" && \
    grep -q '^/follow_path$' <<< "${actions}" && \
    grep -q '^/world/sanitation_structured_world/set_pose$' <<< "${gz_services}" && \
    grep -q '^/world/sanitation_structured_world/set_pose$' <<< "${services}" && \
    grep -q '^/controller_server/get_state$' <<< "${services}" && \
    grep -q '^/planner_server/get_state$' <<< "${services}" && \
    grep -q '^/bt_navigator/get_state$' <<< "${services}" && \
    grep -q '^/coverage_server/get_state$' <<< "${services}" && \
    grep -q '^/keepout_filter_mask_server/get_state$' <<< "${services}" && \
    grep -q '^/keepout_costmap_filter_info_server/get_state$' <<< "${services}" && \
    grep -q '^/speed_filter_mask_server/get_state$' <<< "${services}" && \
    grep -q '^/speed_costmap_filter_info_server/get_state$' <<< "${services}"
  then ready=1; break; fi
  sleep 1
done
test "${ready}" -eq 1
core_lifecycle_nodes=(controller_server planner_server bt_navigator coverage_server)
for node in "${core_lifecycle_nodes[@]}"; do
  active=0
  for _ in $(seq 1 120); do
    ros2 lifecycle get "/${node}" > "${OUT}/${node}_state.txt" 2>&1 || true
    if grep -Fxq 'active [3]' "${OUT}/${node}_state.txt"; then
      active=1; break
    fi
    sleep 1
  done
  test "${active}" -eq 1
done
filter_lifecycle_nodes=(keepout_filter_mask_server keepout_costmap_filter_info_server \
  speed_filter_mask_server speed_costmap_filter_info_server)
for node in "${filter_lifecycle_nodes[@]}"; do
  active=0
  for _ in $(seq 1 120); do
    ros2 lifecycle get "/${node}" > "${OUT}/${node}_state.txt" 2>&1 || true
    if grep -Fxq 'active [3]' "${OUT}/${node}_state.txt"; then
      active=1; break
    fi
    if grep -Fxq 'unconfigured [1]' "${OUT}/${node}_state.txt"; then
      timeout 10 ros2 lifecycle set "/${node}" configure \
        >> "${OUT}/${node}_transition.log" 2>&1 || true
    elif grep -Fxq 'inactive [2]' "${OUT}/${node}_state.txt"; then
      timeout 10 ros2 lifecycle set "/${node}" activate \
        >> "${OUT}/${node}_transition.log" 2>&1 || true
    fi
    sleep 1
  done
  test "${active}" -eq 1
done
ros2 param get /hybrid_global_fuser minimum_refined_variance \
  > "${OUT}/minimum_refined_variance.txt"
ros2 param get /hybrid_global_fuser maximum_refined_variance \
  > "${OUT}/maximum_refined_variance.txt"
ros2 param get /hybrid_global_fuser maximum_refined_age_s \
  > "${OUT}/maximum_refined_age_s.txt"
ros2 param get /hybrid_global_fuser gnss_variance_scale \
  > "${OUT}/gnss_variance_scale.txt"
ros2 param get /controller_server failure_tolerance \
  > "${OUT}/controller_failure_tolerance.txt"
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.inf_is_valid \
  > "${OUT}/local_costmap_inf_is_valid.txt"
ros2 param get /global_costmap/global_costmap obstacle_layer.scan.inf_is_valid \
  > "${OUT}/global_costmap_inf_is_valid.txt"
grep -q '0.00025' "${OUT}/minimum_refined_variance.txt"
grep -q '0.0009' "${OUT}/maximum_refined_variance.txt"
grep -q '5.0' "${OUT}/maximum_refined_age_s.txt"
grep -q '1.3' "${OUT}/gnss_variance_scale.txt"
grep -q '5.0' "${OUT}/controller_failure_tolerance.txt"
grep -qi 'true' "${OUT}/local_costmap_inf_is_valid.txt"
grep -qi 'true' "${OUT}/global_costmap_inf_is_valid.txt"
timeout 20 ros2 run tf2_ros tf2_echo map base_footprint \
  > "${OUT}/tf_map_base.txt" 2>&1 || true
grep -q 'Translation:' "${OUT}/tf_map_base.txt"

setsid ros2 bag record --storage mcap --output "${OUT}/dynamic_coverage_bag" \
  /clock /scan /odom /gnss/fix /localization/fused_pose /localization/refined_pose \
  /ground_truth/odom /tf /tf_static /cmd_vel_gate /cmd_vel /brush_enabled \
  /local_costmap/costmap /global_costmap/costmap /emergency_stop \
  /keepout_filter_mask /speed_filter_mask \
  /collision_monitor_state /speed_limit /coverage/state \
  /coverage/component_state /coverage/current_path /coverage/diagnostics \
  > "${OUT}/rosbag.log" 2>&1 & bag_pid=$!; pids+=("${bag_pid}")
sleep 2
setsid ros2 run sanitation_tasks sanitation_dynamic_obstacle_probe --ros-args \
  -p use_sim_time:=true -p output_path:="${OUT}/dynamic_obstacle_report.json" \
  -p set_pose_script:="${PACK_ROOT}/scripts/gz_set_dynamic_obstacle.sh" \
  -p trial_count:=20 -p world_name:=sanitation_structured_world \
  -p model_name:=dynamic_pedestrian_box -p service_timeout_ms:=3000 \
  -p minimum_remaining_path_m:=0.8 \
  -p minimum_progress_between_trials_m:=0.5 -p hold_sec:=1.0 \
  -p crossing_steps:=5 -p resume_observation_sec:=1.5 \
  > "${OUT}/dynamic_obstacle.log" 2>&1 & dynamic_pid=$!; pids+=("${dynamic_pid}")

set +e
timeout 1200 ros2 run sanitation_coverage coverage_probe --ros-args \
  -p use_sim_time:=true -p output_path:="${OUT}/coverage_report.json" \
  -p path_output_path:="${OUT}/coverage_path.json" \
  -p trajectory_output_path:="${OUT}/coverage_trajectory.csv" \
  > "${OUT}/coverage_probe.log" 2>&1
coverage_code=$?
wait "${dynamic_pid}"; dynamic_code=$?
timeout 600 ros2 run sanitation_tasks sanitation_filter_probe --ros-args \
  -p use_sim_time:=true -p output_path:="${OUT}/filter_report.json" \
  > "${OUT}/filter_probe.log" 2>&1
filter_code=$?
timeout 180 ros2 run sanitation_tasks sanitation_safety_probe --ros-args \
  -p use_sim_time:=true -p trial_count:=30 \
  -p output_path:="${OUT}/safety_latency_report.json" \
  > "${OUT}/safety_probe.log" 2>&1
safety_code=$?
set -e

ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop '{}' \
  > "${OUT}/rosbag_stop.log" 2>&1 || true
stop_group "${bag_pid}"
for pid in "${pids[@]:0:4}"; do stop_group "${pid}"; done
pids=()
ros2 bag info "${OUT}/dynamic_coverage_bag" > "${OUT}/rosbag_info.txt"
setsid timeout 45 ros2 topic echo /coverage/state std_msgs/msg/String --once \
  > "${OUT}/replay_coverage_state.txt" 2>&1 & echo_pid=$!; pids+=("${echo_pid}")
sleep 2
setsid ros2 bag play "${OUT}/dynamic_coverage_bag" --rate 5.0 \
  --delay 5.0 --topics /coverage/state > "${OUT}/replay.log" 2>&1 &
replay_pid=$!; pids+=("${replay_pid}")
wait "${echo_pid}"
stop_group "${replay_pid}"; pids=()

python3 "${PACK_ROOT}/scripts/stage4w_dynamic_aggregate.py" "${OUT}" \
  --coverage-code "${coverage_code}" --dynamic-code "${dynamic_code}" \
  --filter-code "${filter_code}" --safety-code "${safety_code}"
