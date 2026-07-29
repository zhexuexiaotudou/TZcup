#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_WS="${SANITATION_BASE_WS:?SANITATION_BASE_WS required}"
STAGE4V_WS="${SANITATION_STAGE4V_WS:?SANITATION_STAGE4V_WS required}"
WS="${SANITATION_WS:?SANITATION_WS required}"
OUT="${AUTO03_OUT:?AUTO03_OUT required}"
WORLD_ID="${AUTO03_WORLD_ID:?AUTO03_WORLD_ID required}"
WORLD_FILE="${AUTO03_WORLD_FILE:?AUTO03_WORLD_FILE required}"
MATRIX="${AUTO03_MATRIX:?AUTO03_MATRIX required}"
MAX_TRIALS="${AUTO03_MAX_TRIALS:-0}"
TRIAL_OFFSET="${AUTO03_TRIAL_OFFSET:-0}"
mkdir -p "${OUT}"
rm -f \
  "${OUT}/runtime_trials.json" \
  "${OUT}/executive_node_info.txt" \
  "${OUT}/evaluator_node_info.txt" \
  "${OUT}/oracle_node_info.txt" \
  "${OUT}/planner_node_info.txt" \
  "${OUT}/control_node_graph.txt" \
  "${OUT}/gt_semantic_topic_graph.txt"
pids=()

stop_group() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return
  kill -INT -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 100); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  for pid in "${pids[@]:-}"; do stop_group "${pid}"; done
}
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "${BASE_WS}/install/setup.bash"
source "${STAGE4V_WS}/install/setup.bash"
source "${WS}/install/setup.bash"
set -u

map_root="${WS}/install/sanitation_navigation/share/sanitation_navigation/maps"
profile="${WS}/install/sanitation_navigation/share/sanitation_navigation/config/autonomous_navigation_profile_v1.yaml"
nav_params="${OUT}/nav2_autonomous_navigation_profile_v1.yaml"
mission_config="${OUT}/demo_area_autonomous_navigation_profile_v1.yaml"
python3 "${PACK_ROOT}/scripts/stage5br6w_profile.py" \
  --base-nav2 "${WS}/install/sanitation_navigation/share/sanitation_navigation/config/nav2.yaml" \
  --base-mission "${WS}/install/sanitation_tasks/share/sanitation_tasks/config/demo_area.yaml" \
  --profile "${profile}" --nav2-output "${nav_params}" \
  --mission-output "${mission_config}"
python3 - "${nav_params}" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
payload = yaml.safe_load(path.read_text(encoding="utf-8"))
payload["controller_server"]["ros__parameters"]["goal_checker"][
    "yaw_goal_tolerance"
] = 0.50
path.write_text(
    yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
    encoding="utf-8",
)
PY

setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:=3 gnss_profile:=rtk_fixed \
  world_file:="${WORLD_FILE}" camera_profile:=AUTO03_corner enable_training_gt:=true \
  fusion_mode:=hybrid_rtk_scan_imu_wheel enable_scan_refiner:=true \
  > "${OUT}/localization.log" 2>&1 & pids+=("$!")

# The localization launch owns the transient-local /map publisher. Starting
# Nav2 before that lifecycle node is active can leave StaticLayer permanently
# on its fallback 5 x 5 m costmap after a busy cold start.
map_ready=0
for _ in $(seq 1 240); do
  ros2 lifecycle get /map_server > "${OUT}/map_server_state.txt" 2>&1 || true
  ros2 topic info /map > "${OUT}/map_topic_info.txt" 2>&1 || true
  if grep -Fxq 'active [3]' "${OUT}/map_server_state.txt" && \
    grep -Eq 'Publisher count: [1-9][0-9]*' "${OUT}/map_topic_info.txt"
  then
    map_ready=1
    break
  fi
  sleep 1
done
test "${map_ready}" -eq 1

setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  footprint_profile:=autonomous_navigation_profile_v1 \
  map_file:="${map_root}/stage4v_surveyed_reference.yaml" \
  keepout_map:="${map_root}/stage4v_filters/keepout_mask.yaml" \
  speed_map:="${map_root}/stage4v_filters/speed_mask.yaml" \
  operational_profile:=localization_coverage max_linear_velocity:=0.45 \
  max_angular_velocity:=0.35 > "${OUT}/navigation.log" 2>&1 & pids+=("$!")
setsid ros2 launch sanitation_coverage coverage.launch.py \
  footprint_profile:=autonomous_navigation_profile_v1 \
  > "${OUT}/coverage_server.log" 2>&1 & pids+=("$!")
setsid ros2 run sanitation_spot_cleaning stage5br5_observation_pose_node --ros-args \
  --params-file "${WS}/install/sanitation_spot_cleaning/share/sanitation_spot_cleaning/config/auto03_observation_pose.yaml" \
  > "${OUT}/observation_pose.log" 2>&1 & pids+=("$!")

ready=0
for _ in $(seq 1 240); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  actions="$(ros2 action list 2>/dev/null || true)"
  gz_services="$(gz service -l 2>/dev/null || true)"
  if grep -q '^/localization/fused_pose$' <<< "${topics}" && \
    grep -q '^/verification_camera/color/image_raw$' <<< "${topics}" && \
    grep -q '^/ground_truth/verification_semantic/image$' <<< "${topics}" && \
    grep -q '^/compute_path_to_pose$' <<< "${actions}" && \
    grep -q '^/navigate_to_pose$' <<< "${actions}" && \
    grep -q '^/compute_coverage_path$' <<< "${actions}" && \
    grep -q "^/world/${WORLD_ID}/set_pose_vector$" <<< "${gz_services}"
  then
    ready=1
    break
  fi
  sleep 1
done
test "${ready}" -eq 1

lifecycle_checks=()
for node in controller_server planner_server bt_navigator coverage_server; do
  (
    active=0
    for _ in $(seq 1 120); do
      timeout 20 ros2 lifecycle get "/${node}" \
        > "${OUT}/${node}_state.txt" 2>&1 || true
      if grep -Fxq 'active [3]' "${OUT}/${node}_state.txt"; then
        active=1
        break
      fi
      sleep 1
    done
    test "${active}" -eq 1
  ) &
  lifecycle_checks+=("$!")
done
for lifecycle_pid in "${lifecycle_checks[@]}"; do
  wait "${lifecycle_pid}"
done

bag_dir="${OUT}/auto03_runtime_bag"
rm -rf "${bag_dir}"
setsid ros2 bag record -s mcap -o "${bag_dir}" --include-hidden-topics --topics \
  /auto03/oracle_candidate \
  /active_observation/candidate \
  /active_observation/pose_plan \
  /active_observation/selected_pose \
  /auto03/capture_request \
  /auto03/machine_ready_result \
  /auto03/trial_result \
  /auto03/done \
  /coverage/state \
  /brush_enabled \
  /compute_path_to_pose/_action/status \
  /navigate_to_pose/_action/status \
  /spin/_action/status \
  /localization/fused_pose \
  /odom \
  /tf \
  /tf_static \
  /cmd_vel \
  /cmd_vel_nav \
  > "${OUT}/rosbag_record.log" 2>&1 & bag_pid=$!; pids+=("${bag_pid}")
bag_ready=0
for _ in $(seq 1 90); do
  if grep -Fq "Subscribed to topic '/compute_path_to_pose/_action/status'" "${OUT}/rosbag_record.log" && \
    grep -Fq "Subscribed to topic '/navigate_to_pose/_action/status'" "${OUT}/rosbag_record.log" && \
    grep -Fq "Subscribed to topic '/spin/_action/status'" "${OUT}/rosbag_record.log"
  then
    bag_ready=1
    break
  fi
  sleep 1
done
test "${bag_ready}" -eq 1

setsid ros2 run sanitation_spot_cleaning auto03_matrix_probe --ros-args \
  -p use_sim_time:=true -p matrix_path:="${MATRIX}" -p world_id:="${WORLD_ID}" \
  -p max_trials:="${MAX_TRIALS}" -p trial_offset:="${TRIAL_OFFSET}" \
  -p source_start_delay_s:=30.0 \
  -p output_path:="${OUT}/runtime_trials.json" \
  > "${OUT}/matrix_probe.log" 2>&1 & probe_pid=$!; pids+=("${probe_pid}")

workflow_bag_ready=0
for _ in $(seq 1 90); do
  if grep -Fq "Subscribed to topic '/auto03/oracle_candidate'" "${OUT}/rosbag_record.log" && \
    grep -Fq "Subscribed to topic '/auto03/trial_result'" "${OUT}/rosbag_record.log" && \
    grep -Fq "Subscribed to topic '/coverage/state'" "${OUT}/rosbag_record.log"
  then
    workflow_bag_ready=1
    break
  fi
  sleep 1
done
test "${workflow_bag_ready}" -eq 1

for _ in $(seq 1 10800); do
  if [[ -f "${OUT}/runtime_trials.json" ]] && \
    python3 - "${OUT}/runtime_trials.json" <<'PY'
import json
import sys
try:
    payload = json.load(open(sys.argv[1], encoding="utf-8"))
except (OSError, json.JSONDecodeError):
    raise SystemExit(1)
raise SystemExit(0 if payload.get("runtime_complete") else 1)
PY
  then
    break
  fi
  if ! kill -0 "${probe_pid}" 2>/dev/null; then
    echo "AUTO-03 matrix probe exited before completion" >&2
    exit 2
  fi
  sleep 1
done

python3 - "${OUT}/runtime_trials.json" <<'PY'
import json
import sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
assert payload["runtime_complete"], payload
assert payload["result_count"] == payload["expected_count"], payload
assert payload["robot_pose_set_by_oracle"] is False
PY

recorder_info_ready=0
for _ in $(seq 1 3); do
  if timeout 30 ros2 node info /rosbag2_recorder \
    > "${OUT}/rosbag_recorder_node_info.txt" 2>&1 && \
    test -s "${OUT}/rosbag_recorder_node_info.txt"
  then
    recorder_info_ready=1
    break
  fi
  sleep 2
done
test "${recorder_info_ready}" -eq 1

capture_node_info() {
  local node="$1"
  local output="$2"
  for _ in $(seq 1 3); do
    if timeout 30 ros2 node info "${node}" > "${output}" 2>&1 && \
      test -s "${output}"
    then
      return 0
    fi
    sleep 2
  done
  return 1
}
node_info_checks=()
capture_node_info /auto03_observation_executive "${OUT}/executive_node_info.txt" & node_info_checks+=("$!")
capture_node_info /auto03_machine_ready_evaluator "${OUT}/evaluator_node_info.txt" & node_info_checks+=("$!")
capture_node_info /auto03_oracle_scene_source "${OUT}/oracle_node_info.txt" & node_info_checks+=("$!")
capture_node_info /stage5br5_observation_pose_planner "${OUT}/planner_node_info.txt" & node_info_checks+=("$!")
capture_node_info /controller_server "${OUT}/controller_node_info.txt" & node_info_checks+=("$!")
capture_node_info /planner_server "${OUT}/nav_planner_node_info.txt" & node_info_checks+=("$!")
capture_node_info /bt_navigator "${OUT}/bt_navigator_node_info.txt" & node_info_checks+=("$!")
for node_info_pid in "${node_info_checks[@]}"; do
  wait "${node_info_pid}"
done
cat \
  "${OUT}/controller_node_info.txt" \
  "${OUT}/nav_planner_node_info.txt" \
  "${OUT}/bt_navigator_node_info.txt" \
  "${OUT}/planner_node_info.txt" \
  "${OUT}/executive_node_info.txt" \
  > "${OUT}/control_node_graph.txt"
if grep -E '/ground_truth|/g2/.+gt' "${OUT}/control_node_graph.txt"; then
  echo "GT topic reached planner/navigation/control node" >&2
  exit 3
fi
grep -q '/ground_truth/verification_semantic/image' "${OUT}/evaluator_node_info.txt"
timeout 30 ros2 topic info --verbose /ground_truth/verification_semantic/image \
  > "${OUT}/gt_semantic_topic_graph.txt"
grep -q 'Node name: auto03_machine_ready_evaluator' "${OUT}/gt_semantic_topic_graph.txt"
if grep -E 'Node name: (controller_server|planner_server|bt_navigator|stage5br5_observation_pose_planner|auto03_observation_executive)' \
  "${OUT}/gt_semantic_topic_graph.txt"
then
  echo "GT topic has a planner/navigation/control subscriber" >&2
  exit 4
fi

stop_group "${probe_pid}"
stop_group "${bag_pid}"
pids=("${pids[@]:0:4}")
ros2 bag info "${bag_dir}" > "${OUT}/rosbag_info.txt"
python3 "${PACK_ROOT}/scripts/auto03_replay_audit.py" \
  --bag "${bag_dir}" --matrix "${MATRIX}" \
  --runtime "${OUT}/runtime_trials.json" \
  --output "${OUT}/replay_audit.json"
timeout 90 ros2 bag play "${bag_dir}" --rate 100 \
  --topics /auto03/trial_result /coverage/state /brush_enabled \
  --remap \
    /auto03/trial_result:=/replay/auto03/trial_result \
    /coverage/state:=/replay/coverage/state \
    /brush_enabled:=/replay/brush_enabled \
  > "${OUT}/replay_start.log" 2>&1
