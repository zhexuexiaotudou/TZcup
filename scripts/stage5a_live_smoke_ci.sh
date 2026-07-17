#!/usr/bin/env bash
set -euo pipefail

MODEL_PATH="${1:?model path required}"
OUT="${2:?output directory required}"
PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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

setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:=205 gnss_profile:=rtk_fixed \
  fusion_mode:=hybrid_rtk_scan_imu_wheel enable_scan_refiner:=true \
  > "${OUT}/live_simulation.log" 2>&1 & pids+=("$!")

ready=0
for _ in $(seq 1 180); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -q '^/camera/color/image_raw$' <<< "${topics}" && \
     grep -q '^/camera/depth/image_rect_raw$' <<< "${topics}" && \
     grep -q '^/camera/color/camera_info$' <<< "${topics}" && \
     grep -q '^/localization/fused_pose$' <<< "${topics}"; then
    ready=1; break
  fi
  sleep 1
done
test "${ready}" -eq 1

declare -A models=(
  [trash_bottle_01]="trash_bottle/model.sdf|-5.4|-0.55|0.12|0.0"
  [trash_can_01]="trash_can/model.sdf|-4.8|0.0|0.06|0.0"
  [trash_paper_01]="trash_paper/model.sdf|-4.2|0.55|0.015|0.15"
  [leaf_pile_01]="leaf_pile/model.sdf|-3.6|-0.65|0.02|-0.10"
)
for name in "${!models[@]}"; do
  IFS='|' read -r relative x y z yaw <<< "${models[$name]}"
  ros2 run ros_gz_sim create -world sanitation_structured_world \
    -file "${PACK_ROOT}/starter_ws/src/sanitation_worlds/models/${relative}" \
    -name "${name}" -x "${x}" -y "${y}" -z "${z}" -Y "${yaw}" \
    > "${OUT}/spawn_${name}.log" 2>&1
  grep -qi 'success' "${OUT}/spawn_${name}.log"
done

setsid ros2 launch sanitation_perception stage5a_perception.launch.py \
  model_path:="${MODEL_PATH}" use_sim_time:=true \
  > "${OUT}/live_perception.log" 2>&1 & pids+=("$!")

pipeline_ready=0
for _ in $(seq 1 120); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -q '^/garbage/ground_truth$' <<< "${topics}" && \
     grep -q '^/perception/garbage/diagnostics$' <<< "${topics}" && \
     grep -q '^/perception/garbage/segmentation$' <<< "${topics}" && \
     grep -q '^/perception/garbage/targets$' <<< "${topics}" && \
     grep -q '^/spot_clean/state$' <<< "${topics}"; then
    pipeline_ready=1; break
  fi
  sleep 1
done
test "${pipeline_ready}" -eq 1

timeout 30 ros2 topic echo /camera/color/camera_info --once > "${OUT}/camera_info.txt"
timeout 30 ros2 topic echo /camera/color/image_raw --once > "${OUT}/rgb_image.txt"
timeout 30 ros2 topic echo /camera/depth/image_rect_raw --once > "${OUT}/depth_image.txt"
timeout 30 ros2 topic echo /garbage/ground_truth --once > "${OUT}/ground_truth.txt"
timeout 30 ros2 topic echo --full-length /garbage/ground_truth/diagnostics --once > "${OUT}/ground_truth_diagnostics.txt"
timeout 30 ros2 topic echo --full-length /perception/garbage/diagnostics --once > "${OUT}/perception_diagnostics.txt"
timeout 30 ros2 topic echo /perception/garbage/segmentation --once > "${OUT}/segmentation.txt"
timeout 30 ros2 topic echo --full-length /spot_clean/state --once > "${OUT}/spot_clean_state.txt"
ros2 node list > "${OUT}/nodes.txt"
ros2 topic list -t > "${OUT}/topics.txt"

record_bag="${STAGE5A_RECORD_BAG:-false}"
if [[ "${record_bag}" == "true" ]]; then
  setsid ros2 bag record --storage mcap \
    --compression-mode file --compression-format zstd \
    --output "${OUT}/stage5a_live_bag" \
    /clock /camera/color/image_raw /camera/color/camera_info \
    /camera/depth/image_rect_raw /tf /tf_static /localization/fused_pose \
    /cmd_vel /brush_enabled \
    /garbage/ground_truth /garbage/ground_truth/diagnostics \
    /perception/garbage/detections_2d /perception/garbage/detections_3d \
    /perception/garbage/segmentation /perception/garbage/targets \
    /perception/garbage/diagnostics /spot_clean/state /coverage/state \
    /garbage/cleaning_events > "${OUT}/rosbag.log" 2>&1 & bag_pid=$!; pids+=("${bag_pid}")
  bag_ready=0
  for _ in $(seq 1 90); do
    if grep -q "Subscribed to topic '/camera/color/image_raw'" "${OUT}/rosbag.log" && \
       grep -q "Subscribed to topic '/camera/depth/image_rect_raw'" "${OUT}/rosbag.log" && \
       grep -q "Subscribed to topic '/perception/garbage/diagnostics'" "${OUT}/rosbag.log" && \
       grep -q "Subscribed to topic '/brush_enabled'" "${OUT}/rosbag.log"; then
      bag_ready=1
      break
    fi
    sleep 1
  done
  test "${bag_ready}" -eq 1
  sleep 5
  ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop '{}' \
    > "${OUT}/rosbag_stop.log" 2>&1 || true
  stop_group "${bag_pid}"
  ros2 bag info "${OUT}/stage5a_live_bag" > "${OUT}/rosbag_info.txt"
else
  printf 'screening mode: raw MCAP recording disabled\n' > "${OUT}/rosbag_info.txt"
fi

grep -q 'active_backend.*onnxruntime' "${OUT}/perception_diagnostics.txt"
grep -q 'ground_truth_input_used.*false' "${OUT}/perception_diagnostics.txt"
grep -Eq 'last_detection_count.*[1-9][0-9]*' "${OUT}/perception_diagnostics.txt"
grep -Eq 'last_map_target_count.*[1-9][0-9]*' "${OUT}/perception_diagnostics.txt"
grep -q 'map_targets_fail_closed.*false' "${OUT}/perception_diagnostics.txt"
grep -q 'registry_target_count.*5' "${OUT}/ground_truth_diagnostics.txt"
grep -Eq 'published_target_count.*[1-5]' "${OUT}/ground_truth_diagnostics.txt"
grep -q 'occlusion_filter.*geometry_disc_fallback' "${OUT}/ground_truth_diagnostics.txt"
grep -q 'negative_models_published_as_targets.*0' "${OUT}/ground_truth_diagnostics.txt"
grep -q 'ground_truth_control_allowed.*false' "${OUT}/spot_clean_state.txt"

python3 - "${OUT}" <<'PY'
import json
import re
import sys
from pathlib import Path

out = Path(sys.argv[1])
text = (out / 'perception_diagnostics.txt').read_text(encoding='utf-8')
match = re.search(r'data:\s*["\']?(\{.*\})', text)
diagnostics = json.loads(match.group(1).replace('\\"', '"')) if match else {}
ground_truth_text = (out / 'ground_truth_diagnostics.txt').read_text(encoding='utf-8')
ground_truth_match = re.search(r'data:\s*["\']?(\{.*\})', ground_truth_text)
ground_truth_diagnostics = json.loads(
    ground_truth_match.group(1).replace('\\"', '"')
) if ground_truth_match else {}
rosbag_info = (out / 'rosbag_info.txt').read_text(encoding='utf-8')
def bag_topic_has_messages(topic):
    return re.search(
        rf'Topic: {re.escape(topic)} .*? Count: ([1-9][0-9]*)', rosbag_info
    ) is not None

report = {
    'schema_version': 1,
    'stage': 'Stage5A',
    'rgb_topic_received': (out / 'rgb_image.txt').stat().st_size > 0,
    'depth_topic_received': (out / 'depth_image.txt').stat().st_size > 0,
    'camera_info_received': (out / 'camera_info.txt').stat().st_size > 0,
    'segmentation_topic_received': (out / 'segmentation.txt').stat().st_size > 0,
    'ground_truth_topic_received': (out / 'ground_truth.txt').stat().st_size > 0,
    'ground_truth_registry_target_count': ground_truth_diagnostics.get('registry_target_count'),
    'ground_truth_published_target_count': ground_truth_diagnostics.get('published_target_count'),
    'ground_truth_negative_target_count': ground_truth_diagnostics.get('negative_models_published_as_targets'),
    'ground_truth_occlusion_filter': ground_truth_diagnostics.get('occlusion_filter'),
    'active_backend': diagnostics.get('active_backend'),
    'inference_frame_count': diagnostics.get('frame_count', 0),
    'last_detection_count': diagnostics.get('last_detection_count', 0),
    'last_map_target_count': diagnostics.get('last_map_target_count', 0),
    'map_targets_fail_closed': diagnostics.get('map_targets_fail_closed'),
    'ground_truth_input_used': diagnostics.get('ground_truth_input_used'),
    'ground_truth_control_violation_count': diagnostics.get('ground_truth_control_violation_count'),
    'spawned_target_count': 4,
    'structured_world_target_count': 1,
    'rosbag_required': __import__('os').environ.get('STAGE5A_RECORD_BAG', 'false') == 'true',
    'rosbag_info_present': bool(rosbag_info.strip()),
    'rosbag_recorded': 'Files:' in rosbag_info and 'Messages:' in rosbag_info,
    'rosbag_rgb_messages_present': bag_topic_has_messages('/camera/color/image_raw'),
    'rosbag_depth_messages_present': bag_topic_has_messages('/camera/depth/image_rect_raw'),
    'rosbag_perception_messages_present': bag_topic_has_messages('/perception/garbage/diagnostics'),
}
report['live_smoke_pass'] = all([
    report['rgb_topic_received'], report['depth_topic_received'],
    report['camera_info_received'], report['segmentation_topic_received'],
    report['ground_truth_topic_received'], report['active_backend'] == 'onnxruntime',
    report['ground_truth_registry_target_count'] == 5,
    report['ground_truth_published_target_count'] > 0,
    report['ground_truth_negative_target_count'] == 0,
    report['ground_truth_occlusion_filter'] == 'geometry_disc_fallback',
    report['inference_frame_count'] > 0, report['ground_truth_input_used'] is False,
    report['last_detection_count'] > 0, report['last_map_target_count'] > 0,
    report['map_targets_fail_closed'] is False,
    report['ground_truth_control_violation_count'] == 0,
    report['rosbag_info_present'],
    not report['rosbag_required'] or report['rosbag_recorded'],
    not report['rosbag_required'] or report['rosbag_rgb_messages_present'],
    not report['rosbag_required'] or report['rosbag_depth_messages_present'],
    not report['rosbag_required'] or report['rosbag_perception_messages_present'],
])
(out / 'stage5a_live_smoke_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
assert report['live_smoke_pass'], report
PY
