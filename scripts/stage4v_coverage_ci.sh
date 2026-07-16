#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_WS="${SANITATION_BASE_WS:?SANITATION_BASE_WS required}"
WS="${SANITATION_WS:?SANITATION_WS required}"
OUT="${STAGE4V_OUT:?STAGE4V_OUT required}"
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
source "${WS}/install/setup.bash"
set -u

map_root="${WS}/install/sanitation_navigation/share/sanitation_navigation/maps"
nav_params="${WS}/install/sanitation_navigation/share/sanitation_navigation/config/nav2.yaml"

setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:=40 gnss_profile:=rtk_fixed \
  fusion_mode:=hybrid_rtk_scan_imu_wheel enable_scan_refiner:=true \
  > "${OUT}/localization_stack.log" 2>&1 &
pids+=("$!")
setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  map_file:="${map_root}/stage4v_surveyed_reference.yaml" \
  keepout_map:="${map_root}/stage4v_filters/keepout_mask.yaml" \
  speed_map:="${map_root}/stage4v_filters/speed_mask.yaml" \
  operational_profile:=localization_coverage max_linear_velocity:=0.45 \
  max_angular_velocity:=0.35 > "${OUT}/navigation.log" 2>&1 &
pids+=("$!")
setsid ros2 launch sanitation_coverage coverage.launch.py \
  > "${OUT}/coverage_server.log" 2>&1 &
pids+=("$!")

ready=0
for _ in $(seq 1 180); do
  if ros2 topic list 2>/dev/null | grep -q '^/localization/fused_pose$'; then
    ready=1
    break
  fi
  sleep 1
done
test "${ready}" -eq 1
timeout 30 ros2 topic echo /localization/fused_pose \
  geometry_msgs/msg/PoseWithCovarianceStamped --once \
  > "${OUT}/first_fused_pose.txt"
setsid ros2 bag record --storage mcap --output "${OUT}/full_mission_bag" \
  /clock /scan /odom /localization/fused_pose /localization/refined_pose \
  /gnss/fix /ground_truth/odom /tf /tf_static /cmd_vel_gate /cmd_vel \
  /brush_enabled /emergency_stop /collision_monitor_state /speed_limit \
  > "${OUT}/rosbag.log" 2>&1 &
bag_pid=$!; pids+=("${bag_pid}")
setsid ros2 run sanitation_tasks sanitation_dynamic_obstacle_probe --ros-args \
  -p use_sim_time:=true -p output_path:="${OUT}/dynamic_obstacle_report.json" \
  -p set_pose_script:="${PACK_ROOT}/scripts/gz_set_dynamic_obstacle.sh" \
  > "${OUT}/dynamic_obstacle.log" 2>&1 &
dynamic_pid=$!; pids+=("${dynamic_pid}")

set +e
timeout 1200 ros2 run sanitation_coverage coverage_probe --ros-args \
  -p use_sim_time:=true -p output_path:="${OUT}/coverage_report.json" \
  -p path_output_path:="${OUT}/coverage_path.json" \
  -p trajectory_output_path:="${OUT}/coverage_trajectory.csv" \
  > "${OUT}/coverage_probe.log" 2>&1
coverage_code=$?
wait "${dynamic_pid}"; dynamic_code=$?
ros2 run sanitation_tasks sanitation_filter_probe --ros-args \
  -p use_sim_time:=true -p output_path:="${OUT}/filter_report.json" \
  > "${OUT}/filter_probe.log" 2>&1
filter_code=$?
ros2 run sanitation_tasks sanitation_safety_probe --ros-args \
  -p use_sim_time:=true -p trial_count:=30 \
  -p output_path:="${OUT}/safety_latency_report.json" \
  > "${OUT}/safety_probe.log" 2>&1
safety_code=$?
set -e

ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop '{}' \
  > "${OUT}/rosbag_stop.log" 2>&1 || true
stop_group "${bag_pid}"
for pid in "${pids[@]:0:3}"; do stop_group "${pid}"; done
pids=()
ros2 bag info "${OUT}/full_mission_bag" > "${OUT}/rosbag_info.txt"
setsid ros2 bag play "${OUT}/full_mission_bag" --rate 5.0 \
  --topics /localization/fused_pose > "${OUT}/rosbag_replay.log" 2>&1 &
replay_pid=$!; pids+=("${replay_pid}")
timeout 45 ros2 topic echo /localization/fused_pose \
  geometry_msgs/msg/PoseWithCovarianceStamped --once \
  > "${OUT}/replay_fused_pose.txt"
stop_group "${replay_pid}"; pids=()

python3 "${PACK_ROOT}/scripts/stage4t_coverage_aggregate.py" "${OUT}"
python3 - "${OUT}" "${coverage_code}" "${dynamic_code}" "${filter_code}" "${safety_code}" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
path = root / 'stage4t_coverage_report.json'
report = json.loads(path.read_text(encoding='utf-8'))
report['stage'] = 'Stage4V'
report['lane'] = 'hybrid_rtk_scan_imu_wheel'
report['complete_rosbag_replay'] = bool(
    (root / 'full_mission_bag' / 'metadata.yaml').is_file()
    and (root / 'replay_fused_pose.txt').stat().st_size > 0
)
report['exit_codes'] = dict(zip(
    ('coverage', 'dynamic', 'filters', 'safety'), map(int, sys.argv[2:6])
))
report['success'] = bool(
    report['success'] and report['complete_rosbag_replay']
    and not any(report['exit_codes'].values())
)
(root / 'stage4v_coverage_report.json').write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
)
PY

test "${coverage_code}" -eq 0
test "${dynamic_code}" -eq 0
test "${filter_code}" -eq 0
test "${safety_code}" -eq 0
python3 -c "import json; assert json.load(open('${OUT}/stage4v_coverage_report.json'))['success']"
