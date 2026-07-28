#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_WS="${SANITATION_BASE_WS:?SANITATION_BASE_WS required}"
STAGE4V_WS="${SANITATION_STAGE4V_WS:?SANITATION_STAGE4V_WS required}"
WS="${SANITATION_WS:?SANITATION_WS required}"
OUT="${AUTO01_OUT:?AUTO01_OUT required}"
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
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null

map_root="${WS}/install/sanitation_navigation/share/sanitation_navigation/maps"
footprint_profile="${AUTO01_FOOTPRINT_PROFILE:-auto01_g1_height_banded}"
camera_profile="${AUTO01_CAMERA_PROFILE:-V4_engineering}"
profile="${WS}/install/sanitation_navigation/share/sanitation_navigation/config/${footprint_profile}.yaml"
nav_params="${OUT}/nav2_${footprint_profile}.yaml"
mission_config="${OUT}/demo_area_${footprint_profile}.yaml"
python3 "${PACK_ROOT}/scripts/stage5br6w_profile.py" \
  --base-nav2 "${WS}/install/sanitation_navigation/share/sanitation_navigation/config/nav2.yaml" \
  --base-mission "${WS}/install/sanitation_tasks/share/sanitation_tasks/config/demo_area.yaml" \
  --profile "${profile}" --nav2-output "${nav_params}" --mission-output "${mission_config}"

attempt_id="${AUTO01_ATTEMPT_ID:-AUTO-01-G1-C3-STARTUP-C1}"
started_epoch="$(date +%s)"
setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:="${AUTO01_SEED:-0}" gnss_profile:=rtk_fixed \
  camera_profile:="${camera_profile}" fusion_mode:=hybrid_rtk_scan_imu_wheel \
  enable_scan_refiner:=true > "${OUT}/localization.log" 2>&1 & pids+=("$!")
setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  footprint_profile:="${footprint_profile}" \
  map_file:="${map_root}/stage4v_surveyed_reference.yaml" \
  keepout_map:="${map_root}/stage4v_filters/keepout_mask.yaml" \
  speed_map:="${map_root}/stage4v_filters/speed_mask.yaml" \
  operational_profile:=localization_coverage max_linear_velocity:=0.45 \
  max_angular_velocity:=0.35 > "${OUT}/navigation.log" 2>&1 & pids+=("$!")
setsid ros2 launch sanitation_coverage coverage.launch.py \
  footprint_profile:="${footprint_profile}" \
  > "${OUT}/coverage_server.log" 2>&1 & pids+=("$!")

ready=0
services=""
while [[ "$(( $(date +%s) - started_epoch ))" -lt 60 ]]; do
  services="$(timeout 10 ros2 service list 2>/dev/null || true)"
  profile_services_ready=1
  if [[ "${footprint_profile}" == "auto01_g1_height_banded" ]] && \
    { ! grep -q '^/scan_self_filter/get_parameters$' <<< "${services}" || \
      ! grep -q '^/ground_collision_monitor/get_parameters$' <<< "${services}"; }
  then
    profile_services_ready=0
  fi
  if [[ "${profile_services_ready}" -eq 1 ]] && \
    grep -q '^/controller_server/get_parameters$' <<< "${services}" && \
    grep -q '^/planner_server/get_parameters$' <<< "${services}" && \
    grep -q '^/bt_navigator/get_parameters$' <<< "${services}" && \
    grep -q '^/local_costmap/local_costmap/get_parameters$' <<< "${services}" && \
    grep -q '^/global_costmap/global_costmap/get_parameters$' <<< "${services}" && \
    grep -q '^/collision_monitor/get_parameters$' <<< "${services}" && \
    grep -q '^/coverage_server/get_parameters$' <<< "${services}"
  then
    ready=1
    break
  fi
  sleep 1
done
nav2_ready_seconds="$(( $(date +%s) - started_epoch ))"
total_ready_seconds="$(( $(date +%s) - started_epoch ))"

topics="$(timeout 10 ros2 topic list 2>/dev/null || true)"
printf '%s\n' "${topics}" > "${OUT}/observed_topics.txt"
printf '%s\n' "${services}" > "${OUT}/observed_services.txt"

pointcloud_ready=0
pointcloud_topic="/camera/depth/color/points"
if [[ "${footprint_profile}" == "auto01_g2_v5_retracted" ]]; then
  pointcloud_topic="/verification_camera/depth/color/points/navigation"
fi
if [[ "${ready}" -eq 1 ]] && \
  timeout 90 ros2 topic echo "${pointcloud_topic}" \
    sensor_msgs/msg/PointCloud2 --once > "${OUT}/ground_pointcloud_sample.yaml" 2>&1
then
  pointcloud_ready=1
fi
self_filtered_scan_ready=0
if [[ "${footprint_profile}" != "auto01_g1_height_banded" ]]; then
  self_filtered_scan_ready=1
elif [[ "${ready}" -eq 1 ]] && \
  timeout 30 ros2 topic echo /scan/navigation sensor_msgs/msg/LaserScan \
    --once > "${OUT}/self_filtered_scan_sample.yaml" 2>&1
then
  self_filtered_scan_ready=1
fi
verification_camera_ready=0
if [[ "${camera_profile}" == "production" ]]; then
  verification_camera_ready=1
elif [[ "${ready}" -eq 1 ]] && \
  timeout 90 ros2 topic echo /verification_camera/color/image_raw \
    sensor_msgs/msg/Image --field header --once \
    > "${OUT}/verification_camera_sample.yaml" 2>&1
then
  verification_camera_ready=1
fi

if [[ "${ready}" -eq 1 && "${pointcloud_ready}" -eq 1 && "${self_filtered_scan_ready}" -eq 1 && "${verification_camera_ready}" -eq 1 ]]; then
  ros2 param dump /local_costmap/local_costmap > "${OUT}/runtime_local_costmap_params.yaml"
  ros2 param dump /global_costmap/global_costmap > "${OUT}/runtime_global_costmap_params.yaml"
  if [[ "${footprint_profile}" == "auto01_g1_height_banded" ]]; then
    "${PACK_ROOT}/scripts/auto01_capture_collision_params.sh" \
      "${OUT}/runtime_collision_monitor_selected_params.json"
  else
    bash "${PACK_ROOT}/scripts/auto01_capture_g2_params.sh" \
      "${OUT}/runtime_collision_monitor_g2_params.json"
  fi
  timeout 20 ros2 topic echo /local_costmap/published_footprint \
    geometry_msgs/msg/PolygonStamped --once > "${OUT}/runtime_local_published_footprint.yaml"
  timeout 20 ros2 topic echo /global_costmap/published_footprint \
    geometry_msgs/msg/PolygonStamped --once > "${OUT}/runtime_global_published_footprint.yaml"
fi

python3 - "${OUT}" "${ready}" "${nav2_ready_seconds}" "${total_ready_seconds}" "${attempt_id}" "${pointcloud_ready}" "${self_filtered_scan_ready}" "${verification_camera_ready}" "${footprint_profile}" "${camera_profile}" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
ready = bool(int(sys.argv[2]))
seconds = int(sys.argv[3])
total_seconds = int(sys.argv[4])
attempt_id = sys.argv[5]
pointcloud_ready = bool(int(sys.argv[6]))
self_filtered_scan_ready = bool(int(sys.argv[7]))
verification_camera_ready = bool(int(sys.argv[8]))
footprint_profile = sys.argv[9]
camera_profile = sys.argv[10]
topics = set((root / "observed_topics.txt").read_text(encoding="utf-8").splitlines())
collision_parameter_file = (
    "runtime_collision_monitor_selected_params.json"
    if footprint_profile == "auto01_g1_height_banded"
    else "runtime_collision_monitor_g2_params.json"
)
report = {
    "schema_version": 1,
    "stage": "AUTO-01",
    "attempt_id": attempt_id,
    "profile": footprint_profile,
    "camera_profile": camera_profile,
    "observed_runtime_topics": {
        "map_topic": "/map" in topics,
        "scan_topic": "/scan" in topics,
        "odom_topic": bool({"/odom", "/odom/unfiltered"} & topics),
        "cmd_vel_gate_topic": "/cmd_vel_gate" in topics,
        "fused_pose_topic": "/localization/fused_pose" in topics,
    },
    "interfaces_ready": ready,
    "height_banded_pointcloud_message_ready": pointcloud_ready,
    "self_filtered_scan_message_ready": self_filtered_scan_ready,
    "verification_camera_message_ready": verification_camera_ready,
    "nav2_parameter_services_ready_seconds": seconds,
    "nav2_parameter_services_ready_within_60_seconds": ready and seconds <= 60,
    "total_cold_start_seconds": total_seconds,
    "runtime_parameter_dumps_present": all(
        (root / name).is_file() and (root / name).stat().st_size > 0
        for name in (
            "runtime_local_costmap_params.yaml",
            "runtime_global_costmap_params.yaml",
            collision_parameter_file,
            "runtime_local_published_footprint.yaml",
            "runtime_global_published_footprint.yaml",
        )
    ),
}
(root / "cold_start_report.json").write_text(
    json.dumps(report, indent=2) + "\n", encoding="utf-8"
)
PY

test "${ready}" -eq 1
test "${pointcloud_ready}" -eq 1
test "${self_filtered_scan_ready}" -eq 1
test "${verification_camera_ready}" -eq 1
test "${nav2_ready_seconds}" -le 60
