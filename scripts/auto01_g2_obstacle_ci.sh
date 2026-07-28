#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_WS="${SANITATION_BASE_WS:?SANITATION_BASE_WS required}"
STAGE4V_WS="${SANITATION_STAGE4V_WS:?SANITATION_STAGE4V_WS required}"
WS="${SANITATION_WS:?SANITATION_WS required}"
OUT="${AUTO01_OUT:?AUTO01_OUT required}"
TRIALS="${AUTO01_OBSTACLE_TRIALS:-30}"
mkdir -p "${OUT}"
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
cleanup() { for pid in "${pids[@]:-}"; do stop_group "${pid}"; done; }
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "${BASE_WS}/install/setup.bash"
source "${STAGE4V_WS}/install/setup.bash"
source "${WS}/install/setup.bash"
set -u

map_root="${WS}/install/sanitation_navigation/share/sanitation_navigation/maps"
profile="${WS}/install/sanitation_navigation/share/sanitation_navigation/config/auto01_g2_v5_retracted.yaml"
nav_params="${OUT}/nav2_auto01_g2_v5_retracted.yaml"
mission_config="${OUT}/demo_area_auto01_g2_v5_retracted.yaml"
python3 "${PACK_ROOT}/scripts/stage5br6w_profile.py" \
  --base-nav2 "${WS}/install/sanitation_navigation/share/sanitation_navigation/config/nav2.yaml" \
  --base-mission "${WS}/install/sanitation_tasks/share/sanitation_tasks/config/demo_area.yaml" \
  --profile "${profile}" --nav2-output "${nav_params}" \
  --mission-output "${mission_config}"

setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:=0 gnss_profile:=rtk_fixed \
  camera_profile:=V5_retracted \
  fusion_mode:=hybrid_rtk_scan_imu_wheel enable_scan_refiner:=true \
  > "${OUT}/localization.log" 2>&1 & pids+=("$!")
setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  footprint_profile:=auto01_g2_v5_retracted \
  map_file:="${map_root}/stage4v_surveyed_reference.yaml" \
  keepout_map:="${map_root}/stage4v_filters/keepout_mask.yaml" \
  speed_map:="${map_root}/stage4v_filters/speed_mask.yaml" \
  operational_profile:=localization_coverage max_linear_velocity:=0.45 \
  max_angular_velocity:=0.35 > "${OUT}/navigation.log" 2>&1 & pids+=("$!")

ready=0
for _ in $(seq 1 4); do
  if timeout 30 python3 "${PACK_ROOT}/scripts/auto01_capture_g2_params.py" \
    "${OUT}/runtime_collision_monitor_g2_params.json"
  then
    ready=1
    break
  fi
  sleep 2
done
test "${ready}" -eq 1
timeout 90 ros2 topic echo /verification_camera/depth/color/points/navigation \
  sensor_msgs/msg/PointCloud2 --field header --once \
  > "${OUT}/pointcloud_sample.yaml"
timeout 90 ros2 topic echo /verification_camera/color/image_raw \
  sensor_msgs/msg/Image --field header --once \
  > "${OUT}/verification_camera_sample.yaml"
tf_ready=0
for _ in $(seq 1 4); do
  timeout 30 ros2 run tf2_ros tf2_echo odom base_footprint \
    > "${OUT}/tf_odom_base.txt" 2>&1 || true
  if grep -q 'Translation:' "${OUT}/tf_odom_base.txt"; then
    tf_ready=1
    break
  fi
done
test "${tf_ready}" -eq 1
python3 "${PACK_ROOT}/scripts/auto01_g2_obstacle_probe.py" \
  --trials "${TRIALS}" --output "${OUT}/obstacle_report.json"
