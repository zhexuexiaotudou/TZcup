#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_WS="${SANITATION_BASE_WS:-$HOME/sanitation_ws}"
DEMO_WS="${TZCUP_DEMO_WS:-$HOME/tzcup_visual_demo_ws}"
OUTPUT_DIR=""
DASHBOARD_PORT=8877
BUILD=1
GUI=1
RVIZ=1
RECORD_MCAP=1
VIDEO_MODE="auto"
KEEP_OPEN=0
OPEN_DASHBOARD=1
GAZEBO_TRAIL=1
SHOWCASE=0
MAP_SIZE="medium"
MANUAL_CONTROL=0
EXPECTED_COMPONENTS=17
MISSION_TIMEOUT_SEC=1800
RANDOM_SEED=0

usage() {
  cat <<'EOF'
Usage: bash scripts/run_visual_demo.sh [options]

Options:
  --output DIR          Evidence directory (default: artifacts/auto17_visual_demo_<UTC>)
  --workspace DIR       Dedicated overlay workspace
  --base-workspace DIR  Existing ROS 2/Gazebo workspace with upstream dependencies
  --dashboard-port N    Read-only dashboard port (default: 8877)
  --skip-build          Reuse the existing overlay install
  --no-gui              Do not open Gazebo GUI
  --no-rviz             Do not open RViz
  --no-mcap             Do not record MCAP
  --video MODE          auto, on, or off (default: auto)
  --gazebo-only         Show the full mission in Gazebo without browser or RViz
  --showcase            Use the bounded 6 m x 5 m demonstration task
  --map-size SIZE       small (30x20), medium (80x50), or large (200x100)
  --manual-control      Wait for the native Gazebo Start button
  --no-browser          Keep the read-only dashboard server hidden
  --no-gazebo-trail     Disable Gazebo cleaned-swath and mission markers
  --keep-open           Keep GUI/dashboard open after mission termination
  --timeout SEC         Coverage mission timeout (default: 1800)
  --seed N              Gazebo random seed (default: 0)
  -h, --help            Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) OUTPUT_DIR="$2"; shift 2 ;;
    --workspace) DEMO_WS="$2"; shift 2 ;;
    --base-workspace) BASE_WS="$2"; shift 2 ;;
    --dashboard-port) DASHBOARD_PORT="$2"; shift 2 ;;
    --skip-build) BUILD=0; shift ;;
    --no-gui) GUI=0; shift ;;
    --no-rviz) RVIZ=0; shift ;;
    --no-mcap) RECORD_MCAP=0; shift ;;
    --video) VIDEO_MODE="$2"; shift 2 ;;
    --gazebo-only) RVIZ=0; VIDEO_MODE=off; OPEN_DASHBOARD=0; shift ;;
    --showcase) SHOWCASE=1; EXPECTED_COMPONENTS=9; shift ;;
    --map-size) MAP_SIZE="$2"; shift 2 ;;
    --manual-control) MANUAL_CONTROL=1; shift ;;
    --no-browser) OPEN_DASHBOARD=0; shift ;;
    --no-gazebo-trail) GAZEBO_TRAIL=0; shift ;;
    --keep-open) KEEP_OPEN=1; shift ;;
    --timeout) MISSION_TIMEOUT_SEC="$2"; shift 2 ;;
    --seed) RANDOM_SEED="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "${VIDEO_MODE}" in auto|on|off) ;; *) echo "--video must be auto, on, or off" >&2; exit 2 ;; esac
case "${MAP_SIZE}" in small|medium|large) ;; *) echo "--map-size must be small, medium, or large" >&2; exit 2 ;; esac
if [[ "${MAP_SIZE}" == "small" ]]; then SHOWCASE=1; EXPECTED_COMPONENTS=9; fi
[[ "${DASHBOARD_PORT}" =~ ^[0-9]+$ ]] || { echo "dashboard port must be numeric" >&2; exit 2; }
[[ "${MISSION_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || { echo "timeout must be numeric" >&2; exit 2; }

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${ROOT}/artifacts/auto17_visual_demo_$(date -u +%Y%m%dT%H%M%SZ)"
fi
mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 3
fi
if [[ ! -f "${BASE_WS}/install/setup.bash" ]]; then
  echo "Base workspace is missing: ${BASE_WS}/install/setup.bash" >&2
  echo "Build the project dependencies first or pass --base-workspace." >&2
  exit 3
fi
if [[ "${GUI}" -eq 1 && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "Gazebo GUI requested but DISPLAY/WAYLAND_DISPLAY is unavailable." >&2
  exit 3
fi

pids=()
stopped=0
stop_group() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return
  local child_pids=()
  mapfile -t child_pids < <(pgrep -P "${pid}" 2>/dev/null || true)
  for child_pid in "${child_pids[@]}"; do
    kill -INT "${child_pid}" 2>/dev/null || true
  done
  kill -INT -- "-${pid}" 2>/dev/null || true
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 80); do
    local alive=0
    kill -0 "${pid}" 2>/dev/null && alive=1
    for child_pid in "${child_pids[@]}"; do
      kill -0 "${child_pid}" 2>/dev/null && alive=1
    done
    if [[ "${alive}" -eq 0 ]]; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  for child_pid in "${child_pids[@]}"; do
    kill -TERM "${child_pid}" 2>/dev/null || true
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    local alive=0
    kill -0 "${pid}" 2>/dev/null && alive=1
    for child_pid in "${child_pids[@]}"; do
      kill -0 "${child_pid}" 2>/dev/null && alive=1
    done
    if [[ "${alive}" -eq 0 ]]; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  for child_pid in "${child_pids[@]}"; do
    kill -KILL "${child_pid}" 2>/dev/null || true
  done
  kill -KILL -- "-${pid}" 2>/dev/null || true
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
stop_all() {
  [[ "${stopped}" -eq 0 ]] || return
  stopped=1
  for (( index=${#pids[@]}-1; index>=0; index-- )); do
    stop_group "${pids[index]}"
  done
  if [[ -n "${world_file:-}" ]]; then
    pkill -INT -f "gz sim.*${world_file}" 2>/dev/null || true
  fi
  if [[ -n "${rviz_config:-}" ]]; then
    pkill -TERM -f "rviz2 -d ${rviz_config}" 2>/dev/null || true
  fi
}
on_exit() {
  stop_all
}
trap on_exit EXIT INT TERM

set +u
source /opt/ros/jazzy/setup.bash
source "${BASE_WS}/install/setup.bash"
set -u

if [[ "${BUILD}" -eq 1 ]]; then
  echo "[AUTO-17] Synchronizing project packages to ${DEMO_WS}"
  mkdir -p "${DEMO_WS}/src"
  rsync -a --delete "${ROOT}/starter_ws/src/" "${DEMO_WS}/src/"
  (
    cd "${DEMO_WS}"
    set +u
    source /opt/ros/jazzy/setup.bash
    source "${BASE_WS}/install/setup.bash"
    set -u
    colcon build --symlink-install --event-handlers console_direct+ \
      > "${OUTPUT_DIR}/overlay_build.log" 2>&1
  )
fi
if [[ ! -f "${DEMO_WS}/install/setup.bash" ]]; then
  echo "AUTO-17 overlay is missing: ${DEMO_WS}/install/setup.bash" >&2
  exit 3
fi
set +u
source "${DEMO_WS}/install/setup.bash"
set -u

export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export LINOROBOT2_BASE=4wd

runtime="${OUTPUT_DIR}/runtime"
mkdir -p "${runtime}"
navigation_share="$(ros2 pkg prefix sanitation_navigation)/share/sanitation_navigation"
tasks_share="$(ros2 pkg prefix sanitation_tasks)/share/sanitation_tasks"
hmi_share="$(ros2 pkg prefix sanitation_hmi)/share/sanitation_hmi"
control_prefix="$(ros2 pkg prefix sanitation_gazebo_control)"
control_share="${control_prefix}/share/sanitation_gazebo_control"
export GZ_GUI_PLUGIN_PATH="${control_prefix}/lib${GZ_GUI_PLUGIN_PATH:+:${GZ_GUI_PLUGIN_PATH}}"
map_root="${navigation_share}/maps"
nav_params="${runtime}/nav2_autonomous_navigation_profile_v1.yaml"
mission_config="${runtime}/demo_area_autonomous_navigation_profile_v1.yaml"
mission_template="${tasks_share}/config/demo_area.yaml"
if [[ "${SHOWCASE}" -eq 1 ]]; then
  mission_config="${runtime}/showcase_area_autonomous_navigation_profile_v1.yaml"
  mission_template="${tasks_share}/config/showcase_area.yaml"
fi
world_file="$(ros2 pkg prefix sanitation_worlds)/share/sanitation_worlds/worlds/sanitation_campus_${MAP_SIZE}.sdf"
world_name="sanitation_campus_${MAP_SIZE}"
gui_config="${control_share}/config/mission_control_${MAP_SIZE}.config"
rviz_config="${hmi_share}/rviz/visual_demo.rviz"

if python3 - "${DASHBOARD_PORT}" <<'PY'
import socket
import sys

port = int(sys.argv[1])
with socket.socket() as client:
    client.settimeout(0.5)
    raise SystemExit(0 if client.connect_ex(("127.0.0.1", port)) == 0 else 1)
PY
then
  echo "Dashboard port ${DASHBOARD_PORT} is already in use; choose --dashboard-port." >&2
  exit 3
fi

python3 "${ROOT}/scripts/stage5br6w_profile.py" \
  --base-nav2 "${navigation_share}/config/nav2.yaml" \
  --base-mission "${mission_template}" \
  --profile "${navigation_share}/config/autonomous_navigation_profile_v1.yaml" \
  --nav2-output "${nav_params}" \
  --mission-output "${mission_config}"

echo "[AUTO-17] Evidence: ${OUTPUT_DIR}"
echo "[AUTO-17] Dashboard: http://127.0.0.1:${DASHBOARD_PORT}"

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null
sleep 1

existing_nodes="$(
  timeout 10 ros2 node list --no-daemon --spin-time 3 2>/dev/null || true
)"
for node in /controller_server /coverage_server /sanitation_live_dashboard; do
  if grep -Fxq "${node}" <<< "${existing_nodes}"; then
    echo "Existing AUTO-17 runtime node detected: ${node}" >&2
    echo "Stop the previous demo before starting another run." >&2
    exit 3
  fi
done

gui_value=false
[[ "${GUI}" -eq 1 ]] && gui_value=true
setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:="${gui_value}" random_seed:="${RANDOM_SEED}" gnss_profile:=rtk_fixed \
  world_file:="${world_file}" world_name:="${world_name}" \
  gui_config:="${gui_config}" \
  camera_profile:=V5_retracted fusion_mode:=hybrid_rtk_scan_imu_wheel \
  enable_scan_refiner:=true \
  > "${OUTPUT_DIR}/localization.log" 2>&1 &
pids+=("$!")

setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  footprint_profile:=autonomous_navigation_profile_v1 \
  map_file:="${map_root}/stage4v_surveyed_reference.yaml" \
  keepout_map:="${map_root}/stage4v_filters/keepout_mask.yaml" \
  speed_map:="${map_root}/stage4v_filters/speed_mask.yaml" \
  operational_profile:=localization_coverage max_linear_velocity:=0.45 \
  max_angular_velocity:=0.35 \
  > "${OUTPUT_DIR}/navigation.log" 2>&1 &
pids+=("$!")

setsid ros2 launch sanitation_coverage coverage.launch.py \
  footprint_profile:=autonomous_navigation_profile_v1 \
  > "${OUTPUT_DIR}/coverage_server.log" 2>&1 &
pids+=("$!")

setsid ros2 run sanitation_hmi sanitation_live_dashboard --ros-args \
  -p use_sim_time:=true \
  -p port:="${DASHBOARD_PORT}" \
    -p output_dir:="${OUTPUT_DIR}" \
    -p mission_config:="${mission_config}" \
    -p expected_components:="${EXPECTED_COMPONENTS}" \
  > "${OUTPUT_DIR}/dashboard.log" 2>&1 &
dashboard_pid="$!"
pids+=("${dashboard_pid}")

if [[ "${GUI}" -eq 1 && "${GAZEBO_TRAIL}" -eq 1 ]]; then
  setsid ros2 run sanitation_gazebo_visualization cleaning_visualizer --ros-args \
    -p use_sim_time:=true \
    -p operation_width_m:=0.65 \
    -p expected_components:="${EXPECTED_COMPONENTS}" \
    -p mission_config:="${mission_config}" \
    -p world_to_map_x:=8.0 \
    -p world_to_map_y:=0.0 \
    -p world_to_map_yaw:=0.0 \
    -p service_timeout_ms:=3000 \
    > "${OUTPUT_DIR}/gazebo_cleaning_visualizer.log" 2>&1 &
  pids+=("$!")
fi

if [[ "${RVIZ}" -eq 1 ]]; then
  setsid rviz2 -d "${rviz_config}" \
    --ros-args -p use_sim_time:=true \
    > "${OUTPUT_DIR}/rviz.log" 2>&1 &
  pids+=("$!")
fi

ready=0
for _ in $(seq 1 150); do
  if ! kill -0 "${dashboard_pid}" 2>/dev/null; then
    echo "Live dashboard process exited before readiness." >&2
    tail -50 "${OUTPUT_DIR}/dashboard.log" >&2 || true
    exit 4
  fi
  topics="$(
    timeout 8 ros2 topic list --no-daemon --spin-time 3 2>/dev/null || true
  )"
  services="$(
    timeout 8 ros2 service list --no-daemon --spin-time 3 \
      --include-hidden-services \
      2>/dev/null || true
  )"
  dashboard_health="$(
    curl --fail --silent --max-time 2 \
      "http://127.0.0.1:${DASHBOARD_PORT}/healthz" 2>/dev/null || true
  )"
  if grep -q '^/localization/fused_pose$' <<< "${topics}" &&
    grep -q '^/map$' <<< "${topics}" &&
    grep -q '^/scan$' <<< "${topics}" &&
    grep -q '^/cmd_vel$' <<< "${topics}" &&
    grep -q '^/compute_coverage_path/_action/send_goal$' <<< "${services}" &&
    grep -q '^/follow_path/_action/send_goal$' <<< "${services}" &&
    grep -q '^/navigate_to_pose/_action/send_goal$' <<< "${services}" &&
    grep -q '^/controller_server/get_state$' <<< "${services}" &&
    grep -q '"mission_status"' <<< "${dashboard_health}"
  then
    printf '%s\n' "${dashboard_health}" > "${OUTPUT_DIR}/dashboard_health.json"
    ready=1
    break
  fi
  sleep 1
done
if [[ "${ready}" -ne 1 ]]; then
  echo "AUTO-17 runtime did not become ready within 150 seconds." >&2
  exit 4
fi

camera_follow_requested=0
camera_track_request='track_mode: FOLLOW_LOOK_AT, follow_target: {name: "sanitation_vehicle", type: MODEL}, track_target: {name: "sanitation_vehicle", type: MODEL}, follow_offset: {x: -4.5, y: -3.0, z: 3.2}, follow_pgain: 0.35, track_pgain: 0.35'
if [[ "${SHOWCASE}" -eq 1 ]]; then
  camera_track_request='track_mode: FOLLOW_LOOK_AT, follow_target: {name: "sanitation_vehicle", type: MODEL}, track_target: {name: "sanitation_vehicle", type: MODEL}, follow_offset: {x: -8.0, y: -8.0, z: 10.0}, follow_pgain: 0.25, track_pgain: 0.35'
fi
if [[ "${GUI}" -eq 1 ]]; then
  for _ in $(seq 1 10); do
    gz_topics="$(timeout 3 gz topic -l 2>/dev/null || true)"
    if grep -Fxq '/gui/track' <<< "${gz_topics}"; then
      if gz topic -t /gui/track -m gz.msgs.CameraTrack -p \
        "${camera_track_request}" \
        > "${OUTPUT_DIR}/gazebo_camera_follow.log" 2>&1
      then
        camera_follow_requested=1
      fi
      break
    fi
    sleep 0.5
  done
fi

if [[ "${OPEN_DASHBOARD}" -eq 1 ]] && command -v powershell.exe >/dev/null 2>&1; then
  powershell.exe -NoProfile -Command \
    "Start-Process 'http://127.0.0.1:${DASHBOARD_PORT}'" \
    > "${OUTPUT_DIR}/browser_open.log" 2>&1 || true
  powershell.exe -NoProfile -Command \
    "\$shell = New-Object -ComObject WScript.Shell; Start-Sleep -Seconds 2; [void]\$shell.AppActivate('TZcup')" \
    >> "${OUTPUT_DIR}/browser_open.log" 2>&1 || true
elif [[ "${OPEN_DASHBOARD}" -eq 1 ]] && command -v xdg-open >/dev/null 2>&1; then
  xdg-open "http://127.0.0.1:${DASHBOARD_PORT}" \
    > "${OUTPUT_DIR}/browser_open.log" 2>&1 || true
fi

if [[ "${RECORD_MCAP}" -eq 1 ]]; then
  setsid ros2 bag record --storage mcap \
    --output "${OUTPUT_DIR}/visual_demo_bag" \
    /clock /tf /tf_static /scan /odom /localization/fused_pose \
    /ground_truth/odom /cmd_vel /cmd_vel_gate /brush_enabled \
    /emergency_stop /coverage/state /coverage/component_state \
    /coverage/current_path /coverage/evaluation_sample \
    /coverage/diagnostics /local_costmap/costmap /global_costmap/costmap \
    > "${OUTPUT_DIR}/rosbag.log" 2>&1 &
  pids+=("$!")
fi

effective_video_mode="${VIDEO_MODE}"
video_pid=""
video_backend=""
windows_frames_dir="${OUTPUT_DIR}/dashboard_video_frames"
windows_stop_file="${OUTPUT_DIR}/dashboard_video_capture.stop"
if [[ "${VIDEO_MODE}" != "off" ]]; then
  if command -v ffmpeg >/dev/null 2>&1 && command -v py.exe >/dev/null 2>&1 &&
    command -v wslpath >/dev/null 2>&1
  then
    rm -f "${windows_stop_file}"
    mkdir -p "${windows_frames_dir}"
    windows_recorder="$(wslpath -w "${ROOT}/scripts/dashboard_telemetry_frames.py")"
    windows_frames="$(wslpath -w "${windows_frames_dir}")"
    windows_stop="$(wslpath -w "${windows_stop_file}")"
    py.exe -3 "${windows_recorder}" \
      --url "http://127.0.0.1:${DASHBOARD_PORT}/api/v1/telemetry" \
      --output-dir "${windows_frames}" \
      --stop-file "${windows_stop}" \
      --fps 2 \
      > "${OUTPUT_DIR}/video_capture.log" 2>&1 &
    video_pid="$!"
    video_backend=dashboard_telemetry_frames
  elif command -v ffmpeg >/dev/null 2>&1 && command -v xdpyinfo >/dev/null 2>&1 &&
    [[ -n "${DISPLAY:-}" ]]
  then
    xdpy_info="$(xdpyinfo 2>/dev/null || true)"
    dimensions="$(awk '/dimensions:/ {print $2; exit}' <<< "${xdpy_info}")"
    if [[ "${dimensions}" =~ ^[0-9]+x[0-9]+$ ]]; then
      setsid ffmpeg -nostdin -y -f x11grab -draw_mouse 0 -framerate 15 \
        -video_size "${dimensions}" -i "${DISPLAY}" \
        -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p \
        "${OUTPUT_DIR}/visual_demo.mp4" \
        > "${OUTPUT_DIR}/video.log" 2>&1 &
      video_pid="$!"
      pids+=("${video_pid}")
      video_backend=x11grab
    elif [[ "${VIDEO_MODE}" == "on" ]]; then
      echo "Unable to determine X11 dimensions for required video." >&2
      exit 5
    else
      effective_video_mode=off
    fi
  elif [[ "${VIDEO_MODE}" == "on" ]]; then
    echo "Video recording requires ffmpeg, xdpyinfo, and DISPLAY." >&2
    exit 5
  else
    effective_video_mode=off
    echo "Video recording unavailable; continuing because --video auto was used." \
      > "${OUTPUT_DIR}/video_unavailable.txt"
  fi
fi

ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}" \
  > "${OUTPUT_DIR}/emergency_stop_available.log" 2>&1

set +e
timeout "${MISSION_TIMEOUT_SEC}" ros2 run sanitation_coverage coverage_probe --ros-args \
  -p use_sim_time:=true \
  -p manual_start:="$([[ "${MANUAL_CONTROL}" -eq 1 ]] && echo true || echo false)" \
  -p output_path:="${OUTPUT_DIR}/coverage_report.json" \
  -p config_path:="${mission_config}" \
  -p path_output_path:="${OUTPUT_DIR}/coverage_path.json" \
  -p trajectory_output_path:="${OUTPUT_DIR}/coverage_trajectory.csv" \
  > "${OUTPUT_DIR}/coverage_probe.log" 2>&1
coverage_code=$?
set -e

sleep 8
if [[ "${KEEP_OPEN}" -eq 1 ]]; then
  echo "[AUTO-17] Mission ended. Press Ctrl+C when visual inspection is complete."
  keep_open_stop=0
  trap 'keep_open_stop=1' INT TERM
  while [[ "${keep_open_stop}" -eq 0 ]]; do
    if [[ "${GUI}" -eq 1 ]] && ! pgrep -f 'gz sim.*-g' >/dev/null 2>&1; then
      echo "[AUTO-17] Gazebo GUI closed; finishing the launcher."
      break
    fi
    sleep 1
  done
  trap on_exit EXIT INT TERM
fi

if [[ "${video_backend}" == "dashboard_telemetry_frames" ]]; then
  touch "${windows_stop_file}"
  for _ in $(seq 1 50); do
    if ! kill -0 "${video_pid}" 2>/dev/null; then
      wait "${video_pid}" 2>/dev/null || true
      break
    fi
    sleep 0.1
  done
  if kill -0 "${video_pid}" 2>/dev/null; then
    kill "${video_pid}" 2>/dev/null || true
    wait "${video_pid}" 2>/dev/null || true
  fi
  ffmpeg -nostdin -y -framerate 2 \
    -i "${windows_frames_dir}/frame_%06d.jpg" \
    -vf "fps=10,format=yuv420p" \
    -c:v libx264 -preset veryfast -crf 24 \
    "${OUTPUT_DIR}/visual_demo.mp4" \
    > "${OUTPUT_DIR}/video.log" 2>&1
fi

stop_all
trap - EXIT INT TERM

if [[ -f "${OUTPUT_DIR}/visual_demo.mp4" ]] &&
  command -v ffmpeg >/dev/null 2>&1
then
  ffmpeg -nostdin -y -sseof -5 -i "${OUTPUT_DIR}/visual_demo.mp4" \
    -frames:v 1 -update 1 "${OUTPUT_DIR}/visual_demo_frame.png" \
    > "${OUTPUT_DIR}/video_frame.log" 2>&1 || true
fi

summary_args=(
  --output-dir "${OUTPUT_DIR}"
  --coverage-exit-code "${coverage_code}"
  --video-mode "${effective_video_mode}"
)
[[ "${RECORD_MCAP}" -eq 1 ]] && summary_args+=(--mcap-required)
[[ "${camera_follow_requested}" -eq 1 ]] && summary_args+=(--camera-follow-requested)
python3 "${ROOT}/scripts/visual_demo_summary.py" "${summary_args[@]}"
