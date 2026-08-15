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
MAP_SIZE="small"
MANUAL_CONTROL=0
COMPETITION_PROFILE=0
COMPETITION_LANE="representative"
EXPECTED_COMPONENTS=17
MISSION_TIMEOUT_SEC=1800
RANDOM_SEED=0
GAZEBO_GUI_RENDERER="auto"
SIMULATION_SPEED="fast"
COVERAGE_PROFILE="ackermann"
DRIVE_MODEL="ackermann"
DYNAMIC_OBSTACLE_TRIALS=0
SIMULATION_RENDER_ENGINE="ogre2"
REPAIR_EVALUATION_INJECTION=0

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
  --map-size SIZE       small (independent 16x12 demo), medium (80x50), or large (200x100)
  --simulation-speed MODE
                        normal (1x), fast (2x), or turbo (3x) simulator RTF;
                        product Ackermann speed stays physically fixed
  --coverage-profile PROFILE
                        ackermann (default), optimized (legacy skid-steer RTR), or legacy
  --drive-model MODEL   ackermann (default) or skid_steer_legacy
  --dynamic-obstacle-trials N
                        Run N physical SetEntityPose interactions (formal: 20)
  --simulation-render-engine ENGINE
                        ogre2 (default) or ogre (headless software fallback)
  --repair-evaluation-injection
                        Inject one fused-pose brush dropout for repair testing
  --manual-control      Wait for the native Gazebo Start button
  --competition-profile Use the 20,000 m2 map with one representative live
                        zone; map-scale compatibility only, not full-map evidence
  --competition-lane LANE
                        representative (default) or long-lane efficiency candidate
  --no-browser          Keep the read-only dashboard server hidden
  --no-gazebo-trail     Disable Gazebo cleaned-swath and mission markers
  --keep-open           Keep GUI/dashboard open after mission termination
  --timeout SEC         Coverage mission timeout (default: 1800)
  --seed N              Gazebo random seed (default: 0)
  --gazebo-gui-renderer MODE
                        auto, d3d12, or software (default: auto)
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
    --competition-profile) COMPETITION_PROFILE=1; MAP_SIZE=large; EXPECTED_COMPONENTS=7; shift ;;
    --competition-lane) COMPETITION_LANE="$2"; shift 2 ;;
    --no-browser) OPEN_DASHBOARD=0; shift ;;
    --no-gazebo-trail) GAZEBO_TRAIL=0; shift ;;
    --keep-open) KEEP_OPEN=1; shift ;;
    --timeout) MISSION_TIMEOUT_SEC="$2"; shift 2 ;;
    --seed) RANDOM_SEED="$2"; shift 2 ;;
    --gazebo-gui-renderer) GAZEBO_GUI_RENDERER="$2"; shift 2 ;;
    --simulation-speed) SIMULATION_SPEED="$2"; shift 2 ;;
    --coverage-profile) COVERAGE_PROFILE="$2"; shift 2 ;;
    --drive-model) DRIVE_MODEL="$2"; shift 2 ;;
    --dynamic-obstacle-trials) DYNAMIC_OBSTACLE_TRIALS="$2"; shift 2 ;;
    --simulation-render-engine) SIMULATION_RENDER_ENGINE="$2"; shift 2 ;;
    --repair-evaluation-injection) REPAIR_EVALUATION_INJECTION=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "${VIDEO_MODE}" in auto|on|off) ;; *) echo "--video must be auto, on, or off" >&2; exit 2 ;; esac
case "${MAP_SIZE}" in small|medium|large) ;; *) echo "--map-size must be small, medium, or large" >&2; exit 2 ;; esac
case "${GAZEBO_GUI_RENDERER}" in auto|d3d12|software) ;; *) echo "--gazebo-gui-renderer must be auto, d3d12, or software" >&2; exit 2 ;; esac
case "${SIMULATION_SPEED}" in normal|fast|turbo) ;; *) echo "--simulation-speed must be normal, fast, or turbo" >&2; exit 2 ;; esac
case "${COVERAGE_PROFILE}" in ackermann|optimized|legacy) ;; *) echo "--coverage-profile must be ackermann, optimized, or legacy" >&2; exit 2 ;; esac
case "${DRIVE_MODEL}" in ackermann|skid_steer_legacy) ;; *) echo "--drive-model must be ackermann or skid_steer_legacy" >&2; exit 2 ;; esac
case "${COMPETITION_LANE}" in representative|efficiency) ;; *) echo "--competition-lane must be representative or efficiency" >&2; exit 2 ;; esac
if [[ "${COMPETITION_LANE}" != "representative" && "${COMPETITION_PROFILE}" -eq 0 ]]; then
  echo "--competition-lane requires --competition-profile" >&2
  exit 2
fi
if [[ "${DRIVE_MODEL}" == "ackermann" && "${COVERAGE_PROFILE}" != "ackermann" ]]; then
  echo "Ackermann drive requires --coverage-profile ackermann; skid-steer RTR/Spin is forbidden." >&2
  exit 2
fi
if [[ "${DRIVE_MODEL}" == "skid_steer_legacy" && "${COVERAGE_PROFILE}" == "ackermann" ]]; then
  echo "Ackermann connectors require --drive-model ackermann." >&2
  exit 2
fi
if [[ "${DRIVE_MODEL}" == "ackermann" && "${COMPETITION_PROFILE}" -eq 0 ]]; then
  MAP_SIZE="small"
  SHOWCASE=1
  EXPECTED_COMPONENTS=17
fi
case "${SIMULATION_RENDER_ENGINE}" in ogre2|ogre) ;; *) echo "--simulation-render-engine must be ogre2 or ogre" >&2; exit 2 ;; esac
if [[ "${MAP_SIZE}" == "small" ]]; then SHOWCASE=1; EXPECTED_COMPONENTS=17; fi
[[ "${DASHBOARD_PORT}" =~ ^[0-9]+$ ]] || { echo "dashboard port must be numeric" >&2; exit 2; }
[[ "${MISSION_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || { echo "timeout must be numeric" >&2; exit 2; }
[[ "${DYNAMIC_OBSTACLE_TRIALS}" =~ ^[0-9]+$ ]] || { echo "dynamic obstacle trials must be numeric" >&2; exit 2; }
if [[ "${DYNAMIC_OBSTACLE_TRIALS}" -gt 0 && "${MAP_SIZE}" != "small" ]]; then
  echo "dynamic obstacle trials are currently defined only for the independent small field" >&2
  exit 2
fi
if [[ "${REPAIR_EVALUATION_INJECTION}" -eq 1 && ( "${MAP_SIZE}" != "small" || "${COVERAGE_PROFILE}" != "optimized" || "${DRIVE_MODEL}" != "skid_steer_legacy" ) ]]; then
  echo "repair evaluation injection requires the optimized independent small field" >&2
  exit 2
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
  if [[ "${DRIVE_MODEL}" == "ackermann" ]]; then
    OUTPUT_DIR="${ROOT}/artifacts/ackermann_realism_$(date -u +%Y%m%dT%H%M%SZ)"
  else
    OUTPUT_DIR="${ROOT}/artifacts/auto17_visual_demo_$(date -u +%Y%m%dT%H%M%SZ)"
  fi
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
  for signal in INT TERM KILL; do
    for (( index=${#pids[@]}-1; index>=0; index-- )); do
      pid="${pids[index]}"
      [[ -n "${pid}" ]] || continue
      kill -"${signal}" -- "-${pid}" 2>/dev/null || true
      kill -"${signal}" "${pid}" 2>/dev/null || true
    done
    wait_steps=10
    [[ "${signal}" == "INT" ]] && wait_steps=80
    [[ "${signal}" == "KILL" ]] && wait_steps=1
    for _ in $(seq 1 "${wait_steps}"); do
      alive=0
      for pid in "${pids[@]}"; do
        if kill -0 "${pid}" 2>/dev/null || pgrep -g "${pid}" >/dev/null 2>&1; then
          alive=1
          break
        fi
      done
      [[ "${alive}" -eq 0 ]] && break
      sleep 0.1
    done
    [[ "${alive:-0}" -eq 0 ]] && break
  done
  for pid in "${pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  if [[ -n "${world_file:-}" ]]; then
    pkill -INT -f "gz sim.*${world_file}" 2>/dev/null || true
  fi
  if [[ -n "${rviz_config:-}" ]]; then
    pkill -TERM -f "rviz2 -d ${rviz_config}" 2>/dev/null || true
  fi
}
on_exit() {
  exit_code=$?
  trap - EXIT INT TERM
  stop_all
  exit "${exit_code}"
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

gazebo_gui_renderer="${GAZEBO_GUI_RENDERER}"
if [[ "${gazebo_gui_renderer}" == "auto" ]]; then
  if [[ -d /mnt/wslg ]] && grep -qi microsoft /proc/version; then
    # Ogre2 can create a healthy Qt shell but leave the 3D Scene fully black
    # with Mesa's D3D12 driver under WSLg. Keep the headless Gazebo server and
    # sensors on D3D12, while using the reliable X11 llvmpipe path for the GUI.
    gazebo_gui_renderer="software"
  else
    gazebo_gui_renderer="d3d12"
  fi
fi
if [[ "${gazebo_gui_renderer}" == "software" ]]; then
  gazebo_gui_env=(env GALLIUM_DRIVER=llvmpipe LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb)
else
  gazebo_gui_env=(env GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA)
fi

runtime="${OUTPUT_DIR}/runtime"
mkdir -p "${runtime}"
navigation_share="$(ros2 pkg prefix sanitation_navigation)/share/sanitation_navigation"
coverage_share="$(ros2 pkg prefix sanitation_coverage)/share/sanitation_coverage"
tasks_share="$(ros2 pkg prefix sanitation_tasks)/share/sanitation_tasks"
hmi_share="$(ros2 pkg prefix sanitation_hmi)/share/sanitation_hmi"
control_prefix="$(ros2 pkg prefix sanitation_gazebo_control)"
control_share="${control_prefix}/share/sanitation_gazebo_control"
python3 "${ROOT}/scripts/validate_hardware_interface_contract.py" \
  "${tasks_share}/config/hardware_interface_contract.yaml" \
  --output "${OUTPUT_DIR}/hardware_interface_contract_validation.json"
export GZ_GUI_PLUGIN_PATH="${control_prefix}/lib${GZ_GUI_PLUGIN_PATH:+:${GZ_GUI_PLUGIN_PATH}}"
map_root="${navigation_share}/maps"
nav_params="${runtime}/nav2_autonomous_navigation_profile_v1.yaml"
mission_config="${runtime}/demo_area_autonomous_navigation_profile_v1.yaml"
mission_template="${tasks_share}/config/demo_area.yaml"
footprint_profile="autonomous_navigation_profile_v1"
coverage_params=""
map_file="${map_root}/stage4v_surveyed_reference.yaml"
keepout_map="${map_root}/stage4v_filters/keepout_mask.yaml"
speed_map="${map_root}/stage4v_filters/speed_mask.yaml"
spawn_x="-8.0"
spawn_y="0.0"
spawn_yaw="0.0"
world_to_map_x="8.0"
world_to_map_y="0.0"
initial_pose_x="0.0"
initial_pose_y="0.0"
initial_pose_yaw="0.0"
cleaning_width="1.32"
brush_center_y="0.52"
max_linear_velocity="1.0"
max_angular_velocity="0.70"
localization_fusion_mode="hybrid_rtk_scan_imu_wheel"
enable_scan_refiner="true"
profile_label="STANDARD DEMO"
mission_scope="LIVE DEMO AREA"
map_area_m2="4000.0"
if [[ "${SHOWCASE}" -eq 1 ]]; then
  mission_config="${runtime}/showcase_area_autonomous_navigation_profile_v1.yaml"
  mission_template="${tasks_share}/config/showcase_area.yaml"
fi
simulation_rtf="2.0"
simulation_speed_label="2X TARGET"
case "${SIMULATION_SPEED}" in
  normal) simulation_rtf="1.0"; simulation_speed_label="1X NORMAL" ;;
  turbo) simulation_rtf="3.0"; simulation_speed_label="3X TARGET" ;;
esac
if [[ "${MAP_SIZE}" == "small" ]]; then
  mission_config="${runtime}/competition_demo_area_autonomous_navigation_profile_v1.yaml"
  mission_template="${tasks_share}/config/competition_demo_area_skid_steer_optimized.yaml"
  profile_label="SKID-STEER OPTIMIZED DEMO"
  mission_scope="OUTER TASK 30 M2 / CLEANABLE 12 M2"
  map_area_m2="30.0"
  coverage_params="${coverage_share}/config/coverage_skid_steer_optimized.yaml"
  # The independent demo world does not share the frozen Stage4V scan map.
  # Use the deployable RTK + wheel/IMU lane instead of accepting map-mismatched
  # scan corrections. Medium/large retain hybrid scan fallback.
  localization_fusion_mode="rtk_imu_wheel"
  enable_scan_refiner="false"
  if [[ "${DRIVE_MODEL}" != "ackermann" ]]; then
    cleaning_width="0.65"
    brush_center_y="0.23"
    max_linear_velocity="0.65"
    max_angular_velocity="0.55"
  fi
  if [[ "${COVERAGE_PROFILE}" == "legacy" ]]; then
    mission_template="${tasks_share}/config/competition_demo_area.yaml"
    profile_label="LEGACY DUBINS BASELINE"
    coverage_params="${coverage_share}/config/coverage_demo_overlap.yaml"
  fi
  if [[ "${COVERAGE_PROFILE}" == "ackermann" ]]; then
    mission_template="${tasks_share}/config/competition_ackermann_demo_area.yaml"
    mission_config="${runtime}/competition_ackermann_demo_area.yaml"
    nav_params="${runtime}/nav2_ackermann.yaml"
    profile_label="ACKERMANN REALISM DEMO"
    mission_scope="OUTER TURNING APRON 156.0 M2 / CLEANABLE 12 M2"
    map_area_m2="156.0"
    coverage_params="${coverage_share}/config/coverage_ackermann.yaml"
    # The expanded footprint-checked turning apron is intentionally larger
    # than the Stage4V surveyed scan map. Scan correction is disabled for this
    # independent world, so use the repository's bounded free-space map.
    map_file="${map_root}/sanitation_test_map.yaml"
    cleaning_width="1.32"
    brush_center_y="0.52"
    max_linear_velocity="1.0"
    max_angular_velocity="0.70"
  fi
  if [[ "${DRIVE_MODEL}" != "ackermann" && "${SIMULATION_SPEED}" == "fast" ]]; then max_linear_velocity="0.70"; max_angular_velocity="0.60"; fi
  if [[ "${DRIVE_MODEL}" != "ackermann" && "${SIMULATION_SPEED}" == "turbo" ]]; then max_linear_velocity="0.90"; max_angular_velocity="0.75"; fi
fi
if [[ "${COMPETITION_PROFILE}" -eq 1 ]]; then
  competition_runtime="${runtime}/competition_profile"
  python3 "${ROOT}/scripts/generate_competition_gazebo_profile.py" \
    --output "${competition_runtime}" \
    > "${OUTPUT_DIR}/competition_profile_generation.json"
  if [[ "${DRIVE_MODEL}" == "ackermann" ]]; then
    nav_params="${runtime}/nav2_ackermann.yaml"
    mission_config="${competition_runtime}/competition_zone_ackermann.yaml"
    coverage_params="${competition_runtime}/competition_coverage_ackermann.yaml"
    EXPECTED_COMPONENTS=17
    profile_label="ACKERMANN COMPETITION MAP VALIDATION"
    mission_scope="LIVE ACKERMANN ZONE 108 M2 / FULL MAP 20,000 M2"
    if [[ "${COMPETITION_LANE}" == "efficiency" ]]; then
      mission_config="${competition_runtime}/competition_efficiency_ackermann.yaml"
      coverage_params="${competition_runtime}/competition_coverage_efficiency_ackermann.yaml"
      EXPECTED_COMPONENTS=95
      profile_label="ACKERMANN EFFICIENCY CANDIDATE LANE"
      mission_scope="EFFICIENCY CANDIDATE 10,440 M2 / FULL MAP 20,000 M2"
    fi
  else
    nav_params="${navigation_share}/config/nav2_auto12.yaml"
    mission_config="${competition_runtime}/competition_zone_auto12.yaml"
    footprint_profile="auto12_efficiency_v1"
    coverage_params="${DEMO_WS}/install/sanitation_coverage/share/sanitation_coverage/config/coverage_auto12.yaml"
    profile_label="LEGACY COMPETITION VISUALIZATION"
    mission_scope="LIVE ZONE 108 M2 / FULL MAP"
  fi
  map_file="${competition_runtime}/competition_map.yaml"
  keepout_map="${competition_runtime}/competition_keepout.yaml"
  speed_map="${competition_runtime}/competition_speed.yaml"
  spawn_x="-90.0"
  spawn_y="0.0"
  world_to_map_x="100.0"
  world_to_map_y="50.0"
  initial_pose_x="10.0"
  initial_pose_y="50.0"
  cleaning_width="1.32"
  brush_center_y="0.52"
  max_linear_velocity="1.0"
  max_angular_velocity="0.72"
  map_area_m2="20000.0"
  if [[ "${DRIVE_MODEL}" == "ackermann" ]]; then
    # Product entry starts inside the external apron, behind and aligned with
    # the first swath.  This avoids using a long reverse-only map transit as
    # the normal deployment path while preserving a measured brush-off lead-in.
    spawn_x="-94.80"
    spawn_y="-4.05"
    spawn_yaw="0.0"
    initial_pose_x="5.20"
    initial_pose_y="45.95"
    initial_pose_yaw="0.0"
    if [[ "${COMPETITION_LANE}" == "efficiency" ]]; then
      spawn_x="-94.80"
      spawn_y="-30.05"
      initial_pose_x="5.20"
      initial_pose_y="19.95"
    fi
  fi
fi
world_file="$(ros2 pkg prefix sanitation_worlds)/share/sanitation_worlds/worlds/sanitation_campus_${MAP_SIZE}.sdf"
world_name="sanitation_campus_${MAP_SIZE}"
gui_config="${control_share}/config/mission_control_${MAP_SIZE}.config"
if [[ "${MAP_SIZE}" == "small" ]]; then
  world_file="$(ros2 pkg prefix sanitation_worlds)/share/sanitation_worlds/worlds/sanitation_competition_demo.sdf"
  world_name="sanitation_competition_demo"
  gui_config="${control_share}/config/mission_control_demo.config"
fi
if [[ "${DRIVE_MODEL}" == "ackermann" && "${COMPETITION_PROFILE}" -eq 0 ]]; then
  world_file="$(ros2 pkg prefix sanitation_worlds)/share/sanitation_worlds/worlds/sanitation_competition_ackermann_demo.sdf"
  world_name="sanitation_competition_ackermann_demo"
fi
if [[ "${MANUAL_CONTROL}" -eq 1 ]]; then
  manual_gui_config="${runtime}/mission_control_${MAP_SIZE}_manual.config"
  python3 "${ROOT}/scripts/prepare_manual_gazebo_gui.py" \
    --input "${gui_config}" --output "${manual_gui_config}"
  gui_config="${manual_gui_config}"
fi
runtime_world="${runtime}/${world_name}_${SIMULATION_SPEED}.sdf"
python3 - "${world_file}" "${runtime_world}" "${simulation_rtf}" "${SIMULATION_RENDER_ENGINE}" <<'PY'
from pathlib import Path
import re
import sys
source, output, factor, render_engine = sys.argv[1:]
text = Path(source).read_text(encoding="utf-8")
text, count = re.subn(
    r"<real_time_factor>[^<]+</real_time_factor>",
    f"<real_time_factor>{factor}</real_time_factor>",
    text,
    count=1,
)
if count != 1:
    raise SystemExit("world does not define exactly one real_time_factor")
text, render_count = re.subn(
    r"<render_engine>[^<]+</render_engine>",
    f"<render_engine>{render_engine}</render_engine>",
    text,
    count=1,
)
if render_count != 1:
    raise SystemExit("world does not define exactly one sensor render_engine")
Path(output).write_text(text, encoding="utf-8")
PY
world_file="${runtime_world}"
rviz_config="${hmi_share}/rviz/visual_demo.rviz"
server_headless_rendering="true"
if [[ "${SIMULATION_RENDER_ENGINE}" == "ogre" ]]; then
  server_headless_rendering="false"
fi

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

if [[ "${COMPETITION_PROFILE}" -eq 0 && "${DRIVE_MODEL}" != "ackermann" ]]; then
  python3 "${ROOT}/scripts/stage5br6w_profile.py" \
    --base-nav2 "${navigation_share}/config/nav2.yaml" \
    --base-mission "${mission_template}" \
    --profile "${navigation_share}/config/autonomous_navigation_profile_v1.yaml" \
    --nav2-output "${nav_params}" \
    --mission-output "${mission_config}"
fi
if [[ "${DRIVE_MODEL}" == "ackermann" ]]; then
  cp "${navigation_share}/config/nav2_ackermann.yaml" "${nav_params}"
  if [[ "${COMPETITION_PROFILE}" -eq 0 ]]; then
    cp "${mission_template}" "${mission_config}"
  fi
  ackermann_filter_dir="${runtime}/ackermann_filters"
  python3 "${ROOT}/scripts/generate_ackermann_keepout_mask.py" \
    --map-yaml "${map_file}" \
    --mission-yaml "${mission_config}" \
    --output-dir "${ackermann_filter_dir}" \
    > "${OUTPUT_DIR}/ackermann_keepout_generation.txt"
  keepout_map="${ackermann_filter_dir}/ackermann_keepout_mask.yaml"
fi
python3 - "${nav_params}" "${max_linear_velocity}" "${max_angular_velocity}" "${MAP_SIZE}" "${DRIVE_MODEL}" "${COMPETITION_LANE}" <<'PY'
from pathlib import Path
import sys
import yaml
path = Path(sys.argv[1])
config = yaml.safe_load(path.read_text(encoding="utf-8"))
follow = config["controller_server"]["ros__parameters"]["FollowPath"]
linear_velocity = float(sys.argv[2])
angular_velocity = float(sys.argv[3])
map_size = sys.argv[4]
drive_model = sys.argv[5]
competition_lane = sys.argv[6]
smoother = config["velocity_smoother"]["ros__parameters"]
if drive_model != "ackermann":
    follow["desired_linear_vel"] = linear_velocity
    follow["rotate_to_heading_angular_vel"] = angular_velocity
    smoother["max_velocity"] = [linear_velocity, 0.0, angular_velocity]
    smoother["min_velocity"] = [-min(linear_velocity, 0.15), 0.0, -angular_velocity]
else:
    # The Ackermann profile owns its controller and reverse limits. Replacing
    # them with generic demo speeds defeats curvature tracking and can drive a
    # Reeds-Shepp connector into an occupied start pose during replanning.
    assert follow["use_rotate_to_heading"] is False
    assert follow["allow_reversing"] is False
    assert config["controller_server"]["ros__parameters"]["ReversePath"]["allow_reversing"] is True
    if competition_lane == "efficiency":
        controllers = config["controller_server"]["ros__parameters"]
        controllers["CleanPath"]["desired_linear_vel"] = 1.0
        controllers["CleanPath"]["lookahead_dist"] = 2.0
        controllers["CleanPath"]["min_lookahead_dist"] = 2.0
        controllers["CleanPath"]["max_lookahead_dist"] = 2.0
        # A 1 m/s swath needs enough distance to settle onto the extended
        # endpoint before the strict connector hand-off.  Without this ramp,
        # the westbound 186 m swath reached the loose endpoint at 0.73 rad and
        # the next Dubins primitive was pruned to an empty path.
        controllers["CleanPath"]["min_approach_linear_velocity"] = 0.2
        controllers["CleanPath"]["approach_velocity_scaling_dist"] = 5.0
        controllers["DubinsPath"]["desired_linear_vel"] = 0.6
        controllers["DubinsPath"]["regulated_linear_scaling_min_speed"] = 0.5
        controllers["DubinsPath"]["min_approach_linear_velocity"] = 0.2
        controllers["RepairPath"]["desired_linear_vel"] = 0.6
if map_size == "small":
    # Debris is intentionally traversable in the cleaning demonstration. Keep
    # the production RGB-D source alive for observation, but do not let a
    # delayed renderer timestamp stop the demo vehicle. The fresh 2D lidar
    # remains fail-closed and protects the field boundary / true obstacles.
    monitor = config["collision_monitor"]["ros__parameters"]
    monitor["observation_sources"] = ["scan"]
    monitor["source_timeout"] = 1.0
    monitor.pop("ground_cloud", None)
    config["tzcup_demo_safety_profile"] = {
        "ros__parameters": {
            "mode": "SMALL_FIELD_LIDAR_ONLY",
            "production_approved": False,
            "reason": "cleanable debris is traversable; renderer pointcloud is non-blocking",
        },
    }
path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
PY
if [[ "${REPAIR_EVALUATION_INJECTION}" -eq 1 ]]; then
  python3 - "${mission_config}" "${RANDOM_SEED}" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
seed = int(sys.argv[2])
config = yaml.safe_load(path.read_text(encoding="utf-8"))
config["evaluation_brush_dropout"] = {
    "enabled": True,
    "swath_index": seed % 6,
    "start_fraction": 0.25,
    "end_fraction": 0.65,
    "evaluation_only": True,
    "vehicle_control_source": "unchanged_nav2",
}
path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
PY
fi

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

setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false drive_model:="${DRIVE_MODEL}" random_seed:="${RANDOM_SEED}" gnss_profile:=rtk_fixed \
  headless_rendering:="${server_headless_rendering}" \
  world_file:="${world_file}" world_name:="${world_name}" \
  gui_config:="${gui_config}" \
  map_file:="${map_file}" spawn_x:="${spawn_x}" spawn_y:="${spawn_y}" spawn_yaw:="${spawn_yaw}" \
  cleaning_width:="${cleaning_width}" brush_center_y:="${brush_center_y}" \
  world_to_map_x:="${world_to_map_x}" world_to_map_y:="${world_to_map_y}" \
  initial_pose_x:="${initial_pose_x}" initial_pose_y:="${initial_pose_y}" initial_pose_yaw:="${initial_pose_yaw}" \
  camera_profile:=V5_retracted fusion_mode:="${localization_fusion_mode}" \
  enable_scan_refiner:="${enable_scan_refiner}" \
  > "${OUTPUT_DIR}/localization.log" 2>&1 &
localization_pid="$!"
pids+=("${localization_pid}")

# Nav2 lifecycle activation is intentionally delayed until the localization
# pipeline has produced both its public topics and the odom -> base_footprint
# transform. On a cold WSL / Gazebo start the simulator may need several
# seconds to publish its first wheel and IMU samples; activating Nav2 earlier
# makes its costmaps abort and leaves a visible but unusable operator window.
localization_ready=0
localization_topics=""
localization_tf=""
for _ in $(seq 1 120); do
  if ! kill -0 "${localization_pid}" 2>/dev/null; then
    echo "Localization runtime exited before readiness." >&2
    tail -80 "${OUTPUT_DIR}/localization.log" >&2 || true
    exit 4
  fi
  localization_topics="$(
    timeout 12 ros2 topic list --no-daemon --spin-time 3 2>/dev/null || true
  )"
  if grep -Fxq '/localization/fused_pose' <<< "${localization_topics}" &&
    grep -Fxq '/scan' <<< "${localization_topics}" &&
    grep -Fxq '/odom' <<< "${localization_topics}"
  then
    localization_tf="$(
      timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 || true
    )"
    if grep -Fq 'Translation:' <<< "${localization_tf}" &&
      grep -Fq 'Rotation:' <<< "${localization_tf}"
    then
      localization_ready=1
      break
    fi
  fi
  sleep 1
done
printf '%s\n' "${localization_topics}" > "${OUTPUT_DIR}/localization_readiness_topics.txt"
printf '%s\n' "${localization_tf}" > "${OUTPUT_DIR}/localization_readiness_tf.txt"
if [[ "${localization_ready}" -ne 1 ]]; then
  echo "Localization did not publish odom -> base_footprint within the readiness window." >&2
  tail -80 "${OUTPUT_DIR}/localization.log" >&2 || true
  exit 4
fi

setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  footprint_profile:="${footprint_profile}" \
  map_file:="${map_file}" keepout_map:="${keepout_map}" speed_map:="${speed_map}" \
  operational_profile:=localization_coverage max_linear_velocity:="${max_linear_velocity}" \
  max_angular_velocity:="${max_angular_velocity}" \
  > "${OUTPUT_DIR}/navigation.log" 2>&1 &
pids+=("$!")

coverage_launch_args=(footprint_profile:="${footprint_profile}")
[[ -n "${coverage_params}" ]] && coverage_launch_args+=(params_file:="${coverage_params}")
setsid ros2 launch sanitation_coverage coverage.launch.py "${coverage_launch_args[@]}" \
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

if [[ "${GAZEBO_TRAIL}" -eq 1 ]]; then
  setsid ros2 run sanitation_gazebo_visualization cleaning_visualizer --ros-args \
    -p use_sim_time:=true \
    -p operation_width_m:="${cleaning_width}" \
    -p brush_forward_offset_m:="$([[ "${DRIVE_MODEL}" == "ackermann" ]] && echo 0.68 || echo 0.55)" \
    -p configured_min_turning_radius_m:=1.429352 \
    -p expected_components:="${EXPECTED_COMPONENTS}" \
    -p profile_label:="${profile_label}" \
    -p map_area_m2:="${map_area_m2}" \
    -p mission_scope:="${mission_scope}" \
    -p mission_config:="${mission_config}" \
    -p simulation_speed_label:="${simulation_speed_label}" \
    -p world_name:="${world_name}" \
    -p world_to_map_x:="${world_to_map_x}" \
    -p world_to_map_y:="${world_to_map_y}" \
    -p world_to_map_yaw:=0.0 \
    -p service_timeout_ms:=3000 \
    -p telemetry_output_path:="${OUTPUT_DIR}/gazebo_cleaning_telemetry.json" \
    > "${OUTPUT_DIR}/gazebo_cleaning_visualizer.log" 2>&1 &
  pids+=("$!")
fi

if [[ "${RVIZ}" -eq 1 ]]; then
  setsid rviz2 -d "${rviz_config}" \
    --ros-args -p use_sim_time:=true \
    > "${OUTPUT_DIR}/rviz.log" 2>&1 &
  pids+=("$!")
fi

if ! timeout 305 python3 "${ROOT}/scripts/ros_runtime_readiness.py" \
  --timeout 300 \
  --dashboard-url "http://127.0.0.1:${DASHBOARD_PORT}/healthz" \
  --output "${OUTPUT_DIR}/runtime_readiness.json"
then
  if ! kill -0 "${dashboard_pid}" 2>/dev/null; then
    echo "Live dashboard process exited before readiness." >&2
    tail -50 "${OUTPUT_DIR}/dashboard.log" >&2 || true
  fi
  echo "AUTO-17 runtime did not become ready within 300 seconds." >&2
  cat "${OUTPUT_DIR}/runtime_readiness.json" >&2 2>/dev/null || true
  exit 4
fi
cp "${OUTPUT_DIR}/runtime_readiness.json" "${OUTPUT_DIR}/dashboard_health.json"

if [[ "${GUI}" -eq 1 ]]; then
  # Start the WSLg client only after the Gazebo server and ROS graph are ready.
  # Starting it inside ros2 launch, or before the world is discoverable, can
  # load the custom library without instantiating its ROS control backend.
  printf '{"requested":"%s","selected":"%s","server_renderer":"%s"}\n' \
    "${GAZEBO_GUI_RENDERER}" "${gazebo_gui_renderer}" "${GALLIUM_DRIVER}" \
    > "${OUTPUT_DIR}/gazebo_gui_renderer.json"
  setsid "${gazebo_gui_env[@]}" gz sim -g --gui-config "${gui_config}" \
    > "${OUTPUT_DIR}/gazebo_gui.log" 2>&1 &
  gazebo_gui_pid="$!"
  pids+=("${gazebo_gui_pid}")
  gazebo_gui_ready=0
  for _ in $(seq 1 30); do
    if [[ -f "${OUTPUT_DIR}/wslg_window_guard.failed" ]]; then
      echo "WSLg window guard reported: $(cat "${OUTPUT_DIR}/wslg_window_guard.failed")" >&2
      exit 7
    fi
    if ! kill -0 "${gazebo_gui_pid}" 2>/dev/null; then
      echo "Gazebo GUI exited before its native mission control loaded." >&2
      tail -50 "${OUTPUT_DIR}/gazebo_gui.log" >&2 || true
      exit 4
    fi
    gui_nodes="$(
      timeout 5 ros2 node list --no-daemon --spin-time 2 2>/dev/null || true
    )"
    if grep -Fxq '/sanitation_gazebo_mission_control' <<< "${gui_nodes}"; then
      gazebo_gui_ready=1
      break
    fi
    sleep 1
  done
  if [[ "${gazebo_gui_ready}" -ne 1 ]]; then
    echo "Gazebo GUI did not expose its native mission control within 30 seconds." >&2
    exit 4
  fi
  if command -v xwininfo >/dev/null 2>&1 && \
     command -v xwd >/dev/null 2>&1 && \
     command -v ffmpeg >/dev/null 2>&1; then
    if ! python3 "${ROOT}/scripts/gazebo_viewport_probe.py" \
      --output-dir "${OUTPUT_DIR}" --title "Gazebo Sim" --timeout 20; then
      echo "Gazebo 3D viewport is black; refusing to report GUI readiness." >&2
      exit 8
    fi
  else
    printf '{"render_visible":null,"skipped":"xwininfo/xwd/ffmpeg unavailable"}\n' \
      > "${OUTPUT_DIR}/gazebo_viewport_probe.json"
  fi
fi

camera_follow_requested=0
camera_track_request='track_mode: FOLLOW_LOOK_AT, follow_target: {name: "sanitation_vehicle", type: MODEL}, track_target: {name: "sanitation_vehicle", type: MODEL}, follow_offset: {x: -4.5, y: -3.0, z: 3.2}, follow_pgain: 0.35, track_pgain: 0.35'
if [[ "${SHOWCASE}" -eq 1 ]]; then
  camera_track_request='track_mode: FOLLOW_LOOK_AT, follow_target: {name: "sanitation_vehicle", type: MODEL}, track_target: {name: "sanitation_vehicle", type: MODEL}, follow_offset: {x: -8.0, y: -8.0, z: 10.0}, follow_pgain: 0.25, track_pgain: 0.35'
fi
if [[ "${GUI}" -eq 1 && "${MANUAL_CONTROL}" -eq 0 ]]; then
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

if [[ -f "${OUTPUT_DIR}/wslg_window_guard.failed" ]]; then
  echo "WSLg window guard reported: $(cat "${OUTPUT_DIR}/wslg_window_guard.failed")" >&2
  exit 7
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
    /clock /tf /tf_static /scan /odom /wheel/odom_raw /joint_states /localization/fused_pose \
    /ground_truth/odom /cmd_vel /cmd_vel_gate /brush_enabled \
    /emergency_stop /coverage/state /coverage/component_state \
    /coverage/current_path /coverage/evaluation_sample \
    /coverage/full_plan /coverage/planned_swaths \
    /coverage/planned_connectors /coverage/planned_repairs \
    /coverage/current_component_path \
    /coverage/actual_cleaning_trajectory \
    /coverage/actual_transit_trajectory /coverage/actual_repair_trajectory \
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

if ! timeout 20 python3 "${ROOT}/scripts/emergency_stop_availability.py" \
  --telemetry "${OUTPUT_DIR}/dashboard_telemetry.json" \
  --output "${OUTPUT_DIR}/emergency_stop_available.json" \
  --timeout 15
then
  echo "Unable to publish the bounded emergency-stop availability pulse." >&2
  cat "${OUTPUT_DIR}/emergency_stop_available.json" >&2 2>/dev/null || true
  exit 4
fi

dynamic_probe_pid=""
dynamic_probe_code=0
if [[ "${DYNAMIC_OBSTACLE_TRIALS}" -gt 0 ]]; then
  dynamic_probe_executable="$(ros2 pkg prefix sanitation_tasks)/lib/sanitation_tasks/sanitation_dynamic_obstacle_probe"
  if [[ ! -x "${dynamic_probe_executable}" ]]; then
    echo "Dynamic obstacle probe executable is missing: ${dynamic_probe_executable}" >&2
    exit 4
  fi
  setsid --wait timeout "${MISSION_TIMEOUT_SEC}" \
    "${dynamic_probe_executable}" --ros-args \
    -p use_sim_time:=true \
    -p output_path:="${OUTPUT_DIR}/dynamic_obstacle_report.json" \
    -p trial_count:="${DYNAMIC_OBSTACLE_TRIALS}" \
    -p world_name:="${world_name}" \
    -p model_name:="dynamic_pedestrian_box" \
    -p world_to_map_x:="${world_to_map_x}" \
    -p service_timeout_ms:=10000 \
    -p minimum_remaining_path_m:=3.0 \
    -p minimum_progress_between_trials_m:=0.5 \
    -p minimum_injection_distance_m:=1.5 \
    -p maximum_injection_distance_m:=1.8 \
    -p hold_sec:=0.5 \
    -p crossing_steps:=5 \
    > "${OUTPUT_DIR}/dynamic_obstacle_probe.log" 2>&1 &
  dynamic_probe_pid="$!"
  pids+=("${dynamic_probe_pid}")
fi

set +e
coverage_executable="$(ros2 pkg prefix sanitation_coverage)/lib/sanitation_coverage/coverage_probe"
if [[ ! -x "${coverage_executable}" ]]; then
  echo "Coverage executable is missing: ${coverage_executable}" >&2
  exit 4
fi
PYTHONUNBUFFERED=1 setsid --wait timeout "${MISSION_TIMEOUT_SEC}" \
  "${coverage_executable}" --ros-args \
  -p use_sim_time:=true \
  -p manual_start:="$([[ "${MANUAL_CONTROL}" -eq 1 ]] && echo true || echo false)" \
  -p output_path:="${OUTPUT_DIR}/coverage_report.json" \
  -p config_path:="${mission_config}" \
  -p path_output_path:="${OUTPUT_DIR}/coverage_path.json" \
  -p trajectory_output_path:="${OUTPUT_DIR}/coverage_trajectory.csv" \
  -p component_retry_limit:=2 \
  > "${OUTPUT_DIR}/coverage_probe.log" 2>&1 &
coverage_pid="$!"
pids+=("${coverage_pid}")
gui_closed_during_mission=0
runtime_termination_status=""
while kill -0 "${coverage_pid}" 2>/dev/null; do
  if [[ -f "${OUTPUT_DIR}/wslg_window_guard.failed" ]]; then
    gui_closed_during_mission=1
    runtime_termination_status="WSLG_WINDOW_GUARD_FAILED"
    echo "[AUTO-17] WSLg window guard failed; stopping the active mission and runtime."
    break
  fi
  if [[ "${GUI}" -eq 1 ]] && ! kill -0 "${gazebo_gui_pid}" 2>/dev/null; then
    for _ in $(seq 1 10); do
      kill -0 "${coverage_pid}" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "${coverage_pid}" 2>/dev/null; then
      gui_closed_during_mission=1
      runtime_termination_status="OPERATOR_GUI_CLOSED"
      echo "[AUTO-17] Gazebo GUI closed; stopping the active mission and runtime."
    fi
    break
  fi
  sleep 0.5
done
if [[ "${gui_closed_during_mission}" -eq 1 ]]; then
  coverage_code=130
else
  wait "${coverage_pid}"
  coverage_code=$?
fi
printf '%s\n' "${coverage_code}" > "${OUTPUT_DIR}/coverage_process_exit_code.txt"
if [[ -n "${dynamic_probe_pid}" ]]; then
  wait "${dynamic_probe_pid}"
  dynamic_probe_code=$?
  printf '%s\n' "${dynamic_probe_code}" \
    > "${OUTPUT_DIR}/dynamic_obstacle_process_exit_code.txt"
fi
set -e

if [[ "${gui_closed_during_mission}" -eq 1 ]]; then
  printf '{"schema_version":1,"status":"%s","mission_completed":false,"timestamp_utc":"%s"}\n' \
    "${runtime_termination_status}" \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    > "${OUTPUT_DIR}/launcher_termination.json"
  stop_all
  trap - EXIT INT TERM
  if [[ "${runtime_termination_status}" == "WSLG_WINDOW_GUARD_FAILED" ]]; then
    exit 7
  fi
  exit 0
fi

sleep 8
if [[ "${KEEP_OPEN}" -eq 1 ]]; then
  echo "[AUTO-17] Mission ended. Press Ctrl+C when visual inspection is complete."
  keep_open_stop=0
  trap 'keep_open_stop=1' INT TERM
  while [[ "${keep_open_stop}" -eq 0 ]]; do
    if [[ -f "${OUTPUT_DIR}/wslg_window_guard.failed" ]]; then
      echo "[AUTO-17] WSLg window guard failed; finishing the launcher."
      break
    fi
    if [[ "${GUI}" -eq 1 ]] && ! kill -0 "${gazebo_gui_pid}" 2>/dev/null; then
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
[[ "${GUI}" -eq 0 || "${MANUAL_CONTROL}" -eq 1 ]] && summary_args+=(--camera-follow-not-required)
[[ "${GAZEBO_TRAIL}" -eq 1 ]] && summary_args+=(--targets-required)
python3 "${ROOT}/scripts/visual_demo_summary.py" "${summary_args[@]}"
if [[ "${DYNAMIC_OBSTACLE_TRIALS}" -gt 0 && "${dynamic_probe_code}" -ne 0 ]]; then
  echo "Dynamic obstacle matrix failed with exit code ${dynamic_probe_code}." >&2
  exit 8
fi
