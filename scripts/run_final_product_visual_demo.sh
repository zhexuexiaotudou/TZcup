#!/usr/bin/env bash
# Formal visual entry: map once, wait for its sealed hard-stop, then clean once.
# It only orchestrates existing formal runners; vehicle and algorithm contracts
# remain owned by those runners and their validators.
set -euo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

usage() {
  cat <<'EOF'
Usage: run_final_product_visual_demo.sh \
  --run-root ABS --runtime-ws ABS --runtime-closure ABS --vehicle-snapshot ABS --acceptance-session ABS \
  --episode-root ABS --map-root ABS --terminal-output ABS \
  --perception-mode unavailable|pc|s100p --cleaning-planner full_coverage|rl_dirt_priority \
  [--perception-artifact-root ABS --policy-checkpoint ABS] \
  [--s100p-board-bridge ABS --s100p-receipt ABS] \
  [--mapping-ros-domain N --cleaning-ros-domain N] \
  [--mapping-timeout-sec N --cleaning-timeout-sec N --operation-speed-profile NAME] \
  [--full-coverage-distance-m N] [--formal-visual-gui true|false] \
  [--dashboard-port N --dashboard-output ABS] [--preflight]

The run root and all generated map/terminal paths must be fresh.  `pc` binds
the existing RL/perception contract.  `unavailable` is only full coverage and
emits PERCEPTION_BLOCKED_NOT_PRODUCT_PASS.  `s100p` never falls back: until a
formal board bridge and receipt are available it fails before a run root is made.
EOF
}

require_value() {
  local option="$1"
  local value="$2"
  if [[ -z "${value}" ]]; then
    echo "${option} requires a value" >&2
    exit 2
  fi
}

absolute_path() {
  realpath -m -- "$1"
}

path_is_below() {
  local child="$1"
  local parent="$2"
  [[ "${child}" == "${parent}"/* ]]
}

run_root=""
runtime_ws=""
runtime_closure=""
vehicle_snapshot=""
acceptance_session=""
episode_root=""
map_root=""
terminal_output=""
dashboard_output=""
perception_mode=""
cleaning_planner=""
perception_artifact_root=""
policy_checkpoint=""
s100p_board_bridge=""
s100p_receipt=""
mapping_timeout_sec=21600
cleaning_timeout_sec=86400
mapping_ros_domain=99
cleaning_ros_domain=60
full_coverage_distance_m=0.0
operation_speed_profile=dry_cleaning_competition_candidate
formal_visual_gui=false
dashboard_port=8878
preflight_only=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-root|--runtime-ws|--runtime-closure|--vehicle-snapshot|--acceptance-session|--episode-root|--map-root|--terminal-output|--dashboard-output|--perception-mode|--cleaning-planner|--perception-artifact-root|--policy-checkpoint|--s100p-board-bridge|--s100p-receipt|--mapping-ros-domain|--cleaning-ros-domain|--mapping-timeout-sec|--cleaning-timeout-sec|--full-coverage-distance-m|--operation-speed-profile|--formal-visual-gui|--dashboard-port)
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      case "$1" in
        --run-root) run_root="$2" ;; --runtime-ws) runtime_ws="$2" ;;
        --runtime-closure) runtime_closure="$2" ;; --vehicle-snapshot) vehicle_snapshot="$2" ;;
        --acceptance-session) acceptance_session="$2" ;;
        --episode-root) episode_root="$2" ;; --map-root) map_root="$2" ;;
        --terminal-output) terminal_output="$2" ;; --dashboard-output) dashboard_output="$2" ;;
        --perception-mode) perception_mode="$2" ;;
        --cleaning-planner) cleaning_planner="$2" ;; --perception-artifact-root) perception_artifact_root="$2" ;;
        --policy-checkpoint) policy_checkpoint="$2" ;; --s100p-board-bridge) s100p_board_bridge="$2" ;;
        --s100p-receipt) s100p_receipt="$2" ;; --mapping-ros-domain) mapping_ros_domain="$2" ;;
        --cleaning-ros-domain) cleaning_ros_domain="$2" ;; --mapping-timeout-sec) mapping_timeout_sec="$2" ;;
        --cleaning-timeout-sec) cleaning_timeout_sec="$2" ;; --full-coverage-distance-m) full_coverage_distance_m="$2" ;;
        --operation-speed-profile) operation_speed_profile="$2" ;; --formal-visual-gui) formal_visual_gui="$2" ;;
        --dashboard-port) dashboard_port="$2" ;;
      esac
      shift 2 ;;
    --preflight) preflight_only=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for option_and_value in \
  "--run-root:${run_root}" "--runtime-ws:${runtime_ws}" "--runtime-closure:${runtime_closure}" "--vehicle-snapshot:${vehicle_snapshot}" \
  "--acceptance-session:${acceptance_session}" "--episode-root:${episode_root}" \
  "--map-root:${map_root}" "--terminal-output:${terminal_output}" \
  "--perception-mode:${perception_mode}" "--cleaning-planner:${cleaning_planner}"; do
  require_value "${option_and_value%%:*}" "${option_and_value#*:}"
done

case "${formal_visual_gui}" in true|false) ;; *) echo "--formal-visual-gui must be true or false" >&2; exit 2 ;; esac
if [[ ! "${dashboard_port}" =~ ^[1-9][0-9]*$ || "${dashboard_port}" -gt 65535 ]]; then
  echo "--dashboard-port must be an integer in 1..65535" >&2
  exit 2
fi
case "${perception_mode}" in unavailable|pc|s100p) ;; *) echo "--perception-mode must be unavailable, pc, or s100p" >&2; exit 2 ;; esac
case "${cleaning_planner}" in full_coverage|rl_dirt_priority) ;; *) echo "--cleaning-planner must be full_coverage or rl_dirt_priority" >&2; exit 2 ;; esac
if [[ ! "${mapping_timeout_sec}" =~ ^[1-9][0-9]*$ || ! "${cleaning_timeout_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "mapping and cleaning timeouts must be positive integers" >&2
  exit 2
fi
if [[ ! "${mapping_ros_domain}" =~ ^[0-9]+$ || ! "${cleaning_ros_domain}" =~ ^[0-9]+$ \
  || "${mapping_ros_domain}" -gt 232 || "${cleaning_ros_domain}" -gt 232 \
  || "${mapping_ros_domain}" == "${cleaning_ros_domain}" ]]; then
  echo "mapping and cleaning ROS domains must be distinct integers in 0..232" >&2
  exit 2
fi
if [[ "${operation_speed_profile}" != "dry_cleaning_competition_candidate" ]]; then
  echo "operation speed profile must be dry_cleaning_competition_candidate" >&2
  exit 2
fi

run_root="$(absolute_path "${run_root}")"
runtime_ws="$(absolute_path "${runtime_ws}")"
runtime_closure="$(absolute_path "${runtime_closure}")"
vehicle_snapshot="$(absolute_path "${vehicle_snapshot}")"
acceptance_session="$(absolute_path "${acceptance_session}")"
episode_root="$(absolute_path "${episode_root}")"
map_root="$(absolute_path "${map_root}")"
terminal_output="$(absolute_path "${terminal_output}")"
if [[ -z "${dashboard_output}" ]]; then
  dashboard_output="${run_root}/dashboard"
else
  dashboard_output="$(absolute_path "${dashboard_output}")"
fi
final_demo_state_file="${run_root}/final_demo_state.json"

if [[ -e "${run_root}" ]]; then
  echo "refusing stale final-product visual run root: ${run_root}" >&2
  exit 2
fi
if ! path_is_below "${map_root}" "${run_root}" || ! path_is_below "${terminal_output}" "${run_root}"; then
  echo "map root and terminal output must be below the fresh run root" >&2
  exit 2
fi
if [[ -e "${map_root}" || -e "${terminal_output}" ]]; then
  echo "refusing stale map or terminal output path" >&2
  exit 2
fi
if [[ "${formal_visual_gui}" == "true" ]] && { ! path_is_below "${dashboard_output}" "${run_root}" || [[ -e "${dashboard_output}" || -e "${final_demo_state_file}" ]]; }; then
  echo "dashboard output and live state file must be fresh paths below the run root" >&2
  exit 2
fi
if [[ ! -d "${runtime_ws}" || ! -f "${runtime_ws}/install/setup.bash" || ! -f "${runtime_closure}" || ! -f "${vehicle_snapshot}" ]]; then
  echo "formal runtime workspace, install/setup.bash, closure manifest, and vehicle snapshot are required" >&2
  exit 2
fi
if [[ ! -f "${acceptance_session}" ]]; then
  echo "formal acceptance session is required" >&2
  exit 2
fi
for episode_file in public/world.sdf public/episode_manifest.json environment/pedestrian_schedule.json; do
  if [[ ! -f "${episode_root}/${episode_file}" ]]; then
    echo "formal episode root lacks ${episode_file}" >&2
    exit 2
  fi
done

case "${perception_mode}" in
  unavailable)
    if [[ "${cleaning_planner}" != "full_coverage" ]]; then
      echo "unavailable perception permits only full_coverage" >&2
      exit 2
    fi
    if [[ -n "${perception_artifact_root}" || -n "${policy_checkpoint}" ]]; then
      echo "unavailable perception must not accept PC artifact or policy inputs" >&2
      exit 2
    fi
    ;;
  pc)
    if [[ "${cleaning_planner}" != "rl_dirt_priority" || ! -d "${perception_artifact_root}" || ! -f "${policy_checkpoint}" ]]; then
      echo "pc perception requires rl_dirt_priority plus an artifact directory and policy checkpoint" >&2
      exit 2
    fi
    python3 - "${full_coverage_distance_m}" <<'PY'
import math
import sys
value = float(sys.argv[1])
if not math.isfinite(value) or value <= 0.0:
    raise SystemExit("pc perception requires positive --full-coverage-distance-m")
PY
    ;;
  s100p)
    if [[ -z "${s100p_board_bridge}" || -z "${s100p_receipt}" ]]; then
      echo "S100P preflight failed: formal board bridge and receipt are both required; no fallback is allowed" >&2
      exit 3
    fi
    s100p_board_bridge="$(absolute_path "${s100p_board_bridge}")"
    s100p_receipt="$(absolute_path "${s100p_receipt}")"
    if [[ ! -f "${s100p_board_bridge}" || ! -x "${s100p_board_bridge}" || ! -f "${s100p_receipt}" ]]; then
      echo "S100P preflight failed: formal executable board bridge and receipt are required; no fallback is allowed" >&2
      exit 3
    fi
    echo "S100P preflight failed: this repository has no accepted board-bridge execution contract; refusing PC or full-coverage fallback" >&2
    exit 3
    ;;
esac

if "${preflight_only}"; then
  printf 'FORMAL_FINAL_PRODUCT_VISUAL_PREFLIGHT_OK mode=%s gui=%s run_root=%s\n' \
    "${perception_mode}" "${formal_visual_gui}" "${run_root}"
  exit 0
fi

write_final_demo_state() {
  local stage="$1"
  local map_sha256="$2"
  python3 - "${final_demo_state_file}" "${stage}" "${map_sha256}" "${perception_mode}" <<'PY'
import json
import os
import pathlib
import sys

output = pathlib.Path(sys.argv[1])
payload = {
    "field_dimensions_m": [200, 100],
    "vehicle": "A300",
    "stage": sys.argv[2],
    "map_sha256": None if sys.argv[3] == "" else sys.argv[3],
    "perception_provider": sys.argv[4],
    # This visual bridge never performs or claims formal product acceptance.
    "formal_product_acceptance": False,
}
temporary = output.with_name(f".{output.name}.{os.getpid()}.tmp")
temporary.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
os.replace(temporary, output)
PY
}

dashboard_pid=""
state_publisher_pid=""
coverage_stage_monitor_pid=""
stop_visual_stack() {
  local pid
  for pid in "${coverage_stage_monitor_pid}" "${state_publisher_pid}" "${dashboard_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${coverage_stage_monitor_pid}" "${state_publisher_pid}" "${dashboard_pid}"; do
    [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null || true
  done
  dashboard_pid=""
  state_publisher_pid=""
  coverage_stage_monitor_pid=""
}
trap stop_visual_stack EXIT INT TERM

start_visual_stack() {
  local phase="$1"
  local domain="$2"
  [[ "${formal_visual_gui}" == "true" ]] || return 0
  mkdir -p "${dashboard_output}"
  (
    set +u
    source /opt/ros/jazzy/setup.bash
    source "${runtime_ws}/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${domain}"
    export PYTHONPATH="${repo_root}/starter_ws/src/sanitation_hmi${PYTHONPATH:+:${PYTHONPATH}}"
    exec python3 -m sanitation_hmi.live_server --ros-args \
      -p use_sim_time:=true -p port:="${dashboard_port}" \
      -p output_dir:="${dashboard_output}" \
      -p web_root:="${repo_root}/starter_ws/src/sanitation_hmi/web"
  ) >"${dashboard_output}/dashboard.${phase}.log" 2>&1 &
  dashboard_pid=$!
  (
    set +u
    source /opt/ros/jazzy/setup.bash
    source "${runtime_ws}/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${domain}"
    exec python3 "${repo_root}/scripts/publish_final_product_visual_state.py" \
      --state-file "${final_demo_state_file}" --period-sec 1.0
  ) >"${dashboard_output}/final_demo_state_publisher.${phase}.log" 2>&1 &
  state_publisher_pid=$!
}

mkdir -p "${run_root}"
mapping_output="${map_root}"
cleaning_runtime="${run_root}/saved_map_cleaning_runtime"
cleaning_output="${run_root}/map_lifecycle_acceptance.json"

if [[ "${formal_visual_gui}" == "true" ]]; then
  write_final_demo_state MAPPING ""
  start_visual_stack mapping "${mapping_ros_domain}"
fi

FORMAL_DYNAMIC_EPISODE_ROOT="${episode_root}" \
FORMAL_DYNAMIC_SAVED_MAP_ROOT="${mapping_output}" \
FORMAL_VEHICLE_RUNTIME_WS="${runtime_ws}" \
FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST="${runtime_closure}" \
FORMAL_VEHICLE_SNAPSHOT_MANIFEST="${vehicle_snapshot}" \
FORMAL_ACCEPTANCE_SESSION="${acceptance_session}" \
FORMAL_MAPPING_TIMEOUT_S="${mapping_timeout_sec}" \
ROS_DOMAIN_ID="${mapping_ros_domain}" \
FORMAL_VISUAL_GUI="${formal_visual_gui}" \
bash "${repo_root}/scripts/run_formal_first_map_dynamic_prerequisite.sh"

map_sha256="$(sha256sum "${map_root}/map_lifecycle_manifest.json" | awk '{print $1}')"
if [[ "${formal_visual_gui}" == "true" ]]; then
  write_final_demo_state MAP_SAVED "${map_sha256}"
  sleep 1
  write_final_demo_state HARD_RESTART "${map_sha256}"
  sleep 1
  stop_visual_stack
  start_visual_stack cleaning "${cleaning_ros_domain}"
  write_final_demo_state RELOAD_LOCALIZE "${map_sha256}"
fi

cleaning_environment=(
  "FORMAL_DYNAMIC_EPISODE_ROOT=${episode_root}"
  "FORMAL_DYNAMIC_SAVED_MAP_ROOT=${mapping_output}"
  "FORMAL_MAP_CLEANING_RUNTIME_ROOT=${cleaning_runtime}"
  "FORMAL_MAP_LIFECYCLE_OUTPUT=${cleaning_output}"
  "FORMAL_VEHICLE_RUNTIME_WS=${runtime_ws}"
  "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST=${runtime_closure}"
  "FORMAL_VEHICLE_SNAPSHOT_MANIFEST=${vehicle_snapshot}"
  "FORMAL_ACCEPTANCE_SESSION=${acceptance_session}"
  "FORMAL_CLEANING_PLANNER=${cleaning_planner}"
  "FORMAL_CLEANING_TIMEOUT_S=${cleaning_timeout_sec}"
  "FORMAL_FULL_COVERAGE_DISTANCE_M=${full_coverage_distance_m}"
  "FORMAL_OPERATION_SPEED_PROFILE=${operation_speed_profile}"
  "ROS_DOMAIN_ID=${cleaning_ros_domain}"
  "FORMAL_VISUAL_GUI=${formal_visual_gui}"
)
if [[ "${perception_mode}" == "pc" ]]; then
  cleaning_environment+=(
    "FORMAL_PERCEPTION_ARTIFACT_ROOT=$(absolute_path "${perception_artifact_root}")"
    "FORMAL_POLICY_CHECKPOINT=$(absolute_path "${policy_checkpoint}")"
  )
fi
env "${cleaning_environment[@]}" bash "${repo_root}/scripts/run_formal_saved_map_cleaning_lifecycle.sh" &
cleaning_runner_pid=$!
if [[ "${formal_visual_gui}" == "true" ]]; then
  (
    while kill -0 "${cleaning_runner_pid}" 2>/dev/null; do
      if [[ -s "${cleaning_runtime}/hard_restart_record.json" ]]; then
        write_final_demo_state COVERAGE "${map_sha256}"
        exit 0
      fi
      sleep 1
    done
  ) &
  coverage_stage_monitor_pid=$!
fi
wait "${cleaning_runner_pid}"
if [[ -n "${coverage_stage_monitor_pid}" ]]; then
  wait "${coverage_stage_monitor_pid}" 2>/dev/null || true
  coverage_stage_monitor_pid=""
fi

terminal_status="FORMAL_PC_PERCEPTION_COMPLETED_NOT_PRODUCT_PASS"
if [[ "${perception_mode}" == "unavailable" ]]; then
  terminal_status="PERCEPTION_BLOCKED_NOT_PRODUCT_PASS"
fi
python3 - "${terminal_output}" "${terminal_status}" "${perception_mode}" "${cleaning_planner}" \
  "${mapping_output}" "${cleaning_output}" "${formal_visual_gui}" "${runtime_ws}" \
  "${runtime_closure}" "${vehicle_snapshot}" "${acceptance_session}" "${episode_root}" \
  "${mapping_ros_domain}" "${cleaning_ros_domain}" "${operation_speed_profile}" <<'PY'
import datetime
import json
import pathlib
import sys

output = pathlib.Path(sys.argv[1])
output.parent.mkdir(parents=True, exist_ok=True)
value = {
    "schema_version": 1,
    "status": sys.argv[2],
    "product_pass": False,
    "perception_mode": sys.argv[3],
    "cleaning_planner": sys.argv[4],
    "mapping_output": sys.argv[5],
    "hard_restart_cleaning_output": sys.argv[6],
    "formal_visual_gui": sys.argv[7] == "true",
    "runtime_workspace": sys.argv[8],
    "runtime_closure_manifest": sys.argv[9],
    "vehicle_snapshot_manifest": sys.argv[10],
    "acceptance_session": sys.argv[11],
    "episode_root": sys.argv[12],
    "mapping_ros_domain": int(sys.argv[13]),
    "cleaning_ros_domain": int(sys.argv[14]),
    "operation_speed_profile": sys.argv[15],
    "hard_restart_required": True,
    "final_demo_stage": "PRODUCT_TERMINAL",
    "completed_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
output.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
if [[ "${formal_visual_gui}" == "true" ]]; then
  write_final_demo_state PRODUCT_TERMINAL "${map_sha256}"
  sleep 1
fi
printf 'FORMAL_FINAL_PRODUCT_VISUAL_TERMINAL=%s\n' "${terminal_output}"
