#!/usr/bin/env bash
# A live, final-topology visual preview.  It deliberately does not invoke the
# formal acceptance runners: it has no frozen closure/session and therefore
# can never assert a product pass.  The actual Gazebo graph, 200x100m formal
# campus, A300 launch, first-map hard restart, HMI and state publisher are all
# reused from the product implementation.
set -euo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
# A linked worktree has a .git *file*.  Walk upward to the repository that
# owns the .git directory instead of accepting the Windows-formatted path some
# git.exe builds print for --git-common-dir inside WSL.
project_root="${repo_root}"
candidate="${repo_root}"
while [[ "${candidate}" != "/" ]]; do
  if [[ -d "${candidate}/.git" ]]; then
    project_root="${candidate}"
    break
  fi
  candidate="$(dirname "${candidate}")"
done

usage() {
  cat <<'EOF'
Usage: run_final_product_visual_preview.sh --run-root ABS --runtime-ws ABS \
  [--episode-root ABS] [--mapping-timeout-sec N --cleaning-timeout-sec N] \
  [--mapping-ros-domain N --cleaning-ros-domain N --dashboard-port N] \
  [--gazebo-gui true|false] [--preflight]

This is a live final-product visual preview, not a formal acceptance run. It
generates a fresh formal episode when --episode-root is omitted, maps the
200x100m A300 campus, hard-restarts Gazebo, and starts same-map FullCoverage.
Its only terminal result is FINAL_VISUAL_PREVIEW_NOT_PRODUCT_PASS.
EOF
}

absolute_path() { realpath -m -- "$1"; }
path_is_below() { [[ "$1" == "$2"/* ]]; }
require_positive_integer() {
  [[ "$2" =~ ^[1-9][0-9]*$ ]] || { echo "$1 must be a positive integer" >&2; exit 2; }
}
require_nonnegative_integer() {
  [[ "$2" =~ ^[0-9]+$ ]] || { echo "$1 must be a non-negative integer" >&2; exit 2; }
}

run_root=""
runtime_ws=""
episode_root=""
mapping_timeout_sec=21600
cleaning_timeout_sec=86400
mapping_ros_domain=99
cleaning_ros_domain=60
dashboard_port=8879
gazebo_gui=false
preflight_only=false
# Bound visual liveness separately from the long map/coverage completion caps.
# A live PID and healthy memory guard do not prove scan, SLAM, or HMI progress.
phase_progress_timeout_sec=600
hmi_receipt_timeout_sec=90

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-root|--runtime-ws|--episode-root|--mapping-timeout-sec|--cleaning-timeout-sec|--mapping-ros-domain|--cleaning-ros-domain|--dashboard-port|--gazebo-gui)
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      case "$1" in
        --run-root) run_root="$2" ;; --runtime-ws) runtime_ws="$2" ;;
        --episode-root) episode_root="$2" ;; --mapping-timeout-sec) mapping_timeout_sec="$2" ;;
        --cleaning-timeout-sec) cleaning_timeout_sec="$2" ;;
        --mapping-ros-domain) mapping_ros_domain="$2" ;; --cleaning-ros-domain) cleaning_ros_domain="$2" ;;
        --dashboard-port) dashboard_port="$2" ;;
        --gazebo-gui) gazebo_gui="$2" ;;
      esac
      shift 2 ;;
    --preflight) preflight_only=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "${run_root}" && -n "${runtime_ws}" ]] || { usage >&2; exit 2; }
for pair in \
  "--mapping-timeout-sec:${mapping_timeout_sec}" "--cleaning-timeout-sec:${cleaning_timeout_sec}" \
  "--dashboard-port:${dashboard_port}"; do
  require_positive_integer "${pair%%:*}" "${pair#*:}"
done
require_nonnegative_integer "--mapping-ros-domain" "${mapping_ros_domain}"
require_nonnegative_integer "--cleaning-ros-domain" "${cleaning_ros_domain}"
if [[ "${mapping_ros_domain}" == "${cleaning_ros_domain}" ]] \
  || ! formal_runtime_domain_is_linux_safe "${mapping_ros_domain}" \
  || ! formal_runtime_domain_is_linux_safe "${cleaning_ros_domain}"; then
  echo "mapping and cleaning ROS domains must be distinct Linux-safe domains: 0..101 or 215..231" >&2
  exit 2
fi
if (( dashboard_port > 65535 )); then
  echo "dashboard port must be in 1..65535" >&2
  exit 2
fi
[[ "${gazebo_gui}" == "true" || "${gazebo_gui}" == "false" ]] || {
  echo "--gazebo-gui must be true or false" >&2
  exit 2
}

run_root="$(absolute_path "${run_root}")"
runtime_ws="$(absolute_path "${runtime_ws}")"
if [[ -n "${episode_root}" ]]; then episode_root="$(absolute_path "${episode_root}")"; fi

# Keep generated evidence and every fresh preview directory inside the TZcup
# project.  Input episodes may be reused, but must be complete formal inputs.
path_is_below "${run_root}" "${project_root}" || {
  echo "preview run root must remain below TZcup project root: ${project_root}" >&2; exit 2;
}
path_is_below "${runtime_ws}" "${project_root}" || {
  echo "preview runtime workspace must remain below TZcup project root: ${project_root}" >&2; exit 2;
}
[[ ! -e "${run_root}" ]] || { echo "refusing stale preview run root: ${run_root}" >&2; exit 2; }
[[ -d "${runtime_ws}" && -f "${runtime_ws}/install/setup.bash" ]] || {
  echo "runtime workspace must contain install/setup.bash" >&2; exit 2;
}
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS Jazzy is required" >&2; exit 2; }
if [[ -n "${episode_root}" ]]; then
  path_is_below "${episode_root}" "${project_root}" || {
    echo "preview episode root must remain below TZcup project root: ${project_root}" >&2; exit 2;
  }
  for required in public/world.sdf public/episode_manifest.json environment/pedestrian_schedule.json; do
    [[ -f "${episode_root}/${required}" ]] || { echo "episode root lacks ${required}" >&2; exit 2; }
  done
else
  episode_root="${run_root}/episode"
fi

map_root="${run_root}/first_map"
mapping_world="${run_root}/mapping_world.sdf"
cleaning_root="${run_root}/saved_map_cleaning"
cleaning_world="${cleaning_root}/cleaning_world.sdf"
dashboard_output="${run_root}/dashboard"
state_file="${run_root}/final_demo_state.json"
terminal_output="${run_root}/terminal.json"
preview_hmi_mission="${run_root}/preview_hmi_mission.yaml"

if "${preflight_only}"; then
  printf 'FINAL_VISUAL_PREVIEW_PREFLIGHT_OK run_root=%s runtime_ws=%s episode=%s\n' \
    "${run_root}" "${runtime_ws}" "${episode_root}"
  exit 0
fi

mkdir -p "${run_root}" "${cleaning_root}" "${dashboard_output}"

write_state() {
  local stage="$1" map_sha256="${2:-}"
  python3 - "${state_file}" "${stage}" "${map_sha256}" <<'PY'
import json, os, pathlib, sys
path = pathlib.Path(sys.argv[1])
payload = {
    "field_dimensions_m": [200, 100], "vehicle": "A300", "stage": sys.argv[2],
    "map_sha256": sys.argv[3] or None, "perception_provider": "unavailable",
    "formal_product_acceptance": False,
}
temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
temporary.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
os.replace(temporary, path)
PY
}

write_terminal() {
  local detail="$1"
  python3 - "${terminal_output}" "${detail}" "${hmi_telemetry_sha256}" <<'PY'
import datetime, json, pathlib, sys
pathlib.Path(sys.argv[1]).write_text(json.dumps({
  "schema_version": 1, "status": "FINAL_VISUAL_PREVIEW_NOT_PRODUCT_PASS",
  "product_pass": False, "perception_provider": "unavailable",
  "detail": sys.argv[2], "field_dimensions_m": [200, 100], "vehicle": "A300",
  "hmi_terminal_telemetry_sha256": None if sys.argv[3] == "" else sys.argv[3],
  "completed_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
}

dashboard_pid=""
state_publisher_pid=""
mapping_launch_pid=""
cleaning_launch_pid=""
emergency_stop_pid=""
main_power_pid=""
terminal_written=false
hmi_telemetry_sha256=""
mapping_partition="tzcup_final_visual_mapping_${mapping_ros_domain}_$$"
cleaning_partition="tzcup_final_visual_cleaning_${cleaning_ros_domain}_$$"
stop_pid() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 -- "-${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 -- "-${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  echo "process group ${pid} survived TERM then KILL" >&2
  return 1
}
cleanup() {
  local cleanup_status=0
  # The formal helper owns exact process-group plus exact GZ_PARTITION
  # cleanup.  It never relies on a broad ros2/gz process match.
  formal_runtime_cleanup_groups "${mapping_partition}" "${mapping_launch_pid}" || cleanup_status=1
  formal_runtime_cleanup_groups "${cleaning_partition}" "${cleaning_launch_pid}" || cleanup_status=1
  stop_pid "${emergency_stop_pid}" || cleanup_status=1
  stop_pid "${main_power_pid}" || cleanup_status=1
  stop_pid "${state_publisher_pid}" || cleanup_status=1
  stop_pid "${dashboard_pid}" || cleanup_status=1
  for pid in "${mapping_launch_pid}" "${cleaning_launch_pid}" "${emergency_stop_pid}" "${main_power_pid}" "${state_publisher_pid}" "${dashboard_pid}"; do
    [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null || true
  done
  if [[ -d "${run_root}" && "${terminal_written}" != true && ! -e "${terminal_output}" ]]; then
    write_terminal "preview interrupted or failed before completion; formal product acceptance was not evaluated"
  fi
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup

require_phase_processes() {
  local label="$1" pid
  shift
  for pid in "$@"; do
    if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
      echo "${label} process exited unexpectedly: ${pid:-missing}" >&2
      return 125
    fi
  done
}

hmi_receipt_matches() {
  local expected_stage="$1" expected_hash="$2"
  python3 - "${dashboard_port}" "${dashboard_output}/dashboard_telemetry.json" \
    "${expected_stage}" "${expected_hash}" <<'PY'
import http.client
import json
import os
import pathlib
import sys

port, telemetry_path, stage, digest = sys.argv[1:]
try:
    def fetch_json(route):
        connection = http.client.HTTPConnection("127.0.0.1", int(port), timeout=3.0)
        try:
            connection.request("GET", route)
            response = connection.getresponse()
            body = response.read()
            if response.status != 200:
                raise RuntimeError(f"{route} status")
            return json.loads(body)
        finally:
            connection.close()

    if fetch_json("/healthz").get("status") != "ok":
        raise RuntimeError("health payload")
    payload = fetch_json("/api/v1/telemetry")
    final_demo = payload["final_demo"]
    expected_digest = None if digest == "" else digest
    if (
        final_demo.get("status") != "live"
        or final_demo.get("stage") != stage
        or final_demo.get("map_sha256") != expected_digest
    ):
        raise RuntimeError("telemetry stage/hash mismatch")
    path = pathlib.Path(telemetry_path)
    temporary = path.with_name(f".{path.name}.runner.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)
except (OSError, KeyError, TypeError, ValueError, RuntimeError):
    raise SystemExit(1)
PY
}

wait_for_hmi_receipt() {
  local expected_stage="$1" expected_hash="$2" deadline
  deadline=$((SECONDS + hmi_receipt_timeout_sec))
  while (( SECONDS < deadline )); do
    if kill -0 "${dashboard_pid}" 2>/dev/null \
      && kill -0 "${state_publisher_pid}" 2>/dev/null \
      && hmi_receipt_matches "${expected_stage}" "${expected_hash}"; then
      hmi_telemetry_sha256="$(sha256sum "${dashboard_output}/dashboard_telemetry.json" | awk '{print $1}')"
      return 0
    fi
    sleep 1
  done
  echo "HMI receipt did not become healthy for ${expected_stage}" >&2
  return 125
}

require_hmi_receipt() {
  local attempt
  for attempt in {1..5}; do
    if kill -0 "${dashboard_pid}" 2>/dev/null \
      && kill -0 "${state_publisher_pid}" 2>/dev/null \
      && hmi_receipt_matches "$1" "$2"; then
      return 0
    fi
    sleep 1
  done
  echo "HMI health, PID, or telemetry receipt failed during $1 after retries" >&2
  return 125
}

dashboard_has_first_map() {
  python3 - "${dashboard_output}/dashboard_telemetry.json" <<'PY'
import json
import pathlib
import sys

try:
    grid = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))["visualization"]["occupancy_grid"]
    ok = int(grid["width"]) > 0 and int(grid["height"]) > 0 and len(grid["data"]) == int(grid["width"]) * int(grid["height"])
except (OSError, KeyError, TypeError, ValueError):
    ok = False
raise SystemExit(0 if ok else 1)
PY
}

require_memory_watchdog() {
  local watchdog_pid="${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}" watchdog_status
  [[ -n "${watchdog_pid}" ]] || return 125
  kill -0 "${watchdog_pid}" 2>/dev/null && return 0
  set +e
  wait "${watchdog_pid}" 2>/dev/null
  watchdog_status=$?
  set -e
  formal_runtime_record_memory_watchdog_exit "${watchdog_pid}" "${watchdog_status}" || return 125
  formal_runtime_memory_watchdog_tripped && return "${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
  return 125
}

finish_memory_watchdog() {
  formal_runtime_stop_memory_watchdog
  formal_runtime_memory_watchdog_tripped && return "${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
  (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT == 0 )) || return 125
}

# Refuse a memory-starved start, then pin ROS discovery to loopback and take
# the shared Gazebo lease before any DDS participant exists.
formal_runtime_memory_preflight "${run_root}/windows_memory_preflight"
formal_runtime_configure "${mapping_ros_domain}"

set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_ws}/install/setup.bash"
set -u
export TZCUP_REPOSITORY_ROOT="${repo_root}"

if [[ ! -f "${episode_root}/public/world.sdf" ]]; then
  ROS_DOMAIN_ID="${mapping_ros_domain}" ros2 run sanitation_campus_scenario sanitation-campus-scenario generate \
    --config "${repo_root}/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" \
    --profile formal --split train --map-index 0 --mission-index 0 --output "${episode_root}" \
    >"${run_root}/episode_generator.log" 2>&1
fi
for required in public/world.sdf public/episode_manifest.json environment/pedestrian_schedule.json; do
  [[ -f "${episode_root}/${required}" ]] || { echo "episode generation failed: missing ${required}" >&2; exit 3; }
done

# The HMI is read-only.  Give it the same public, declared map-frame geofence
# that starts this episode so an otherwise quiet mapping phase still renders
# the real 200x100m campus rather than the legacy empty-canvas fallback.  It
# does not add world/evaluator truth, a path, or a vehicle pose.
python3 - "${episode_root}/public/episode_manifest.json" "${preview_hmi_mission}" <<'PY'
import json
import pathlib
import sys
import yaml

episode = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
field = episode.get("field", {})
polygon = field.get("localization_map_geofence", {}).get("polygon_m")
if (
    episode.get("profile") != "formal"
    or field.get("area_m2") != 20000.0
    or not isinstance(polygon, list)
    or len(polygon) < 3
):
    raise SystemExit("preview HMI requires the declared formal 200x100m episode geofence")
pathlib.Path(sys.argv[2]).write_text(yaml.safe_dump({
    "mission_id": episode.get("episode_id", "formal-preview"),
    "outer_polygon": polygon,
    "keepout_polygons": [],
    "exclusion_polygons": [],
}, allow_unicode=True, sort_keys=True), encoding="utf-8")
PY

start_dashboard() {
  local domain="$1" phase="$2"
  (
    export ROS_DOMAIN_ID="${domain}"
    export PYTHONPATH="${repo_root}/starter_ws/src/sanitation_hmi${PYTHONPATH:+:${PYTHONPATH}}"
    exec "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 -m sanitation_hmi.live_server --ros-args \
      -p use_sim_time:=true -p port:="${dashboard_port}" -p output_dir:="${dashboard_output}" \
      -p mission_config:="${preview_hmi_mission}" \
      -p web_root:="${repo_root}/starter_ws/src/sanitation_hmi/web"
  ) >"${dashboard_output}/dashboard.${phase}.log" 2>&1 & dashboard_pid=$!
  (
    export ROS_DOMAIN_ID="${domain}"
    exec "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${repo_root}/scripts/publish_final_product_visual_state.py" \
      --state-file "${state_file}" --period-sec 1.0
  ) >"${dashboard_output}/state.${phase}.log" 2>&1 & state_publisher_pid=$!
}

start_safety_heartbeat() {
  local domain="$1" phase="$2"
  (
    export ROS_DOMAIN_ID="${domain}"
    exec "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 topic pub /formal_vehicle/simulation/command/emergency_stop \
      std_msgs/msg/Bool "{data: false}" -r 10
  ) >"${run_root}/emergency_stop.${phase}.log" 2>&1 & emergency_stop_pid=$!
  (
    export ROS_DOMAIN_ID="${domain}"
    exec "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 topic pub /formal_vehicle/simulation/command/main_power \
      std_msgs/msg/Bool "{data: true}" -r 10
  ) >"${run_root}/main_power.${phase}.log" 2>&1 & main_power_pid=$!
}

prepare_mapping_world() {
  python3 "${repo_root}/scripts/prepare_formal_mapping_world.py" \
    --source "${episode_root}/public/world.sdf" --episode-manifest "${episode_root}/public/episode_manifest.json" \
    --output "${mapping_world}" --report "${run_root}/mapping_world_preparation.json"
}
prepare_cleaning_world() {
  python3 "${repo_root}/scripts/prepare_formal_dynamic_runtime_world.py" \
    --source "${episode_root}/public/world.sdf" --output "${cleaning_world}" \
    --manifest "${cleaning_root}/cleaning_world_manifest.json"
}

prepare_mapping_world
write_state MAPPING
start_dashboard "${mapping_ros_domain}" mapping
wait_for_hmi_receipt MAPPING ""
export ROS_DOMAIN_ID="${mapping_ros_domain}"
export GZ_PARTITION="${mapping_partition}"
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus_map_lifecycle.launch.py \
  mission_mode:=mapping gui:="${gazebo_gui}" world:="${mapping_world}" \
  simulation_initial_estop_active:=false \
  lidar_bridge_ready_timeout_sec:=600 \
  episode_manifest:="${episode_root}/public/episode_manifest.json" map_artifact_dir:="${map_root}" \
  pedestrian_schedule:="${episode_root}/environment/pedestrian_schedule.json" \
  start_pedestrians:=false start_coverage:=false operation_speed_profile:=mapping_safe \
  >"${run_root}/mapping.launch.log" 2>&1 & mapping_launch_pid=$!
formal_runtime_start_memory_watchdog \
  "${mapping_launch_pid}" "${run_root}/mapping.memory_watchdog"
start_safety_heartbeat "${mapping_ros_domain}" mapping

# The preview needs a real sealed map before it can demonstrate a hard restart,
# but does not evaluate or present that map as a formal acceptance artifact.
deadline=$((SECONDS + mapping_timeout_sec))
mapping_progress_deadline=$((SECONDS + phase_progress_timeout_sec))
mapping_scan_ready=false
mapping_first_map=false
while [[ ! -f "${map_root}/map_lifecycle_manifest.json" ]]; do
  kill -0 "${mapping_launch_pid}" 2>/dev/null || { echo "mapping launch exited; see ${run_root}/mapping.launch.log" >&2; exit 3; }
  require_memory_watchdog
  require_phase_processes mapping \
    "${dashboard_pid}" "${state_publisher_pid}" "${emergency_stop_pid}" "${main_power_pid}"
  require_hmi_receipt MAPPING ""
  if [[ "${mapping_scan_ready}" == "false" ]] \
    && grep -Fq "canonical scan ready; starting standard autostart SLAM lifecycle" \
      "${run_root}/mapping.launch.log" 2>/dev/null; then
    mapping_scan_ready=true
  fi
  if [[ "${mapping_first_map}" == "false" ]] && dashboard_has_first_map; then
    mapping_first_map=true
  fi
  if (( SECONDS >= mapping_progress_deadline )) \
    && { [[ "${mapping_scan_ready}" != "true" ]] || [[ "${mapping_first_map}" != "true" ]]; }; then
    echo "mapping progress watchdog timed out: scan_ready=${mapping_scan_ready} first_map=${mapping_first_map}" >&2
    exit 4
  fi
  (( SECONDS < deadline )) || { echo "preview mapping timed out before a sealed map" >&2; exit 4; }
  sleep 2
done
if [[ "${mapping_scan_ready}" != "true" || "${mapping_first_map}" != "true" ]]; then
  echo "sealed map arrived without required scan-ready and first-map progress" >&2
  exit 4
fi
map_sha256="$(sha256sum "${map_root}/map_lifecycle_manifest.json" | awk '{print $1}')"
write_state MAP_SAVED "${map_sha256}"
wait_for_hmi_receipt MAP_SAVED "${map_sha256}"
formal_runtime_cleanup_groups "${mapping_partition}" "${mapping_launch_pid}" || exit 125
finish_memory_watchdog || exit $?
mapping_launch_pid=""
stop_pid "${emergency_stop_pid}" || exit 125
stop_pid "${main_power_pid}" || exit 125
wait "${emergency_stop_pid}" 2>/dev/null || true; wait "${main_power_pid}" 2>/dev/null || true
emergency_stop_pid=""; main_power_pid=""
write_state HARD_RESTART "${map_sha256}"
wait_for_hmi_receipt HARD_RESTART "${map_sha256}"
stop_pid "${state_publisher_pid}" || exit 125
stop_pid "${dashboard_pid}" || exit 125
wait "${state_publisher_pid}" 2>/dev/null || true; wait "${dashboard_pid}" 2>/dev/null || true
state_publisher_pid=""; dashboard_pid=""

prepare_cleaning_world
write_state RELOAD_LOCALIZE "${map_sha256}"
start_dashboard "${cleaning_ros_domain}" cleaning
wait_for_hmi_receipt RELOAD_LOCALIZE "${map_sha256}"
export ROS_DOMAIN_ID="${cleaning_ros_domain}"
export GZ_PARTITION="${cleaning_partition}"
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus_map_lifecycle.launch.py \
  mission_mode:=cleaning cleaning_planner:=full_coverage gui:="${gazebo_gui}" world:="${cleaning_world}" \
  simulation_initial_estop_active:=false \
  episode_manifest:="${episode_root}/public/episode_manifest.json" map_artifact_dir:="${map_root}" \
  pedestrian_schedule:="${episode_root}/environment/pedestrian_schedule.json" start_pedestrians:=true \
  start_coverage:=true coverage_evidence_dir:="${cleaning_root}" \
  operation_speed_profile:=dry_cleaning_competition_candidate \
  >"${cleaning_root}/cleaning.launch.log" 2>&1 & cleaning_launch_pid=$!
formal_runtime_start_memory_watchdog \
  "${cleaning_launch_pid}" "${cleaning_root}/cleaning.memory_watchdog"
start_safety_heartbeat "${cleaning_ros_domain}" cleaning

deadline=$((SECONDS + cleaning_timeout_sec))
cleaning_progress_deadline=$((SECONDS + phase_progress_timeout_sec))
coverage_stage_published=false
coverage_report="${cleaning_root}/coverage_execution.json"
while [[ ! -s "${coverage_report}" ]]; do
  require_memory_watchdog
  require_phase_processes cleaning \
    "${dashboard_pid}" "${state_publisher_pid}" "${emergency_stop_pid}" "${main_power_pid}"
  if [[ "${coverage_stage_published}" == "true" ]]; then
    require_hmi_receipt COVERAGE "${map_sha256}"
  else
    require_hmi_receipt RELOAD_LOCALIZE "${map_sha256}"
    if [[ -s "${cleaning_root}/hard_restart_record.json" ]]; then
      write_state COVERAGE "${map_sha256}"
      wait_for_hmi_receipt COVERAGE "${map_sha256}"
      coverage_stage_published=true
    elif (( SECONDS >= cleaning_progress_deadline )); then
      echo "cleaning progress watchdog timed out before hard-restart receipt" >&2
      exit 4
    fi
  fi
  if ! kill -0 "${cleaning_launch_pid}" 2>/dev/null; then
    write_state PRODUCT_TERMINAL "${map_sha256}"
    wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"
    write_terminal "same-map FullCoverage launch exited before a terminal report"
    terminal_written=true
    echo "cleaning launch exited before coverage report; see ${cleaning_root}/cleaning.launch.log" >&2
    exit 5
  fi
  if (( SECONDS >= deadline )); then
    write_state PRODUCT_TERMINAL "${map_sha256}"
    wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"
    write_terminal "same-map FullCoverage timed out before a terminal report"
    terminal_written=true
    echo "preview cleaning timed out; terminal remains NOT_PRODUCT_PASS" >&2
    exit 4
  fi
  sleep 2
done
if [[ "${coverage_stage_published}" != "true" ]]; then
  write_state PRODUCT_TERMINAL "${map_sha256}"
  wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"
  write_terminal "coverage report appeared without a hard-restart coverage receipt"
  terminal_written=true
  echo "preview coverage report appeared before hard-restart receipt" >&2
  exit 5
fi
if ! python3 - "${coverage_report}" <<'PY'
import json
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if report.get("success") is not True or report.get("terminal_state") != "COMPLETED":
    raise SystemExit(1)
PY
then
  write_state PRODUCT_TERMINAL "${map_sha256}"
  wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"
  write_terminal "same-map FullCoverage produced a non-success terminal report"
  terminal_written=true
  echo "preview coverage report is non-success: ${coverage_report}" >&2
  exit 5
fi
formal_runtime_cleanup_groups "${cleaning_partition}" "${cleaning_launch_pid}" || exit 125
finish_memory_watchdog || exit $?
cleaning_launch_pid=""
stop_pid "${emergency_stop_pid}" || exit 125
stop_pid "${main_power_pid}" || exit 125
wait "${emergency_stop_pid}" 2>/dev/null || true; wait "${main_power_pid}" 2>/dev/null || true
emergency_stop_pid=""; main_power_pid=""
write_state PRODUCT_TERMINAL "${map_sha256}"
wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"
write_terminal "live mapping and same-map FullCoverage preview; no formal closure/session/perception acceptance"
terminal_written=true
printf 'FINAL_VISUAL_PREVIEW_TERMINAL=%s\n' "${terminal_output}"
