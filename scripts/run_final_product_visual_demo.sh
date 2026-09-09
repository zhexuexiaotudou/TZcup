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
# A visual short run must prove that the live graph has progressed; a healthy
# PID or memory guard alone is not evidence that SLAM/HMI is advancing.
phase_progress_timeout_sec=600
hmi_receipt_timeout_sec=90

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
mapping_runner_pid=""
cleaning_runner_pid=""
hmi_telemetry_sha256=""

stop_exact_child() {
  local pid="$1" signal attempt
  [[ -n "${pid}" ]] || return 0
  for signal in INT TERM KILL; do
    kill -0 "${pid}" 2>/dev/null || break
    kill -"${signal}" "${pid}" 2>/dev/null || true
    for attempt in {1..80}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
  done
  wait "${pid}" 2>/dev/null || true
  ! kill -0 "${pid}" 2>/dev/null
}

stop_exact_visual_pid() {
  local pid="$1" attempt
  [[ -n "${pid}" ]] || return 0
  kill -TERM "${pid}" 2>/dev/null || true
  for attempt in {1..40}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.25
  done
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
  ! kill -0 "${pid}" 2>/dev/null
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
        connection = http.client.HTTPConnection("127.0.0.1", int(port), timeout=1.0)
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
  [[ "${formal_visual_gui}" == "true" ]] || return 0
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
  [[ "${formal_visual_gui}" == "true" ]] || return 0
  kill -0 "${dashboard_pid}" 2>/dev/null \
    && kill -0 "${state_publisher_pid}" 2>/dev/null \
    && hmi_receipt_matches "$1" "$2" || {
      echo "HMI health, PID, or telemetry receipt failed during $1" >&2
      return 125
    }
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

stop_visual_stack() {
  local cleanup_status=0
  stop_exact_child "${cleaning_runner_pid}" || cleanup_status=1
  stop_exact_child "${mapping_runner_pid}" || cleanup_status=1
  stop_exact_visual_pid "${state_publisher_pid}" || cleanup_status=1
  stop_exact_visual_pid "${dashboard_pid}" || cleanup_status=1
  cleaning_runner_pid=""
  mapping_runner_pid=""
  dashboard_pid=""
  state_publisher_pid=""
  return "${cleanup_status}"
}
trap stop_visual_stack EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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
  wait_for_hmi_receipt MAPPING ""
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
bash "${repo_root}/scripts/run_formal_first_map_dynamic_prerequisite.sh" &
mapping_runner_pid=$!
mapping_progress_deadline=$((SECONDS + phase_progress_timeout_sec))
mapping_scan_ready=false
mapping_first_map=false
while kill -0 "${mapping_runner_pid}" 2>/dev/null; do
  require_hmi_receipt MAPPING ""
  if [[ "${mapping_scan_ready}" == "false" ]] \
    && grep -Fq "canonical scan ready; starting standard autostart SLAM lifecycle" \
      "${map_root}/mapping.launch.log" 2>/dev/null; then
    mapping_scan_ready=true
  fi
  if [[ "${formal_visual_gui}" == "true" && "${mapping_first_map}" == "false" ]] \
    && dashboard_has_first_map; then
    mapping_first_map=true
  fi
  if (( SECONDS >= mapping_progress_deadline )) \
    && { [[ "${mapping_scan_ready}" != "true" ]] \
      || { [[ "${formal_visual_gui}" == "true" && "${mapping_first_map}" != "true" ]]; }; }; then
    echo "mapping progress watchdog timed out: scan_ready=${mapping_scan_ready} first_map=${mapping_first_map}" >&2
    exit 4
  fi
  sleep 2
done
wait "${mapping_runner_pid}"
mapping_runner_pid=""
if [[ "${mapping_scan_ready}" != "true" ]] \
  || { [[ "${formal_visual_gui}" == "true" ]] && [[ "${mapping_first_map}" != "true" ]]; }; then
  echo "mapping ended without the required scan-ready and first-map progress" >&2
  exit 4
fi

map_sha256="$(sha256sum "${map_root}/map_lifecycle_manifest.json" | awk '{print $1}')"
if [[ "${formal_visual_gui}" == "true" ]]; then
  write_final_demo_state MAP_SAVED "${map_sha256}"
  wait_for_hmi_receipt MAP_SAVED "${map_sha256}"
  write_final_demo_state HARD_RESTART "${map_sha256}"
  wait_for_hmi_receipt HARD_RESTART "${map_sha256}"
  stop_visual_stack
  write_final_demo_state RELOAD_LOCALIZE "${map_sha256}"
  start_visual_stack cleaning "${cleaning_ros_domain}"
  wait_for_hmi_receipt RELOAD_LOCALIZE "${map_sha256}"
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
cleaning_progress_deadline=$((SECONDS + phase_progress_timeout_sec))
coverage_stage_published=false
while kill -0 "${cleaning_runner_pid}" 2>/dev/null; do
  if [[ "${coverage_stage_published}" == "true" ]]; then
    require_hmi_receipt COVERAGE "${map_sha256}"
  else
    require_hmi_receipt RELOAD_LOCALIZE "${map_sha256}"
    if [[ -s "${cleaning_runtime}/hard_restart_record.json" ]]; then
      write_final_demo_state COVERAGE "${map_sha256}"
      wait_for_hmi_receipt COVERAGE "${map_sha256}"
      coverage_stage_published=true
    elif (( SECONDS >= cleaning_progress_deadline )); then
      echo "cleaning progress watchdog timed out before hard-restart receipt" >&2
      exit 4
    fi
  fi
  sleep 2
done
wait "${cleaning_runner_pid}"
cleaning_runner_pid=""
if [[ "${coverage_stage_published}" != "true" ]]; then
  echo "cleaning ended without a hard-restart coverage stage receipt" >&2
  exit 4
fi

terminal_status="FORMAL_PC_PERCEPTION_COMPLETED_NOT_PRODUCT_PASS"
if [[ "${perception_mode}" == "unavailable" ]]; then
  terminal_status="PERCEPTION_BLOCKED_NOT_PRODUCT_PASS"
fi
if [[ "${formal_visual_gui}" == "true" ]]; then
  write_final_demo_state PRODUCT_TERMINAL "${map_sha256}"
  wait_for_hmi_receipt PRODUCT_TERMINAL "${map_sha256}"
fi
python3 - "${terminal_output}" "${terminal_status}" "${perception_mode}" "${cleaning_planner}" \
  "${mapping_output}" "${cleaning_output}" "${formal_visual_gui}" "${runtime_ws}" \
  "${runtime_closure}" "${vehicle_snapshot}" "${acceptance_session}" "${episode_root}" \
  "${mapping_ros_domain}" "${cleaning_ros_domain}" "${operation_speed_profile}" "${hmi_telemetry_sha256}" \
  "${cleaning_runtime}" <<'PY'
import datetime
import hashlib
import json
import pathlib
import sys

output = pathlib.Path(sys.argv[1])
output.parent.mkdir(parents=True, exist_ok=True)
map_root = pathlib.Path(sys.argv[5])
cleaning_output = pathlib.Path(sys.argv[6])
cleaning_runtime = pathlib.Path(sys.argv[17])

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

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
    "hmi_terminal_telemetry_sha256": None if sys.argv[16] == "" else sys.argv[16],
    "artifact_sha256": {
        "map_lifecycle_manifest": digest(map_root / "map_lifecycle_manifest.json"),
        "mapping_runtime": digest(map_root / "mapping_runtime.json"),
        "mapping_handoff": digest(map_root / "mapping_handoff_record.json"),
        "hard_restart_record": digest(cleaning_runtime / "hard_restart_record.json"),
        "cleaning_runtime_binding": digest(cleaning_runtime / "runtime_gate_binding.json"),
        "map_lifecycle_acceptance": digest(cleaning_output),
    },
    "completed_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
}
output.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
printf 'FORMAL_FINAL_PRODUCT_VISUAL_TERMINAL=%s\n' "${terminal_output}"
