#!/usr/bin/env bash
# One hard-restarted saved-map Gazebo process, one FullCoverage server/probe,
# and one source-recomputable formal baseline. This script is intentionally
# long-running; callers retain its entire output directory as evidence.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/scripts/run_formal_runtime_isolation.sh"
source "${ROOT}/scripts/formal_source_bound_preflight.sh"
OPERATION_SPEED_PROFILE="${FORMAL_OPERATION_SPEED_PROFILE:-dry_cleaning_competition_candidate}"
EPISODE_ROOT=""
MAP_ROOT=""
SESSION=""
OVERLAY="${FORMAL_BASELINE_RUNTIME_OVERLAY:-}"
OUTPUT=""
FORMAL_OUTPUT="${FORMAL_SAME_MAP_BASELINE_OUTPUT:-${ROOT}/artifacts/formal_same_map_full_coverage_baseline.json}"
ROS_DOMAIN="61"
TIMEOUT="86400"
SNAPSHOT="${ROOT}/reports/engineering/formal_vehicle_snapshot_manifest.json"
RUNTIME_WS="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen colcon workspace}"
RUNTIME_INSTALL="${RUNTIME_WS}/install"
RUNTIME_CLOSURE_MANIFEST="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${RUNTIME_WS}/final_runtime_closure_manifest.json}"

while (($#)); do
  case "$1" in
    --episode-root) EPISODE_ROOT="$2"; shift 2 ;;
    --map-root) MAP_ROOT="$2"; shift 2 ;;
    --session) SESSION="$2"; shift 2 ;;
    --runtime-overlay) OVERLAY="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    --formal-output) FORMAL_OUTPUT="$2"; shift 2 ;;
    --ros-domain-id) ROS_DOMAIN="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
formal_runtime_register_evidence_paths "${FORMAL_OUTPUT}"
for value in EPISODE_ROOT MAP_ROOT SESSION OUTPUT; do
  [[ -n "${!value}" ]] || { echo "missing required argument for ${value}" >&2; exit 2; }
done
if [[ -n "${OVERLAY}" && "$(realpath "${OVERLAY}")" != "$(realpath "${RUNTIME_INSTALL}")" ]]; then
  echo "formal same-map runtime overlay must be the one frozen runtime install" >&2
  exit 2
fi
OVERLAY="${RUNTIME_INSTALL}"
formal_runtime_configure "${ROS_DOMAIN}"
if [[ ! "${TIMEOUT}" =~ ^[0-9]+$ ]] || (( TIMEOUT < 300 )); then
  echo "timeout must be an integer >=300 seconds" >&2; exit 2
fi
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "missing ROS Jazzy" >&2; exit 2; }
[[ -f "${OVERLAY}/setup.bash" ]] || { echo "missing frozen runtime overlay: ${OVERLAY}/setup.bash" >&2; exit 2; }
[[ ! -e "${OUTPUT}" ]] || { echo "refusing retained run directory: ${OUTPUT}" >&2; exit 3; }
[[ ! -e "${FORMAL_OUTPUT}" ]] || { echo "refusing retained formal baseline: ${FORMAL_OUTPUT}" >&2; exit 3; }

EPISODE_MANIFEST="${EPISODE_ROOT}/public/episode_manifest.json"
WORLD="${EPISODE_ROOT}/public/world.sdf"
SCHEDULE="${EPISODE_ROOT}/environment/pedestrian_schedule.json"
MAPPING_RUNTIME="${MAP_ROOT}/mapping_runtime.json"
for file in "${EPISODE_MANIFEST}" "${WORLD}" "${SCHEDULE}" "${MAPPING_RUNTIME}" \
  "${MAP_ROOT}/map_lifecycle_manifest.json" "${MAP_ROOT}/mission_geometry.yaml" \
  "${SESSION}" "${SNAPSHOT}"; do
  [[ -f "${file}" ]] || { echo "missing required file: ${file}" >&2; exit 3; }
done
mkdir -p "${OUTPUT}"
RUNTIME_BINDING="${OUTPUT}/runtime_gate_binding.json"
formal_source_bound_preflight \
  "${ROOT}" "${RUNTIME_WS}" "${RUNTIME_CLOSURE_MANIFEST}" \
  "${SESSION}" "${SNAPSHOT}" "${RUNTIME_BINDING}"

source /opt/ros/jazzy/setup.bash
source "${OVERLAY}/setup.bash"
formal_source_bound_verify_overlay "${RUNTIME_INSTALL}"
export TZCUP_REPOSITORY_ROOT="${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN}"
export GZ_PARTITION="tzcup-formal-same-map-baseline-${ROS_DOMAIN}-$$"

# Verify the saved map against the same public episode before Gazebo starts.
python3 - "${EPISODE_MANIFEST}" "${MAP_ROOT}" <<'PY'
import pathlib, sys
from sanitation_formal_campus_integration.map_lifecycle_core import (
    load_campus_map_contract, validate_saved_map_artifact,
)
contract = load_campus_map_contract(pathlib.Path(sys.argv[1]))
validate_saved_map_artifact(pathlib.Path(sys.argv[2]), contract)
PY

readarray -t START < <(python3 - "${EPISODE_MANIFEST}" "${SESSION}" "${SNAPSHOT}" <<'PY'
import hashlib, json, sys
episode=json.load(open(sys.argv[1],encoding='utf-8'))
session=json.load(open(sys.argv[2],encoding='utf-8'))
snapshot=json.load(open(sys.argv[3],encoding='utf-8'))
outputs=snapshot.get('outputs',{})
urdf=outputs.get('reports/engineering/formal_competition_vehicle.urdf',{}).get('sha256')
identity={'snapshot_manifest_sha256':hashlib.sha256(open(sys.argv[3],'rb').read()).hexdigest(),
          'source_inventory_sha256':snapshot.get('source_inventory_sha256'),
          'expanded_urdf_sha256':urdf}
if session.get('status') != 'FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING' or session.get('snapshot') != identity:
    raise SystemExit('formal session is not RUNNING on the current frozen snapshot')
if episode.get('profile') != 'formal' or float(episode.get('field',{}).get('area_m2',0.0)) < 20000.0:
    raise SystemExit('episode is not a formal >=20000 m2 field')
pose=episode.get('vehicle_start_pose_map',{})
for key in ('x_m','y_m','yaw_rad'):
    if not isinstance(pose.get(key),(int,float)): raise SystemExit('episode fixed start is missing')
print(float(pose['x_m'])); print(float(pose['y_m'])); print(float(pose['yaw_rad']))
PY
)
START_X="${START[0]}"; START_Y="${START[1]}"; START_YAW="${START[2]}"

python3 "${ROOT}/scripts/prepare_formal_same_map_coverage.py" \
  --mission "${MAP_ROOT}/mission_geometry.yaml" \
  --motion-profile "${ROOT}/config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml" \
  --probe-output "${OUTPUT}/coverage_probe_config.yaml" \
  --server-output "${OUTPUT}/coverage_server_params.yaml"
python3 "${ROOT}/scripts/prepare_formal_dynamic_runtime_world.py" \
  --source "${WORLD}" --output "${OUTPUT}/world.sdf" \
  --manifest "${OUTPUT}/world_manifest.json"
WORLD_NAME="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1],encoding="utf-8"))["world_name"])' "${OUTPUT}/world_manifest.json")"
[[ -n "${WORLD_NAME}" ]] || { echo "runtime world has no name" >&2; exit 3; }

mapping_count="$({ ps -eo args | grep -F 'mission_mode:=mapping' | grep -F "map_artifact_dir:=${MAP_ROOT}" | grep -v grep || true; } | wc -l)"
mapping_count="${mapping_count//[[:space:]]/}"
[[ "${mapping_count}" == "0" ]] || { echo "mapping process still alive" >&2; exit 3; }
python3 - "${OUTPUT}/hard_restart_record.json" <<'PY'
import datetime,json,pathlib,sys
now=datetime.datetime.now(datetime.timezone.utc).isoformat()
pathlib.Path(sys.argv[1]).write_text(json.dumps({
  'schema_version':1,'mapping_stopped_before_cleaning':True,
  'mapping_process_count_before_cleaning':0,'mapping_stop_wall_time':now,
  'cleaning_start_wall_time':now,'restart_type':'separate_process_hard_restart',
},indent=2,sort_keys=True)+'\n',encoding='utf-8')
PY

PIDS=()
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${PIDS[@]:-}"
}
formal_runtime_install_traps cleanup

# The lifecycle launch owns one Gazebo/Nav2/AMCL graph, but deliberately does
# not start its hard-coded server. The baseline server below consumes configs
# derived from the frozen real cleaning footprint and effective swath width.
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus_map_lifecycle.launch.py \
  mission_mode:=cleaning cleaning_planner:=full_coverage start_coverage:=false \
  gui:=false world:="${OUTPUT}/world.sdf" world_name:="${WORLD_NAME}" episode_manifest:="${EPISODE_MANIFEST}" \
  map_artifact_dir:="${MAP_ROOT}" pedestrian_schedule:="${SCHEDULE}" \
  operation_speed_profile:="${OPERATION_SPEED_PROFILE}" \
  start_pedestrians:=true >"${OUTPUT}/cleaning.launch.log" 2>&1 &
LAUNCH_PID=$!; PIDS+=("${LAUNCH_PID}")
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  "/world/${WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
  --ros-args -r "/world/${WORLD_NAME}/dynamic_pose/info:=/evaluation/formal_same_map/dynamic_pose" \
  >"${OUTPUT}/ground_truth_bridge.log" 2>&1 & PIDS+=("$!")
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${ROOT}/scripts/formal_same_map_baseline_support.py" \
  --start-x "${START_X}" --start-y "${START_Y}" --start-yaw "${START_YAW}" \
  >"${OUTPUT}/baseline_support.log" 2>&1 & PIDS+=("$!")

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${ROOT}/scripts/collect_formal_map_lifecycle_runtime.py" \
  --mode cleaning --map-root "${MAP_ROOT}" --timeout 180 \
  --restart-record "${OUTPUT}/hard_restart_record.json" \
  --output "${OUTPUT}/cleaning_runtime.json" \
  >"${OUTPUT}/cleaning_runtime.collector.log" 2>&1 &
COLLECTOR_PID=$!; PIDS+=("${COLLECTOR_PID}")
set +e; wait "${COLLECTOR_PID}"; COLLECTOR_STATUS=$?; set -e
unset 'PIDS[-1]'
(( COLLECTOR_STATUS == 0 )) || { echo "saved-map readiness collector failed" >&2; exit 4; }
kill -0 "${LAUNCH_PID}" 2>/dev/null || { echo "cleaning launch exited before FullCoverage" >&2; exit 4; }

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run opennav_coverage opennav_coverage --ros-args \
  --params-file "${OUTPUT}/coverage_server_params.yaml" \
  >"${OUTPUT}/coverage_server.log" 2>&1 & SERVER_PID=$!; PIDS+=("${SERVER_PID}")
for _ in {1..120}; do
  ros2 service list 2>/dev/null | grep -q '^/coverage_server/change_state$' && break
  kill -0 "${SERVER_PID}" 2>/dev/null || { echo "coverage_server exited" >&2; exit 4; }
  sleep 1
done
ros2 service list | grep -q '^/coverage_server/change_state$' || { echo "coverage_server not ready" >&2; exit 4; }
ros2 lifecycle set /coverage_server configure >"${OUTPUT}/coverage_server.configure.log"
grep -q 'Transitioning successful' "${OUTPUT}/coverage_server.configure.log" || { echo "coverage_server configure failed" >&2; exit 4; }
ros2 lifecycle set /coverage_server activate >"${OUTPUT}/coverage_server.activate.log"
grep -q 'Transitioning successful' "${OUTPUT}/coverage_server.activate.log" || { echo "coverage_server activate failed" >&2; exit 4; }

for _ in {1..180}; do
  actions="$(ros2 action list 2>/dev/null || true)"
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -q '^/compute_coverage_path$' <<<"${actions}" && \
     grep -q '^/navigate_to_pose$' <<<"${actions}" && \
     grep -q '^/ground_truth/odom$' <<<"${topics}"; then break; fi
  kill -0 "${LAUNCH_PID}" 2>/dev/null || { echo "cleaning launch exited during readiness" >&2; exit 4; }
  sleep 1
done
timeout 30 ros2 topic echo --once /ground_truth/odom nav_msgs/msg/Odometry \
  >"${OUTPUT}/first_ground_truth_odom.txt" 2>&1 || { echo "evaluator ground truth unavailable" >&2; exit 4; }

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run sanitation_coverage coverage_probe --ros-args \
  -p use_sim_time:=true -p config_path:="${OUTPUT}/coverage_probe_config.yaml" \
  -p output_path:="${OUTPUT}/coverage_runtime.json" \
  -p path_output_path:="${OUTPUT}/coverage_path.json" \
  -p trajectory_output_path:="${OUTPUT}/coverage_trajectory.csv" \
  >"${OUTPUT}/coverage_probe.log" 2>&1 & PROBE_PID=$!; PIDS+=("${PROBE_PID}")
deadline=$((SECONDS + TIMEOUT))
while kill -0 "${PROBE_PID}" 2>/dev/null; do
  (( SECONDS < deadline )) || { echo "FullCoverage probe timed out" >&2; exit 4; }
  kill -0 "${LAUNCH_PID}" 2>/dev/null || { echo "cleaning launch exited during FullCoverage" >&2; exit 4; }
  sleep 2
done
set +e; wait "${PROBE_PID}"; PROBE_STATUS=$?; set -e
unset 'PIDS[-1]'
(( PROBE_STATUS == 0 )) || { echo "FullCoverage probe failed" >&2; exit 4; }
[[ -f "${OUTPUT}/coverage_runtime.json" ]] || { echo "FullCoverage report missing" >&2; exit 4; }

python3 "${ROOT}/scripts/validate_formal_map_lifecycle_runtime.py" \
  --map-root "${MAP_ROOT}" --mapping-runtime "${MAPPING_RUNTIME}" \
  --cleaning-runtime "${OUTPUT}/cleaning_runtime.json" \
  --runtime-binding "${RUNTIME_BINDING}" \
  --output "${OUTPUT}/lifecycle_acceptance.json"
bash "${ROOT}/scripts/run_formal_same_map_baseline.sh" \
  --episode-manifest "${EPISODE_MANIFEST}" --map-root "${MAP_ROOT}" \
  --mapping-runtime "${MAPPING_RUNTIME}" \
  --cleaning-runtime "${OUTPUT}/cleaning_runtime.json" \
  --lifecycle-acceptance "${OUTPUT}/lifecycle_acceptance.json" \
  --coverage-runtime "${OUTPUT}/coverage_runtime.json" \
  --session "${SESSION}" --snapshot "${SNAPSHOT}" --output "${FORMAL_OUTPUT}"
echo "formal same-map FullCoverage baseline passed: ${FORMAL_OUTPUT}"
