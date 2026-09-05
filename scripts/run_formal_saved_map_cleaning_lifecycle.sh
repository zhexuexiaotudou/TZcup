#!/usr/bin/env bash
# Hard-restart the qualified first-task map into AMCL/Nav2 cleaning mode.
set -eo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
episode="${FORMAL_DYNAMIC_EPISODE_ROOT:-${repo_root}/.work/formal_campus_episode_runtime}"
map_root="${FORMAL_DYNAMIC_SAVED_MAP_ROOT:-${repo_root}/.work/formal_first_map_acceptance}"
output="${FORMAL_MAP_LIFECYCLE_OUTPUT:-${repo_root}/artifacts/formal_map_lifecycle_acceptance.json}"
domain="${ROS_DOMAIN_ID:-60}"
runtime="${FORMAL_MAP_CLEANING_RUNTIME_ROOT:-${map_root}/saved_map_cleaning_runtime}"
formal_runtime_register_evidence_paths "${output}" "${runtime}"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen colcon workspace}"
runtime_install="${runtime_ws}/install"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
runtime_binding="${runtime}/runtime_gate_binding.json"
cleaning_planner="${FORMAL_CLEANING_PLANNER:-full_coverage}"
operation_speed_profile="${FORMAL_OPERATION_SPEED_PROFILE:-dry_cleaning_competition_candidate}"
perception_artifact_root="${FORMAL_PERCEPTION_ARTIFACT_ROOT:-}"
policy_checkpoint="${FORMAL_POLICY_CHECKPOINT:-}"
maximum_task_distance_m="${FORMAL_FULL_COVERAGE_DISTANCE_M:-0.0}"
cleaning_timeout_sec="${FORMAL_CLEANING_TIMEOUT_S:-86400}"
if [[ ! "${cleaning_timeout_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "FORMAL_CLEANING_TIMEOUT_S must be a positive integer" >&2
  exit 2
fi
formal_runtime_configure "${domain}"
if [[ -e "${runtime}" || -e "${output}" ]]; then
  echo "Refusing stale saved-map cleaning evidence; use fresh runtime and output paths" >&2
  exit 2
fi
mkdir -p "$(dirname "${runtime}")" "$(dirname "${output}")"
mkdir "${runtime}"

if [[ "${cleaning_planner}" != "full_coverage" && "${cleaning_planner}" != "rl_dirt_priority" ]]; then
  echo "FORMAL_CLEANING_PLANNER must be full_coverage or rl_dirt_priority" >&2
  exit 2
fi
if [[ "${operation_speed_profile}" != "dry_cleaning_competition_candidate" ]]; then
  echo "FORMAL_OPERATION_SPEED_PROFILE must be dry_cleaning_competition_candidate for dry cleaning" >&2
  exit 2
fi
if [[ "${cleaning_planner}" == "rl_dirt_priority" ]]; then
  if [[ ! -d "${perception_artifact_root}" ]]; then
    echo "RL cleaning requires FORMAL_PERCEPTION_ARTIFACT_ROOT" >&2
    exit 2
  fi
  if [[ ! -f "${policy_checkpoint}" ]]; then
    echo "RL cleaning requires FORMAL_POLICY_CHECKPOINT" >&2
    exit 2
  fi
  python3 - "${maximum_task_distance_m}" <<'PY'
import math
import sys

value = float(sys.argv[1])
if not math.isfinite(value) or value <= 0.0:
    raise SystemExit("RL cleaning requires positive FORMAL_FULL_COVERAGE_DISTANCE_M")
PY
fi

source /opt/ros/jazzy/setup.bash
formal_source_bound_preflight \
  "${repo_root}" "${runtime_ws}" "${runtime_closure_manifest}" \
  "${session}" "${snapshot}" "${runtime_binding}"
source "${runtime_install}/setup.bash"
formal_source_bound_verify_overlay "${runtime_install}"
export TZCUP_REPOSITORY_ROOT="${repo_root}"
export ROS_DOMAIN_ID="${domain}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_saved_map_cleaning_${domain}_$$}"

# Reject a missing, low-coverage, wrong-map or tampered map before Gazebo,
# AMCL or coverage is allowed to start. Bind the restart to the exact mapping
# process record and byte-identical mapping evidence.
mapping_handoff_record="${map_root}/mapping_handoff_record.json"
python3 - \
  "${episode}/public/episode_manifest.json" \
  "${map_root}" \
  "${mapping_handoff_record}" <<'PY'
import hashlib
import json
import pathlib
import sys
from sanitation_formal_campus_integration.map_lifecycle_core import (
    load_campus_map_contract,
    validate_saved_map_artifact,
)

contract = load_campus_map_contract(pathlib.Path(sys.argv[1]))
root = pathlib.Path(sys.argv[2])
validate_saved_map_artifact(root, contract)
handoff = json.loads(pathlib.Path(sys.argv[3]).read_text(encoding="utf-8"))
if (
    handoff.get("schema_version") != 2
    or handoff.get("mapping_runner_completed") is not True
    or handoff.get("mapping_runner_exit_code") != 0
    or handoff.get("mapping_process_groups_stopped") is not True
    or handoff.get("map_lifecycle_manifest_sha256")
    != hashlib.sha256((root / "map_lifecycle_manifest.json").read_bytes()).hexdigest()
    or handoff.get("mapping_runtime_sha256")
    != hashlib.sha256((root / "mapping_runtime.json").read_bytes()).hexdigest()
):
    raise SystemExit("mapping handoff record or hashes failed closed")
PY

# The mapping launch must be gone, not merely lifecycle-inactive.  Use the
# exact map artifact argument so unrelated ROS processes are outside scope.
mapping_process_count="$({
  ps -eo args \
    | grep -F "mission_mode:=mapping" \
    | grep -F "map_artifact_dir:=${map_root}" \
    | grep -v grep || true
} | wc -l)"
mapping_process_count="${mapping_process_count//[[:space:]]/}"
if [[ "${mapping_process_count}" != "0" ]]; then
  echo "mapping launch still present; refusing saved-map restart" >&2
  exit 3
fi
mapping_pid_alive_count="$(python3 - "${mapping_handoff_record}" <<'PY'
import json
import os
import pathlib
import sys

handoff = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
alive = 0
for key in ("mapping_runner_pid", "mapping_launch_pid", "mapping_collector_pid"):
    pid = int(handoff[key])
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        continue
    except PermissionError:
        alive += 1
    else:
        alive += 1
print(alive)
PY
)"
if [[ "${mapping_pid_alive_count}" != "0" ]]; then
  echo "recorded mapping PID still alive; refusing saved-map restart" >&2
  exit 3
fi

runtime_world="${runtime}/cleaning_world.with_contact_system.sdf"
runtime_world_manifest="${runtime}/cleaning_world_manifest.json"
python3 "${repo_root}/scripts/prepare_formal_dynamic_runtime_world.py" \
  --source "${episode}/public/world.sdf" \
  --output "${runtime_world}" \
  --manifest "${runtime_world_manifest}"

if [[ "${cleaning_planner}" == "rl_dirt_priority" ]]; then
  launch_command=(
    ros2 launch sanitation_product_demo_integration product_demo.launch.py
    gui:=false
    world:="${runtime_world}"
    episode_manifest:="${episode}/public/episode_manifest.json"
    saved_map_artifact_dir:="${map_root}"
    pedestrian_schedule:="${episode}/environment/pedestrian_schedule.json"
    start_pedestrians:=true
    perception_artifact_root:="${perception_artifact_root}"
    policy_checkpoint:="${policy_checkpoint}"
    maximum_task_distance_m:="${maximum_task_distance_m}"
    operation_speed_profile:="${operation_speed_profile}"
  )
else
  launch_command=(
    ros2 launch sanitation_formal_campus_integration
    formal_campus_map_lifecycle.launch.py
    mission_mode:=cleaning
    cleaning_planner:=full_coverage
    gui:=false
    world:="${runtime_world}"
    episode_manifest:="${episode}/public/episode_manifest.json"
    map_artifact_dir:="${map_root}"
    pedestrian_schedule:="${episode}/environment/pedestrian_schedule.json"
    start_pedestrians:=true
    start_coverage:=true
    coverage_evidence_dir:="${runtime}"
    operation_speed_profile:="${operation_speed_profile}"
  )
fi
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" "${launch_command[@]}" >"${runtime}/cleaning.launch.log" 2>&1 &
launch_pid=$!
restart_record="${runtime}/hard_restart_record.json"
python3 - \
  "${restart_record}" \
  "${mapping_handoff_record}" \
  "${map_root}/map_lifecycle_manifest.json" \
  "${map_root}/mapping_runtime.json" \
  "${mapping_process_count}" "${mapping_pid_alive_count}" "$$" "${launch_pid}" "${ROS_DOMAIN_ID}" "${GZ_PARTITION}" <<'PY'
import datetime
import hashlib
import json
import pathlib
import sys

output, handoff_path, manifest, runtime = map(pathlib.Path, sys.argv[1:5])
handoff = json.loads(handoff_path.read_text(encoding="utf-8"))
value = {
    "schema_version": 2,
    "mapping_stopped_before_cleaning": int(sys.argv[5]) == 0,
    "mapping_process_count_before_cleaning": int(sys.argv[5]),
    "mapping_pid_alive_count_before_cleaning": int(sys.argv[6]),
    "mapping_runner_pid": handoff["mapping_runner_pid"],
    "mapping_runner_exit_code": handoff["mapping_runner_exit_code"],
    "mapping_launch_pid": handoff["mapping_launch_pid"],
    "mapping_collector_pid": handoff["mapping_collector_pid"],
    "mapping_completion_wall_time": handoff["mapping_completion_wall_time"],
    "mapping_cleanup_wall_time": handoff["mapping_cleanup_wall_time"],
    "cleaning_runner_pid": int(sys.argv[7]),
    "cleaning_launch_pid": int(sys.argv[8]),
    "cleaning_ros_domain_id": int(sys.argv[9]),
    "cleaning_gz_partition": sys.argv[10],
    "cleaning_start_wall_time": datetime.datetime.now(
        datetime.timezone.utc
    ).isoformat(),
    "restart_type": "separate_process_hard_restart",
    "mapping_handoff_record_sha256": hashlib.sha256(
        handoff_path.read_bytes()
    ).hexdigest(),
    "map_lifecycle_manifest_sha256": hashlib.sha256(
        manifest.read_bytes()
    ).hexdigest(),
    "mapping_runtime_sha256": hashlib.sha256(runtime.read_bytes()).hexdigest(),
}
output.write_text(
    json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
PY
collector_pid=""
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${collector_pid}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

sleep 2
cleaning_runtime="${runtime}/cleaning_runtime.json"
coverage_execution="${runtime}/coverage_execution.json"
set +e
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${repo_root}/scripts/collect_formal_map_lifecycle_runtime.py" \
  --mode cleaning --map-root "${map_root}" --timeout "${cleaning_timeout_sec}" \
  --restart-record "${restart_record}" \
  --mission-geometry "${map_root}/mission_geometry.yaml" \
  --coverage-report "${coverage_execution}" \
  --output "${cleaning_runtime}" \
  >"${runtime}/cleaning_runtime.collector.log" 2>&1 &
collector_pid=$!
collector_status=0
while kill -0 "${collector_pid}" 2>/dev/null; do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${collector_pid}" 2>/dev/null || true
    wait "${collector_pid}" 2>/dev/null
    collector_status=3
    break
  fi
  sleep 1
done
if (( collector_status == 0 )); then
  wait "${collector_pid}"
  collector_status=$?
fi
set -e

set +e
python3 "${repo_root}/scripts/validate_formal_map_lifecycle_runtime.py" \
  --map-root "${map_root}" \
  --mapping-runtime "${map_root}/mapping_runtime.json" \
  --cleaning-runtime "${cleaning_runtime}" \
  --runtime-binding "${runtime_binding}" \
  --output "${output}"
validation_status=$?
set -e
if (( collector_status != 0 || validation_status != 0 )); then
  echo "saved-map cleaning lifecycle failed closed: ${output}" >&2
  exit 2
fi
echo "formal map lifecycle passed: ${output}"
