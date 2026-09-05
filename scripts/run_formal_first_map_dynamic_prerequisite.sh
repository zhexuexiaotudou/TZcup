#!/usr/bin/env bash
# Produce the sealed saved-map prerequisite; never substitutes a world-derived map.
set -eo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
episode="${FORMAL_DYNAMIC_EPISODE_ROOT:-${repo_root}/.work/formal_campus_episode_runtime}"
output="${FORMAL_DYNAMIC_SAVED_MAP_ROOT:-${repo_root}/.work/formal_first_map_acceptance}"
formal_runtime_register_evidence_paths "${output}"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen colcon workspace}"
runtime_install="${runtime_ws}/install"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
runtime_binding="${output}/runtime_gate_binding.json"
domain="${ROS_DOMAIN_ID:-99}"
mapping_timeout_sec="${FORMAL_MAPPING_TIMEOUT_S:-21600}"
mapping_poll_period_sec=15
if [[ ! "${mapping_timeout_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "formal mapping timeout must be a positive integer" >&2
  exit 2
fi
mapping_polls="${FORMAL_MAPPING_POLLS:-}"
if [[ -z "${mapping_polls}" ]]; then
  mapping_polls=$((
    (mapping_timeout_sec + mapping_poll_period_sec - 1) / mapping_poll_period_sec
  ))
fi
if [[ ! "${mapping_polls}" =~ ^[1-9][0-9]*$ ]]; then
  echo "formal mapping timeout and poll count must be positive integers" >&2
  exit 2
fi
if [[ -e "${output}" ]]; then
  echo "refusing to reuse a saved-map run root: ${output}" >&2
  exit 2
fi
mkdir -p "$(dirname "${output}")"
mkdir "${output}"

source /opt/ros/jazzy/setup.bash
formal_source_bound_preflight \
  "${repo_root}" "${runtime_ws}" "${runtime_closure_manifest}" \
  "${session}" "${snapshot}" "${runtime_binding}"
source "${runtime_install}/setup.bash"
formal_source_bound_verify_overlay "${runtime_install}"
export TZCUP_REPOSITORY_ROOT="${repo_root}"
export ROS_DOMAIN_ID="${domain}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_mapping_${domain}_$$}"

mapping_world="${output}/mapping_without_parked_pedestrians.sdf"
python3 "${repo_root}/scripts/prepare_formal_mapping_world.py" \
  --source "${episode}/public/world.sdf" \
  --episode-manifest "${episode}/public/episode_manifest.json" \
  --output "${mapping_world}" \
  --report "${output}/mapping_world_preparation.json"

launch_pid=""
estop_pid=""
power_pid=""
collector_pid=""
handoff_record="${output}/mapping_handoff_record.json"
cleanup() {
  local cleanup_status=0
  formal_runtime_cleanup_groups "${GZ_PARTITION}" \
    "${estop_pid}" "${power_pid}" "${collector_pid}" "${launch_pid}" || cleanup_status=1
  if [[ -f "${handoff_record}" ]]; then
    python3 - "${handoff_record}" "${cleanup_status}" <<'PY'
import datetime
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["mapping_process_groups_stopped"] = int(sys.argv[2]) == 0
value["mapping_cleanup_wall_time"] = datetime.datetime.now(
    datetime.timezone.utc
).isoformat()
temporary = path.with_suffix(path.suffix + ".tmp")
temporary.write_text(
    json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
temporary.replace(path)
PY
  fi
  # Mapping alone is not the contract gate. Keep its quantified boundary in
  # this fresh map root; only the hard-restart cleaning runner may publish the
  # formal lifecycle artifact.
  python3 "${repo_root}/scripts/validate_formal_map_lifecycle_runtime.py" \
    --map-root "${output}" \
    --mapping-runtime "${output}/mapping_runtime.json" \
    --cleaning-runtime "${output}/saved_map_cleaning_runtime/cleaning_runtime.json" \
    --output "${output}/mapping_only_lifecycle_boundary.json" >/dev/null 2>&1 || true
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration \
  formal_campus_map_lifecycle.launch.py \
  mission_mode:=mapping gui:=false \
  world:="${mapping_world}" \
  episode_manifest:="${episode}/public/episode_manifest.json" \
  map_artifact_dir:="${output}" \
  pedestrian_schedule:="${episode}/environment/pedestrian_schedule.json" \
  start_pedestrians:=false start_coverage:=false operation_speed_profile:=mapping_safe \
  >"${output}/mapping.launch.log" 2>&1 &
launch_pid=$!
sleep 30
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 topic pub /formal_vehicle/simulation/command/emergency_stop \
  std_msgs/msg/Bool "{data: false}" -r 10 >"${output}/estop.log" 2>&1 &
estop_pid=$!
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 topic pub /formal_vehicle/simulation/command/main_power \
  std_msgs/msg/Bool "{data: true}" -r 10 >"${output}/power.log" 2>&1 &
power_pid=$!
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${repo_root}/scripts/collect_formal_map_lifecycle_runtime.py" \
  --mode mapping --map-root "${output}" --timeout "${mapping_timeout_sec}" \
  --output "${output}/mapping_runtime.json" \
  >"${output}/mapping_runtime.collector.log" 2>&1 &
collector_pid=$!

for index in $(seq 1 "${mapping_polls}"); do
  if [[ -f "${output}/map_lifecycle_manifest.json" ]]; then
    for _ in $(seq 1 30); do
      [[ -f "${output}/mapping_runtime.json" ]] && break
      sleep 1
    done
    if ! python3 - "${output}/mapping_runtime.json" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
try:
    report = json.loads(path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError):
    raise SystemExit(1)
raise SystemExit(0 if report.get("passed") is True else 1)
PY
    then
      echo "saved map was sealed but live mapping runtime evidence failed" >&2
      exit 5
    fi
    python3 - \
      "${handoff_record}" \
      "${output}/map_lifecycle_manifest.json" \
      "${output}/mapping_runtime.json" \
      "$$" "${launch_pid}" "${collector_pid}" "${ROS_DOMAIN_ID}" "${GZ_PARTITION}" <<'PY'
import datetime
import hashlib
import json
import pathlib
import sys

output, manifest, runtime = map(pathlib.Path, sys.argv[1:4])
mapping_report = json.loads(runtime.read_text(encoding="utf-8"))
if mapping_report.get("passed") is not True:
    raise SystemExit("mapping runtime did not pass")
value = {
    "schema_version": 2,
    "mapping_runner_completed": True,
    "mapping_runner_exit_code": 0,
    "mapping_process_groups_stopped": False,
    "mapping_runner_pid": int(sys.argv[4]),
    "mapping_launch_pid": int(sys.argv[5]),
    "mapping_collector_pid": int(sys.argv[6]),
    "mapping_ros_domain_id": int(sys.argv[7]),
    "mapping_gz_partition": sys.argv[8],
    "mapping_completion_wall_time": datetime.datetime.now(
        datetime.timezone.utc
    ).isoformat(),
    "map_lifecycle_manifest_sha256": hashlib.sha256(
        manifest.read_bytes()
    ).hexdigest(),
    "mapping_runtime_sha256": hashlib.sha256(runtime.read_bytes()).hexdigest(),
}
output.write_text(
    json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
PY
    echo "formal saved map finalized: ${output}"
    exit 0
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "formal mapping launch exited; see ${output}/mapping.launch.log" >&2
    exit 3
  fi
  if (( index % 4 == 0 )); then
    echo "mapping checkpoint ${index}"
    ros2 topic echo /formal_mapping/lifecycle_status std_msgs/msg/String \
      --once --timeout 5 2>/dev/null || true
    ros2 topic echo /formal_mapping/explorer_status std_msgs/msg/String \
      --once --timeout 5 2>/dev/null || true
  fi
  sleep "${mapping_poll_period_sec}"
done
echo "formal 200x100 mapping did not reach the sealed 95% gate in the bounded run" >&2
exit 4
