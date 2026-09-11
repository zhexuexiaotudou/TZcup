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
localization_bag_pid=""
localization_bag_stop_rc=1
localization_bag_finalizer_rc=1
localization_bag_finalized=false
localization_bag_started_epoch_ns=""
localization_bag_dir="${output}/mapping_localization_diagnostic"
localization_bag_topics="${output}/mapping_localization_diagnostic.topics"
localization_bag_receipt="${output}/mapping_localization_diagnostic.json"
handoff_record="${output}/mapping_handoff_record.json"
finalize_localization_bag() {
  local stop_rc=0 finalizer_rc=1 bag_pid="" bag_pgid="" bag_state="" attempt optional_args=()
  if [[ "${localization_bag_finalized}" == true ]]; then
    return "${localization_bag_finalizer_rc}"
  fi
  if [[ -n "${localization_bag_pid}" ]]; then
    bag_pid="${localization_bag_pid}"
    bag_pgid="$(ps -o pgid= -p "${bag_pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    kill -INT "${bag_pid}" 2>/dev/null || true
    for attempt in {1..40}; do
      bag_state="$(ps -o stat= -p "${bag_pid}" 2>/dev/null | tr -d '[:space:]' || true)"
      [[ -z "${bag_state}" || "${bag_state}" == Z* ]] && break
      sleep 0.25
    done
    if [[ -n "${bag_state}" && "${bag_state}" != Z* ]]; then
      echo "localization diagnostic recorder PID ${bag_pid} remained alive after SIGINT" >&2
      stop_rc=1
    fi
    if (( stop_rc == 0 )); then
      wait "${bag_pid}" || stop_rc=1
      if kill -0 "${bag_pid}" 2>/dev/null; then
        echo "localization diagnostic recorder PID ${bag_pid} was not reaped" >&2
        stop_rc=1
      fi
      if [[ -n "${bag_pgid}" && "${bag_pgid}" == "${bag_pid}" ]] && \
          kill -0 -- "-${bag_pid}" 2>/dev/null; then
        echo "localization diagnostic recorder private PGID ${bag_pid} remained alive after reap" >&2
        stop_rc=1
      fi
      (( stop_rc == 0 )) && localization_bag_pid=""
    fi
  fi
  localization_bag_stop_rc="${stop_rc}"
  if [[ -f "${localization_bag_topics}" ]] && \
      grep -Fxq '/ground_truth/odom' "${localization_bag_topics}"; then
    optional_args+=(--optional-topic /ground_truth/odom)
  fi
  if python3 "${repo_root}/scripts/finalize_formal_first_map_localization_diagnostic.py" \
      --run-root "${output}" --bag-dir "${localization_bag_dir}" \
      --topic-manifest "${localization_bag_topics}" \
      --output "${localization_bag_receipt}" \
      --recorder-stop-rc "${localization_bag_stop_rc}" \
      --runtime-binding "${runtime_binding}" \
      --started-epoch-ns "${localization_bag_started_epoch_ns:-0}" \
      "${optional_args[@]}"; then
    finalizer_rc=0
  else
    finalizer_rc=$?
  fi
  localization_bag_finalized=true
  localization_bag_finalizer_rc="${finalizer_rc}"
  return "${finalizer_rc}"
}
cleanup() {
  local cleanup_status=0
  formal_runtime_cleanup_groups "${GZ_PARTITION}" \
    "${estop_pid}" "${power_pid}" "${collector_pid}" "${launch_pid}" || cleanup_status=1
  if [[ "${localization_bag_finalized}" != true ]]; then
    finalize_localization_bag || cleanup_status=1
  fi
  if [[ -n "${localization_bag_pid}" ]]; then
    formal_runtime_cleanup_groups "${GZ_PARTITION}" "${localization_bag_pid}" || cleanup_status=1
  fi
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
    --episode-manifest "${episode}/public/episode_manifest.json" \
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

# This MCAP is a read-only localization-drift diagnostic.  It neither
# publishes nor consumes simulator truth for product control.  The optional
# ground-truth topic is captured only when the formal graph already exposes it.
localization_topics=(
  /odom
  /odom/unfiltered
  /odometry/gps
  /gnss/fix
  /formal_mapping/lifecycle_status
  /ground_truth/odom
)
printf '%s\n' "${localization_topics[@]}" >"${localization_bag_topics}"
localization_bag_started_epoch_ns="$(date -u +%s%N)"
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 bag record --storage mcap --regex \
  '^(\/odom|\/odom\/unfiltered|\/odometry\/gps|\/gnss\/fix|\/formal_mapping\/lifecycle_status|\/ground_truth\/odom)$' \
  --output "${localization_bag_dir}" \
  >"${output}/mapping_localization_diagnostic.recorder.log" 2>&1 &
localization_bag_pid=$!
sleep 3
if ! kill -0 "${localization_bag_pid}" 2>/dev/null; then
  echo "localization diagnostic rosbag recorder exited; see ${output}/mapping_localization_diagnostic.recorder.log" >&2
  exit 6
fi

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
    if ! finalize_localization_bag; then
      echo "localization diagnostic rosbag failed; see ${localization_bag_receipt}" >&2
      exit 6
    fi
    python3 - \
      "${handoff_record}" \
      "${output}/map_lifecycle_manifest.json" \
      "${output}/mapping_runtime.json" \
      "${localization_bag_receipt}" \
      "$$" "${launch_pid}" "${collector_pid}" "${ROS_DOMAIN_ID}" "${GZ_PARTITION}" <<'PY'
import datetime
import hashlib
import json
import pathlib
import sys

output, manifest, runtime, diagnostic = map(pathlib.Path, sys.argv[1:5])
mapping_report = json.loads(runtime.read_text(encoding="utf-8"))
diagnostic_report = json.loads(diagnostic.read_text(encoding="utf-8"))
if mapping_report.get("passed") is not True or diagnostic_report.get("passed") is not True:
    raise SystemExit("mapping runtime or localization diagnostic did not pass")
value = {
    "schema_version": 2,
    "mapping_runner_completed": True,
    "mapping_runner_exit_code": 0,
    "mapping_process_groups_stopped": False,
    "mapping_runner_pid": int(sys.argv[5]),
    "mapping_launch_pid": int(sys.argv[6]),
    "mapping_collector_pid": int(sys.argv[7]),
    "mapping_ros_domain_id": int(sys.argv[8]),
    "mapping_gz_partition": sys.argv[9],
    "mapping_completion_wall_time": datetime.datetime.now(
        datetime.timezone.utc
    ).isoformat(),
    "map_lifecycle_manifest_sha256": hashlib.sha256(
        manifest.read_bytes()
    ).hexdigest(),
    "mapping_runtime_sha256": hashlib.sha256(runtime.read_bytes()).hexdigest(),
    "mapping_localization_diagnostic_sha256": hashlib.sha256(
        diagnostic.read_bytes()
    ).hexdigest(),
    "mapping_runtime_gate_binding_sha256": hashlib.sha256(
        (output.parent / "runtime_gate_binding.json").read_bytes()
    ).hexdigest(),
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
