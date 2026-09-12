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
early_recording_pid=""
localization_bag_stop_rc=1
localization_bag_finalizer_rc=1
localization_bag_finalized=false
localization_bag_started_epoch_ns=""
localization_bag_dir="${output}/mapping_localization_diagnostic"
localization_bag_topics="${output}/mapping_localization_diagnostic.topics"
localization_bag_receipt="${output}/mapping_localization_diagnostic.json"
early_recording_closed_receipt="${output}/formal_recording_closed.json"
formal_recording_arm_receipt="${output}/formal_recording_arm.json"
formal_recording_invalid_receipt="${output}/formal_recording_invalid.json"
recording_helper="${repo_root}/scripts/capture_formal_first_map_early_recording_audit.py"
recording_ready_timeout_sec="${FORMAL_RECORDING_READY_TIMEOUT_S:-120}"
if [[ ! "${recording_ready_timeout_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "formal recording ready timeout must be a positive integer" >&2
  exit 2
fi
export FORMAL_ACCEPTANCE_SESSION="${session}"
export FORMAL_RECORDING_RUN_TOKEN="${FORMAL_RECORDING_RUN_TOKEN:-$(cat /proc/sys/kernel/random/uuid)}"
if [[ ! "${FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS:-}" =~ ^[1-9][0-9]*$ ]]; then
  export FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS="$((
    $(python3 -c 'import time; print(time.monotonic_ns())') + mapping_timeout_sec * 1000000000
  ))"
fi
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
wait_for_recording_writer_open() {
  local receipt="$1" role="$2" bag_dir="$3"
  python3 - "${receipt}" "${role}" "${bag_dir}" \
    "${FORMAL_ACCEPTANCE_SESSION}" "${FORMAL_RECORDING_RUN_TOKEN}" \
    "${FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS}" "${recording_ready_timeout_sec}" <<'PY'
import hashlib
import json
import os
import pathlib
import sys
import time

receipt, role, bag_dir, session, token, shared_deadline, wait_seconds = sys.argv[1:]
deadline = min(int(shared_deadline), time.monotonic_ns() + int(wait_seconds) * 1_000_000_000)
session_path = pathlib.Path(session)
session_hash = hashlib.sha256(session_path.read_bytes()).hexdigest()
while time.monotonic_ns() < deadline:
    path = pathlib.Path(receipt)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        owner = value["owner_identity"]
        pid = int(owner["pid"])
        stat = pathlib.Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = stat[stat.rfind(")") + 2:].split()
        valid = (
            value.get("schema_version") == 1
            and value.get("status") == "FORMAL_RECORDING_WRITER_OPEN"
            and value.get("opened") is True
            and value.get("role") == role
            and value.get("bag_dir") == bag_dir
            and value.get("formal_acceptance_session") == session
            and value.get("formal_acceptance_session_sha256") == session_hash
            and value.get("run_token") == token
            and owner.get("boot_id") == pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip()
            and int(owner["pgid"]) == os.getpgid(pid)
            and int(owner["process_starttime_ticks"]) == int(fields[19])
            and owner.get("owner_token") == f"{token}:{role}"
        )
        if valid:
            raise SystemExit(0)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        pass
    time.sleep(0.1)
raise SystemExit(1)
PY
}
wait_for_formal_recording_arm() {
  python3 - "${formal_recording_arm_receipt}" "${formal_recording_invalid_receipt}" \
    "${output}/formal_localization_recording_invalid.json" \
    "${output}/formal_recording_ready.json" \
    "${output}/formal_localization_ready.json" \
    "${FORMAL_ACCEPTANCE_SESSION}" "${FORMAL_RECORDING_RUN_TOKEN}" \
    "${FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS}" "${recording_ready_timeout_sec}" <<'PY'
import hashlib
import json
import os
import pathlib
import sys
import time

arm_path, invalid_path, local_invalid_path, early_ready_path, local_ready_path, session, token, shared_deadline, wait_seconds = sys.argv[1:]
deadline = min(int(shared_deadline), time.monotonic_ns() + int(wait_seconds) * 1_000_000_000)
session_hash = hashlib.sha256(pathlib.Path(session).read_bytes()).hexdigest()
required = ("/odom", "/odom/unfiltered", "/odometry/gps", "/gnss/fix", "/formal_mapping/lifecycle_status")
def live_owner(value, role, bag_dir):
    try:
        owner = value["owner_identity"]
        pid = int(owner["pid"])
        stat = pathlib.Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = stat[stat.rfind(")") + 2:].split()
        return (
            value.get("role") == role and value.get("bag_dir") == bag_dir
            and value.get("formal_acceptance_session") == session
            and value.get("formal_acceptance_session_sha256") == session_hash
            and value.get("run_token") == token and owner.get("owner_token") == f"{token}:{role}"
            and owner.get("boot_id") == pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip()
            and int(owner["pgid"]) == os.getpgid(pid)
            and int(owner["process_starttime_ticks"]) == int(fields[19])
        )
    except (OSError, KeyError, TypeError, ValueError):
        return False
def current_ready(path, role, bag_dir, now):
    value = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    if not value.get("ready") or not live_owner(value, role, bag_dir): return None
    age = now - value.get("ready_monotonic_ns", -1)
    clock_age = now - value.get("last_clock_advance_monotonic_ns", -1)
    if not (0 <= age <= 2_000_000_000 and 0 <= clock_age <= 2_000_000_000): return None
    if value.get("clock_ns", 0) <= 0 or value.get("clock_advances", 0) < 2 or value.get("clock_rollback") is not False: return None
    counts = value.get("writer_message_counts", {}); last = value.get("last_message_monotonic_ns_by_topic", {})
    if not all(isinstance(counts.get(topic), int) and counts[topic] > 0 and isinstance(last.get(topic), int) and 0 <= now-last[topic] <= 2_000_000_000 for topic in required): return None
    return value
while time.monotonic_ns() < deadline:
    if pathlib.Path(invalid_path).exists() or pathlib.Path(local_invalid_path).exists():
        raise SystemExit(1)
    try:
        arm = json.loads(pathlib.Path(arm_path).read_text(encoding="utf-8"))
        now = time.monotonic_ns()
        early = current_ready(early_ready_path, "early", str(pathlib.Path(arm_path).parent / "early_recording_audit"), now)
        local = current_ready(local_ready_path, "localization", str(pathlib.Path(arm_path).parent / "mapping_localization_diagnostic"), now)
        valid = (
            arm.get("schema_version") == 1
            and arm.get("status") == "FORMAL_RECORDING_ARMED"
            and arm.get("armed") is True
            and arm.get("formal_acceptance_session") == session
            and arm.get("formal_acceptance_session_sha256") == session_hash
            and arm.get("run_token") == token
            and isinstance(arm.get("armed_epoch_ns"), int)
            and isinstance(arm.get("armed_monotonic_ns"), int)
            and early is not None and local is not None
        )
        if valid:
            raise SystemExit(0)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        pass
    time.sleep(0.1)
raise SystemExit(1)
PY
}
finalize_early_recording_audit() {
  local stop_rc=0 state="" bag_pgid="" attempt
  if [[ -n "${early_recording_pid}" ]]; then
    bag_pgid="$(ps -o pgid= -p "${early_recording_pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    kill -INT "${early_recording_pid}" 2>/dev/null || true
    for attempt in {1..40}; do
      state="$(ps -o stat= -p "${early_recording_pid}" 2>/dev/null | tr -d '[:space:]' || true)"
      [[ -z "${state}" || "${state}" == Z* ]] && break
      sleep 0.25
    done
    if [[ -n "${state}" && "${state}" != Z* ]]; then
      echo "early recording audit PID ${early_recording_pid} remained alive after SIGINT" >&2
      stop_rc=1
    elif ! wait "${early_recording_pid}"; then
      stop_rc=1
    fi
    if (( stop_rc == 0 )) && [[ -n "${bag_pgid}" && "${bag_pgid}" == "${early_recording_pid}" ]] && \
        kill -0 -- "-${early_recording_pid}" 2>/dev/null; then
      echo "early recording audit private PGID ${early_recording_pid} remained alive after reap" >&2
      stop_rc=1
    fi
    (( stop_rc == 0 )) && early_recording_pid=""
  fi
  if ! python3 - "${early_recording_closed_receipt}" "${formal_recording_invalid_receipt}" <<'PY'
import json
import pathlib
import sys

closed, invalid = map(pathlib.Path, sys.argv[1:])
if invalid.exists():
    raise SystemExit(1)
value = json.loads(closed.read_text(encoding="utf-8"))
raise SystemExit(0 if value.get("status") == "FORMAL_RECORDING_CLOSED"
                 and value.get("normal_close") is True
                 and isinstance(value.get("bag_validation"), dict)
                 and "validation_error" not in value["bag_validation"] else 1)
PY
  then
    stop_rc=1
  fi
  return "${stop_rc}"
}
cleanup() {
  local cleanup_status=0
  # Stop and reap the recorder while its publishers are still alive.  This
  # lets rosbag2 write metadata and the closing MCAP footer before any launch
  # or Gazebo process is interrupted.  A process-group broadcast here would
  # race the recorder's SIGINT handler and can leave a truncated active copy.
  if ! finalize_early_recording_audit; then
    cleanup_status=1
  fi
  if [[ -n "${early_recording_pid}" ]]; then
    formal_runtime_cleanup_groups "${GZ_PARTITION}" "${early_recording_pid}" || cleanup_status=1
  fi
  if [[ "${localization_bag_finalized}" != true ]]; then
    finalize_localization_bag || cleanup_status=1
  fi
  if [[ -n "${localization_bag_pid}" ]]; then
    formal_runtime_cleanup_groups "${GZ_PARTITION}" "${localization_bag_pid}" || cleanup_status=1
  fi
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
    --episode-manifest "${episode}/public/episode_manifest.json" \
    --output "${output}/mapping_only_lifecycle_boundary.json" >/dev/null 2>&1 || true
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup

# Both bags must have real rosbag2 writers before launch: process liveness or
# subscriber discovery cannot establish that an early fault was recorded.
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
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${recording_helper}" \
  --output "${output}" --role localization --timeout "${recording_ready_timeout_sec}" \
  >"${output}/mapping_localization_diagnostic.recorder.log" 2>&1 &
localization_bag_pid=$!
if ! wait_for_recording_writer_open \
    "${output}/formal_localization_writer_open.json" localization "${localization_bag_dir}"; then
  echo "localization rosbag2 writer did not open before mapping launch" >&2
  exit 6
fi
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${recording_helper}" \
  --output "${output}" --role early --timeout "${recording_ready_timeout_sec}" \
  >"${output}/formal_recording_audit.log" 2>&1 &
early_recording_pid=$!
if ! wait_for_recording_writer_open \
    "${output}/formal_recording_writer_open.json" early "${output}/early_recording_audit"; then
  echo "early rosbag2 writer did not open before mapping launch" >&2
  exit 6
fi

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
if ! wait_for_formal_recording_arm; then
  echo "recording writers did not establish a valid formal arm" >&2
  exit 6
fi

for index in $(seq 1 "${mapping_polls}"); do
  if [[ -e "${formal_recording_invalid_receipt}" || \
        -e "${output}/formal_localization_recording_invalid.json" ]] || \
      ! kill -0 "${early_recording_pid}" 2>/dev/null || \
      ! kill -0 "${localization_bag_pid}" 2>/dev/null; then
    echo "recording writer invalidated or exited before mapping completion" >&2
    exit 6
  fi
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
    if ! finalize_early_recording_audit; then
      echo "early recording audit failed to close" >&2
      exit 6
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
