#!/usr/bin/env bash
# Isolate Gazebo camera generation, Gazebo Transport, ros_gz_image and DDS.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"

runtime_setup=""
output_root=""
domain_id=225
topic="/formal_visual/front_left"
while (( $# )); do
  case "$1" in
    --runtime-setup) runtime_setup="$2"; shift 2 ;;
    --output-root) output_root="$2"; shift 2 ;;
    --domain-id) domain_id="$2"; shift 2 ;;
    --topic) topic="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ -f "${runtime_setup}" ]] || {
  echo "missing --runtime-setup: ${runtime_setup}" >&2
  exit 2
}
[[ "${output_root}" = /* && "${output_root}" != / ]] || {
  echo "--output-root must be an absolute path other than /" >&2
  exit 2
}
[[ "${domain_id}" =~ ^[0-9]+$ && "${domain_id}" -le 231 ]] || {
  echo "--domain-id must be in [0, 231]" >&2
  exit 2
}
[[ "${topic}" == /formal_visual/* ]] || {
  echo "--topic must be a formal visual topic" >&2
  exit 2
}
if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" == "1" ]]; then
  echo "single-topic diagnostic is standalone-only; refusing nested orchestration" >&2
  exit 2
fi
if ! mkdir -p "$(dirname -- "${output_root}")" || ! mkdir "${output_root}"; then
  echo "refusing stale diagnostic output root: ${output_root}" >&2
  exit 2
fi

source "${runtime_setup}"
world="$(ros2 pkg prefix --share sanitation_vehicle_description)/worlds/formal_vehicle_visual_acceptance.sdf"
[[ -f "${world}" ]] || {
  echo "installed visual world is missing: ${world}" >&2
  exit 2
}
single_world="${output_root}/single_topic_world.sdf"
python3 "${repo_root}/scripts/prepare_formal_visual_single_topic_world.py" \
  --source-world "${world}" \
  --output-world "${single_world}" \
  --topic "${topic}" \
  --report "${output_root}/single_topic_world_report.json"

partition="tzcup_visual_single_topic_${domain_id}_$$_${RANDOM}"
formal_runtime_configure "${domain_id}" 1
formal_runtime_register_evidence_paths "${output_root}"
formal_runtime_memory_preflight "${output_root}/windows_memory_preflight"

leader_pid=""
cleanup() {
  local result=0
  if [[ -n "${leader_pid}" ]]; then
    formal_runtime_cleanup_groups "${partition}" "${leader_pid}" || result=$?
    leader_pid=""
  fi
  return "${result}"
}
formal_runtime_install_traps cleanup

setsid bash -c '
set -eo pipefail
runtime_setup="$1"
world="$2"
output_root="$3"
domain_id="$4"
partition="$5"
topic="$6"
repo_root="$7"
source /opt/ros/jazzy/setup.bash
source "${runtime_setup}"
export ROS_DOMAIN_ID="${domain_id}"
export GZ_PARTITION="${partition}"
export ROS2CLI_DISABLE_DAEMON=1
gz_pid=""
bridge_pid=""
ros_echo_pid=""
cleanup_children() {
  [[ -z "${ros_echo_pid}" ]] || kill -INT "${ros_echo_pid}" 2>/dev/null || true
  [[ -z "${bridge_pid}" ]] || kill -INT "${bridge_pid}" 2>/dev/null || true
  [[ -z "${gz_pid}" ]] || kill -INT "${gz_pid}" 2>/dev/null || true
}
trap cleanup_children EXIT INT TERM

gz sim -r -s "${world}" --physics-engine gz-physics-dartsim-plugin \
  >"${output_root}/gazebo.log" 2>&1 &
gz_pid=$!
sleep 10
ros2 run ros_gz_image image_bridge "${topic}" --ros-args -p qos:=sensor_data \
  >"${output_root}/image_bridge.log" 2>&1 &
bridge_pid=$!
sleep 8
kill -0 "${gz_pid}"
kill -0 "${bridge_pid}"

closure_manifest="$(dirname -- "$(dirname -- "${runtime_setup}")")/final_runtime_closure_manifest.json"
python3 "${repo_root}/scripts/capture_formal_transport_process_maps.py" \
  --gazebo-pid "${gz_pid}" \
  --image-bridge-pid "${bridge_pid}" \
  --runtime-setup "${runtime_setup}" \
  --closure-manifest "${closure_manifest}" \
  --output "${output_root}/transport_process_maps.json" \
  >"${output_root}/transport_process_maps.stdout" \
  2>"${output_root}/transport_process_maps.stderr"

timeout -k 5 45 ros2 topic echo "${topic}" sensor_msgs/msg/Image \
  --once --field width >"${output_root}/ros_width.txt" \
  2>"${output_root}/ros_width.stderr" &
ros_echo_pid=$!
discovery_ready=0
for attempt in $(seq 1 20); do
  gz topic -i -t "${topic}" >"${output_root}/gz_topic_info.txt" 2>&1 || true
  ros2 topic info -v "${topic}" >"${output_root}/ros_topic_info.txt" 2>&1 || true
  if grep -q "gz.msgs.Image" "${output_root}/gz_topic_info.txt" \
    && grep -q "Subscriber" "${output_root}/gz_topic_info.txt" \
    && grep -q "Publisher count: 1" "${output_root}/ros_topic_info.txt" \
    && grep -q "Subscription count: 1" "${output_root}/ros_topic_info.txt"; then
    discovery_ready=1
    break
  fi
  sleep 0.5
done
if (( discovery_ready != 1 )); then
  exit 88
fi
bridge_executable="$(ros2 pkg prefix ros_gz_image)/lib/ros_gz_image/image_bridge"
readlink -f "${bridge_executable}" >"${output_root}/image_bridge_executable.txt"
ldd "${bridge_executable}" >"${output_root}/image_bridge_ldd.txt" 2>&1
set +e
timeout 45 gz topic -e -t "${topic}" -n 1 \
  2>"${output_root}/gz_sample.stderr" \
  | python3 "${repo_root}/scripts/extract_formal_gz_image_metadata.py" \
      >"${output_root}/gz_sample_metadata.json" \
      2>"${output_root}/gz_sample_parser.stderr"
sample_pipe_status=("${PIPESTATUS[@]}")
set -e
printf "%s\n" "${sample_pipe_status[*]}" >"${output_root}/gz_sample_pipe_status.txt"
if (( sample_pipe_status[0] != 0 || sample_pipe_status[1] != 0 )); then
  exit 87
fi
kill -0 "${gz_pid}"
kill -0 "${bridge_pid}"
grep -q "\"width\": 1600" "${output_root}/gz_sample_metadata.json"
grep -q "\"height\": 1000" "${output_root}/gz_sample_metadata.json"
grep -q "\"expected_uncompressed_data_bytes_from_step\": 4800000" \
  "${output_root}/gz_sample_metadata.json"
set +e
wait "${ros_echo_pid}"
echo_status=$?
set -e
ros_echo_pid=""
printf "%d\n" "${echo_status}" >"${output_root}/echo_status.txt"
if (( echo_status != 0 )); then
  exit "${echo_status}"
fi
grep -qx "1600" "${output_root}/ros_width.txt"
' bash "${runtime_setup}" "${single_world}" "${output_root}" "${domain_id}" \
  "${partition}" "${topic}" "${repo_root}" &
leader_pid=$!
formal_runtime_start_memory_watchdog "${leader_pid}" "${output_root}/memory_watchdog"

set +e
wait "${leader_pid}"
result=$?
set -e
formal_runtime_stop_memory_watchdog
leader_pid=""
formal_runtime_cleanup_partition "${partition}"
set +e
python3 "${repo_root}/scripts/finalize_formal_visual_single_topic_diagnostic.py" \
  --output-root "${output_root}" --child-result "${result}"
summary_result=$?
set -e
if formal_runtime_memory_watchdog_tripped; then
  exit "${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
fi
if (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT != 0 )); then
  exit "${FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT}"
fi
if (( summary_result != 0 )); then
  if (( result == 0 )); then
    exit 1
  fi
  exit "${result}"
fi
exit "${result}"
