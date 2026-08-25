#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${INTEGRATED_ACCEPTANCE_RUNTIME_WS:?set INTEGRATED_ACCEPTANCE_RUNTIME_WS to the fresh colcon workspace}"
build_manifest="${INTEGRATED_ACCEPTANCE_BUILD_MANIFEST:?set INTEGRATED_ACCEPTANCE_BUILD_MANIFEST to a post-build snapshot}"
evidence_root="${INTEGRATED_ACCEPTANCE_OUTPUT_DIR:-${repo_root}/artifacts/integrated_functional_acceptance}"
domain_base="${INTEGRATED_ACCEPTANCE_DOMAIN_BASE:-180}"
material="${INTEGRATED_ACCEPTANCE_MATERIAL:-PET}"
aggregator="${repo_root}/scripts/aggregate_integrated_functional_acceptance.py"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing fresh runtime workspace: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
if [[ ! -f "${build_manifest}" ]]; then
  echo "Missing required source-bound build manifest: ${build_manifest}" >&2
  exit 2
fi
if ! [[ "${domain_base}" =~ ^[0-9]+$ ]] || (( domain_base < 0 || domain_base > 229 )); then
  echo "INTEGRATED_ACCEPTANCE_DOMAIN_BASE must leave four valid ROS domains (0..232)" >&2
  exit 2
fi
case "${material}" in
  paperboard|PP|PET|aluminum) ;;
  *) echo "Unsupported cube material: ${material}" >&2; exit 2 ;;
esac

source "${runtime_ws}/install/setup.bash"

run_id="${INTEGRATED_ACCEPTANCE_RUN_ID:-$(date -u +%Y%m%dT%H%M%S)_$$_${RANDOM}}"
run_dir="${evidence_root}/${run_id}"
context="${run_dir}/run_context.json"
manifest="${run_dir}/integrated_acceptance_manifest.json"
manifest_tmp="${manifest}.pending.$$"
mkdir -p "${evidence_root}"
if ! mkdir "${run_dir}" 2>/dev/null; then
  echo "Refusing to reuse integrated acceptance run directory: ${run_dir}" >&2
  exit 2
fi

python3 "${aggregator}" preflight \
  --repo-root "${repo_root}" --runtime-ws "${runtime_ws}" \
  --build-manifest "${build_manifest}"

run_started_ns="$(date +%s%N)"
python3 "${aggregator}" init-run \
  --repo-root "${repo_root}" --runtime-ws "${runtime_ws}" \
  --build-manifest "${build_manifest}" --context "${context}" \
  --run-id "${run_id}" --started-epoch-ns "${run_started_ns}"

active_group_pid=""
active_partition=""
cleanup_active_group() {
  if [[ -n "${active_group_pid}" ]] && kill -0 "${active_group_pid}" 2>/dev/null; then
    kill -INT -- "-${active_group_pid}" 2>/dev/null || true
    sleep 1
    kill -TERM -- "-${active_group_pid}" 2>/dev/null || true
    wait "${active_group_pid}" 2>/dev/null || true
  fi
  active_group_pid=""
}

cleanup_partition() {
  local partition="$1"
  python3 - "${partition}" <<'PY'
import os
import signal
import sys
import time

needle = ("GZ_PARTITION=" + sys.argv[1]).encode()

def matching():
    found = []
    for raw in os.listdir("/proc"):
        if not raw.isdigit() or int(raw) == os.getpid():
            continue
        try:
            env = open(f"/proc/{raw}/environ", "rb").read().split(b"\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if needle in env:
            found.append(int(raw))
    return found

for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
    pids = matching()
    if not pids:
        break
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + (2.0 if sig != signal.SIGKILL else 1.0)
    while time.monotonic() < deadline and matching():
        time.sleep(0.1)
print(len(matching()))
PY
}

cleanup_active_scenario() {
  cleanup_active_group
  if [[ -n "${active_partition}" ]]; then
    cleanup_partition "${active_partition}" >/dev/null 2>&1 || true
    active_partition=""
  fi
  if [[ -n "${manifest_tmp}" ]]; then
    rm -f -- "${manifest_tmp}"
  fi
}

handle_signal() {
  local status="$1"
  cleanup_active_scenario
  exit "${status}"
}

trap cleanup_active_scenario EXIT
trap 'handle_signal 130' INT
trap 'handle_signal 143' TERM

record_scenario() {
  local name="$1" started_ns="$2" finished_ns="$3" exit_code="$4"
  local domain="$5" partition="$6" result="$7" launch_log="$8" runner_log="$9"
  local remaining="${10}"
  python3 "${aggregator}" record-scenario \
    --context "${context}" --name "${name}" \
    --started-epoch-ns "${started_ns}" --finished-epoch-ns "${finished_ns}" \
    --exit-code "${exit_code}" --ros-domain-id "${domain}" \
    --gz-partition "${partition}" --result "${result}" \
    --launch-log "${launch_log}" --runner-log "${runner_log}" \
    --cleanup-remaining-pids "${remaining}"
}

run_wrapped_scenario() {
  local name="$1" offset="$2" child_runner="$3"
  local domain=$((domain_base + offset))
  local partition="tzcup_integrated_${run_id}_${name}"
  local result="${run_dir}/${name}.json"
  local launch_log="${run_dir}/${name}.launch.log"
  local runner_log="${run_dir}/${name}.runner.log"
  local started_ns finished_ns exit_code remaining
  rm -f -- "${result}" "${launch_log}" "${runner_log}"
  active_partition="${partition}"
  started_ns="$(date +%s%N)"
  set +e
  if [[ "${name}" == "mobility" ]]; then
    setsid env ROS_DOMAIN_ID="${domain}" GZ_PARTITION="${partition}" \
      FORMAL_VEHICLE_RUNTIME_WS="${runtime_ws}" \
      FORMAL_VEHICLE_MOBILITY_OUTPUT="${result}" \
      FORMAL_VEHICLE_MOBILITY_LOG="${launch_log}" \
      bash "${child_runner}" >"${runner_log}" 2>&1 &
  else
    setsid env ROS_DOMAIN_ID="${domain}" GZ_PARTITION="${partition}" \
      FORMAL_MANIPULATION_RUNTIME_WS="${runtime_ws}" \
      FORMAL_MANIPULATION_OUTPUT="${result}" \
      FORMAL_MANIPULATION_LOG="${launch_log}" \
      FORMAL_MANIPULATION_MATERIAL="${material}" \
      bash "${child_runner}" >"${runner_log}" 2>&1 &
  fi
  active_group_pid=$!
  wait "${active_group_pid}"
  exit_code=$?
  set -e
  cleanup_active_group
  remaining="$(cleanup_partition "${partition}")"
  active_partition=""
  finished_ns="$(date +%s%N)"
  record_scenario "${name}" "${started_ns}" "${finished_ns}" "${exit_code}" \
    "${domain}" "${partition}" "${result}" "${launch_log}" "${runner_log}" "${remaining}"
}

run_water_scenario() {
  local name="$1" offset="$2" validator_scenario="$3"
  local domain=$((domain_base + offset))
  local partition="tzcup_integrated_${run_id}_${name}"
  local result="${run_dir}/${name}.json"
  local launch_log="${run_dir}/${name}.launch.log"
  local runner_log="${run_dir}/${name}.runner.log"
  local started_ns finished_ns exit_code remaining
  rm -f -- "${result}" "${launch_log}" "${runner_log}"
  active_partition="${partition}"
  started_ns="$(date +%s%N)"
  env ROS_DOMAIN_ID="${domain}" GZ_PARTITION="${partition}" \
    setsid ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
      gui:=false bodywork_visible:=true start_controllers:=true \
      water_evaluation_interfaces:=true >"${launch_log}" 2>&1 &
  active_group_pid=$!
  set +e
  env ROS_DOMAIN_ID="${domain}" GZ_PARTITION="${partition}" \
    python3 "${repo_root}/scripts/validate_formal_water_recovery_runtime.py" \
      --scenario "${validator_scenario}" --output "${result}" >"${runner_log}" 2>&1
  exit_code=$?
  set -e
  cleanup_active_group
  remaining="$(cleanup_partition "${partition}")"
  active_partition=""
  finished_ns="$(date +%s%N)"
  record_scenario "${name}" "${started_ns}" "${finished_ns}" "${exit_code}" \
    "${domain}" "${partition}" "${result}" "${launch_log}" "${runner_log}" "${remaining}"
}

# Run all four even if one fails so the manifest can report the complete fresh
# attempt.  The final aggregator still fails on the first violated contract.
run_wrapped_scenario "mobility" 0 "${repo_root}/scripts/run_formal_vehicle_mobility_runtime.sh"
run_water_scenario "water_normal" 1 "normal"
run_water_scenario "water_full" 2 "full"
run_wrapped_scenario "manipulation" 3 "${repo_root}/scripts/run_formal_cube_pick_place_runtime.sh"

run_finished_ns="$(date +%s%N)"
python3 "${aggregator}" aggregate \
  --context "${context}" --output "${manifest_tmp}" \
  --finished-epoch-ns "${run_finished_ns}"
mv -- "${manifest_tmp}" "${manifest}"
echo "Integrated acceptance manifest: ${manifest}"
