#!/usr/bin/env bash
# Run the complete formal sensor gate in one fresh, isolated evidence root.
#
# This wrapper is intentionally not a reduced diagnostic.  It creates a fresh
# session bound to the canonical snapshot and frozen runtime, then delegates to
# the existing full-specification sensor runner.  Its extra responsibility is
# evidence placement and an after-run cleanup attestation; it never deletes
# failed-run evidence or attempts to repair a surviving Gazebo process.
set -uo pipefail

usage() {
  cat >&2 <<'EOF'
Usage: run_formal_sensor_transport_probe.sh --runtime-ws PATH --attempt-root PATH --domain N

The attempt root must not yet exist and must be directly below this repository's
.work directory.  The frozen runtime workspace must contain install/setup.bash
and final_runtime_closure_manifest.json.
EOF
}

fail() {
  echo "formal sensor transport probe: $*" >&2
  exit 2
}

runtime_ws_arg=""
attempt_root_arg=""
domain=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime-ws)
      runtime_ws_arg="${2:-}"
      shift 2
      ;;
    --attempt-root)
      attempt_root_arg="${2:-}"
      shift 2
      ;;
    --domain)
      domain="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      usage
      fail "unknown argument: $1"
      ;;
  esac
done

[[ -n "${runtime_ws_arg}" && -n "${attempt_root_arg}" && -n "${domain}" ]] || {
  usage
  fail "--runtime-ws, --attempt-root and --domain are all required"
}
[[ "${domain}" =~ ^[0-9]+$ && ${#domain} -le 3 ]] || fail "domain must be a decimal integer"
domain_number=$((10#${domain}))
(( (domain_number >= 0 && domain_number <= 101) || (domain_number >= 215 && domain_number <= 231) )) || \
  fail "domain intersects the Linux ephemeral UDP port range or exceeds the formal range"

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)}"
repo_root="$(cd -- "${repo_root}" && pwd -P)"
work_root="${repo_root}/.work"
[[ -d "${work_root}" && ! -L "${work_root}" ]] || fail "repository .work root is missing or symbolic: ${work_root}"

runtime_ws="$(realpath -e -- "${runtime_ws_arg}")" || fail "runtime workspace does not exist: ${runtime_ws_arg}"
[[ -d "${runtime_ws}" && ! -L "${runtime_ws}" ]] || fail "runtime workspace is not a regular directory: ${runtime_ws}"
runtime_setup="${runtime_ws}/install/setup.bash"
closure_manifest="${runtime_ws}/final_runtime_closure_manifest.json"
[[ -f "${runtime_setup}" && ! -L "${runtime_setup}" ]] || fail "missing frozen runtime setup: ${runtime_setup}"
[[ -f "${closure_manifest}" && ! -L "${closure_manifest}" ]] || fail "missing frozen runtime closure: ${closure_manifest}"

attempt_root="$(realpath -m -- "${attempt_root_arg}")"
case "${attempt_root}" in
  "${work_root}"/*) ;;
  *) fail "attempt root must be below repository .work: ${attempt_root}" ;;
esac
[[ "${attempt_root}" != "${work_root}" ]] || fail "attempt root must not be repository .work itself"
[[ ! -e "${attempt_root}" && ! -L "${attempt_root}" ]] || fail "attempt root already exists: ${attempt_root}"
[[ -d "$(dirname -- "${attempt_root}")" ]] || fail "attempt root parent must already exist: $(dirname -- "${attempt_root}")"
mkdir -- "${attempt_root}" || fail "cannot create fresh attempt root: ${attempt_root}"

canonical_snapshot="${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json"
[[ -f "${canonical_snapshot}" && ! -L "${canonical_snapshot}" ]] || fail "canonical snapshot is missing or symbolic: ${canonical_snapshot}"

session="${attempt_root}/formal_sensor_probe_session.json"
sensor_output="${attempt_root}/formal_vehicle_runtime_report.json"
sensor_log="${attempt_root}/formal_vehicle_sensor_runtime.launch.log"
fov_output="${attempt_root}/formal_vehicle_fov_occlusion_report.json"
preembedded_world="${attempt_root}/preembedded_sensor_world.sdf"
preembedded_report="${attempt_root}/preembedded_sensor_world.json"
runtime_binding="${sensor_output}.runtime_binding.json"
memory_base="${sensor_output%.json}"
memory_preflight_json="${memory_base}.windows_memory_preflight.json"
memory_preflight_log="${memory_base}.windows_memory_preflight.log"
memory_watchdog_json="${memory_base}.memory_watchdog.json"
memory_watchdog_log="${memory_base}.memory_watchdog.log"
attestation="${attempt_root}/cleanup_attestation.json"

for path in \
  "${session}" "${sensor_output}" "${sensor_log}" "${fov_output}" \
  "${preembedded_world}" "${preembedded_report}" "${runtime_binding}" \
  "${memory_preflight_json}" "${memory_preflight_log}" \
  "${memory_watchdog_json}" "${memory_watchdog_log}" "${attestation}"; do
  [[ ! -e "${path}" && ! -L "${path}" ]] || fail "fresh attempt unexpectedly contains evidence: ${path}"
done

# The inner runner owns this lock.  The unique path is tested for availability
# only after it returns, so this wrapper never contends with its child.
partition="tzcup_formal_sensor_transport_probe_${domain}_$$_$(date +%s)"
lock_path="/tmp/tzcup_formal_sensor_transport_probe_${domain}_$$_lock"

export TZCUP_REPOSITORY_ROOT="${repo_root}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${domain}"
export GZ_PARTITION="${partition}"
export FORMAL_GAZEBO_LOCK_FILE="${lock_path}"
export FORMAL_SENSOR_RUNTIME_SETUP="${runtime_setup}"
export FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST="${closure_manifest}"
export FORMAL_ACCEPTANCE_SESSION="${session}"
export FORMAL_VEHICLE_SNAPSHOT_MANIFEST="${canonical_snapshot}"
unset FORMAL_SENSOR_SNAPSHOT
export FORMAL_SENSOR_RUNTIME_OUTPUT="${sensor_output}"
export FORMAL_SENSOR_RUNTIME_LOG="${sensor_log}"
export FORMAL_SENSOR_FOV_OUTPUT="${fov_output}"
export FORMAL_SENSOR_PREEMBEDDED_WORLD="${preembedded_world}"
export FORMAL_SENSOR_PREEMBEDDED_REPORT="${preembedded_report}"
export FORMAL_ORCHESTRATED_STEP_SESSION=0
export FORMAL_MEMORY_WATCHDOG_ENABLED=1
export FORMAL_WINDOWS_MEMORY_GUARD_ENABLED=1

# Do the canonical pure-Python check before a fresh session is minted.  The
# snapshot may not be copied or regenerated into this diagnostic attempt.
if ! python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${canonical_snapshot}"; then
  exit 2
fi

if ! python3 "${repo_root}/scripts/formal_acceptance_session.py" start \
  --snapshot "${canonical_snapshot}" --output "${session}" \
  --runtime-closure-manifest "${closure_manifest}" \
  --runtime-install-root "${runtime_ws}/install" \
  --repository-root "${repo_root}"; then
  exit 2
fi

runner_status=125
"${repo_root}/scripts/run_formal_vehicle_sensor_runtime.sh"
runner_status=$?

# Do not call formal_runtime_cleanup_partition here: any survivor is evidence
# of failed cleanup and must remain observable.  Scan the same exact partition
# while excluding this wrapper and the short-lived scanner process.
scan_ok=true
survivors_json="$(/usr/bin/python3 - "${partition}" "$$" <<'PY'
import json
import os
import sys

needle = ("GZ_PARTITION=" + sys.argv[1]).encode()
excluded = {os.getpid(), int(sys.argv[2])}
rows = []
for raw in os.listdir("/proc"):
    if not raw.isdigit() or int(raw) in excluded:
        continue
    try:
        environment = open(f"/proc/{raw}/environ", "rb").read().split(b"\0")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if needle in environment:
        rows.append(int(raw))
print(json.dumps(sorted(rows)))
PY
)" || scan_ok=false
if [[ "${scan_ok}" != true ]]; then
  survivors_json='[]'
fi

lock_reacquirable=true
(
  exec 9>"${lock_path}"
  flock -n 9
) || lock_reacquirable=false

attestation_ok=true
/usr/bin/python3 - \
  "${attestation}" "${attempt_root}" "${runtime_ws}" "${domain}" "${partition}" \
  "${lock_path}" "${runner_status}" "${scan_ok}" "${lock_reacquirable}" "${survivors_json}" \
  "${memory_watchdog_json}" <<'PY' || attestation_ok=false
import json
import os
import sys
import time
from pathlib import Path

(
    output_raw,
    attempt_root,
    runtime_ws,
    domain_raw,
    partition,
    lock_path,
    runner_status_raw,
    scan_ok_raw,
    lock_reacquirable_raw,
    survivors_raw,
    watchdog_raw,
) = sys.argv[1:]
output = Path(output_raw)
watchdog_path = Path(watchdog_raw)
survivors = json.loads(survivors_raw)
if not isinstance(survivors, list) or not all(isinstance(pid, int) and pid > 1 for pid in survivors):
    raise SystemExit("cleanup survivor scan result is invalid")
scan_ok = scan_ok_raw == "true"
lock_reacquirable = lock_reacquirable_raw == "true"
runner_status = int(runner_status_raw)
watchdog_survivors = 1
if watchdog_path.is_file() and not watchdog_path.is_symlink():
    watchdog = json.loads(watchdog_path.read_text(encoding="utf-8"))
    value = watchdog.get("surviving_group_processes")
    if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
        watchdog_survivors = value
passed = (
    runner_status == 0
    and scan_ok
    and not survivors
    and watchdog_survivors == 0
    and lock_reacquirable
)
payload = {
    "report_id": "tzcup_formal_sensor_transport_probe_cleanup_attestation_v1",
    "status": "FORMAL_SENSOR_TRANSPORT_CLEANUP_PASSED" if passed else "FORMAL_SENSOR_TRANSPORT_CLEANUP_FAILED",
    "passed": passed,
    "recorded_epoch_ns": time.time_ns(),
    "attempt_root": attempt_root,
    "runtime_workspace": runtime_ws,
    "ros_domain_id": int(domain_raw),
    "gz_partition": partition,
    "gazebo_lock_path": lock_path,
    "sensor_runner_exit_code": runner_status,
    "partition_scan_succeeded": scan_ok,
    "partition_survivors": survivors,
    "partition_survivor_pids": survivors,
    "partition_survivor_count": len(survivors),
    "surviving_group_processes": watchdog_survivors,
    "lock_released": lock_reacquirable,
    "lock_reacquirable": lock_reacquirable,
    "evidence_deleted_by_probe": False,
}
if output.exists() or output.is_symlink():
    raise SystemExit(f"refusing to overwrite cleanup attestation: {output}")
pending = output.with_name(f"{output.name}.pending.{os.getpid()}")
pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
pending.replace(output)
PY

if [[ "${attestation_ok}" != true || "${scan_ok}" != true || "${lock_reacquirable}" != true ]] || \
  [[ "${survivors_json}" != '[]' ]]; then
  exit 125
fi
exit "${runner_status}"
