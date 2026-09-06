#!/usr/bin/env bash
# Launch the formal vehicle and collect physical service-door joint evidence.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_setup="${FORMAL_SERVICE_DOOR_RUNTIME_SETUP:-${repo_root}/.work/final_frozen_runtime/install/setup.bash}"
output="${FORMAL_SERVICE_DOOR_RUNTIME_OUTPUT:-${repo_root}/artifacts/formal_service_door_runtime.json}"
log="${FORMAL_SERVICE_DOOR_RUNTIME_LOG:-${repo_root}/artifacts/formal_service_door_runtime.launch.log}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
install_root="$(dirname "${runtime_setup}")"
closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${install_root}")/final_runtime_closure_manifest.json}"
runtime_binding="${output}.runtime_binding.json"
gz_sidecar="${output}.gz_joint_state_sidecar.json"
formal_runtime_register_evidence_paths "${output}" "${runtime_binding}"
if [[ ! -f "${runtime_setup}" ]]; then
  echo "Missing runtime setup: ${runtime_setup}" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-79}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_service_door_${ROS_DOMAIN_ID}_$$}"
if [[ -e "${output}" || -e "${log}" || -e "${runtime_binding}" || -e "${gz_sidecar}" ]]; then
  echo "Refusing stale service-door evidence; move the existing output/log before a fresh run" >&2
  exit 2
fi
mkdir -p "$(dirname "${output}")" "$(dirname "${log}")"

write_runner_failure_report() {
  local reason="$1"
  python3 - "${output}" "${reason}" "${gz_sidecar}" <<'PY'
import json
import sys
from pathlib import Path

output, reason, sidecar = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
try:
    sidecar_evidence = json.loads(sidecar.read_text(encoding="utf-8"))
except (OSError, UnicodeError, json.JSONDecodeError) as exc:
    sidecar_evidence = {"status": "FAILED", "error": str(exc)}
report = {
    "report_id": "tzcup_formal_service_door_runtime_v1",
    "status": "FORMAL_BODYWORK_SERVICE_DOOR_RUNTIME_FAILED",
    "passed": False,
    "checks": {
        "runner_completed": False,
        "independent_gazebo_joint_state_sidecar_is_complete": False,
    },
    "runner_failure": reason,
    "gazebo_joint_state_sidecar": sidecar_evidence,
    "claim_boundary": "Launcher or independent Gazebo sidecar failure is rejected fail-closed.",
}
output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(json.dumps(report, sort_keys=True))
PY
}

python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${install_root}" \
  --closure-manifest "${closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${runtime_setup}"
set -u

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false start_controllers:=true enable_safety_manager:=false \
  high_bandwidth_sensor_runtime:=false \
  service_door_evaluation_interfaces:=true >"${log}" 2>&1 &
launch_pid=$!
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

physical_joint_states_topic="/formal/service_door_joint_states"
for _ in $(seq 1 120); do
  if ros2 topic list 2>/dev/null | grep -Fxq "${physical_joint_states_topic}"; then
    break
  fi
  sleep 0.25
done
ros2 topic list 2>/dev/null | grep -Fxq "${physical_joint_states_topic}" || {
  echo "Timed out waiting for ${physical_joint_states_topic}" >&2
  exit 3
}

gz_sidecar_args=(
  --partition "${GZ_PARTITION}"
  --launcher-pid "${launch_pid}"
  --topic /formal_vehicle/evaluation/bodywork_service/joint_states
  --output "${gz_sidecar}"
)
for joint in \
  bodywork_power_service_door_hinge_joint bodywork_power_service_door_latch_joint \
  bodywork_compute_service_door_hinge_joint bodywork_compute_service_door_latch_joint \
  bodywork_wet_service_door_hinge_joint bodywork_wet_service_door_latch_joint \
  bodywork_rear_dry_service_door_hinge_joint bodywork_rear_dry_service_door_latch_joint; do
  gz_sidecar_args+=(--joint "${joint}")
done
set +e
python3 "${repo_root}/scripts/collect_formal_service_door_gz_sidecar.py" "${gz_sidecar_args[@]}"
gz_sidecar_rc=$?
set -e
if [[ "${gz_sidecar_rc}" != 0 ]]; then
  echo "Independent Gazebo joint-state sidecar failed" >&2
  write_runner_failure_report "independent_gazebo_joint_state_sidecar_failed"
  exit 3
fi
if ! kill -0 "${launch_pid}" 2>/dev/null; then
  echo "Gazebo launcher exited before service-door collector" >&2
  write_runner_failure_report "gazebo_launcher_exited_before_service_door_collector"
  exit 3
fi

python3 "${repo_root}/scripts/collect_formal_service_door_runtime.py" \
  --output "${output}" \
  --snapshot-manifest "${snapshot}" --session "${session}" \
  --runtime-binding "${runtime_binding}" \
  --plugin-diagnostic-log "${log}" \
  --gazebo-sidecar "${gz_sidecar}"
