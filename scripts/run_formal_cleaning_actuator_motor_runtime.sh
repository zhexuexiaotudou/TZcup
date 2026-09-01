#!/usr/bin/env bash
# Run the cleaning-motor gate against an explicitly selected frozen overlay.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen colcon workspace}"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
snapshot_manifest="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
output="${FORMAL_CLEANING_MOTOR_OUTPUT:-${repo_root}/artifacts/formal_cleaning_actuator_motor_runtime.json}"
raw="${FORMAL_CLEANING_MOTOR_RAW_OUTPUT:-${output%.json}.capture.json}"
launch_log="${FORMAL_CLEANING_MOTOR_LOG:-${repo_root}/artifacts/formal_cleaning_actuator_motor_runtime.launch.log}"
runtime_binding="${FORMAL_CLEANING_MOTOR_RUNTIME_BINDING:-${output}.runtime_binding.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"

# Retire the complete canonical evidence unit before any runtime preflight.
# A new sidecar may never overwrite a retained binding and make an earlier
# result appear attached to the active final-acceptance session.
for retained in "${output}" "${raw}" "${launch_log}" "${runtime_binding}"; do
  if [[ -e "${retained}" || -L "${retained}" ]]; then
    superseded="${retained}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"
    [[ ! -e "${superseded}" && ! -L "${superseded}" ]] || {
      echo "Refusing stale cleaning-motor evidence overwrite: ${superseded}" >&2
      exit 2
    }
    mv -- "${retained}" "${superseded}"
  fi
done
formal_runtime_register_evidence_paths "${output}" "${raw}" "${runtime_binding}"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing frozen runtime overlay: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
if [[ ! -f "${runtime_closure_manifest}" ]]; then
  echo "Missing frozen final runtime closure: ${runtime_closure_manifest}" >&2
  exit 2
fi
if [[ ! -f "${snapshot_manifest}" ]]; then
  echo "Missing frozen vehicle snapshot manifest: ${snapshot_manifest}" >&2
  exit 2
fi
if [[ ! -f "${session}" ]]; then
  echo "Missing running formal acceptance session: ${session}" >&2
  exit 2
fi

# Verify the checkout snapshot and create the unique sidecar before Gazebo or
# any ROS launch is admitted.  The validator repeats these identities from
# its live process environment before it can publish a PASS report.
python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot_manifest}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${runtime_ws}/install" \
  --closure-manifest "${runtime_closure_manifest}" --session "${session}" \
  --snapshot "${snapshot_manifest}" --output "${runtime_binding}"

set +u
source "${runtime_ws}/install/setup.bash"
formal_source_bound_verify_overlay "${runtime_ws}/install"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-85}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_cleaning_motor_${ROS_DOMAIN_ID}_$$}"
mkdir -p "$(dirname "${output}")" "$(dirname "${raw}")" "$(dirname "${launch_log}")"

launch_pid=""
cleanup() {
  local pid="${launch_pid}"
  launch_pid=""
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${pid}"
}
formal_runtime_install_traps cleanup

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true start_controllers:=true \
  enable_safety_manager:=true start_localization:=false \
  start_simulation_safety_inputs:=true start_power_system_simulators:=true \
  high_bandwidth_sensor_runtime:=false simulation_initial_estop_active:=true \
  >"${launch_log}" 2>&1 &
launch_pid=$!

ready="false"
for _ in $(seq 1 120); do
  if ros2 topic list 2>/dev/null | grep -Fxq \
    /model/tzcup_formal_sanitation_vehicle/cleaning_motors/telemetry_snapshot; then
    ready="true"
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Formal vehicle launch exited before cleaning-motor status became ready" >&2
    exit 3
  fi
  sleep 0.25
done
if [[ "${ready}" != "true" ]]; then
  echo "Timed out waiting for physical cleaning-motor status" >&2
  exit 3
fi

python3 "${repo_root}/scripts/collect_formal_cleaning_actuator_motor_runtime.py" \
  --exercise-live --snapshot-manifest "${snapshot_manifest}" --output "${raw}"
python3 "${repo_root}/scripts/validate_formal_cleaning_actuator_motor_runtime.py" \
  "${raw}" --output "${output}" --snapshot "${snapshot_manifest}" \
  --session "${session}" --runtime-binding "${runtime_binding}"
echo "Cleaning motor runtime acceptance: ${output}"
