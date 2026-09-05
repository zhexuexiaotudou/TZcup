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
formal_runtime_register_evidence_paths "${output}" "${runtime_binding}"
if [[ ! -f "${runtime_setup}" ]]; then
  echo "Missing runtime setup: ${runtime_setup}" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-79}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_service_door_${ROS_DOMAIN_ID}_$$}"
if [[ -e "${output}" || -e "${log}" || -e "${runtime_binding}" ]]; then
  echo "Refusing stale service-door evidence; move the existing output/log before a fresh run" >&2
  exit 2
fi
mkdir -p "$(dirname "${output}")" "$(dirname "${log}")"

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

for _ in $(seq 1 120); do
  if ros2 topic list 2>/dev/null | grep -Fxq /joint_states; then
    break
  fi
  sleep 0.25
done
ros2 topic list 2>/dev/null | grep -Fxq /joint_states || {
  echo "Timed out waiting for /joint_states" >&2
  exit 3
}

python3 "${repo_root}/scripts/collect_formal_service_door_runtime.py" \
  --output "${output}" \
  --snapshot-manifest "${snapshot}" --session "${session}" \
  --runtime-binding "${runtime_binding}" \
  --plugin-diagnostic-log "${log}"
