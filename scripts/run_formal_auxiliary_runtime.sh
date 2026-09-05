#!/usr/bin/env bash
# Runtime acceptance for charge, power distribution, E-stop and lighting.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
underlay="${FORMAL_AUXILIARY_UNDERLAY:-${repo_root}/.work/final_frozen_runtime/install}"
output="${FORMAL_AUXILIARY_OUTPUT:-${repo_root}/artifacts/formal_auxiliary_power_lighting_runtime.json}"
log="${FORMAL_AUXILIARY_LOG:-${repo_root}/artifacts/formal_auxiliary_power_lighting_runtime.launch.log}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${underlay}")/final_runtime_closure_manifest.json}"
runtime_binding="${output}.runtime_binding.json"
formal_runtime_register_evidence_paths "${output}" "${runtime_binding}"

if [[ ! -f "${underlay}/setup.bash" ]]; then
  echo "Missing ROS underlay: ${underlay}/setup.bash" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-221}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_auxiliary_${ROS_DOMAIN_ID}_$$}"
if [[ -e "${output}" || -e "${log}" || -e "${runtime_binding}" ]]; then
  echo "Refusing stale auxiliary evidence; move the existing output/log before a fresh run" >&2
  exit 2
fi
mkdir -p "$(dirname "${output}")" "$(dirname "${log}")"

python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${underlay}" \
  --closure-manifest "${closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${underlay}/setup.bash"
set -u

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true start_controllers:=false \
  enable_safety_manager:=true start_simulation_safety_inputs:=false \
  start_power_system_simulators:=false \
  simulation_initial_estop_active:=true \
  high_bandwidth_sensor_runtime:=false >"${log}" 2>&1 &
launch_pid=$!
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

python3 "${repo_root}/scripts/validate_formal_auxiliary_runtime.py" \
  --output "${output}" --startup-timeout 120 --in-process-product-node \
  --snapshot "${snapshot}" --session "${session}" \
  --runtime-binding "${runtime_binding}"
