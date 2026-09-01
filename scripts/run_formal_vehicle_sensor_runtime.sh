#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_setup="${FORMAL_SENSOR_RUNTIME_SETUP:-${repo_root}/.work/final_frozen_runtime/install/setup.bash}"
output="${FORMAL_SENSOR_RUNTIME_OUTPUT:-${repo_root}/reports/engineering/formal_vehicle_runtime_report.json}"
log="${FORMAL_SENSOR_RUNTIME_LOG:-${repo_root}/artifacts/formal_vehicle_sensor_runtime.launch.log}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${FORMAL_SENSOR_SNAPSHOT:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}}"
install_root="$(cd -- "$(dirname -- "${runtime_setup}")" && pwd -P)"
closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${install_root}")/final_runtime_closure_manifest.json}"
runtime_binding="${output}.runtime_binding.json"
fov_output="${FORMAL_SENSOR_FOV_OUTPUT:-${repo_root}/reports/engineering/formal_vehicle_fov_occlusion_report.json}"
preembedded_world="${FORMAL_SENSOR_PREEMBEDDED_WORLD:-${output%.json}.preembedded_sensor_world.sdf}"
preembedded_report="${FORMAL_SENSOR_PREEMBEDDED_REPORT:-${output%.json}.preembedded_sensor_world.json}"
preembedded_model_pose="${FORMAL_SENSOR_PREEMBEDDED_MODEL_POSE:-0 0 0.005 0 0 0}"
memory_evidence_base="${output%.json}"
memory_preflight_prefix="${memory_evidence_base}.windows_memory_preflight"
memory_watchdog_prefix="${memory_evidence_base}.memory_watchdog"
loopback_attestation="${FORMAL_SENSOR_LOOPBACK_ATTESTATION:-${memory_evidence_base}.loopback_attestation.json}"
formal_runtime_register_evidence_paths \
  "${output}" "${fov_output}" "${runtime_binding}" \
  "${preembedded_world}" "${preembedded_report}" \
  "${memory_preflight_prefix}.json" "${memory_preflight_prefix}.log" \
  "${memory_watchdog_prefix}.json" "${memory_watchdog_prefix}.log" \
  "${loopback_attestation}"
if [[ ! -f "${runtime_setup}" ]]; then
  echo "Missing fresh sensor runtime overlay: ${runtime_setup}" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-81}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_sensor_runtime_${ROS_DOMAIN_ID}_$$}"
for required in "${session}" "${snapshot}"; do
  if [[ ! -f "${required}" ]]; then
    echo "Missing formal sensor acceptance input: ${required}" >&2
    exit 2
  fi
done
stale_paths=(
  "${output}" "${fov_output}" "${log}" "${runtime_binding}"
  "${preembedded_world}" "${preembedded_report}"
  "${memory_preflight_prefix}.json" "${memory_preflight_prefix}.log"
  "${memory_watchdog_prefix}.json" "${memory_watchdog_prefix}.log"
  "${loopback_attestation}"
)
stale_existing=()
for candidate in "${stale_paths[@]}"; do
  [[ -e "${candidate}" ]] && stale_existing+=("${candidate}")
done
if (( ${#stale_existing[@]} > 0 )); then
  printf 'Refusing stale sensor/FOV evidence; archive or isolate every prior attempt artifact before a fresh run:\n' >&2
  printf '  %s\n' "${stale_existing[@]}" >&2
  exit 2
fi
mkdir -p "$(dirname "${output}")" "$(dirname "${fov_output}")" "$(dirname "${log}")"

python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${install_root}" \
  --closure-manifest "${closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${runtime_setup}"
set -u

# Harmonic can miss render / GPU sensors inserted later through UserCommands.
# Convert the frozen expanded vehicle and embed it before the Sensors system
# starts.  The generator fails closed if conversion loses any physical sensor
# attachment; the normal collector still requires every one of the 12 streams.
installed_package_share="$(ros2 pkg prefix --share sanitation_vehicle_description)"
expected_package_share="${install_root}/share/sanitation_vehicle_description"
if [[ ! -d "${expected_package_share}" ]] || \
   [[ "$(cd -- "${installed_package_share}" && pwd -P)" != "$(cd -- "${expected_package_share}" && pwd -P)" ]]; then
  echo "sanitation_vehicle_description resolves outside the frozen runtime install: ${installed_package_share}" >&2
  exit 2
fi
installed_world="${installed_package_share}/worlds/formal_vehicle_validation.sdf"
installed_controller_config="${installed_package_share}/config/formal_vehicle_controllers.yaml"
python3 "${repo_root}/scripts/prepare_formal_preembedded_sensor_world.py" \
  --source-world "${installed_world}" \
  --vehicle-urdf "${repo_root}/reports/engineering/formal_competition_vehicle.urdf" \
  --controller-config "${installed_controller_config}" \
  --runtime-install-root "${install_root}" \
  --output-world "${preembedded_world}" --report "${preembedded_report}" \
  --model-pose "${preembedded_model_pose}"

# This deterministic mesh-ray gate is part of the same frozen session.  A
# blocked FOV, mount, range or configured-rate contract stops before Gazebo.
python3 "${repo_root}/scripts/validate_formal_fov_occlusion.py" \
  --urdf "${repo_root}/reports/engineering/formal_competition_vehicle.urdf" \
  --layout "${repo_root}/config/high_fidelity_vehicle/formal_vehicle_layout.yaml" \
  --output "${fov_output}" --compact

launch_pid=""
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" != "1" ]]; then
  formal_runtime_memory_preflight "${memory_preflight_prefix}"
fi
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  world:="${preembedded_world}" spawn_robot:=false \
  gui:=false headless_rendering:=true bodywork_visible:=true start_controllers:=true \
  enable_safety_manager:=true simulation_initial_estop_active:=true \
  high_bandwidth_sensor_runtime:=true >"${log}" 2>&1 &
launch_pid=$!
if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" != "1" ]]; then
  formal_runtime_start_memory_watchdog "${launch_pid}" "${memory_watchdog_prefix}"
fi

python3 "${repo_root}/scripts/capture_formal_sensor_loopback_attestation.py" \
  --partition "${GZ_PARTITION}" \
  --expected-cyclonedds-uri "file://${repo_root}/config/cyclonedds_localhost.xml" \
  --session "${session}" --closure-manifest "${closure_manifest}" \
  --output "${loopback_attestation}" --timeout 90

python3 "${repo_root}/scripts/collect_formal_vehicle_sensor_runtime.py" \
  --output "${output}" --timeout 180 \
  --session "${session}" --snapshot "${snapshot}" --fov-report "${fov_output}" \
  --runtime-binding "${runtime_binding}" \
  --preembedded-report "${preembedded_report}" --preembedded-world "${preembedded_world}" \
  --preembedded-model-pose "${preembedded_model_pose}"
