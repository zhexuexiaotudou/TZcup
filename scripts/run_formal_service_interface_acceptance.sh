#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
formal_service_setup="${FORMAL_SERVICE_SETUP:-${repo_root}/.work/final_frozen_runtime/install/setup.bash}"
if [[ ! -f "${formal_service_setup}" ]]; then
  printf 'formal service overlay is missing: %s\n' "${formal_service_setup}" >&2
  exit 2
fi
episodes_dir="${1:-${repo_root}/artifacts/formal_service_interface_episodes}"
aggregate_output="${2:-${repo_root}/artifacts/formal_service_interface_acceptance.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
install_root="$(dirname "${formal_service_setup}")"
closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${install_root}")/final_runtime_closure_manifest.json}"
runtime_binding="${aggregate_output}.runtime_binding.json"
formal_runtime_register_evidence_paths "${aggregate_output}" "${runtime_binding}"
scenarios=(
  charge_allow charge_reject_no_contact charge_reject_door_closed
  charge_reject_lock_open drain_allow drain_reject_no_contact
  drain_reject_cap_closed mutual_interlock_charge_wins
)
base_domain="${FORMAL_SERVICE_DOMAIN_BASE:-87}"
formal_runtime_configure "${base_domain}" "${#scenarios[@]}"
if [[ -e "${episodes_dir}" || -e "${aggregate_output}" || -e "${runtime_binding}" ]]; then
  echo "Refusing stale service-interface evidence; use fresh episode and aggregate paths" >&2
  exit 2
fi
mkdir -p "$(dirname "${episodes_dir}")" "$(dirname "${aggregate_output}")"

python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${install_root}" \
  --closure-manifest "${closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${formal_service_setup}"
set -u
mkdir "${episodes_dir}"

vehicle_xacro="${repo_root}/starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
vehicle_model="${episodes_dir}/formal_service_acceptance_vehicle.urdf"
xacro "${vehicle_xacro}" \
  use_sim:=true \
  high_bandwidth_sensor_runtime:=false \
  service_acceptance_interfaces:=true \
  wastewater_load_mass_kg:=8.30 \
  > "${vehicle_model}.tmp"
mv "${vehicle_model}.tmp" "${vehicle_model}"

active_launch_pid=""
active_partition=""
cleanup_active() {
  local cleanup_status=0
  if [[ -n "${active_launch_pid}" ]]; then
    formal_runtime_kill_group "${active_launch_pid}" || cleanup_status=1
    active_launch_pid=""
  fi
  if [[ -n "${active_partition}" ]]; then
    formal_runtime_cleanup_partition "${active_partition}" || cleanup_status=1
    active_partition=""
  fi
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup_active

for index in "${!scenarios[@]}"; do
  scenario="${scenarios[$index]}"
  episode_output="${episodes_dir}/${scenario}.json"
  episode_log="${episodes_dir}/${scenario}.launch.log"
  station_x_offset="0.0"
  if [[ "${scenario}" == *"no_contact"* ]]; then
    station_x_offset="4.0"
  fi
  export ROS_DOMAIN_ID="$((base_domain + index))"
  export GZ_PARTITION="formal_service_${scenario}_${ROS_DOMAIN_ID}_$$"
  active_partition="${GZ_PARTITION}"
  set +e
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" timeout --foreground \
    --signal=INT --kill-after=20s 240s \
    ros2 launch sanitation_service_acceptance formal_service_acceptance.launch.py \
      scenario:="${scenario}" \
      output:="${episode_output}" \
      vehicle_model:="${vehicle_model}" \
      station_x_offset:="${station_x_offset}" \
      > "${episode_log}" 2>&1 &
  launch_pid=$!
  active_launch_pid="${launch_pid}"
  wait "${launch_pid}"
  launch_rc=$?
  set -e
  if ! cleanup_active; then
    printf 'scenario=%s cleanup_failed_closed=true\n' \
      "${scenario}" >> "${episode_log}"
    if [[ -f "${episode_output}" ]]; then
      mv -- "${episode_output}" "${episode_output}.cleanup_failed"
    fi
    launch_rc=125
  fi
  if [[ ! -f "${episode_output}" ]]; then
    printf 'scenario=%s launch_rc=%s artifact_missing=true\n' \
      "${scenario}" "${launch_rc}" >> "${episode_log}"
  fi
done

python3 "${repo_root}/scripts/validate_formal_service_interface_acceptance.py" \
  --episodes-dir "${episodes_dir}" \
  --output "${aggregate_output}" --snapshot "${snapshot}" --session "${session}" \
  --runtime-binding "${runtime_binding}"
