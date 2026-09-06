#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime}"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
output="${FORMAL_VEHICLE_MOBILITY_OUTPUT:-${repo_root}/artifacts/formal_a300_drivetrain_runtime.json}"
runtime_binding="${FORMAL_VEHICLE_MOBILITY_RUNTIME_BINDING:-${output}.runtime_binding.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
launch_log="${FORMAL_VEHICLE_MOBILITY_LOG:-${output%.json}.launch.log}"
dry_load_mass_kg="${FORMAL_VEHICLE_DRY_LOAD_MASS_KG:-0.0}"
wastewater_load_mass_kg="${FORMAL_VEHICLE_WASTEWATER_LOAD_MASS_KG:-0.0}"
ready_timeout_s="${FORMAL_VEHICLE_MOBILITY_READY_TIMEOUT_S:-150}"
forward_speed_mps="${FORMAL_VEHICLE_MOBILITY_FORWARD_SPEED_MPS:-0.25}"
forward_duration_s="${FORMAL_VEHICLE_MOBILITY_FORWARD_DURATION_S:-4.0}"
safety_max_linear_velocity="${FORMAL_VEHICLE_MOBILITY_SAFETY_MAX_LINEAR_VELOCITY:-0.45}"
exercise_estop="${FORMAL_VEHICLE_MOBILITY_EXERCISE_ESTOP:-0}"
if [[ "${FORMAL_DRY_SPEED_REQUALIFICATION:-}" == "1" ]]; then
  [[ "${FORMAL_DRY_SPEED_REQUALIFICATION:-}" == "1" && -n "${FORMAL_DRY_SPEED_REQUALIFICATION_MARKER:-}" && -n "${FORMAL_DRY_SPEED_REQUALIFICATION_ROOT:-}" ]] || {
    echo "speed requalification requires the run-scoped opt-in marker" >&2; exit 2;
  }
  python3 "${repo_root}/scripts/formal_dry_speed_requalification_token.py" --validate \
    --profile "${repo_root}/config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml" \
    --run-root "${FORMAL_DRY_SPEED_REQUALIFICATION_ROOT}" --token "${FORMAL_DRY_SPEED_REQUALIFICATION_MARKER}" \
    --requested-cap "${safety_max_linear_velocity}"
elif [[ "${safety_max_linear_velocity}" != "0.45" ]]; then
  echo "non-default safety cap requires the requalification wrapper opt-in marker" >&2; exit 2;
fi
# Retire every canonical output before any setup/preflight work.  A failed
# fresh run must never leave a prior PASS artifact or its binding appearing
# current in the active final-acceptance session.
for retained in "${output}" "${runtime_binding}" "${launch_log}"; do
  if [[ -e "${retained}" ]]; then
    superseded="${retained}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"
    [[ ! -e "${superseded}" ]] || {
      echo "Refusing stale mobility evidence overwrite: ${superseded}" >&2
      exit 2
    }
    mv -- "${retained}" "${superseded}"
  fi
done

formal_runtime_register_evidence_paths "${output}" "${runtime_binding}"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing frozen ROS runtime overlay: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
if [[ ! -f "${runtime_closure_manifest}" ]]; then
  echo "Missing frozen final runtime closure: ${runtime_closure_manifest}" >&2
  exit 2
fi
if [[ ! -f "${session}" ]]; then
  echo "Missing running formal acceptance session: ${session}" >&2
  exit 2
fi

python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${runtime_ws}/install" \
  --closure-manifest "${runtime_closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${runtime_ws}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_mobility_${ROS_DOMAIN_ID}_$$}"
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true start_controllers:=true \
  enable_safety_manager:=true start_simulation_safety_inputs:=true \
  simulation_initial_estop_active:=false high_bandwidth_sensor_runtime:=false \
  start_localization:=false \
  dry_load_mass_kg:="${dry_load_mass_kg}" \
  wastewater_load_mass_kg:="${wastewater_load_mass_kg}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

if [[ "${safety_max_linear_velocity}" != "0.45" ]]; then
  for _ in $(seq 1 90); do
    timeout 10s ros2 param set /whole_vehicle_safety_manager max_linear_velocity "${safety_max_linear_velocity}" && break
    sleep 1
  done
  timeout 10s ros2 param get /whole_vehicle_safety_manager max_linear_velocity | grep -Fq "${safety_max_linear_velocity}" || {
    echo "requalification safety-cap override was not applied" >&2; exit 3;
  }
fi

probe_args=()
if [[ "${exercise_estop}" == "1" ]]; then
  probe_args+=(--exercise-estop)
elif [[ "${exercise_estop}" != "0" ]]; then
  echo "FORMAL_VEHICLE_MOBILITY_EXERCISE_ESTOP must be 0 or 1" >&2
  exit 2
fi

python3 "${repo_root}/scripts/validate_formal_vehicle_mobility_runtime.py" \
  --output "${output}" --timeout "${ready_timeout_s}" --forward-speed "${forward_speed_mps}" --forward-duration "${forward_duration_s}" \
  --safety-max-linear-velocity "${safety_max_linear_velocity}" \
  --snapshot "${snapshot}" --session "${session}" --runtime-binding "${runtime_binding}" \
  "${probe_args[@]}"
