#!/usr/bin/env bash
# Run all cleaning, storage and recovery position actuators in isolated Gazebo.
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen colcon workspace}"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
snapshot_manifest="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
output="${FORMAL_FUNCTION_POSITIONS_OUTPUT:-${repo_root}/reports/engineering/formal_function_positions_runtime_report.json}"
runtime_binding="${FORMAL_FUNCTION_POSITIONS_RUNTIME_BINDING:-${output}.runtime_binding.json}"
launch_log="${FORMAL_FUNCTION_POSITIONS_LOG:-${repo_root}/artifacts/formal_function_positions_runtime.launch.log}"

for retained in "${output}" "${runtime_binding}" "${launch_log}"; do
  if [[ -e "${retained}" || -L "${retained}" ]]; then
    superseded="${retained}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"
    [[ ! -e "${superseded}" && ! -L "${superseded}" ]] || {
      echo "Refusing stale function-position evidence overwrite: ${superseded}" >&2
      exit 2
    }
    mv -- "${retained}" "${superseded}"
  fi
done

source /opt/ros/jazzy/setup.bash
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
formal_runtime_register_evidence_paths "${output}" "${runtime_binding}"

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
python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot_manifest}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${runtime_ws}/install" \
  --closure-manifest "${runtime_closure_manifest}" --session "${session}" \
  --snapshot "${snapshot_manifest}" --output "${runtime_binding}"
set +u
source "${runtime_ws}/install/setup.bash"
set -u
formal_source_bound_verify_overlay "${runtime_ws}/install"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_function_positions_${ROS_DOMAIN_ID}_$$}"
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

launch_pid=""
cleanup() {
  local pid="${launch_pid}"
  launch_pid=""
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${pid}"
}
formal_runtime_install_traps cleanup

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true start_controllers:=true \
  enable_safety_manager:=false start_localization:=false \
  squeegee_evaluation_interfaces:=true \
  high_bandwidth_sensor_runtime:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

required_topics=(
  /joint_states
  /cleaning_controller/joint_trajectory
  /service_controller/joint_trajectory
  /cleaning/squeegee/contact
  /model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_force_n
)
ready="false"
missing_topics=("${required_topics[@]}")
for _ in $(seq 1 160); do
  topic_snapshot="$(ros2 topic list 2>/dev/null || true)"
  missing_topics=()
  for required_topic in "${required_topics[@]}"; do
    if ! grep -Fxq -- "${required_topic}" <<<"${topic_snapshot}"; then
      missing_topics+=("${required_topic}")
    fi
  done
  if ((${#missing_topics[@]} == 0)); then
    ready="true"
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Formal vehicle launch exited before function-position controllers and squeegee telemetry became ready" >&2
    exit 3
  fi
  sleep 0.25
done
if [[ "${ready}" != "true" ]]; then
  printf 'Timed out waiting for cleaning/storage/recovery readiness; missing topics: %s\n' \
    "${missing_topics[*]}" >&2
  exit 3
fi

python3 "${repo_root}/scripts/validate_formal_function_positions_runtime.py" \
  --snapshot-manifest "${snapshot_manifest}" --session "${session}" \
  --runtime-binding "${runtime_binding}" --output "${output}"
echo "Function-position runtime acceptance: ${output}"
