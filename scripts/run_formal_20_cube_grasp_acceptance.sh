#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${FORMAL_MANIPULATION_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime/install}"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${runtime_ws}")/final_runtime_closure_manifest.json}"
manifest="${FORMAL_20_CUBE_MANIFEST:-${repo_root}/artifacts/formal_20_cube_grasp_manifest.json}"
output="${FORMAL_20_CUBE_OUTPUT:-${repo_root}/artifacts/formal_20_cube_grasp_runtime.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
runtime_binding="${FORMAL_20_CUBE_RUNTIME_BINDING:-${output}.runtime_binding.json}"
launch_log="${FORMAL_20_CUBE_LOG:-${output%.json}.launch.log}"
timeout_s="${FORMAL_20_CUBE_PER_TARGET_TIMEOUT_S:-180}"

# A failed preflight must never leave an older canonical PASS artifact in
# place.  This comes before ROS setup as well as every runtime gate, because a
# missing host setup must not make retained evidence look current.
for retained in "${output}" "${runtime_binding}" "${manifest}" "${launch_log}"; do
  if [[ -e "${retained}" || -L "${retained}" ]]; then
    mv -- "${retained}" "${retained}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"
  fi
done

source /opt/ros/jazzy/setup.bash
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
formal_runtime_register_evidence_paths "${output}" "${runtime_binding}"

if [[ ! -f "${runtime_ws}/setup.bash" ]]; then
  echo "Missing ROS runtime overlay: ${runtime_ws}/setup.bash" >&2
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
  --repository-root "${repo_root}" --install-root "${runtime_ws}" \
  --closure-manifest "${runtime_closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${runtime_ws}/setup.bash"
formal_source_bound_verify_overlay "${runtime_ws}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_20_cube_${ROS_DOMAIN_ID}_$$}"
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

python3 "${repo_root}/scripts/prepare_formal_20_cube_grasp_acceptance.py" --output "${manifest}"

simulation_pid=""
bridge_pid=""
executor_pid=""
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" \
    "${executor_pid}" "${bridge_pid}" "${simulation_pid}"
}
formal_runtime_install_traps cleanup

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_manipulation formal_20_cube_pick_place.launch.py \
  manifest:="${manifest}" >"${launch_log}" 2>&1 &
simulation_pid=$!
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/observed_status_json@std_msgs/msg/String[gz.msgs.StringMsg" \
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json@std_msgs/msg/String[gz.msgs.StringMsg" \
  "/manipulation/gripper/dual_contact@std_msgs/msg/Bool[gz.msgs.Boolean" \
  --ros-args \
  -r "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json:=/formal_acceptance/evaluator/dry_bin/status_json" \
  >>"${launch_log}" 2>&1 &
bridge_pid=$!
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_manipulation formal_physical_grasp.launch.py \
  >>"${launch_log}" 2>&1 &
executor_pid=$!

# Grasp attachment has no preselected entity.  This release only establishes
# a fail-closed detached baseline after the bridge and contact gate are live.
sleep 13
gz topic -t /manipulation/grasp/detach -m gz.msgs.Empty -p "" \
  >>"${launch_log}" 2>&1

python3 "${repo_root}/scripts/validate_formal_20_cube_grasp_runtime.py" \
  --manifest "${manifest}" --output "${output}" \
  --snapshot "${snapshot}" \
  --session "${session}" --runtime-binding "${runtime_binding}" \
  --per-target-timeout "${timeout_s}"
