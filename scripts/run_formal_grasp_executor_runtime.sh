#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_ws="${FORMAL_MANIPULATION_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime/install}"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${runtime_ws}")/final_runtime_closure_manifest.json}"
output="${FORMAL_GRASP_EXECUTOR_OUTPUT:-${repo_root}/artifacts/formal_grasp_executor_runtime.json}"
runtime_binding="${FORMAL_GRASP_EXECUTOR_RUNTIME_BINDING:-${output}.runtime_binding.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
launch_log="${FORMAL_GRASP_EXECUTOR_LOG:-${output%.json}.launch.log}"
preembedded_world="${FORMAL_GRASP_PREEMBEDDED_WORLD:-${output%.json}.preembedded_grasp_world.sdf}"
preembedded_report="${FORMAL_GRASP_PREEMBEDDED_REPORT:-${output%.json}.preembedded_grasp_world.json}"
preembedded_vehicle_urdf="${FORMAL_GRASP_PREEMBEDDED_VEHICLE_URDF:-${output%.json}.preembedded_vehicle.urdf}"
preembedded_cube_urdf="${FORMAL_GRASP_PREEMBEDDED_CUBE_URDF:-${output%.json}.preembedded_cube.urdf}"
material="${FORMAL_MANIPULATION_MATERIAL:-PET}"
cube_name="${FORMAL_MANIPULATION_CUBE_NAME:-material_cube}"
timeout_s="${FORMAL_MANIPULATION_TIMEOUT_S:-180}"
startup_wait_s="${FORMAL_MANIPULATION_STARTUP_WAIT_S:-30}"

# A failed preflight must never leave retained canonical PASS evidence (or its
# runtime-identity sidecar) looking current.  Rotate both before ROS setup and
# before any frozen-runtime/session gate so every attempted acceptance starts
# from an absent canonical output pair.
for retained in "${output}" "${runtime_binding}" "${launch_log}" \
  "${preembedded_world}" "${preembedded_report}" \
  "${preembedded_vehicle_urdf}" "${preembedded_cube_urdf}"; do
  if [[ -e "${retained}" || -L "${retained}" ]]; then
    superseded="${retained}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"
    [[ ! -e "${superseded}" && ! -L "${superseded}" ]] || {
      echo "Refusing stale grasp evidence overwrite: ${superseded}" >&2
      exit 2
    }
    mv -- "${retained}" "${superseded}"
  fi
done

source /opt/ros/jazzy/setup.bash
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
formal_runtime_register_evidence_paths \
  "${output}" "${runtime_binding}" "${preembedded_world}" "${preembedded_report}" \
  "${preembedded_vehicle_urdf}" "${preembedded_cube_urdf}"

if [[ ! -f "${runtime_ws}/setup.bash" ]]; then
  echo "Missing frozen ROS runtime overlay: ${runtime_ws}/setup.bash" >&2
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

vehicle_share="$(ros2 pkg prefix --share sanitation_vehicle_description)"
manipulation_share="$(ros2 pkg prefix --share sanitation_manipulation)"
expected_vehicle_share="${runtime_ws}/share/sanitation_vehicle_description"
expected_manipulation_share="${runtime_ws}/share/sanitation_manipulation"
for package_share in "${vehicle_share}" "${manipulation_share}"; do
  [[ -d "${package_share}" ]] || {
    echo "missing frozen runtime package share: ${package_share}" >&2
    exit 2
  }
done
if [[ "$(cd -- "${vehicle_share}" && pwd -P)" != "$(cd -- "${expected_vehicle_share}" && pwd -P)" ]] || \
   [[ "$(cd -- "${manipulation_share}" && pwd -P)" != "$(cd -- "${expected_manipulation_share}" && pwd -P)" ]]; then
  echo "grasp packages resolve outside the frozen runtime install" >&2
  exit 2
fi
vehicle_model="${manipulation_share}/urdf/formal_manipulation_acceptance.urdf.xacro"
cube_model="${manipulation_share}/urdf/material_cube.urdf.xacro"
controller_config="${vehicle_share}/config/formal_vehicle_controllers.yaml"
source_world="${manipulation_share}/worlds/formal_cube_manipulation.sdf"
for required in "${vehicle_model}" "${cube_model}" "${controller_config}" "${source_world}"; do
  [[ -f "${required}" ]] || { echo "missing frozen grasp input: ${required}" >&2; exit 2; }
done
# The generated source URDFs and preembedded world are retained gate evidence,
# so their parent must exist before shell redirection starts xacro expansion.
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

xacro "${vehicle_model}" use_sim:=true bodywork_visible:=true \
  dry_accounting_mode:=physical_resident initial_estop_latched:=false \
  >"${preembedded_vehicle_urdf}"
xacro "${cube_model}" material:="${material}" >"${preembedded_cube_urdf}"
python3 "${repo_root}/scripts/prepare_formal_preembedded_sensor_world.py" \
  --source-world "${source_world}" --vehicle-urdf "${preembedded_vehicle_urdf}" \
  --additional-urdf "${preembedded_cube_urdf}" \
  --additional-model-pose "0.300 -0.950 0.017 0 0 0" \
  --controller-config "${controller_config}" --runtime-install-root "${runtime_ws}" \
  --output-world "${preembedded_world}" --report "${preembedded_report}"

if [[ -n "${FORMAL_MANIPULATION_UNDERLAY:-}" ]]; then
  # This direct final-acceptance runner admits exactly the frozen merged
  # overlay validated above.  An arbitrary developer underlay would make a
  # same-topic executor impossible to attribute to the recorded closure.
  echo "FORMAL_MANIPULATION_UNDERLAY is not permitted for formal acceptance" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-218}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_product_grasp_${ROS_DOMAIN_ID}_$$}"

simulation_pid=""
bridge_pid=""
executor_pid=""
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" \
    "${executor_pid}" "${bridge_pid}" "${simulation_pid}"
}
formal_runtime_install_traps cleanup

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_manipulation formal_cube_pick_place.launch.py \
  world:="${preembedded_world}" vehicle_model:="${vehicle_model}" cube_model:="${cube_model}" \
  spawn_vehicle:=false spawn_single_cube:=false \
  gui:=false material:="${material}" cube_name:="${cube_name}" >"${launch_log}" 2>&1 &
simulation_pid=$!
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/observed_status_json@std_msgs/msg/String[gz.msgs.StringMsg" \
  "/manipulation/gripper/dual_contact@std_msgs/msg/Bool[gz.msgs.Boolean" \
  >>"${launch_log}" 2>&1 &
bridge_pid=$!
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_manipulation formal_physical_grasp.launch.py \
  >>"${launch_log}" 2>&1 &
executor_pid=$!

# DetachableJoint is attached on construction.  Re-issue the evaluator-only
# initialization release after DDS and the product subscriber are both live;
# this is outside the task phase and does not move, delete or identify a cube.
sleep 13
gz topic -t /manipulation/grasp/detach -m gz.msgs.Empty -p "" \
  >>"${launch_log}" 2>&1

python3 "${repo_root}/scripts/validate_formal_grasp_executor_runtime.py" \
  --output "${output}" --startup-wait "${startup_wait_s}" --timeout "${timeout_s}" \
  --snapshot "${snapshot}" --session "${session}" --runtime-binding "${runtime_binding}" \
  --preembedded-report "${preembedded_report}" --preembedded-world "${preembedded_world}" \
  --preembedded-vehicle-urdf "${preembedded_vehicle_urdf}" \
  --preembedded-cube-urdf "${preembedded_cube_urdf}" \
  --preembedded-source-world "${source_world}"
