#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime}"
output="${1:-${repo_root}/artifacts/formal_vehicle_safety/whole_vehicle_actuator_interlock.json}"
launch_log="${WHOLE_VEHICLE_SAFETY_LAUNCH_LOG:-${repo_root}/artifacts/formal_vehicle_safety/launch.log}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
runtime_binding="${output}.runtime_binding.json"
formal_runtime_register_evidence_paths "${output}" "${runtime_binding}" "${launch_log}"

if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then
  echo "Missing built ROS workspace: ${runtime_ws}/install/setup.bash" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-220}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_safety_${ROS_DOMAIN_ID}_$$}"
if [[ -e "${output}" || -e "${launch_log}" || -e "${runtime_binding}" ]]; then
  echo "Refusing stale whole-vehicle interlock evidence; use fresh output/log paths" >&2
  exit 2
fi
mkdir -p "$(dirname "${output}")" "$(dirname "${launch_log}")"

python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
  --check --output "${snapshot}"
python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${runtime_ws}/install" \
  --closure-manifest "${closure_manifest}" --session "${session}" \
  --snapshot "${snapshot}" --output "${runtime_binding}"
source "${runtime_ws}/install/setup.bash"
set -u

launch_pid=""
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true start_controllers:=true \
  enable_safety_manager:=true start_simulation_safety_inputs:=false \
  start_power_system_simulators:=false simulation_initial_estop_active:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

ready="false"
for _ in $(seq 1 90); do
  # Every formal step intentionally reuses one bounded ROS domain.  Bypass the
  # long-lived ROS CLI daemon here so discovery cannot be satisfied or delayed
  # by graph cache entries from the previous, already-terminated Gazebo step.
  if timeout 20s ros2 node list --no-daemon --spin-time 3.0 2>/dev/null \
      | grep -qx '/whole_vehicle_safety_manager' \
    && timeout 20s ros2 service list -t --no-daemon --spin-time 3.0 2>/dev/null \
      | grep -Fxq '/controller_manager/list_controllers [controller_manager_msgs/srv/ListControllers]' \
    && timeout 20s ros2 topic echo /safety/actuators_enabled std_msgs/msg/Bool \
      --once --no-daemon --spin-time 3.0 --timeout 4 >/dev/null 2>&1 \
    && timeout 20s ros2 topic echo /joint_states sensor_msgs/msg/JointState \
      --once --no-daemon --spin-time 3.0 --timeout 4 >/dev/null 2>&1; then
    ready="true"
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Formal vehicle launch exited before the safety manager was ready" >&2
    exit 1
  fi
  sleep 1
done
if [[ "${ready}" != "true" ]]; then
  echo "Timed out waiting for the current no-daemon whole-vehicle safety graph" >&2
  exit 1
fi

python3 "${repo_root}/scripts/validate_whole_vehicle_actuator_interlock.py" \
  --output "${output}" --snapshot "${snapshot}" --session "${session}" \
  --runtime-binding "${runtime_binding}"

launch_state="$(ps -o stat= -p "${launch_pid}" 2>/dev/null || true)"
if ! kill -0 "${launch_pid}" 2>/dev/null \
  || [[ -z "${launch_state}" || "${launch_state}" == *Z* ]]; then
  echo "Formal vehicle launch exited before safety acceptance completed" >&2
  exit 1
fi
