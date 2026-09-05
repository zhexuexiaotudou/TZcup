#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/.work/final_functional_build/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_description_diag_${ROS_DOMAIN_ID}}"
log="${repo_root}/.work/robot_description_diag.log"
setsid ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false start_controllers:=false start_localization:=false \
  start_simulation_safety_inputs:=false high_bandwidth_sensor_runtime:=false \
  >"${log}" 2>&1 &
launch_pid=$!

cleanup() {
  local candidate_pid
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 3
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true

  # Gazebo can daemonize outside the ros2 launch process group.  Match the
  # unique partition from this diagnostic run before signaling any fallback
  # process, so unrelated Gazebo sessions are never touched.
  while IFS= read -r candidate_pid; do
    [[ -n "${candidate_pid}" ]] || continue
    kill -INT "${candidate_pid}" 2>/dev/null || true
  done < <(
    for proc_env in /proc/[0-9]*/environ; do
      [[ -r "${proc_env}" ]] || continue
      if tr '\0' '\n' <"${proc_env}" 2>/dev/null | grep -Fxq "GZ_PARTITION=${GZ_PARTITION}"; then
        basename "$(dirname "${proc_env}")"
      fi
    done
  )
  sleep 2
  while IFS= read -r candidate_pid; do
    [[ -n "${candidate_pid}" ]] || continue
    kill -TERM "${candidate_pid}" 2>/dev/null || true
  done < <(
    for proc_env in /proc/[0-9]*/environ; do
      [[ -r "${proc_env}" ]] || continue
      if tr '\0' '\n' <"${proc_env}" 2>/dev/null | grep -Fxq "GZ_PARTITION=${GZ_PARTITION}"; then
        basename "$(dirname "${proc_env}")"
      fi
    done
  )
}
trap cleanup EXIT

sleep 25
echo "LAUNCH_PID=${launch_pid}"
gz_pid="$(pgrep -f 'gz sim .*formal_vehicle_validation' | head -1 || true)"
echo "GZ_PID=${gz_pid}"
if [[ -n "${gz_pid}" ]]; then
  tr '\0' '\n' <"/proc/${gz_pid}/environ" | \
    grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_LOCALHOST_ONLY)=' || true
fi
ros2 node list --no-daemon || true
ros2 topic info -v /robot_description --no-daemon || true
