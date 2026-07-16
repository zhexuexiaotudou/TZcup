#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -n "${SANITATION_BASE_WS:-}" ]]; then
  source "${SANITATION_BASE_WS}/install/setup.bash"
fi
source "${SANITATION_WS}/install/setup.bash"
set -u

artifact="${STAGE4V_OUT:-/work/artifacts/stage4v_runtime_smoke}"
mkdir -p "${artifact}"

ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:="${STAGE4V_SEED:-74}" \
  spawn_x:="${STAGE4V_SPAWN_X:--8.0}" spawn_y:="${STAGE4V_SPAWN_Y:-0.0}" \
  spawn_yaw:="${STAGE4V_SPAWN_YAW:-0.0}" \
  > "${artifact}/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT "${launch_pid}" 2>/dev/null || true
  for _ in $(seq 1 100); do
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      wait "${launch_pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -TERM "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

ready=0
for _ in $(seq 1 150); do
  topics=$(ros2 topic list 2>/dev/null || true)
  if grep -q '^/gnss/fix$' <<< "${topics}" && \
    grep -q '^/localization/fused_pose$' <<< "${topics}" && \
    grep -q '^/scan$' <<< "${topics}"
  then
    ready=1
    break
  fi
  sleep 1
done
echo "READY=${ready}"
ros2 topic list | sort > "${artifact}/topics.txt"

timeout 12 ros2 topic echo /gnss/fix --once > "${artifact}/gnss_fix.txt"
timeout 12 ros2 topic echo /localization/fused_pose --once > "${artifact}/fused_pose.txt"
timeout 12 ros2 topic echo /localization/fusion_diagnostics --once \
  > "${artifact}/fusion_diagnostics.txt"
timeout 12 ros2 topic echo /localization/refiner_diagnostics --once \
  > "${artifact}/refiner_diagnostics.txt" || true
timeout 15 ros2 run sanitation_tasks sanitation_tf_ownership_audit --ros-args \
  -p duration_s:=5.0 -p output_path:="${artifact}/tf_ownership.json" \
  > "${artifact}/tf_audit.log" 2>&1

test "${ready}" -eq 1
test -s "${artifact}/gnss_fix.txt"
test -s "${artifact}/fused_pose.txt"
test -s "${artifact}/fusion_diagnostics.txt"
test -s "${artifact}/tf_ownership.json"
