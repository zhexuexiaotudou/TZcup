#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS must point to the reusable Stage 1 workspace}"
OUT="$PACK_ROOT/artifacts/stage4s_fit_$STAMP"
LAUNCH_PID=""
mkdir -p "$OUT/radius_grid" "$OUT/separation_grid"

if [[ -n "${STAGE4S_BASE_FIT_DIR:-}" ]]; then
  cp "${STAGE4S_BASE_FIT_DIR}/radius_grid/"*.json "$OUT/radius_grid/"
  cp "${STAGE4S_BASE_FIT_DIR}/separation_grid/"*.json "$OUT/separation_grid/"
fi

stop_launch() {
  if [[ -z "$LAUNCH_PID" ]] || ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    return 0
  fi
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
  for _ in {1..80}; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || kill -KILL "$LAUNCH_PID" 2>/dev/null || true
}
trap stop_launch EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"
cd "$WS"
colcon build --packages-select-regex '^sanitation_' --symlink-install \
  --event-handlers console_direct+ 2>&1 | tee "$OUT/build.log"
set +u
source "$WS/install/setup.bash"
set -u

run_case() {
  local mode="$1"
  local radius="$2"
  local separation="$3"
  local output="$4"
  local label="$5"
  setsid ros2 launch sanitation_bringup motion_calibration.launch.py \
    gui:=false headless_rendering:=true \
    drive_wheel_radius:="$radius" drive_wheel_separation:="$separation" \
    > "$OUT/${label}_simulation.log" 2>&1 &
  LAUNCH_PID=$!
  timeout 60s ros2 topic echo --once /ground_truth/model_odom_raw nav_msgs/msg/Odometry \
    > "$OUT/${label}_first_odom.txt"
  timeout 100s ros2 run sanitation_tasks sanitation_motion_fit_probe --ros-args \
    -p use_sim_time:=true \
    -p mode:="$mode" -p output_path:="$output" \
    -p drive_wheel_radius:="$radius" -p drive_wheel_separation:="$separation" \
    > "$OUT/${label}_probe.log" 2>&1
  stop_launch
  LAUNCH_PID=""
  sleep 2
}

if [[ -z "${STAGE4S_BASE_FIT_DIR:-}" ]]; then
  for radius in 0.132 0.134 0.136 0.138 0.140; do
    key="${radius/./p}"
    run_case radius "$radius" 0.80 "$OUT/radius_grid/${key}.json" "radius_${key}"
    if [[ "${STAGE4S_FIT_SMOKE_ONLY:-false}" == "true" ]]; then
      echo "$OUT"
      exit 0
    fi
  done
fi

SELECTED_RADIUS="$(python3 - "$OUT/radius_grid" <<'PY'
import json, sys
from pathlib import Path
cases = [json.loads(p.read_text()) for p in Path(sys.argv[1]).glob('*.json')]
print(min(cases, key=lambda c: c['body_distance_error_pct'])['drive_wheel_radius'])
PY
)"

for separation in ${STAGE4S_SEPARATION_CANDIDATES:-1.15 1.20 1.25 1.30 1.35}; do
  key="${separation/./p}"
  run_case separation "$SELECTED_RADIUS" "$separation" \
    "$OUT/separation_grid/${key}.json" "separation_${key}"
done

python3 "$PACK_ROOT/scripts/stage4s_parameter_fit.py" "$OUT" \
  | tee "$OUT/selected_parameters.txt"
echo "$OUT"
