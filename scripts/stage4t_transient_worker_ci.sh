#!/usr/bin/env bash
set -euo pipefail
PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; WS="${SANITATION_WS:?SANITATION_WS required}"
OUT="${STAGE4T_OUT:?STAGE4T_OUT required}"; SHARD_INDEX="${SHARD_INDEX:?}"; SHARD_COUNT="${SHARD_COUNT:?}"
mkdir -p "$OUT/angular_rate_trials"; SIM_PID=""
stop_sim(){ [[ -n "$SIM_PID" ]] || return; kill -INT -- "-$SIM_PID" 2>/dev/null || true; for _ in {1..60}; do kill -0 "$SIM_PID" 2>/dev/null || { SIM_PID=""; return; }; sleep 0.1; done; kill -KILL -- "-$SIM_PID" 2>/dev/null || true; SIM_PID=""; }
trap stop_sim EXIT
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u

start_sim(){
  local key="$1"
  setsid ros2 launch sanitation_bringup motion_calibration.launch.py gui:=false headless_rendering:=true \
    drive_wheel_radius:=0.14 drive_wheel_separation:=1.22 operational_profile:=stress \
    max_linear_velocity:=0.45 max_angular_velocity:=0.60 random_seed:="$key" \
    > "$OUT/angular_rate_trials/sim_${key}.log" 2>&1 & SIM_PID=$!
  timeout 75s ros2 topic echo --once /ground_truth/odom nav_msgs/msg/Odometry > /dev/null
}
run_trial(){
  local id="$1" type="$2" thermal="$3" rate="$4" target="$5"
  [[ -s "$OUT/angular_rate_trials/$id.json" ]] && return
  timeout 120s ros2 run sanitation_tasks sanitation_transient_response_runner --ros-args \
    -p use_sim_time:=true -p trial_id:="$id" -p trial_type:="$type" -p thermal_state:="$thermal" \
    -p angular_rate:="$rate" -p target_heading_rad:="$target" \
    -p output_path:="$OUT/angular_rate_trials/$id.json" -p csv_path:="$OUT/angular_rate_trials/$id.csv" \
    > "$OUT/angular_rate_trials/$id.log" 2>&1
}

rates=(-0.60 -0.45 -0.35 -0.25 -0.10 0.10 0.25 0.35 0.45 0.60)
job=0
for rate in "${rates[@]}"; do
  rate_label="${rate/-/m}"; rate_label="${rate_label/./p}"
  for repeat in {0..9}; do
    if (( job % SHARD_COUNT == SHARD_INDEX )); then
      key=$((1000 + job)); cold="fixed_cold_${rate_label}_r$repeat"; hot="fixed_hot_${rate_label}_r$repeat"
      if [[ ! -s "$OUT/angular_rate_trials/$cold.json" || ! -s "$OUT/angular_rate_trials/$hot.json" ]]; then
        start_sim "$key"; run_trial "$cold" fixed_time cold "$rate" 0.0; run_trial "$hot" fixed_time hot "$rate" 0.0; stop_sim
      fi
    fi
    job=$((job + 1))
  done
done

headings=(-6.283185307179586 -3.141592653589793 -1.5707963267948966 1.5707963267948966 3.141592653589793 6.283185307179586)
for target in "${headings[@]}"; do
  target_label="${target/-/m}"; target_label="${target_label/./p}"
  for repeat in {0..9}; do
    if (( job % SHARD_COUNT == SHARD_INDEX )); then
      key=$((2000 + job)); sign_rate=0.35; [[ "$target" == -* ]] && sign_rate=-0.35
      cold="heading_cold_${target_label}_r$repeat"; hot="heading_hot_${target_label}_r$repeat"
      if [[ ! -s "$OUT/angular_rate_trials/$cold.json" || ! -s "$OUT/angular_rate_trials/$hot.json" ]]; then
        start_sim "$key"; run_trial "$cold" closed_loop_heading cold "$sign_rate" "$target"; run_trial "$hot" closed_loop_heading hot "$sign_rate" "$target"; stop_sim
      fi
    fi
    job=$((job + 1))
  done
done
echo "shard $SHARD_INDEX complete"
