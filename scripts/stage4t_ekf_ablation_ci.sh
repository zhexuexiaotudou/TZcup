#!/usr/bin/env bash
set -euo pipefail
PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"; WS="${SANITATION_WS:?SANITATION_WS required}"
OUT="${STAGE4T_OUT:-$PACK_ROOT/artifacts/stage4t_ekf_ablation_$STAMP}"
REPEATS="${TZCUP_EKF_REPEATS:-1}"; mkdir -p "$OUT/ekf_trials"
SIM_PID=""
stop_sim(){ [[ -n "$SIM_PID" ]] || return 0; kill -INT -- "-$SIM_PID" 2>/dev/null || true; for _ in {1..80}; do kill -0 "$SIM_PID" 2>/dev/null || { SIM_PID=""; return; }; sleep 0.1; done; kill -KILL -- "-$SIM_PID" 2>/dev/null || true; SIM_PID=""; }
trap stop_sim EXIT
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
if [[ "${SKIP_BUILD:-false}" != true ]]; then
  rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"; cd "$WS"
  colcon build --packages-select-regex '^sanitation_' --symlink-install --event-handlers console_direct+ > "$OUT/build.log" 2>&1
fi
set +u; source "$WS/install/setup.bash"; set -u

declare -A CONFIGS=(
  [A]=ekf_a_wheel_vx_imu_vyaw.yaml
  [B]=ekf_b_wheel_vx_imu_yaw_vyaw.yaml
  [C]=ekf_c_current_wheel_twist_imu_vyaw.yaml
  [D]=ekf_d_wheel_vx_vyaw_no_imu.yaml
)
candidates=(A B C D)
[[ -z "${CANDIDATE_ONLY:-}" ]] || candidates=("$CANDIDATE_ONLY")
for candidate in "${candidates[@]}"; do
  for ((seed=0; seed<REPEATS; seed++)); do
    trial="$OUT/ekf_trials/$candidate/seed_$seed"; mkdir -p "$trial"
    if [[ -s "$trial/motion_calibration_report.json" ]]; then
      echo "reuse completed $candidate seed $seed"
      continue
    fi
    config="$WS/install/sanitation_bringup/share/sanitation_bringup/config/${CONFIGS[$candidate]}"
    setsid ros2 launch sanitation_bringup motion_calibration.launch.py gui:=false headless_rendering:=true \
      drive_wheel_radius:=0.14 drive_wheel_separation:=1.22 operational_profile:=stress \
      max_linear_velocity:=0.45 max_angular_velocity:=0.60 random_seed:="$seed" ekf_config:="$config" \
      > "$trial/simulation.log" 2>&1 &
    SIM_PID=$!
    timeout 75s ros2 topic echo --once /ground_truth/odom nav_msgs/msg/Odometry > "$trial/first_truth.txt"
    timeout 900s ros2 run sanitation_tasks sanitation_motion_calibration_runner --ros-args \
      -p use_sim_time:=true -p output_dir:="$trial" -p drive_wheel_radius:=0.14 \
      -p drive_wheel_separation:=1.22 -p calibration_label:="ekf_$candidate" \
      -p schedule_profile:=stage4t_ablation -p random_seed:="$seed" \
      > "$trial/runner.log" 2>&1
    stop_sim
  done
done
if [[ "${SKIP_AGGREGATE:-false}" != true ]]; then
  python3 "$PACK_ROOT/scripts/stage4t_ekf_ablation.py" "$OUT" --required-repeats 5 \
    --config-dir "$PACK_ROOT/starter_ws/src/sanitation_bringup/config"
fi
echo "$OUT"
