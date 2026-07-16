#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -n "${SANITATION_BASE_WS:-}" ]]; then
  source "${SANITATION_BASE_WS}/install/setup.bash"
fi
source "${SANITATION_WS:?SANITATION_WS required}/install/setup.bash"
set -u

seed="${STAGE4V_SEED:-0}"
lane="${STAGE4V_LANE:-hybrid_rtk_scan_imu_wheel}"
out="${STAGE4V_OUT:?STAGE4V_OUT required}/seed_${seed}"
mkdir -p "${out}"

case "${lane}" in
  rtk_imu_wheel)
    gnss_profile=rtk_fixed
    fusion_mode=rtk_imu_wheel
    enable_scan_refiner=false
    ;;
  hybrid_rtk_scan_imu_wheel)
    gnss_profile=rtk_fixed
    fusion_mode=hybrid_rtk_scan_imu_wheel
    enable_scan_refiner=true
    ;;
  gnss_denied_scan_fallback)
    gnss_profile=gnss_denied
    fusion_mode=gnss_denied_scan_fallback
    enable_scan_refiner=true
    ;;
  *)
    echo "Unsupported Stage4V lane: ${lane}" >&2
    exit 3
    ;;
esac

map_root="${SANITATION_WS}/install/sanitation_navigation/share/sanitation_navigation/maps"
map_yaml="${map_root}/stage4v_surveyed_reference.yaml"
calibration="${map_root}/stage4v_map_frame_calibration.yaml"
keepout_map="${map_root}/stage4v_filters/keepout_mask.yaml"
speed_map="${map_root}/stage4v_filters/speed_mask.yaml"
nav_params="${SANITATION_WS}/install/sanitation_navigation/share/sanitation_navigation/config/nav2.yaml"

pids=()
stop_group() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return
  kill -INT -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 100); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  for pid in "${pids[@]:-}"; do
    stop_group "${pid}"
  done
}
trap cleanup EXIT

setsid ros2 launch sanitation_bringup stage4v_localization.launch.py \
  gui:=false random_seed:="${seed}" gnss_profile:="${gnss_profile}" \
  fusion_mode:="${fusion_mode}" enable_scan_refiner:="${enable_scan_refiner}" \
  > "${out}/localization_stack.log" 2>&1 &
stack_pid=$!
pids+=("${stack_pid}")

ready=0
for _ in $(seq 1 180); do
  topics=$(ros2 topic list 2>/dev/null || true)
  if grep -q '^/localization/fused_pose$' <<< "${topics}" && \
    grep -q '^/ground_truth/odom$' <<< "${topics}"
  then
    ready=1
    break
  fi
  sleep 1
done
test "${ready}" -eq 1
timeout 30 ros2 topic echo /localization/fused_pose --once \
  > "${out}/first_fused_pose.txt"

setsid ros2 launch sanitation_navigation navigation.launch.py \
  rviz:=false localization_backend:=external params_file:="${nav_params}" \
  map_file:="${map_yaml}" keepout_map:="${keepout_map}" speed_map:="${speed_map}" \
  operational_profile:=localization_coverage max_linear_velocity:=0.45 \
  max_angular_velocity:=0.35 \
  > "${out}/navigation_stack.log" 2>&1 &
navigation_pid=$!
pids+=("${navigation_pid}")

setsid ros2 run sanitation_tasks sanitation_localization_evaluator --ros-args \
  -p use_sim_time:=true -p duration_sec:=120.0 -p rmse_threshold_m:=0.05 \
  -p map_frame_calibration:="${calibration}" \
  -p localization_backend:="${lane}" \
  -p estimate_topic:=/localization/fused_pose \
  -p require_particle_instrumentation:=false \
  -p output_path:="${out}/localization_report.json" \
  -p csv_path:="${out}/localization_trajectory.csv" \
  -p plot_path:="${out}/localization_error.png" \
  > "${out}/evaluator.log" 2>&1 &
evaluator_pid=$!
pids+=("${evaluator_pid}")

setsid ros2 run sanitation_tasks sanitation_tf_ownership_audit --ros-args \
  -p duration_s:=120.0 -p output_path:="${out}/tf_ownership.json" \
  > "${out}/tf_audit.log" 2>&1 &
tf_audit_pid=$!
pids+=("${tf_audit_pid}")

set +e
timeout 300 ros2 run sanitation_tasks sanitation_navigation_probe --ros-args \
  -p use_sim_time:=true -p timeout_sec:=240.0 \
  -p output_path:="${out}/navigation_probe.json" \
  -p initial_pose_x:=0.0 -p initial_pose_y:=0.0 -p initial_pose_yaw:=0.0 \
  > "${out}/navigation_probe.log" 2>&1
navigation_code=$?
wait "${evaluator_pid}"
evaluator_code=$?
wait "${tf_audit_pid}"
tf_audit_code=$?
set -e

timeout 15 ros2 topic echo /localization/fusion_diagnostics --once \
  > "${out}/fusion_diagnostics.txt" || true
timeout 15 ros2 topic echo /localization/refiner_diagnostics --once \
  > "${out}/refiner_diagnostics.txt" || true

python3 - "${out}" "${lane}" "${seed}" "${navigation_code}" \
  "${evaluator_code}" "${tf_audit_code}" <<'PY'
import json
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
lane = sys.argv[2]
seed = int(sys.argv[3])
navigation_code, evaluator_code, tf_audit_code = map(int, sys.argv[4:7])


def load(name):
    path = root / name
    return json.loads(path.read_text(encoding='utf-8')) if path.exists() else None


def load_refiner_diagnostics():
    path = root / 'refiner_diagnostics.txt'
    text = path.read_text(encoding='utf-8') if path.exists() else ''

    def value(key):
        match = re.search(
            rf"key:\s*'?{re.escape(key)}'?\s+value:\s*'?([^'\s]+)'?", text
        )
        return match.group(1) if match else None

    accepted_count = value('accepted_count')
    attempt_count = value('attempt_count')
    return {
        'available': bool(text.strip()),
        'accepted_ever': value('accepted_ever') in {'1', 'true', 'True'},
        'accepted_count': int(accepted_count) if accepted_count else 0,
        'attempt_count': int(attempt_count) if attempt_count else 0,
        'last_reason': value('reason'),
    }


localization = load('localization_report.json')
navigation = load('navigation_probe.json')
tf_ownership = load('tf_ownership.json')
summary = {
    'schema_version': 1,
    'lane': lane,
    'seed': seed,
    'localization': localization,
    'navigation': navigation,
    'tf_ownership': tf_ownership,
    'scan_refiner': load_refiner_diagnostics(),
    'exit_codes': {
        'navigation': navigation_code,
        'evaluator': evaluator_code,
        'tf_audit': tf_audit_code,
    },
    'ground_truth_used_for_control': False,
    'ground_truth_direct_fusion': False,
}
(root / 'trial_summary.json').write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
)
PY

test "${navigation_code}" -eq 0
test "${evaluator_code}" -eq 0
test "${tf_audit_code}" -eq 0
