#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS must point to the reusable Stage 1 workspace}"
OUT="$PACK_ROOT/artifacts/stage4s_friction_$STAMP"
LAUNCH_PID=""
mkdir -p "$OUT/cases"

stop_launch() {
  if [[ -z "$LAUNCH_PID" ]] || ! kill -0 "$LAUNCH_PID" 2>/dev/null; then return 0; fi
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
  for _ in {1..80}; do kill -0 "$LAUNCH_PID" 2>/dev/null || return 0; sleep 0.1; done
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  for _ in {1..50}; do kill -0 "$LAUNCH_PID" 2>/dev/null || return 0; sleep 0.1; done
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
  local label="$1" mu_lat="$2" slip_enabled="$3" slip_lat="$4"
  local output="$OUT/cases/${label}.json"
  setsid ros2 launch sanitation_bringup motion_calibration.launch.py \
    gui:=false headless_rendering:=true drive_wheel_radius:=0.14 drive_wheel_separation:=1.22 \
    wheel_mu_longitudinal:=1.0 wheel_mu_lateral:="$mu_lat" \
    enable_wheel_slip:="$slip_enabled" slip_compliance_longitudinal:=0.0 \
    slip_compliance_lateral:="$slip_lat" > "$OUT/${label}_simulation.log" 2>&1 &
  LAUNCH_PID=$!
  timeout 60s ros2 topic echo --once /ground_truth/model_odom_raw nav_msgs/msg/Odometry \
    > "$OUT/${label}_first_odom.txt"
  timeout 70s ros2 run sanitation_tasks sanitation_motion_fit_probe --ros-args \
    -p use_sim_time:=true -p mode:=friction -p output_path:="$output" \
    -p drive_wheel_radius:=0.14 -p drive_wheel_separation:=1.22 \
    -p wheel_mu_longitudinal:=1.0 -p wheel_mu_lateral:="$mu_lat" \
    -p enable_wheel_slip:="$slip_enabled" -p slip_compliance_longitudinal:=0.0 \
    -p slip_compliance_lateral:="$slip_lat" > "$OUT/${label}_probe.log" 2>&1
  stop_launch
  LAUNCH_PID=""
  sleep 2
}

run_case mu_lat_1p00 1.00 false 0.0
run_case mu_lat_0p80 0.80 false 0.0
run_case mu_lat_0p60 0.60 false 0.0
run_case mu_lat_0p40 0.40 false 0.0
run_case wheelslip_lat_0p01 1.00 true 0.01

python3 - "$OUT" <<'PY'
import json, sys, yaml
from pathlib import Path
root = Path(sys.argv[1])
cases = [json.loads(p.read_text()) for p in sorted((root / "cases").glob("*.json"))]
passing = [c for c in cases if c["body_tracking_pass"]]
selected = min(passing or cases, key=lambda c: c["objective_body_yaw_error_deg"])
report = {
    "schema_version": 1,
    "eligible": True,
    "executed": True,
    "selection_objective": "minimum high-speed +360 body yaw error; final 13-segment validation required",
    "grid_complete": len(cases) == 5,
    "cases": cases,
    "selected": selected["contact_parameters"],
    "selected_probe": selected,
}
(root / "friction_slip_scan.json").write_text(json.dumps(report, indent=2) + "\n")
dynamics = {
    "physical_wheel_radius": 0.14, "physical_track_width": 0.80,
    "drive_wheel_radius": 0.14, "drive_wheel_separation": 1.22,
    **selected["contact_parameters"],
}
(root / "selected_vehicle_dynamics.yaml").write_text(
    yaml.safe_dump({"vehicle_dynamics": dynamics}, sort_keys=False)
)
print(json.dumps({"selected": dynamics, "error_deg": selected["objective_body_yaw_error_deg"]}))
PY
echo "$OUT"
