#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS is required}"
OUT="$PACK_ROOT/artifacts/stage4_$STAMP"
PIDS=()
mkdir -p "$OUT"

stop_group() {
  local pid="$1"
  if ! kill -0 -- "-$pid" 2>/dev/null; then return; fi
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 -- "-$pid" 2>/dev/null || break
    sleep 0.2
  done
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in {1..25}; do
      kill -0 -- "-$pid" 2>/dev/null || break
      sleep 0.2
    done
  fi
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do stop_group "$pid"; done
}
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"
cd "$WS"

colcon build \
  --packages-select sanitation_coverage sanitation_navigation sanitation_safety sanitation_tasks \
  --symlink-install --event-handlers console_direct+ \
  2>&1 | tee "$OUT/build.log"
set +u
source "$WS/install/setup.bash"
set -u
colcon test --packages-select sanitation_coverage \
  --event-handlers console_direct+ 2>&1 | tee "$OUT/test.log"
colcon test-result --all --verbose > "$OUT/test_results.txt"

setsid ros2 launch sanitation_bringup sim.launch.py \
  gui:=false headless_rendering:=true > "$OUT/simulation.log" 2>&1 &
SIM_PID=$!
PIDS+=("$SIM_PID")

setsid ros2 launch sanitation_navigation navigation.launch.py rviz:=false \
  > "$OUT/navigation.log" 2>&1 &
NAV_PID=$!
PIDS+=("$NAV_PID")

setsid ros2 launch sanitation_coverage coverage.launch.py \
  > "$OUT/coverage_server.log" 2>&1 &
COVERAGE_PID=$!
PIDS+=("$COVERAGE_PID")

setsid ros2 bag record -o "$OUT/coverage_bag" \
  /coverage_server/coverage_path /coverage_server/swaths \
  /odom /amcl_pose /cmd_vel /brush_enabled \
  > "$OUT/rosbag.log" 2>&1 &
BAG_PID=$!
PIDS+=("$BAG_PID")

ros2 run sanitation_coverage coverage_probe --ros-args \
  -p output_path:="$OUT/coverage_metrics.json" \
  -p path_output_path:="$OUT/coverage_path.json" \
  -p handoff_duration_sec:=20.0 \
  2>&1 | tee "$OUT/coverage_probe.log"
cp "$OUT/coverage_metrics.json" "$OUT/coverage_report.json"

stop_group "$BAG_PID"
PIDS=("$SIM_PID" "$NAV_PID" "$COVERAGE_PID")
ros2 node list | sort > "$OUT/nodes.txt"
ros2 topic list -t | sort > "$OUT/topics.txt"
ros2 action list -t | sort > "$OUT/actions.txt"
ros2 service list -t | sort > "$OUT/services.txt"

if ! kill -0 -- "-$SIM_PID" 2>/dev/null; then
  echo "ERROR: simulation exited before Stage 4 collection" >&2
  exit 4
fi

stop_group "$COVERAGE_PID"
stop_group "$NAV_PID"
stop_group "$SIM_PID"
PIDS=()
trap - EXIT

export STAGE4_OUT="$OUT"
export STAGE4_WS="$WS"
python3 - <<'PY'
import datetime as dt
import json
import os
from pathlib import Path

out = Path(os.environ["STAGE4_OUT"])
coverage = json.loads((out / "coverage_metrics.json").read_text(encoding="utf-8"))
bag = out / "coverage_bag" / "metadata.yaml"
summary = {
    "schema_version": 1,
    "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "success": bool(coverage["success"] and bag.is_file()),
    "workspace": os.environ["STAGE4_WS"],
    "coverage": coverage,
    "rosbag_metadata": str(bag.relative_to(out)) if bag.is_file() else None,
    "review_boundary": "Stage 4 complete; stop before perception training or J6 quantization.",
    "artifacts": sorted(path.name for path in out.iterdir()),
}
(out / "stage4_summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
if not summary["success"]:
    raise SystemExit(5)
PY

echo "$OUT"
