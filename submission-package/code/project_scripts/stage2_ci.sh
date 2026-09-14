#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:?SANITATION_WS must point to a passed Stage 1 workspace}"
OUT="$PACK_ROOT/artifacts/stage2_$STAMP"
LAUNCH_PID=""

mkdir -p "$OUT"

cleanup() {
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM "$LAUNCH_PID" 2>/dev/null || true
      for _ in {1..25}; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 0.2
      done
    fi
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"
cd "$WS"
colcon build \
  --packages-select-regex '^sanitation_' \
  --symlink-install \
  --event-handlers console_direct+ \
  2>&1 | tee "$OUT/build.log"

set +u
source "$WS/install/setup.bash"
set -u

colcon test \
  --packages-select sanitation_tasks \
  --event-handlers console_direct+ \
  2>&1 | tee "$OUT/test.log"
colcon test-result --all --verbose > "$OUT/test_results.txt"

vehicle_xacro="$WS/install/sanitation_vehicle_description/share/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro"
world_sdf="$WS/install/sanitation_worlds/share/sanitation_worlds/worlds/sanitation_test_world.sdf"
xacro "$vehicle_xacro" > "$OUT/sanitation_vehicle.urdf"
check_urdf "$OUT/sanitation_vehicle.urdf" | tee "$OUT/check_urdf.log"
gz sdf -k "$OUT/sanitation_vehicle.urdf" 2>&1 | tee "$OUT/check_vehicle_sdf.log"
gz sdf -k "$world_sdf" 2>&1 | tee "$OUT/check_world_sdf.log"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader \
  > "$OUT/gpu.txt"

ros2 launch sanitation_bringup sim.launch.py \
  gui:=false headless_rendering:=true \
  > "$OUT/simulation.log" 2>&1 &
LAUNCH_PID=$!

ros2 run sanitation_tasks sanitation_runtime_probe --ros-args \
  -p timeout_sec:=120.0 \
  -p motion_sec:=5.0 \
  -p output_path:="$OUT/runtime_probe.json" \
  2>&1 | tee "$OUT/runtime_probe.log"

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "ERROR: simulation launch exited before Stage 2 evidence collection completed" >&2
  exit 4
fi

ros2 node list | sort > "$OUT/nodes.txt"
ros2 topic list -t | sort > "$OUT/topics.txt"
gz topic -l | sort > "$OUT/gz_topics.txt"

cleanup
LAUNCH_PID=""
trap - EXIT

export STAGE2_OUT="$OUT"
export STAGE2_WS="$WS"
python3 - <<'PY'
import datetime as dt
import json
import os
from pathlib import Path

out = Path(os.environ["STAGE2_OUT"])
probe = json.loads((out / "runtime_probe.json").read_text(encoding="utf-8"))
summary = {
    "schema_version": 1,
    "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "success": bool(probe["success"]),
    "workspace": os.environ["STAGE2_WS"],
    "headless": True,
    "render_engine": "ogre2",
    "gpu_passthrough": True,
    "urdf_valid": True,
    "world_sdf_valid": True,
    "runtime_probe": probe,
    "artifacts": sorted(path.name for path in out.iterdir()),
}
(out / "stage2_summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
if not summary["success"]:
    raise SystemExit(5)
PY

echo "$OUT"
