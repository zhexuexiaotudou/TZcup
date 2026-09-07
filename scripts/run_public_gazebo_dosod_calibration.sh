#!/usr/bin/env bash
# Public-train Gazebo camera calibration supervisor. It never drives a robot.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/run_formal_runtime_isolation.sh"
: "${PUBLIC_GAZEBO_CALIBRATION_PLAN:?approved public scene plan is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_OUTPUT:?new run root is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_LOCK:?shared Gazebo lock path is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC:?camera Image topic is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC:?camera-info topic is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC:?whole collection timeout is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA:?per-scene quota is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP:?frozen stage-1 setup is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP:?frozen runtime setup is required}"
: "${PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP:?frozen campus setup is required}"
: "${ROS_DOMAIN_ID:?isolated ROS domain is required}"

RUN_ROOT="$(realpath --no-symlinks -e "$PUBLIC_GAZEBO_CALIBRATION_OUTPUT")"
[[ "$RUN_ROOT" == "$(realpath -e "$PUBLIC_GAZEBO_CALIBRATION_OUTPUT")" && ! -L "$RUN_ROOT" && -d "$RUN_ROOT" ]] || {
  echo 'BLOCKED: run root must be a real non-link directory' >&2; exit 2;
}
[[ -z "$(find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" ]] || {
  echo 'BLOCKED: run root must be new and empty' >&2; exit 2;
}
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ && "$ROS_DOMAIN_ID" -ge 1 && "$ROS_DOMAIN_ID" -le 232 ]] || {
  echo 'BLOCKED: ROS domain must be in 1..232' >&2; exit 2;
}
[[ "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" =~ ^[1-9][0-9]*$ ]] || {
  echo 'BLOCKED: per-scene quota must be a positive integer' >&2; exit 2;
}
for required in "$PUBLIC_GAZEBO_CALIBRATION_PLAN" "$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP" "$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP" "$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP"; do
  [[ -f "$required" && ! -L "$required" ]] || { echo "BLOCKED: required regular file missing: $required" >&2; exit 2; }
done

# The established formal-campus child is the one and only owner of this exact
# shared lock.  Keeping the collector selector INACTIVE between child runs
# prevents any frame from an unrelated owner being attributed to a scene.

DATASET="$RUN_ROOT/dataset"
SELECTOR="$RUN_ROOT/scene_selector.json"
PROGRESS="$RUN_ROOT/collector_progress.json"
RECEIPT="$RUN_ROOT/public_gazebo_dosod_calibration_receipt.json"
COLLECTOR_PID=""
PRIMARY_ERROR=""

write_receipt() {
  local state="$1" code="$2" survivors="$3"
  python3 - "$RECEIPT" "$state" "$code" "$PRIMARY_ERROR" "$survivors" <<'PY'
import json, os, sys
from pathlib import Path
target = Path(sys.argv[1])
payload = {
    "report_id": "tzcup_public_gazebo_dosod_calibration_runner_v1",
    "status": sys.argv[2],
    "formal_passed": False,
    "classification": "NON_FORMAL",
    "exit_code": int(sys.argv[3]),
    "primary_error": sys.argv[4],
    "zero_survivor_check": sys.argv[5] == "true",
}
pending = target.with_name(f".{target.name}.pending.{os.getpid()}")
pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
os.replace(pending, target)
PY
}

cleanup() {
  local code=$? survivor=true
  set +e
  if [[ -n "$COLLECTOR_PID" ]] && kill -0 "$COLLECTOR_PID" 2>/dev/null; then
    kill -TERM "$COLLECTOR_PID" 2>/dev/null
    wait "$COLLECTOR_PID" 2>/dev/null
  fi
  if [[ -n "$COLLECTOR_PID" ]] && kill -0 "$COLLECTOR_PID" 2>/dev/null; then survivor=false; fi
  if (( code != 0 )) && [[ ! -e "$RECEIPT" ]]; then write_receipt "BLOCKED" "$code" "$survivor"; fi
  exit "$code"
}
trap 'PRIMARY_ERROR="${BASH_COMMAND}"' ERR
trap cleanup EXIT

# Preflight rejects non-public plans and checks the frozen four-class contract.
python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" \
  --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" \
  --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" >"$RUN_ROOT/preflight.json"

set +u
source /opt/ros/jazzy/setup.bash
source "$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP"
source "$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP"
source "$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP"
set -u

formal_runtime_configure_networking
python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" \
  --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" \
  --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" \
  --live-output "$DATASET" --image-topic "$PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC" \
  --camera-info-topic "$PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC" \
  --timeout-sec "$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC" --scene-selector "$SELECTOR" \
  --per-scene-quota "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" \
  --progress-output "$PROGRESS" >"$RUN_ROOT/collector.stdout" 2>"$RUN_ROOT/collector.stderr" &
COLLECTOR_PID=$!

scene_rows() {
  python3 - "$PUBLIC_GAZEBO_CALIBRATION_PLAN" <<'PY'
import json
import sys
plan = json.loads(open(sys.argv[1], encoding="utf-8").read())
for role in ("calibration", "holdout"):
    for scene in plan["scene_groups"][role]:
        print(f"{role}\t{scene}")
PY
}

scene_has_quota() {
  python3 - "$PROGRESS" "$1" "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" <<'PY'
import json
import sys
path, scene, quota = sys.argv[1], sys.argv[2], int(sys.argv[3])
try:
    value = json.load(open(path, encoding="utf-8"))
except (OSError, ValueError):
    raise SystemExit(1)
for key in ("calibration_scene_counts", "holdout_scene_counts"):
    if int(value.get(key, {}).get(scene, 0)) >= quota:
        raise SystemExit(0)
raise SystemExit(1)
PY
}

while IFS=$'\t' read -r role scene; do
  [[ "$scene" =~ ^map-([0-9]+)-mission-([0-9]+)$ ]] || {
    echo "BLOCKED: scene id must be map-N-mission-N: $scene" >&2; exit 2;
  }
  map_index="${BASH_REMATCH[1]}"; mission_index="${BASH_REMATCH[2]}"
  scene_root="$RUN_ROOT/scenes/$scene"
  [[ ! -e "$scene_root" && ! -L "$scene_root" ]] || { echo "BLOCKED: scene root already exists: $scene" >&2; exit 2; }
  mkdir -p "$scene_root"
  ros2 run sanitation_campus_scenario sanitation-campus-scenario generate \
    --config "$ROOT/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" \
    --profile formal --split train --map-index "$map_index" --mission-index "$mission_index" \
    --output "$scene_root/episode" >"$scene_root/materialize.stdout"
  manifest="$scene_root/episode/public/episode_manifest.json"
  [[ -f "$manifest" && ! -L "$manifest" ]] || { echo "BLOCKED: public manifest missing for $scene" >&2; exit 2; }
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" \
    --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" \
    --write-scene-selector --scene-selector "$SELECTOR" --scene-id "$scene" \
    --episode-manifest "$manifest" >"$scene_root/selector.json"

  TZCUP_REPOSITORY_ROOT="$ROOT" FORMAL_CAMPUS_STAGE1_SETUP="$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP" \
  FORMAL_CAMPUS_RUNTIME_SETUP="$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP" \
  FORMAL_CAMPUS_AGENT_SETUP="$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP" \
  FORMAL_CAMPUS_EPISODE_ROOT="$scene_root/episode" FORMAL_CAMPUS_OUTPUT_ROOT="$scene_root/runtime" \
  FORMAL_GAZEBO_LOCK_FILE="$PUBLIC_GAZEBO_CALIBRATION_LOCK" \
  GZ_PARTITION="tzcup_public_calibration_${ROS_DOMAIN_ID}_$$_${map_index}_${mission_index}" \
  bash "$ROOT/scripts/run_formal_campus_runtime.sh" >"$scene_root/campus.stdout" 2>&1

  drain_deadline=$((SECONDS + ${PUBLIC_GAZEBO_CALIBRATION_SCENE_DRAIN_TIMEOUT_SEC:-15}))
  until scene_has_quota "$scene"; do
    kill -0 "$COLLECTOR_PID" 2>/dev/null || { echo "BLOCKED: collector exited during $scene" >&2; exit 2; }
    (( SECONDS < drain_deadline )) || { echo "BLOCKED: fresh paired-frame quota not reached for $scene" >&2; exit 2; }
    sleep 0.2
  done
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" \
    --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" \
    --deactivate-scene-selector --scene-selector "$SELECTOR" >"$scene_root/selector_inactive.json"
done < <(scene_rows)

wait "$COLLECTOR_PID"; COLLECTOR_PID=""
[[ -f "$DATASET/calibration_manifest.json" && ! -L "$DATASET/calibration_manifest.json" ]] || {
  echo 'BLOCKED: collector returned without a frozen manifest' >&2; exit 2;
}
write_receipt "NON_FORMAL_CALIBRATION_FROZEN" 0 true
echo "NON_FORMAL: frozen public-train calibration dataset at $DATASET"
