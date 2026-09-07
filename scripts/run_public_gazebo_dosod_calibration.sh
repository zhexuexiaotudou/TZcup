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
: "${PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC:?whole public-calibration wall deadline is required}"
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
[[ "$PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ ]] || {
  echo 'BLOCKED: public-calibration wall deadline must be a positive integer' >&2; exit 2;
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
COLLECTOR_PGID=""
ACTIVE_CAMPUS_PID=""
ACTIVE_GZ_PARTITION=""
ACTIVE_CAMPUS_SESSION_TOKEN=""
ACTIVE_MATERIALIZER_PID=""
ACTIVE_MATERIALIZER_PGID=""
PRIMARY_ERROR=""
DEADLINE_EPOCH=$((SECONDS + PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC))

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
    "wall_deadline_seconds": int(os.environ["PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC"]),
    "zero_survivor_check": sys.argv[5] == "true",
}
pending = target.with_name(f".{target.name}.pending.{os.getpid()}")
pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
os.replace(pending, target)
PY
}

stop_exact_group() {
  local pgid="$1" signal attempt
  [[ "$pgid" =~ ^[0-9]+$ ]] || return 0
  for signal in TERM KILL; do
    kill -0 -- "-$pgid" 2>/dev/null || break
    kill -"$signal" -- "-$pgid" 2>/dev/null || true
    for attempt in {1..40}; do
      kill -0 -- "-$pgid" 2>/dev/null || break
      sleep 0.25
    done
  done
  ! kill -0 -- "-$pgid" 2>/dev/null
}

remaining_seconds() {
  local remaining=$((DEADLINE_EPOCH - SECONDS))
  (( remaining > 0 )) || return 124
  printf '%s\n' "$remaining"
}

wait_until_deadline() {
  local pid="$1" status
  while kill -0 "$pid" 2>/dev/null; do
    remaining_seconds >/dev/null || return 124
    sleep 0.2
  done
  set +e
  wait "$pid"
  status=$?
  set -e
  return "$status"
}

cleanup() {
  local code=$? zero_survivor=true
  set +e
  if [[ -n "$ACTIVE_CAMPUS_PID" ]]; then
    formal_runtime_cleanup_groups "$ACTIVE_GZ_PARTITION" "$ACTIVE_CAMPUS_PID" || zero_survivor=false
    stop_exact_group "$ACTIVE_CAMPUS_PID" || zero_survivor=false
    wait "$ACTIVE_CAMPUS_PID" 2>/dev/null || true
    ACTIVE_CAMPUS_PID=""
    ACTIVE_CAMPUS_SESSION_TOKEN=""
  fi
  stop_exact_group "$ACTIVE_MATERIALIZER_PGID" || zero_survivor=false
  [[ -n "$ACTIVE_MATERIALIZER_PID" ]] && wait "$ACTIVE_MATERIALIZER_PID" 2>/dev/null || true
  stop_exact_group "$COLLECTOR_PGID" || zero_survivor=false
  [[ -n "$COLLECTOR_PID" ]] && wait "$COLLECTOR_PID" 2>/dev/null || true
  formal_runtime_stop_memory_watchdog || zero_survivor=false
  if (( code != 0 )) && [[ ! -e "$RECEIPT" ]]; then write_receipt "BLOCKED" "$code" "$zero_survivor"; fi
  [[ "$zero_survivor" == true ]]
}
trap 'PRIMARY_ERROR="${BASH_COMMAND}"' ERR
formal_runtime_install_traps cleanup
formal_runtime_register_evidence_paths "$RECEIPT" "$RUN_ROOT/memory_preflight.json" "$RUN_ROOT/memory_preflight.log"

# This is the first owner write after the fresh-root admission succeeds.
formal_runtime_memory_preflight "$RUN_ROOT/memory_preflight"

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
setsid python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" \
  --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" \
  --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" \
  --live-output "$DATASET" --image-topic "$PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC" \
  --camera-info-topic "$PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC" \
  --timeout-sec "$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC" --scene-selector "$SELECTOR" \
  --per-scene-quota "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" \
  --progress-output "$PROGRESS" >"$RUN_ROOT/collector.stdout" 2>"$RUN_ROOT/collector.stderr" &
COLLECTOR_PID=$!
COLLECTOR_PGID="$COLLECTOR_PID"

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
  setsid ros2 run sanitation_campus_scenario sanitation-campus-scenario generate \
    --config "$ROOT/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" \
    --profile formal --split train --map-index "$map_index" --mission-index "$mission_index" \
    --output "$scene_root/episode" >"$scene_root/materialize.stdout" &
  ACTIVE_MATERIALIZER_PID=$!
  ACTIVE_MATERIALIZER_PGID="$ACTIVE_MATERIALIZER_PID"
  wait_until_deadline "$ACTIVE_MATERIALIZER_PID"
  stop_exact_group "$ACTIVE_MATERIALIZER_PGID"
  ACTIVE_MATERIALIZER_PID=""; ACTIVE_MATERIALIZER_PGID=""
  manifest="$scene_root/episode/public/episode_manifest.json"
  [[ -f "$manifest" && ! -L "$manifest" ]] || { echo "BLOCKED: public manifest missing for $scene" >&2; exit 2; }
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" \
    --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" \
    --write-scene-selector --scene-selector "$SELECTOR" --scene-id "$scene" \
    --episode-manifest "$manifest" >"$scene_root/selector.json"

  ACTIVE_GZ_PARTITION="tzcup_public_calibration_${ROS_DOMAIN_ID}_$$_${map_index}_${mission_index}"
  ACTIVE_CAMPUS_SESSION_TOKEN="$(od -An -N32 -tx1 /dev/urandom | tr -d '[:space:]')"
  if [[ ! "$ACTIVE_CAMPUS_SESSION_TOKEN" =~ ^[0-9a-f]{64}$ ]]; then
    echo "failed to create the campus session attestation token" >&2
    exit 2
  fi
  TZCUP_REPOSITORY_ROOT="$ROOT" FORMAL_CAMPUS_STAGE1_SETUP="$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP" \
  FORMAL_CAMPUS_RUNTIME_SETUP="$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP" \
  FORMAL_CAMPUS_AGENT_SETUP="$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP" \
  FORMAL_CAMPUS_EPISODE_ROOT="$scene_root/episode" FORMAL_CAMPUS_OUTPUT_ROOT="$scene_root/runtime" \
  FORMAL_GAZEBO_LOCK_FILE="$PUBLIC_GAZEBO_CALIBRATION_LOCK" \
  GZ_PARTITION="$ACTIVE_GZ_PARTITION" \
  FORMAL_ORCHESTRATED_STEP_SESSION=1 \
  FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN="$ACTIVE_CAMPUS_SESSION_TOKEN" \
  setsid bash "$ROOT/scripts/run_formal_campus_runtime.sh" >"$scene_root/campus.stdout" 2>&1 &
  ACTIVE_CAMPUS_PID=$!
  formal_runtime_register_evidence_paths "$scene_root/campus_memory_watchdog.json" "$scene_root/campus_memory_watchdog.log"
  formal_runtime_start_memory_watchdog "$ACTIVE_CAMPUS_PID" "$scene_root/campus_memory_watchdog"
  wait_until_deadline "$ACTIVE_CAMPUS_PID"
  formal_runtime_stop_memory_watchdog
  stop_exact_group "$ACTIVE_CAMPUS_PID"
  ACTIVE_CAMPUS_PID=""
  ACTIVE_CAMPUS_SESSION_TOKEN=""

  drain_deadline=$((SECONDS + ${PUBLIC_GAZEBO_CALIBRATION_SCENE_DRAIN_TIMEOUT_SEC:-15}))
  until scene_has_quota "$scene"; do
    kill -0 "$COLLECTOR_PID" 2>/dev/null || { echo "BLOCKED: collector exited during $scene" >&2; exit 2; }
    (( SECONDS < drain_deadline )) || { echo "BLOCKED: fresh paired-frame quota not reached for $scene" >&2; exit 2; }
    remaining_seconds >/dev/null || { echo 'BLOCKED: public-calibration wall deadline exceeded' >&2; exit 124; }
    sleep 0.2
  done
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" \
    --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" \
    --deactivate-scene-selector --scene-selector "$SELECTOR" >"$scene_root/selector_inactive.json"
  ACTIVE_GZ_PARTITION=""
done < <(scene_rows)

wait_until_deadline "$COLLECTOR_PID"
stop_exact_group "$COLLECTOR_PGID"
COLLECTOR_PID=""; COLLECTOR_PGID=""
[[ -f "$DATASET/calibration_manifest.json" && ! -L "$DATASET/calibration_manifest.json" ]] || {
  echo 'BLOCKED: collector returned without a frozen manifest' >&2; exit 2;
}
write_receipt "NON_FORMAL_CALIBRATION_FROZEN" 0 true
echo "NON_FORMAL: frozen public-train calibration dataset at $DATASET"
