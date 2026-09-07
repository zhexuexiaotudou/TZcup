#!/usr/bin/env bash
# One exact public RGB/CameraInfo readiness pair.  It is never a dataset run.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/run_formal_runtime_isolation.sh"

: "${PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT:?fresh output root is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK:?exclusive Gazebo lock is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC:?whole-run deadline is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP:?frozen stage setup is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP:?frozen runtime setup is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP:?frozen campus setup is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD:?public world is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST:?public episode manifest is required}"
: "${ROS_DOMAIN_ID:?isolated ROS domain is required}"

RUN_ROOT="$(realpath --no-symlinks -e "$PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT")"
[[ -d "$RUN_ROOT" && ! -L "$RUN_ROOT" && -z "$(find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" ]] || {
  echo 'BLOCKED: output root must be a fresh empty regular directory' >&2; exit 2;
}
[[ "$PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ ]] || {
  echo 'BLOCKED: whole-run deadline must be a positive integer' >&2; exit 2;
}
for required in "$PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP" \
  "$PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD" \
  "$PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST"; do
  [[ -f "$required" && ! -L "$required" ]] || { echo "BLOCKED: regular input missing: $required" >&2; exit 2; }
done

READINESS="${PUBLIC_GAZEBO_CAMERA_SMOKE_READINESS:-$ROOT/scripts/public_gazebo_camera_pair_readiness.py}"
[[ -f "$READINESS" && ! -L "$READINESS" ]] || { echo "BLOCKED: readiness probe must be a regular file" >&2; exit 2; }
[[ "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] || { echo 'BLOCKED: ROS domain must be numeric' >&2; exit 2; }

RECEIPT="$RUN_ROOT/public_gazebo_camera_readiness_smoke_receipt.json"
READINESS_REPORT="$RUN_ROOT/camera_pair_readiness.json"
LAUNCH_PID=""
LAUNCH_PGID=""
PRIMARY_ERROR=""
DEADLINE_EPOCH=$((SECONDS + PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC))
export TZCUP_REPOSITORY_ROOT="$ROOT" ROS_DOMAIN_ID
export GZ_PARTITION="tzcup_public_camera_smoke_${ROS_DOMAIN_ID}_$$_$(date +%s)"

write_receipt() {
  local status="$1" code="$2" zero="$3"
  python3 - "$RECEIPT" "$status" "$code" "$zero" "$PRIMARY_ERROR" "$GZ_PARTITION" "$LAUNCH_PGID" <<'PY'
import json, os, sys
from pathlib import Path
p, status, code, zero, error, partition, launch_pid = sys.argv[1:]
payload = {
    "report_id": "tzcup_public_gazebo_camera_readiness_smoke_v1",
    "status": status, "formal_passed": False, "classification": "NON_FORMAL",
    "exit_code": int(code), "primary_error": error, "gazebo_partition": partition,
    "launch_pgid": int(launch_pid) if launch_pid.isdigit() else None,
    "wall_deadline_seconds": int(os.environ["PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC"]),
    "memory_watchdog_max_group_rss_kib": int(os.environ.get("FORMAL_MEMORY_MAX_GROUP_RSS_KIB", "9437184")),
    "zero_survivor_check": zero == "true",
    "claim_boundary": "One paired readiness observation only; it is not a pilot, full calibration, GT artifact, or formal acceptance.",
}
target = Path(p); pending = target.with_name(f".{target.name}.pending.{os.getpid()}")
pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
os.replace(pending, target)
PY
}

cleanup() {
  local code=$? zero=true
  trap - EXIT INT TERM
  set +e
  if [[ -n "$LAUNCH_PID" ]]; then
    formal_runtime_cleanup_groups "$GZ_PARTITION" "$LAUNCH_PID" || zero=false
  else
    formal_runtime_cleanup_partition "$GZ_PARTITION" || zero=false
  fi
  formal_runtime_stop_memory_watchdog || zero=false
  if formal_runtime_memory_watchdog_tripped; then code="$FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE"; fi
  if (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT != 0 && code == 0 )); then code=125; fi
  [[ "$zero" == true ]] || code=125
  write_receipt "$([[ "$code" == 0 ]] && echo NON_FORMAL_CAMERA_READY || echo BLOCKED)" "$code" "$zero" || code=125
  exit "$code"
}
trap 'PRIMARY_ERROR="${BASH_COMMAND}"' ERR
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# The root remains empty until this runner owns the first guard evidence write.
formal_runtime_register_evidence_paths "$RECEIPT" "$READINESS_REPORT" "$RUN_ROOT/memory_preflight.json" "$RUN_ROOT/memory_watchdog.json"
formal_runtime_memory_preflight "$RUN_ROOT/memory_preflight"
set +u
source /opt/ros/jazzy/setup.bash
source "$PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP"
source "$PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP"
source "$PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP"
set -u
FORMAL_GAZEBO_LOCK_FILE="$PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK" formal_runtime_configure "$ROS_DOMAIN_ID"

remaining=$((DEADLINE_EPOCH - SECONDS))
(( remaining > 0 )) || exit 124
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus.launch.py \
  gui:=false world:="$PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD" \
  episode_manifest:="$PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST" world_name:=campus_formal \
  runtime_artifact_dir:="$RUN_ROOT/materialized" simulation_initial_estop_active:=true \
  start_navigation:=false start_coverage:=false >"$RUN_ROOT/launch.log" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID="$LAUNCH_PID"
formal_runtime_start_memory_watchdog "$LAUNCH_PID" "$RUN_ROOT/memory_watchdog"

remaining=$((DEADLINE_EPOCH - SECONDS))
(( remaining > 0 )) || exit 124
set +e
timeout -k 3 "$remaining" python3 "$READINESS" \
  --image-topic "${PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC:-/camera/color/image_raw}" \
  --camera-info-topic "${PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC:-/camera/color/camera_info}" \
  --timeout "$remaining" --output "$READINESS_REPORT"
status=$?
set -e
(( status == 0 )) || exit "$status"
kill -0 "$LAUNCH_PID" 2>/dev/null || { echo 'BLOCKED: launch exited before readiness completion' >&2; exit 125; }
