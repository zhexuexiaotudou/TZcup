#!/usr/bin/env bash
# One exact public RGB/CameraInfo readiness pair. It is never a dataset run.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/run_formal_runtime_isolation.sh"

: "${PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT:?fresh output root is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK:?exclusive Gazebo lock is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC:?whole-run deadline is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_ROS_SETUP:?ROS setup is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP:?frozen stage setup is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP:?frozen runtime setup is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP:?frozen campus setup is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD:?public world is required}"
: "${PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST:?public episode manifest is required}"
: "${ROS_DOMAIN_ID:?isolated ROS domain is required}"
case "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" in ''|0) ;; *) echo 'BLOCKED: standalone smoke rejects orchestrated sessions' >&2; exit 2 ;; esac
[[ "$PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ && "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || exit 2

has_symlink_ancestor() { local path="$1"; [[ "$path" == /* && "$path" != *'/../'* && "$path" != */.. && "$path" != .. ]] || return 1; while [[ "$path" != / ]]; do [[ ! -L "$path" ]] || return 1; path="$(dirname "$path")"; done; }
real_regular() { [[ -f "$1" && ! -L "$1" ]] && has_symlink_ancestor "$1" && realpath --no-symlinks -e "$1"; }
within_root() { [[ "$1" == "$ROOT"/* ]]; }
has_symlink_ancestor "$PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT" || exit 2
RUN_ROOT="$(realpath --no-symlinks -e "$PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT")"
[[ -d "$RUN_ROOT" && ! -L "$RUN_ROOT" ]] && [[ -z "$(find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" ]] && within_root "$RUN_ROOT" || {
  echo 'BLOCKED: output must be fresh, empty, regular, and inside this worktree' >&2; exit 2;
}
LOCK_PARENT="$(dirname "$PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK")"
has_symlink_ancestor "$LOCK_PARENT" || exit 2
LOCK_PARENT="$(realpath --no-symlinks -e "$LOCK_PARENT")"
[[ "$LOCK_PARENT" == "$ROOT/.work/locks" ]] || { echo 'BLOCKED: lock parent must be worktree .work/locks' >&2; exit 2; }
[[ ! -e "$PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK" || ( -f "$PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK" && ! -L "$PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK" ) ]] || exit 2
LOCK_PATH="$LOCK_PARENT/$(basename "$PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK")"
for variable in PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST; do
  value="$(real_regular "${!variable}")" || { echo "BLOCKED: regular input missing: ${!variable}" >&2; exit 2; }
  within_root "$value" || { echo "BLOCKED: input must be inside worktree: $value" >&2; exit 2; }
  printf -v "$variable" '%s' "$value"
done
ROS_SETUP="$(real_regular "$PUBLIC_GAZEBO_CAMERA_SMOKE_ROS_SETUP")" || exit 2
if ! within_root "$ROS_SETUP" && [[ "$ROS_SETUP" != /opt/ros/jazzy/setup.bash ]]; then echo 'BLOCKED: ROS setup outside contract' >&2; exit 2; fi
READINESS="$ROOT/scripts/public_gazebo_camera_pair_readiness.py"
[[ -f "$READINESS" && ! -L "$READINESS" ]] || exit 2

RECEIPT="$RUN_ROOT/public_gazebo_camera_readiness_smoke_receipt.json"; READINESS_REPORT="$RUN_ROOT/camera_pair_readiness.json"
SETUP_SNAPSHOT="$RUN_ROOT/setup_environment.sh"; LAUNCH_LOG="$RUN_ROOT/launch.log"
PREFLIGHT_PID=""; PREFLIGHT_PGID=""; LAUNCH_PID=""; LAUNCH_PGID=""; PROBE_PID=""; PROBE_PGID=""; DEADLINE_PID=""; PRIMARY_ERROR=""
GZ_PARTITION="tzcup_public_camera_smoke_${ROS_DOMAIN_ID}_$$_$(date +%s)"; export TZCUP_REPOSITORY_ROOT="$ROOT" ROS_DOMAIN_ID GZ_PARTITION
REQUESTED_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"; REQUESTED_GZ_PARTITION="$GZ_PARTITION"
IMAGE_TOPIC="/camera/color/image_raw"; CAMERA_INFO_TOPIC="/camera/color/camera_info"
git -C "$ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1 || { echo 'BLOCKED: worktree must be a git repository' >&2; exit 2; }
GIT_STATUS_AT_ADMISSION="$(git -C "$ROOT" status --porcelain=v1)"; [[ -z "$GIT_STATUS_AT_ADMISSION" ]] || { echo 'BLOCKED: worktree must be clean' >&2; exit 2; }; export GIT_STATUS_AT_ADMISSION
binding_digest() { sha256sum "$ROOT/scripts/run_public_gazebo_camera_readiness_smoke.sh" "$READINESS" "$ROS_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD" "$PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST" | sha256sum | awk '{print $1}'; }
ADMISSION_BINDING_SHA256="$(binding_digest)"; export ADMISSION_BINDING_SHA256
remaining() { local n=$((DEADLINE_EPOCH - SECONDS)); (( n > 0 )) && printf '%s\n' "$n"; }
stop_private_group() {
  local pgid="$1" signal; [[ "$pgid" =~ ^[2-9][0-9]*$ ]] || return 0
  for signal in TERM KILL; do
    kill -0 -- "-$pgid" 2>/dev/null || return 0; kill -"$signal" -- "-$pgid" 2>/dev/null || true
    for _ in {1..10}; do kill -0 -- "-$pgid" 2>/dev/null || return 0; sleep .05; done
  done
  ! kill -0 -- "-$pgid" 2>/dev/null
}
stop_deadline() { [[ -n "$DEADLINE_PID" ]] && stop_private_group "$DEADLINE_PID"; DEADLINE_PID=""; }

write_receipt() {
  local status="$1" code="$2" zero="$3"
  WATCHDOG_JSON="$RUN_ROOT/memory_watchdog.json" python3 - "$RECEIPT" "$status" "$code" "$zero" "$PRIMARY_ERROR" "$GZ_PARTITION" "$PREFLIGHT_PGID" "$LAUNCH_PGID" "$PROBE_PGID" "$ROOT" "$READINESS" "$READINESS_REPORT" "$ROS_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD" "$PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST" <<'PY'
import hashlib, json, os, subprocess, sys
from pathlib import Path
r,status,code,zero,error,partition,preflight,launch,probe,root,script,report,*inputs=sys.argv[1:]
def ident(name):
 p=Path(name); return {"path":str(p.relative_to(root)) if p.is_relative_to(root) else str(p),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}
def git(*args):
 try:return subprocess.check_output(["git",*args],cwd=root,text=True,stderr=subprocess.DEVNULL).strip()
 except (OSError,subprocess.CalledProcessError):return None
watchdog=Path(os.environ["WATCHDOG_JSON"])
def maybe(name): return ident(name) if Path(name).is_file() else None
roles=("ros_setup","stage1_setup","runtime_setup","campus_setup","world","manifest")
payload={"report_id":"tzcup_public_gazebo_camera_readiness_smoke_v1","status":status,"formal_passed":False,"classification":"NON_FORMAL","exit_code":int(code),"primary_error":error,"gazebo_partition":partition,"preflight_pgid":int(preflight) if preflight.isdigit() else None,"launch_pgid":int(launch) if launch.isdigit() else None,"probe_pgid":int(probe) if probe.isdigit() else None,"wall_deadline_seconds":int(os.environ["PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC"]),"memory_watchdog_max_group_rss_kib":int(os.environ.get("FORMAL_MEMORY_MAX_GROUP_RSS_KIB","9437184")),"zero_survivor_check":zero=="true","runner":ident(Path(root)/"scripts/run_public_gazebo_camera_readiness_smoke.sh"),"probe":ident(script),"readiness_report":maybe(report),"inputs":dict(zip(roles,(ident(p) for p in inputs))),"admission_binding_sha256":os.environ["ADMISSION_BINDING_SHA256"],"final_binding_sha256":os.environ["FINAL_BINDING_SHA256"],"binding_unchanged":os.environ["BINDING_UNCHANGED"]=="true","evidence":{"setup_snapshot":maybe(Path(r).parent/"setup_environment.sh"),"preflight_json":maybe(Path(r).parent/"memory_preflight.json"),"preflight_log":maybe(Path(r).parent/"memory_preflight.log"),"watchdog_json":maybe(watchdog),"watchdog_log":maybe(Path(r).parent/"memory_watchdog.log"),"launch_log":maybe(Path(r).parent/"launch.log")},"git_head":git("rev-parse","HEAD"),"git_tree":git("rev-parse","HEAD^{tree}"),"git_worktree_clean_at_admission":not bool(os.environ.get("GIT_STATUS_AT_ADMISSION")),"git_worktree_status_sha256":hashlib.sha256(os.environ.get("GIT_STATUS_AT_ADMISSION","").encode()).hexdigest(),"claim_boundary":"One paired readiness observation only; it is not a pilot, full calibration, GT artifact, or formal acceptance."}
p=Path(r); tmp=p.with_name(f".{p.name}.pending.{os.getpid()}"); tmp.write_text(json.dumps(payload,indent=2,sort_keys=True)+"\n",encoding="utf-8"); os.replace(tmp,p)
PY
}
cleanup_finalize() {
  local code="${FORMAL_RUNTIME_EXIT_STATUS:-125}" zero=true label; local -a leaders=()
  set +e; stop_deadline || zero=false
  [[ -n "$PREFLIGHT_PID" ]] && leaders+=("$PREFLIGHT_PID"); [[ -n "$PROBE_PID" ]] && leaders+=("$PROBE_PID"); [[ -n "$LAUNCH_PID" ]] && leaders+=("$LAUNCH_PID")
  if (( ${#leaders[@]} )); then formal_runtime_cleanup_groups "$GZ_PARTITION" "${leaders[@]}" || zero=false; else formal_runtime_cleanup_partition "$GZ_PARTITION" || zero=false; fi
  formal_runtime_stop_memory_watchdog || zero=false
  if formal_runtime_memory_watchdog_tripped; then
    code="$FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE"
  elif (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT != 0 )); then
    code=125
  fi
  FINAL_BINDING_SHA256="$(binding_digest)"; BINDING_UNCHANGED=false; [[ "$FINAL_BINDING_SHA256" == "$ADMISSION_BINDING_SHA256" ]] && BINDING_UNCHANGED=true
  [[ "$code" != 0 || "$BINDING_UNCHANGED" == true ]] || code=125
  export FINAL_BINDING_SHA256 BINDING_UNCHANGED
  [[ "$zero" == true ]] || code=125; label=BLOCKED; [[ "$code" == 0 ]] && label=NON_FORMAL_CAMERA_READY
  write_receipt "$label" "$code" "$zero" || return 1; [[ "$zero" == true ]]
}
trap 'PRIMARY_ERROR="${BASH_COMMAND}"' ERR
formal_runtime_install_traps cleanup_finalize
formal_runtime_register_evidence_paths "$RECEIPT" "$READINESS_REPORT" "$SETUP_SNAPSHOT" "$RUN_ROOT/memory_preflight.json" "$RUN_ROOT/memory_preflight.log" "$RUN_ROOT/memory_watchdog.json" "$RUN_ROOT/memory_watchdog.log" "$LAUNCH_LOG" "$RUN_ROOT/materialized"
DEADLINE_EPOCH=$((SECONDS + PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC)); setsid sleep "$PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC" & DEADLINE_PID=$!

# The preflight and setup child cannot outlive the same wall deadline.
setsid bash -c 'source "$1"; formal_runtime_memory_preflight "$2"' bash "$ROOT/scripts/run_formal_runtime_isolation.sh" "$RUN_ROOT/memory_preflight" & PREFLIGHT_PID=$!
PREFLIGHT_PGID="$(ps -o pgid= -p "$PREFLIGHT_PID" 2>/dev/null | tr -d ' ' || true)"
set +e; wait -n -p finished "$PREFLIGHT_PID" "$DEADLINE_PID"; status=$?; set -e
[[ "$finished" == "$PREFLIGHT_PID" ]] || exit 124
(( status == 0 )) || { (( status == FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE )) && exit "$status"; exit 125; }
left="$(remaining)" || exit 124
set +e
setsid timeout -k 1 "$left" bash -c '
  set -Eeuo pipefail; for setup in "$@"; do source "$setup"; done
  while IFS= read -r name; do
    case "$name" in
      PATH|PYTHONPATH|LD_LIBRARY_PATH|PKG_CONFIG_PATH|AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|COLCON_PREFIX_PATH|COLCON_CURRENT_PREFIX|ROS_DISTRO|ROS_VERSION|ROS_PYTHON_VERSION|ROS_PACKAGE_PATH|RMW_IMPLEMENTATION|GZ_SIM_RESOURCE_PATH|GZ_SIM_SYSTEM_PLUGIN_PATH|GZ_CONFIG_PATH|IGN_GAZEBO_RESOURCE_PATH|IGN_GAZEBO_SYSTEM_PLUGIN_PATH|IGN_CONFIG_PATH|GAZEBO_RESOURCE_PATH|GAZEBO_PLUGIN_PATH|GAZEBO_MODEL_PATH) [[ "$name" != *TOKEN* && "$name" != *PASSWORD* && "$name" != *SECRET* && "$name" != *CREDENTIAL* && "$name" != *AUTH* && "$name" != *KEY* ]] && declare -px "$name" ;;
    esac
  done < <(compgen -v)
' bash "$ROS_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP" "$PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP" >"$SETUP_SNAPSHOT"
status=$?
set -e
(( status == 0 )) || { [[ "$status" == 124 || "$status" == 137 ]] && exit 124; exit 125; }
python3 - "$SETUP_SNAPSHOT" <<'PY'
import re,sys
raw=open(sys.argv[1],"rb").read()
try: lines=raw.decode("utf-8").splitlines()
except UnicodeDecodeError: raise SystemExit(2)
safe=re.compile(r'declare -x [A-Za-z_][A-Za-z0-9_]*(?:="(?:[^"\\\n]|\\.)*")?$')
raise SystemExit(0 if raw and b"\0" not in raw and b"$'" not in raw and all(safe.fullmatch(x) for x in lines) else 2)
PY
source "$SETUP_SNAPSHOT"
ROS_DOMAIN_ID="$REQUESTED_ROS_DOMAIN_ID"; GZ_PARTITION="$REQUESTED_GZ_PARTITION"; export ROS_DOMAIN_ID GZ_PARTITION
FORMAL_GAZEBO_LOCK_FILE="$LOCK_PATH" formal_runtime_configure "$ROS_DOMAIN_ID"

left="$(remaining)" || exit 124
setsid ros2 launch sanitation_formal_campus_integration formal_campus.launch.py gui:=false world:="$PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD" episode_manifest:="$PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST" world_name:=campus_formal runtime_artifact_dir:="$RUN_ROOT/materialized" simulation_initial_estop_active:=true start_navigation:=false start_coverage:=false >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!; LAUNCH_PGID="$(formal_runtime_wait_for_setsid_pgid "$LAUNCH_PID")" || exit 125
formal_runtime_start_memory_watchdog "$LAUNCH_PID" "$RUN_ROOT/memory_watchdog" || exit $?
WATCHDOG_PID="$FORMAL_RUNTIME_MEMORY_WATCHDOG_PID"
left="$(remaining)" || exit 124
setsid python3 "$READINESS" --image-topic "$IMAGE_TOPIC" --camera-info-topic "$CAMERA_INFO_TOPIC" --timeout "$left" --output "$READINESS_REPORT" &
PROBE_PID=$!; PROBE_PGID="$(formal_runtime_wait_for_setsid_pgid "$PROBE_PID")" || exit 125
set +e; wait -n -p finished "$PROBE_PID" "$WATCHDOG_PID" "$LAUNCH_PID" "$DEADLINE_PID"; status=$?; set -e
case "$finished" in
  "$PROBE_PID") (( status == 0 )) && python3 "$READINESS" --validate-report "$READINESS_REPORT" --image-topic "$IMAGE_TOPIC" --camera-info-topic "$CAMERA_INFO_TOPIC" && [[ "$(binding_digest)" == "$ADMISSION_BINDING_SHA256" ]] || exit 125 ;;
  "$WATCHDOG_PID") formal_runtime_record_memory_watchdog_exit "$WATCHDOG_PID" "$status" || exit 125; formal_runtime_memory_watchdog_tripped && exit "$FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE"; exit 125 ;;
  "$LAUNCH_PID") exit 125 ;;
  "$DEADLINE_PID") exit 124 ;;
  *) exit 125 ;;
esac
