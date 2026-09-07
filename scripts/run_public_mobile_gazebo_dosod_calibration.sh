#!/usr/bin/env bash
# NON_FORMAL public-train mobile camera collection; the collector never controls Nav2.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/run_formal_runtime_isolation.sh"
source "$ROOT/scripts/public_gazebo_mobile_readiness.sh"
export PUBLIC_GAZEBO_CALIBRATION_PARSER="$ROOT/scripts/parse_public_gazebo_topic_info.py"
: "${PUBLIC_GAZEBO_CALIBRATION_PLAN:?}" "${PUBLIC_GAZEBO_CALIBRATION_OUTPUT:?}"
: "${PUBLIC_GAZEBO_CALIBRATION_LOCK:?}" "${PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC:?}" "${PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC:?}"
: "${PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC:?}" "${PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC:?}"
: "${PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA:?}" "${PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP:?}"
: "${PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP:?}" "${PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP:?}" "${ROS_DOMAIN_ID:?}"
MODE="${PUBLIC_GAZEBO_CALIBRATION_MODE:-full}"
[[ "$MODE" == pilot || "$MODE" == full ]] || { echo 'BLOCKED: mode must be pilot or full' >&2; exit 2; }
[[ "$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ && "$PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ ]] || { echo 'BLOCKED: calibration deadlines must be positive integer seconds' >&2; exit 2; }
[[ "$PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC" == /camera/color/image_raw && "$PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC" == /camera/color/camera_info ]] || { echo 'BLOCKED: only the approved public RGB/CameraInfo pair is accepted' >&2; exit 2; }
PILOT_SCENE="map-0-mission-0"
if [[ "$MODE" == pilot ]]; then
  [[ "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" == 25 ]] || { echo 'BLOCKED: pilot quota must be 25' >&2; exit 2; }
  (( PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC >= 900 && PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC >= 900 )) || { echo 'BLOCKED: pilot collector and total deadlines must be at least 900 seconds' >&2; exit 2; }
  EXPECTED_MANIFEST="pilot_manifest.json"
else
  : "${PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT:?}" "${PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST:?}" "${PUBLIC_GAZEBO_CALIBRATION_PREPROCESSING_ORACLE:?}"
  [[ "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" == 25 ]] || { echo 'BLOCKED: full quota must be 25 (20+4 scenes = 500+100)' >&2; exit 2; }
  (( PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC >= 14400 && PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC >= 14400 )) || { echo 'BLOCKED: full collector and total deadlines must be at least 14400 seconds' >&2; exit 2; }
  EXPECTED_MANIFEST="calibration_manifest.json"
fi
has_symlink_ancestor() { local path="$1"; [[ "$path" == /* && "$path" != *'/../'* && "$path" != */.. && "$path" != .. ]] || return 1; while [[ "$path" != / ]]; do [[ ! -L "$path" ]] || return 1; path="$(dirname "$path")"; done; }
real_regular() { [[ -f "$1" && ! -L "$1" ]] && has_symlink_ancestor "$1" && realpath --no-symlinks -e "$1"; }
within_root() { [[ "$1" == "$ROOT"/* ]]; }
has_symlink_ancestor "$PUBLIC_GAZEBO_CALIBRATION_OUTPUT" || exit 2
RUN_ROOT="$(realpath --no-symlinks -e "$PUBLIC_GAZEBO_CALIBRATION_OUTPUT")"
[[ -d "$RUN_ROOT" && ! -L "$RUN_ROOT" && -z "$(find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" ]] && within_root "$RUN_ROOT" || { echo 'BLOCKED: output must be fresh, empty, regular, and inside this worktree' >&2; exit 2; }
LOCK_PARENT="$(dirname "$PUBLIC_GAZEBO_CALIBRATION_LOCK")"; has_symlink_ancestor "$LOCK_PARENT" || exit 2
LOCK_PARENT="$(realpath --no-symlinks -e "$LOCK_PARENT")"; [[ "$LOCK_PARENT" == "$ROOT/.work/locks" ]] || { echo 'BLOCKED: lock parent must be worktree .work/locks' >&2; exit 2; }
[[ ! -e "$PUBLIC_GAZEBO_CALIBRATION_LOCK" || ( -f "$PUBLIC_GAZEBO_CALIBRATION_LOCK" && ! -L "$PUBLIC_GAZEBO_CALIBRATION_LOCK" ) ]] || exit 2
LOCK_PATH="$LOCK_PARENT/$(basename "$PUBLIC_GAZEBO_CALIBRATION_LOCK")"
for variable in PUBLIC_GAZEBO_CALIBRATION_PLAN PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP; do value="$(real_regular "${!variable}")" || { echo "BLOCKED: required regular input missing: ${!variable}" >&2; exit 2; }; within_root "$value" || { echo "BLOCKED: input must be inside worktree: $value" >&2; exit 2; }; printf -v "$variable" '%s' "$value"; done
CANONICAL_PLAN="$(real_regular "$ROOT/config/public_gazebo_dosod_train_scene_plan.json")" || { echo 'BLOCKED: canonical public plan is unavailable' >&2; exit 2; }
[[ "$PUBLIC_GAZEBO_CALIBRATION_PLAN" == "$CANONICAL_PLAN" ]] || { echo 'BLOCKED: only config/public_gazebo_dosod_train_scene_plan.json is accepted' >&2; exit 2; }
CANONICAL_PLAN_SHA256="$(sha256sum "$CANONICAL_PLAN" | awk '{print $1}')"
if [[ "$MODE" == full ]]; then for variable in PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST PUBLIC_GAZEBO_CALIBRATION_PREPROCESSING_ORACLE; do value="$(real_regular "${!variable}")" || { echo "BLOCKED: full input missing or unsafe: ${!variable}" >&2; exit 2; }; within_root "$value" || { echo "BLOCKED: full input must be inside worktree: $value" >&2; exit 2; }; printf -v "$variable" '%s' "$value"; done; fi
git -C "$ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1 || { echo 'BLOCKED: worktree must be a git repository' >&2; exit 2; }
GIT_STATUS_AT_ADMISSION="$(git -C "$ROOT" status --porcelain=v1)"; [[ -z "$GIT_STATUS_AT_ADMISSION" ]] || { echo 'BLOCKED: worktree must be clean' >&2; exit 2; }
binding_digest() { local -a bindings=("$ROOT/scripts/run_public_mobile_gazebo_dosod_calibration.sh" "$ROOT/scripts/public_gazebo_dosod_calibration.py" "$ROOT/scripts/public_gazebo_mobile_readiness.sh" "$PUBLIC_GAZEBO_CALIBRATION_PLAN" "$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP" "$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP" "$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP"); [[ "$MODE" == full ]] && bindings+=("$PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT" "$PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST" "$PUBLIC_GAZEBO_CALIBRATION_PREPROCESSING_ORACLE"); sha256sum "${bindings[@]}" | sha256sum | awk '{print $1}'; }
ADMISSION_BINDING_SHA256="$(binding_digest)"; export ADMISSION_BINDING_SHA256 CANONICAL_PLAN_SHA256
DEADLINE_EPOCH=$((SECONDS + PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC))
DEADLINE_PID=""
REQUESTED_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
set +u
source /opt/ros/jazzy/setup.bash
source "$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP"; source "$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP"; source "$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP"
set -u
ROS_DOMAIN_ID="$REQUESTED_ROS_DOMAIN_ID"; export ROS_DOMAIN_ID
FORMAL_GAZEBO_LOCK_FILE="$LOCK_PATH" formal_runtime_configure "$ROS_DOMAIN_ID"
export TZCUP_REPOSITORY_ROOT="$ROOT"
export PUBLIC_GAZEBO_CALIBRATION_RECEIPT_STRICT=1
DATASET="$RUN_ROOT/dataset"; SELECTOR="$RUN_ROOT/scene_selector.json"; PROGRESS="$RUN_ROOT/collector_progress.json"; SCENE_RUNTIME_INDEX="$RUN_ROOT/scene_runtime_index.json"
RECEIPT="$RUN_ROOT/public_mobile_gazebo_dosod_calibration_receipt.json"; PRIMARY_ERROR=""; DESIRED_STATE="BLOCKED"
collector_pid=""; launch_pid=""; operator_pid=""; stop_estop_pid=""
scene_operator_started=false; scene_stop_attempted=false; scene_stop_verified=false
RUNNER_EXIT_CODE=0
VALIDATION_SNAPSHOT=""; ORACLE_FINAL_SUMMARY=""
GENERATOR_DEADLINE_SEC=60
READINESS_DEADLINE_SEC=60
FINAL_VALIDATION_DEADLINE_SEC=120
COLLECTOR_DEADLINE_SEC="$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC"
QUOTA_PID=""; WATCHDOG_PID=""; GZ_PGID=""; PREFLIGHT_PID=""; PREFLIGHT_PGID=""; READINESS_PID=""; FINAL_ORACLE_PID=""; FINAL_BINDING_SHA256=""
export FORMAL_MEMORY_WATCHDOG_ENABLED=1 FORMAL_MEMORY_MAX_GROUP_RSS_KIB=9437184
deadline_run() {
  # Each short operation owns a fresh session.  TERM waits ten seconds before
  # KILL and the caller receives failure unless that exact group is gone.
  local limit="$1" log="$2"; shift 2
  setsid "$@" >"$log" 2>&1 & local pid=$! elapsed=0
  while kill -0 "$pid" 2>/dev/null && (( elapsed < limit )); do sleep 1; ((elapsed+=1)); done
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true; sleep 10
    kill -0 -- "-$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    ! kill -0 -- "-$pid" 2>/dev/null || return 125
    return 124
  fi
  local rc=0
  wait "$pid" || rc=$?
  # A leader exiting first is not cleanup: any exact child still in its
  # private PGID turns this operation into a fail-closed cleanup failure.
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true; sleep 10
    kill -0 -- "-$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null || true
    ! kill -0 -- "-$pid" 2>/dev/null || return 125
    return 125
  fi
  return "$rc"
}
remaining_seconds() { local remaining=$((DEADLINE_EPOCH - SECONDS)); (( remaining > 0 )) && printf '%s\n' "$remaining"; }
stop_private_group() {
  local pgid="$1" signal
  [[ "$pgid" =~ ^[2-9][0-9]*$ ]] || return 0
  for signal in TERM KILL; do
    kill -0 -- "-$pgid" 2>/dev/null || return 0
    kill -"$signal" -- "-$pgid" 2>/dev/null || true
    for _ in {1..40}; do kill -0 -- "-$pgid" 2>/dev/null || return 0; sleep .25; done
  done
  ! kill -0 -- "-$pgid" 2>/dev/null
}
stop_deadline() { [[ -n "$DEADLINE_PID" ]] && stop_private_group "$DEADLINE_PID" || true; DEADLINE_PID=""; }
start_quota_waiter() {
  local scene="$1"
  setsid bash -c '
    set -Eeuo pipefail
    progress="$1"; scene="$2"; quota="$3"
    while :; do
      if python3 - "$progress" "$scene" "$quota" <<"PY"
import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception: raise SystemExit(1)
count=max(d.get("calibration_scene_counts",{}).get(sys.argv[2],0), d.get("holdout_scene_counts",{}).get(sys.argv[2],0))
raise SystemExit(0 if count >= int(sys.argv[3]) else 1)
PY
      then exit 0; fi
      sleep .2
    done
  ' bash "$PROGRESS" "$scene" "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" & QUOTA_PID=$!
}
require_mapping_readiness() {
  local expected=""
  if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" == 1 ]]; then expected="$(ps -o pgid= -p "$$" | tr -d '[:space:]')"; fi
  public_mobile_mapping_readiness "$launch_pid" "$1" "$READINESS_DEADLINE_SEC" "$expected"
}
start_mapping_readiness() {
  local output="$1" expected=""
  if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" == 1 ]]; then expected="$(ps -o pgid= -p "$$" | tr -d '[:space:]')"; fi
  setsid bash -c 'source "$1"; public_mobile_mapping_readiness "$2" "$3" "$4" "$5"' bash "$ROOT/scripts/public_gazebo_mobile_readiness.sh" "$launch_pid" "$output" "$READINESS_DEADLINE_SEC" "$expected" & READINESS_PID=$!
}
record_scene_runtime() {
  python3 - "$SCENE_RUNTIME_INDEX" "$scene" "$GZ_PGID" "$scene_root/memory_watchdog.json" "$scene_root/memory_watchdog.log" <<'PY'
import hashlib,json,os,sys
from pathlib import Path
index,scene,pgid,watch_json,watch_log=map(Path,sys.argv[1:])
def item(path): return {'relative_path':str(path.relative_to(index.parent)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'byte_size':path.stat().st_size}
if not pgid.name.isdigit() or int(pgid.name) <= 1: raise SystemExit(2)
for path in (watch_json,watch_log):
 if not path.is_file() or path.is_symlink(): raise SystemExit(2)
watch=json.loads(watch_json.read_text())
if watch.get('target_pgid') != int(pgid.name) or watch.get('surviving_group_processes') != 0: raise SystemExit(2)
rows=[] if not index.exists() else json.loads(index.read_text())
rows.append({'scene_id':scene.name,'gazebo_pgid':int(pgid.name),'watchdog_status':watch.get('status'),'watchdog_json':item(watch_json),'watchdog_log':item(watch_log)})
tmp=index.with_name('.'+index.name+'.pending.'+str(os.getpid())); tmp.write_text(json.dumps(rows,sort_keys=True)+'\n'); os.replace(tmp,index)
PY
}
write_receipt() { python3 - "$RECEIPT" "$1" "$2" "$PRIMARY_ERROR" "$3" "$MODE" "$DATASET" "$EXPECTED_MANIFEST" "${PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT:-}" "${PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST:-}" "${PUBLIC_GAZEBO_CALIBRATION_PREPROCESSING_ORACLE:-}" "${VALIDATION_SNAPSHOT:-}" <<'PY'
import hashlib,json,os,sys
from pathlib import Path
p,status,code,error,zero,mode,dataset,expected,review,pilot,oracle,snapshot=(Path(sys.argv[1]),sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5],sys.argv[6],Path(sys.argv[7]),sys.argv[8],sys.argv[9],sys.argv[10],sys.argv[11],sys.argv[12])
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def regular(path): return path.is_file() and not path.is_symlink()
strict=os.environ.get('PUBLIC_GAZEBO_CALIBRATION_RECEIPT_STRICT') == '1'
root=Path(os.environ.get('TZCUP_REPOSITORY_ROOT',Path.cwd()))
def ident(path): return {'path':str(path.relative_to(root)),'sha256':digest(path),'byte_size':path.stat().st_size}
roles={} if not strict else {'plan':Path(os.environ['PUBLIC_GAZEBO_CALIBRATION_PLAN']),'stage1_setup':Path(os.environ['PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP']),'runtime_setup':Path(os.environ['PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP']),'campus_setup':Path(os.environ['PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP'])}
scene_index=p.parent/'scene_runtime_index.json'
scenes=[] if not regular(scene_index) else json.loads(scene_index.read_text())
v={'report_id':'tzcup_public_mobile_gazebo_dosod_calibration_runner_v1','status':status,'formal_passed':False,'classification':'NON_FORMAL','exit_code':int(code),'primary_error':error,'zero_survivor_check':zero=='true','mode':mode,'budgets_sec':{'whole_runner':int(os.environ.get('PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC',os.environ['PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC'])),'collector':int(os.environ['PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC']),'scene_generator':60,'readiness':60,'final_validation':120,'term_grace':10},'memory_watchdog_max_group_rss_kib':9437184,'bindings':{'admission_sha256':os.environ.get('ADMISSION_BINDING_SHA256'),'final_sha256':os.environ.get('FINAL_BINDING_SHA256'),'unchanged':os.environ.get('ADMISSION_BINDING_SHA256')==os.environ.get('FINAL_BINDING_SHA256'),'canonical_plan_sha256':os.environ.get('CANONICAL_PLAN_SHA256'),'inputs':{name:ident(path) for name,path in roles.items()}},'git':None if not strict else {'head':__import__('subprocess').check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'tree':__import__('subprocess').check_output(['git','rev-parse','HEAD^{tree}'],cwd=root,text=True).strip()},'preflight':{'pid':int(os.environ.get('PREFLIGHT_PID','0') or 0),'pgid':int(os.environ.get('PREFLIGHT_PGID','0') or 0)},'scene_runtime':scenes}
invalid=False
if status in {'NON_FORMAL_PILOT_CAPTURED','NON_FORMAL_CALIBRATION_FROZEN'}:
 try:
  manifest=dataset/expected
  if not regular(manifest): raise ValueError('manifest')
  data=json.loads(manifest.read_text()); artifact={'relative_path':expected,'sha256':digest(manifest),'byte_size':manifest.stat().st_size}
  if mode == 'pilot':
   sheet=data.get('contact_sheet',{}); path=dataset/sheet.get('relative_path','')
   if data.get('status') != status or data.get('formal_passed') is not False or data.get('pilot_scene') != 'map-0-mission-0' or data.get('record_count') != 25 or sheet.get('relative_path') != 'pilot_contact_sheet.png' or not regular(path) or sheet.get('sha256') != digest(path) or sheet.get('byte_size') != path.stat().st_size: raise ValueError('pilot')
   artifact.update({'pilot_scene':data['pilot_scene'],'record_count':data['record_count'],'record_sha256':data.get('record_sha256'),'contact_sheet':sheet})
  else:
   if data.get('status') != 'FROZEN' or not all(regular(Path(x)) for x in (review,pilot,oracle,snapshot)): raise ValueError('full')
   checked=json.loads(Path(snapshot).read_text())
   pilot_data=json.loads(Path(pilot).read_text()); sheet=pilot_data.get('contact_sheet',{}); sheet_path=Path(pilot).parent/sheet.get('relative_path','')
   if checked.get('status') != 'NON_FORMAL_REVIEW_APPROVED' or checked.get('review_receipt_sha256') != digest(Path(review)) or checked.get('pilot_manifest_sha256') != digest(Path(pilot)) or checked.get('contact_sheet_sha256') != sheet.get('sha256') or checked.get('record_sha256') != pilot_data.get('record_sha256') or not regular(sheet_path) or digest(sheet_path) != checked['contact_sheet_sha256']: raise ValueError('full_snapshot')
   final_oracle=Path(os.environ.get('ORACLE_FINAL_SUMMARY',''))
   if strict and not regular(final_oracle): raise ValueError('final_oracle_validation_missing')
   artifact.update({'review_receipt_sha256':checked['review_receipt_sha256'],'pilot_manifest_sha256':checked['pilot_manifest_sha256'],'preprocessing_oracle_sha256':digest(Path(oracle)),'preprocessing_oracle_final_validation_sha256':digest(final_oracle) if regular(final_oracle) else None,'contact_sheet_sha256':checked['contact_sheet_sha256'],'record_sha256':checked['record_sha256'],'review_validation_snapshot_sha256':digest(Path(snapshot))})
  v['artifact']=artifact
  if strict and status != 'BLOCKED' and (not v['bindings']['unchanged'] or not scenes or any(row.get('watchdog_status') != 'FORMAL_MEMORY_WATCHDOG_COMPLETED' for row in scenes)): raise ValueError('final_binding_or_watchdog_invalid')
 except Exception:
  v.update({'status':'BLOCKED','exit_code':125,'primary_error':'runner_artifact_binding_invalid'}); status='BLOCKED'; invalid=True
t=p.with_name('.'+p.name+'.pending.'+str(os.getpid())); t.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n'); os.replace(t,p)
raise SystemExit(1 if invalid else 0)
PY
}
stop_verified() {
  local pre status rc=0 verified=false stop_root="${scene_root:-$RUN_ROOT}"
  scene_stop_attempted=true
  pre=-1
  status="$stop_root/safety-stop-pre.yaml"
  timeout 5s ros2 topic echo --once --full-length /safety/status_json std_msgs/msg/String >"$status" 2>&1 || rc=1
  pre="$(python3 "$ROOT/scripts/formal_w1_operator_safety_status.py" count "$status" 2>/dev/null)" || rc=1
  if [[ -n "$operator_pid" ]]; then
    formal_runtime_kill_group "$operator_pid" || rc=1
    wait "$operator_pid" 2>/dev/null || true; operator_pid=""
  fi
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 topic pub --rate 10 /formal_vehicle/simulation/command/emergency_stop std_msgs/msg/Bool '{data: true}' >"$stop_root/operator-estop-rearm.log" 2>&1 & stop_estop_pid=$!
  for _ in $(seq 1 30); do
    status="$stop_root/safety-stop-${_}.yaml"; timeout 5s ros2 topic echo --once --full-length /safety/status_json std_msgs/msg/String >"$status" 2>&1 || continue
    if python3 "$ROOT/scripts/formal_w1_operator_safety_status.py" require "$status" INHIBITED manual_estop "$pre"; then verified=true; break; fi
  done
  if [[ "$verified" == true && "$rc" == 0 ]]; then scene_stop_verified=true; else rc=1; fi
  return "$rc"
}
stop_estop_publisher() {
  local rc=0
  if [[ -n "$stop_estop_pid" ]]; then
    formal_runtime_kill_group "$stop_estop_pid" || rc=1
    wait "$stop_estop_pid" 2>/dev/null || true
    kill -0 "$stop_estop_pid" 2>/dev/null && rc=1
    stop_estop_pid=""
  fi
  return "$rc"
}
cleanup() {
  local survivor=true cleanup_failed=0 state="$DESIRED_STATE" pid receipt_code="${FORMAL_RUNTIME_EXIT_STATUS:-$RUNNER_EXIT_CODE}"
  trap - ERR
  set +e
  if [[ -e "$SELECTOR" ]]; then
    python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --deactivate-scene-selector --scene-selector "$SELECTOR" >/dev/null || cleanup_failed=1
  fi
  if [[ "$scene_operator_started" == true && "$scene_stop_verified" != true ]] && [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    if [[ "$scene_stop_attempted" != true ]]; then stop_verified || cleanup_failed=1; fi
    [[ "$scene_stop_verified" == true ]] || cleanup_failed=1
  fi
  stop_private_group "$QUOTA_PID" || cleanup_failed=1; QUOTA_PID=""
  stop_deadline || cleanup_failed=1
  formal_runtime_cleanup_groups "${GZ_PARTITION:-}" "$PREFLIGHT_PID" "$READINESS_PID" "$FINAL_ORACLE_PID" "$launch_pid" "$operator_pid" "$collector_pid" || cleanup_failed=1
  for pid in "$PREFLIGHT_PID" "$READINESS_PID" "$FINAL_ORACLE_PID" "$launch_pid" "$operator_pid" "$collector_pid"; do
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && survivor=false
  done
  launch_pid=""; operator_pid=""; collector_pid=""; READINESS_PID=""; FINAL_ORACLE_PID=""
  stop_estop_publisher || cleanup_failed=1
  FINAL_BINDING_SHA256="$(binding_digest)"; export FINAL_BINDING_SHA256 PREFLIGHT_PID PREFLIGHT_PGID
  [[ "$FINAL_BINDING_SHA256" == "$ADMISSION_BINDING_SHA256" ]] || { cleanup_failed=1; PRIMARY_ERROR=binding_drift; }
  if [[ "$survivor" != true || "$cleanup_failed" != 0 ]]; then cleanup_failed=1; state=BLOCKED; receipt_code=125; fi
  write_receipt "$state" "$receipt_code" "$survivor" || cleanup_failed=1
  (( cleanup_failed == 0 ))
}
trap 'RUNNER_EXIT_CODE=$?; PRIMARY_ERROR="${BASH_COMMAND}"' ERR
formal_runtime_register_evidence_paths "$RUN_ROOT" "$RECEIPT" "$RUN_ROOT/memory_preflight.json" "$RUN_ROOT/memory_preflight.log" "$RUN_ROOT/memory_watchdog.json" "$RUN_ROOT/memory_watchdog.log"
formal_runtime_install_traps cleanup
trap 'RUNNER_EXIT_CODE=$?; formal_runtime_exit_trap "$RUNNER_EXIT_CODE"' EXIT
trap 'RUNNER_EXIT_CODE=130; PRIMARY_ERROR=signal:INT; exit 130' INT
trap 'RUNNER_EXIT_CODE=143; PRIMARY_ERROR=signal:TERM; exit 143' TERM
remaining_seconds >/dev/null || { RUNNER_EXIT_CODE=124; exit 124; }
setsid sleep "$(remaining_seconds)" & DEADLINE_PID=$!
setsid bash -c 'source "$1"; formal_runtime_memory_preflight "$2"' bash "$ROOT/scripts/run_formal_runtime_isolation.sh" "$RUN_ROOT/memory_preflight" & PREFLIGHT_PID=$!
PREFLIGHT_PGID="$(formal_runtime_wait_for_setsid_pgid "$PREFLIGHT_PID")" || { RUNNER_EXIT_CODE=125; exit 125; }
set +e; wait -n -p finished "$PREFLIGHT_PID" "$DEADLINE_PID"; preflight_status=$?; set -e
if [[ "$finished" == "$PREFLIGHT_PID" ]]; then PREFLIGHT_PID=""; else RUNNER_EXIT_CODE=124; exit 124; fi
(( preflight_status == 0 )) || { RUNNER_EXIT_CODE=125; (( preflight_status == FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE )) && RUNNER_EXIT_CODE=$preflight_status; exit "$RUNNER_EXIT_CODE"; }
preflight_args=(--scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json")
if [[ "$MODE" == full ]]; then
  preflight_args+=(--review-receipt "$PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT" --pilot-manifest "$PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST")
  python3 "$ROOT/scripts/validate_dosod_single_frame_preprocessing_oracle.py" --receipt "$PUBLIC_GAZEBO_CALIBRATION_PREPROCESSING_ORACLE" >"$RUN_ROOT/preprocessing_oracle_validation.json"
  python3 - "$PUBLIC_GAZEBO_CALIBRATION_PLAN" <<'PY'
import json,sys
groups=json.load(open(sys.argv[1],encoding='utf-8')).get('scene_groups',{})
raise SystemExit(0 if len(groups.get('calibration',[])) == 20 and len(groups.get('holdout',[])) == 4 else 2)
PY
fi
python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" "${preflight_args[@]}" >"$RUN_ROOT/preflight.json"
scene_rows() { python3 - "$PUBLIC_GAZEBO_CALIBRATION_PLAN" "$MODE" <<'PY'
import json,sys
plan=json.load(open(sys.argv[1])); mode=sys.argv[2]
if mode == 'pilot':
 scene=plan['scene_groups']['calibration'][0]
 if scene != 'map-0-mission-0': raise SystemExit('canonical pilot scene mismatch')
 print('calibration\t'+scene)
 raise SystemExit(0)
for role in ('calibration','holdout'):
 for scene in plan['scene_groups'][role]: print(role+'\t'+scene)
PY
}
started=false; collection_complete=false
while IFS=$'\t' read -r role scene; do
  [[ "$scene" =~ ^map-([0-9]+)-mission-([0-9]+)$ ]] || { RUNNER_EXIT_CODE=2; exit 2; }
  scene_root="$RUN_ROOT/scenes/$scene"; mkdir -p "$scene_root"
  remaining_seconds >/dev/null || { RUNNER_EXIT_CODE=124; exit 124; }; generator_budget="$(remaining_seconds)"; (( generator_budget > GENERATOR_DEADLINE_SEC )) && generator_budget="$GENERATOR_DEADLINE_SEC"
  deadline_run "$generator_budget" "$scene_root/generator.log" ros2 run sanitation_campus_scenario sanitation-campus-scenario generate --config "$ROOT/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" --profile formal --split train --map-index "${BASH_REMATCH[1]}" --mission-index "${BASH_REMATCH[2]}" --output "$scene_root/episode" || { rc=$?; PRIMARY_ERROR=scene_generator_deadline_or_failure; RUNNER_EXIT_CODE=$rc; exit "$RUNNER_EXIT_CODE"; }
  manifest="$scene_root/episode/public/episode_manifest.json"
  scene_operator_started=false; scene_stop_attempted=false; scene_stop_verified=false
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --write-scene-selector --scene-selector "$SELECTOR" --scene-id "$scene" --episode-manifest "$manifest" >"$scene_root/selector.json"
  if [[ "$started" == false ]]; then
    collector_args=(--live-output "$DATASET" --image-topic /camera/color/image_raw --camera-info-topic /camera/color/camera_info --timeout-sec "$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC" --scene-selector "$SELECTOR" --per-scene-quota "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" --progress-output "$PROGRESS")
    if [[ "$MODE" == pilot ]]; then collector_args+=(--pilot-scene "$PILOT_SCENE"); else collector_args+=(--review-receipt "$PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT" --pilot-manifest "$PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST"); fi
    "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" "${collector_args[@]}" 9>&- >"$RUN_ROOT/collector.stdout" 2>"$RUN_ROOT/collector.stderr" & collector_pid=$!; started=true
  fi
  remaining_seconds >/dev/null || { RUNNER_EXIT_CODE=124; exit 124; }
  GZ_PARTITION="tzcup_public_mobile_${ROS_DOMAIN_ID}_$$_${scene}"; export GZ_PARTITION
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus_map_lifecycle.launch.py mission_mode:=mapping gui:=false mapping_high_bandwidth_sensor_runtime:=true enable_training_gt:=true world:="$scene_root/episode/public/world.sdf" episode_manifest:="$manifest" map_artifact_dir:="$scene_root/runtime" pedestrian_schedule:="$scene_root/episode/environment/pedestrian_schedule.json" start_pedestrians:=false start_coverage:=false >"$scene_root/mapping.launch.log" 2>&1 & launch_pid=$!
  GZ_PGID="$(formal_runtime_wait_for_setsid_pgid "$launch_pid")" || { RUNNER_EXIT_CODE=125; exit 125; }
  formal_runtime_register_evidence_paths "$scene_root/memory_watchdog.json" "$scene_root/memory_watchdog.log"
  formal_runtime_start_memory_watchdog "$launch_pid" "$scene_root/memory_watchdog" || { RUNNER_EXIT_CODE=$?; exit "$RUNNER_EXIT_CODE"; }
  WATCHDOG_PID="$FORMAL_RUNTIME_MEMORY_WATCHDOG_PID"
  start_mapping_readiness "$scene_root/readiness"
  set +e; wait -n -p finished "$READINESS_PID" "$WATCHDOG_PID" "$launch_pid" "$DEADLINE_PID"; readiness_status=$?; set -e
  case "$finished" in
    "$READINESS_PID") [[ "$readiness_status" == 0 ]] || { RUNNER_EXIT_CODE=4; exit 4; }; READINESS_PID="" ;;
    "$WATCHDOG_PID") formal_runtime_record_memory_watchdog_exit "$WATCHDOG_PID" "$readiness_status" || { RUNNER_EXIT_CODE=125; exit 125; }; WATCHDOG_PID=""; formal_runtime_memory_watchdog_tripped && { RUNNER_EXIT_CODE="$FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE"; exit "$RUNNER_EXIT_CODE"; }; RUNNER_EXIT_CODE=125; exit 125 ;;
    "$DEADLINE_PID") RUNNER_EXIT_CODE=124; exit 124 ;;
    "$launch_pid") RUNNER_EXIT_CODE=4; exit 4 ;;
    *) RUNNER_EXIT_CODE=125; exit 125 ;;
  esac
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "$ROOT/scripts/collect_formal_map_lifecycle_runtime.py" --mode mapping --map-root "$scene_root/runtime" --timeout "$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC" --output "$scene_root/mapping_runtime.json" >"$scene_root/operator.log" 2>&1 & operator_pid=$!
  scene_operator_started=true
  start_quota_waiter "$scene"
  set +e; wait -n -p finished "$collector_pid" "$launch_pid" "$operator_pid" "$WATCHDOG_PID" "$DEADLINE_PID" "$QUOTA_PID"; wait_status=$?; set -e
  case "$finished" in
    "$QUOTA_PID") (( wait_status == 0 )) || { RUNNER_EXIT_CODE=125; exit 125; }; QUOTA_PID="" ;;
    "$WATCHDOG_PID") formal_runtime_record_memory_watchdog_exit "$WATCHDOG_PID" "$wait_status" || { RUNNER_EXIT_CODE=125; exit 125; }; WATCHDOG_PID=""; formal_runtime_memory_watchdog_tripped && { RUNNER_EXIT_CODE="$FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE"; exit "$RUNNER_EXIT_CODE"; }; RUNNER_EXIT_CODE=125; exit 125 ;;
    "$DEADLINE_PID") RUNNER_EXIT_CODE=124; exit 124 ;;
    "$collector_pid")
      [[ "$wait_status" == 0 && -f "$DATASET/$EXPECTED_MANIFEST" ]] || { RUNNER_EXIT_CODE=4; exit 4; }
      collection_complete=true; collector_pid="" ;;
    "$launch_pid"|"$operator_pid") RUNNER_EXIT_CODE=4; exit 4 ;;
    *) RUNNER_EXIT_CODE=125; exit 125 ;;
  esac
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --deactivate-scene-selector --scene-selector "$SELECTOR" >"$scene_root/selector_inactive.json"
  if ! stop_verified; then PRIMARY_ERROR=verified_stop_failed; RUNNER_EXIT_CODE=4; exit 4; fi
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "$operator_pid" "$launch_pid"
  launch_pid=""; operator_pid=""
  formal_runtime_stop_memory_watchdog || { PRIMARY_ERROR=memory_watchdog_cleanup_failed; RUNNER_EXIT_CODE=125; exit 125; }; WATCHDOG_PID=""
  record_scene_runtime || { PRIMARY_ERROR=scene_watchdog_evidence_invalid; RUNNER_EXIT_CODE=125; exit 125; }
  if ! stop_estop_publisher; then PRIMARY_ERROR=stop_estop_cleanup_failed; RUNNER_EXIT_CODE=4; exit 4; fi
  scene_operator_started=false; scene_stop_attempted=false; scene_stop_verified=false
  [[ "${collection_complete:-false}" == true ]] && break
done < <(scene_rows)
if [[ -n "$collector_pid" ]]; then
  set +e; wait -n -p finished "$collector_pid" "$DEADLINE_PID"; collector_status=$?; set -e
  [[ "$finished" == "$collector_pid" && "$collector_status" == 0 ]] || { RUNNER_EXIT_CODE=124; [[ "$finished" == "$collector_pid" ]] && RUNNER_EXIT_CODE=4; exit "$RUNNER_EXIT_CODE"; }
  collector_pid=""
fi
if [[ "$MODE" == pilot ]]; then
  deadline_run "$FINAL_VALIDATION_DEADLINE_SEC" "$RUN_ROOT/pilot_manifest_validation.json" python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --validate-pilot-manifest "$DATASET/pilot_manifest.json"
else
  VALIDATION_SNAPSHOT="$RUN_ROOT/full_review_validation.json"
  remaining_seconds >/dev/null && (( $(remaining_seconds) >= FINAL_VALIDATION_DEADLINE_SEC )) || { RUNNER_EXIT_CODE=124; exit 124; }
  deadline_run "$FINAL_VALIDATION_DEADLINE_SEC" "$VALIDATION_SNAPSHOT" python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --review-receipt "$PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT" --pilot-manifest "$PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST"
  ORACLE_FINAL_SUMMARY="$RUN_ROOT/preprocessing_oracle_final_validation.json"; export ORACLE_FINAL_SUMMARY
  remaining_seconds >/dev/null && (( $(remaining_seconds) >= FINAL_VALIDATION_DEADLINE_SEC )) || { RUNNER_EXIT_CODE=124; exit 124; }
  setsid python3 "$ROOT/scripts/validate_dosod_single_frame_preprocessing_oracle.py" --receipt "$PUBLIC_GAZEBO_CALIBRATION_PREPROCESSING_ORACLE" >"$ORACLE_FINAL_SUMMARY" & FINAL_ORACLE_PID=$!
  set +e; wait -n -p finished "$FINAL_ORACLE_PID" "$DEADLINE_PID"; final_oracle_status=$?; set -e
  [[ "$finished" == "$FINAL_ORACLE_PID" && "$final_oracle_status" == 0 ]] || { RUNNER_EXIT_CODE=124; [[ "$finished" == "$FINAL_ORACLE_PID" ]] && RUNNER_EXIT_CODE=125; exit "$RUNNER_EXIT_CODE"; }
fi
DESIRED_STATE="$([[ "$MODE" == pilot ]] && echo NON_FORMAL_PILOT_CAPTURED || echo NON_FORMAL_CALIBRATION_FROZEN)"
