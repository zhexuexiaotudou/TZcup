#!/usr/bin/env bash
# NON_FORMAL public-train mobile camera collection; the collector never controls Nav2.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/scripts/run_formal_runtime_isolation.sh"
: "${PUBLIC_GAZEBO_CALIBRATION_PLAN:?}" "${PUBLIC_GAZEBO_CALIBRATION_OUTPUT:?}"
: "${PUBLIC_GAZEBO_CALIBRATION_LOCK:?}" "${PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC:?}"
: "${PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA:?}" "${PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP:?}"
: "${PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP:?}" "${PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP:?}" "${ROS_DOMAIN_ID:?}"
MODE="${PUBLIC_GAZEBO_CALIBRATION_MODE:-full}"
[[ "$MODE" == pilot || "$MODE" == full ]] || { echo 'BLOCKED: mode must be pilot or full' >&2; exit 2; }
PILOT_SCENE="map-0-mission-0"
if [[ "$MODE" == pilot ]]; then
  [[ "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" == 25 ]] || { echo 'BLOCKED: pilot quota must be 25' >&2; exit 2; }
  EXPECTED_MANIFEST="pilot_manifest.json"
else
  : "${PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT:?}" "${PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST:?}"
  EXPECTED_MANIFEST="calibration_manifest.json"
fi
RUN_ROOT="$(realpath --no-symlinks -e "$PUBLIC_GAZEBO_CALIBRATION_OUTPUT")"
[[ -d "$RUN_ROOT" && ! -L "$RUN_ROOT" && -z "$(find "$RUN_ROOT" -mindepth 1 -print -quit)" ]] || { echo 'BLOCKED: fresh empty run root required' >&2; exit 2; }
for required in "$PUBLIC_GAZEBO_CALIBRATION_PLAN" "$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP" "$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP" "$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP"; do [[ -f "$required" && ! -L "$required" ]] || { echo "BLOCKED: required regular file missing: $required" >&2; exit 2; }; done
set +u
source /opt/ros/jazzy/setup.bash
source "$PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP"; source "$PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP"; source "$PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP"
set -u
FORMAL_GAZEBO_LOCK_FILE="$PUBLIC_GAZEBO_CALIBRATION_LOCK" formal_runtime_configure "$ROS_DOMAIN_ID"
export TZCUP_REPOSITORY_ROOT="$ROOT"
DATASET="$RUN_ROOT/dataset"; SELECTOR="$RUN_ROOT/scene_selector.json"; PROGRESS="$RUN_ROOT/collector_progress.json"
RECEIPT="$RUN_ROOT/public_mobile_gazebo_dosod_calibration_receipt.json"; PRIMARY_ERROR=""; DESIRED_STATE="BLOCKED"
collector_pid=""; launch_pid=""; operator_pid=""; stop_estop_pid=""
scene_operator_started=false; scene_stop_attempted=false; scene_stop_verified=false
RUNNER_EXIT_CODE=0
VALIDATION_SNAPSHOT=""
write_receipt() { python3 - "$RECEIPT" "$1" "$2" "$PRIMARY_ERROR" "$3" "$MODE" "$DATASET" "$EXPECTED_MANIFEST" "${PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT:-}" "${PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST:-}" "${VALIDATION_SNAPSHOT:-}" <<'PY'
import hashlib,json,os,sys
from pathlib import Path
p,status,code,error,zero,mode,dataset,expected,review,pilot,snapshot=(Path(sys.argv[1]),sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5],sys.argv[6],Path(sys.argv[7]),sys.argv[8],sys.argv[9],sys.argv[10],sys.argv[11])
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def regular(path): return path.is_file() and not path.is_symlink()
v={'report_id':'tzcup_public_mobile_gazebo_dosod_calibration_runner_v1','status':status,'formal_passed':False,'classification':'NON_FORMAL','exit_code':int(code),'primary_error':error,'zero_survivor_check':zero=='true','mode':mode}
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
   if data.get('status') != 'FROZEN' or not all(regular(Path(x)) for x in (review,pilot,snapshot)): raise ValueError('full')
   checked=json.loads(Path(snapshot).read_text())
   pilot_data=json.loads(Path(pilot).read_text()); sheet=pilot_data.get('contact_sheet',{}); sheet_path=Path(pilot).parent/sheet.get('relative_path','')
   if checked.get('status') != 'NON_FORMAL_REVIEW_APPROVED' or checked.get('review_receipt_sha256') != digest(Path(review)) or checked.get('pilot_manifest_sha256') != digest(Path(pilot)) or checked.get('contact_sheet_sha256') != sheet.get('sha256') or checked.get('record_sha256') != pilot_data.get('record_sha256') or not regular(sheet_path) or digest(sheet_path) != checked['contact_sheet_sha256']: raise ValueError('full_snapshot')
   artifact.update({'review_receipt_sha256':checked['review_receipt_sha256'],'pilot_manifest_sha256':checked['pilot_manifest_sha256'],'contact_sheet_sha256':checked['contact_sheet_sha256'],'record_sha256':checked['record_sha256'],'review_validation_snapshot_sha256':digest(Path(snapshot))})
  v['artifact']=artifact
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
  local survivor=true cleanup_failed=0 state="$DESIRED_STATE" pid receipt_code="$RUNNER_EXIT_CODE"
  trap - ERR
  set +e
  if [[ -e "$SELECTOR" ]]; then
    python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --deactivate-scene-selector --scene-selector "$SELECTOR" >/dev/null || cleanup_failed=1
  fi
  if [[ "$scene_operator_started" == true && "$scene_stop_verified" != true ]] && [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    if [[ "$scene_stop_attempted" != true ]]; then stop_verified || cleanup_failed=1; fi
    [[ "$scene_stop_verified" == true ]] || cleanup_failed=1
  fi
  formal_runtime_cleanup_groups "${GZ_PARTITION:-}" "$launch_pid" "$operator_pid" "$collector_pid" || cleanup_failed=1
  for pid in "$launch_pid" "$operator_pid" "$collector_pid"; do
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && survivor=false
  done
  launch_pid=""; operator_pid=""; collector_pid=""
  stop_estop_publisher || cleanup_failed=1
  if [[ "$survivor" != true || "$cleanup_failed" != 0 ]]; then cleanup_failed=1; state=BLOCKED; receipt_code=125; fi
  write_receipt "$state" "$receipt_code" "$survivor" || cleanup_failed=1
  (( cleanup_failed == 0 ))
}
trap 'RUNNER_EXIT_CODE=$?; PRIMARY_ERROR="${BASH_COMMAND}"' ERR
formal_runtime_register_evidence_paths "$RUN_ROOT"
formal_runtime_install_traps cleanup
trap 'RUNNER_EXIT_CODE=$?; formal_runtime_exit_trap "$RUNNER_EXIT_CODE"' EXIT
trap 'RUNNER_EXIT_CODE=130; PRIMARY_ERROR=signal:INT; exit 130' INT
trap 'RUNNER_EXIT_CODE=143; PRIMARY_ERROR=signal:TERM; exit 143' TERM
preflight_args=(--scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json")
if [[ "$MODE" == full ]]; then preflight_args+=(--review-receipt "$PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT" --pilot-manifest "$PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST"); fi
python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" "${preflight_args[@]}" >"$RUN_ROOT/preflight.json"
whole_deadline=$((SECONDS + PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC))
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
started=false
while IFS=$'\t' read -r role scene; do
  [[ "$scene" =~ ^map-([0-9]+)-mission-([0-9]+)$ ]] || { RUNNER_EXIT_CODE=2; exit 2; }
  scene_root="$RUN_ROOT/scenes/$scene"; mkdir -p "$scene_root"
  ros2 run sanitation_campus_scenario sanitation-campus-scenario generate --config "$ROOT/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" --profile formal --split train --map-index "${BASH_REMATCH[1]}" --mission-index "${BASH_REMATCH[2]}" --output "$scene_root/episode"
  manifest="$scene_root/episode/public/episode_manifest.json"
  scene_operator_started=false; scene_stop_attempted=false; scene_stop_verified=false
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --write-scene-selector --scene-selector "$SELECTOR" --scene-id "$scene" --episode-manifest "$manifest" >"$scene_root/selector.json"
  if [[ "$started" == false ]]; then
    collector_args=(--live-output "$DATASET" --image-topic /camera/color/image_raw --camera-info-topic /camera/color/camera_info --timeout-sec "$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC" --scene-selector "$SELECTOR" --per-scene-quota "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" --progress-output "$PROGRESS")
    if [[ "$MODE" == pilot ]]; then collector_args+=(--pilot-scene "$PILOT_SCENE"); else collector_args+=(--review-receipt "$PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT" --pilot-manifest "$PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST"); fi
    "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" "${collector_args[@]}" 9>&- >"$RUN_ROOT/collector.stdout" 2>"$RUN_ROOT/collector.stderr" & collector_pid=$!; started=true
  fi
  GZ_PARTITION="tzcup_public_mobile_${ROS_DOMAIN_ID}_$$_${scene}"; export GZ_PARTITION
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus_map_lifecycle.launch.py mission_mode:=mapping gui:=false mapping_high_bandwidth_sensor_runtime:=true world:="$scene_root/episode/public/world.sdf" episode_manifest:="$manifest" map_artifact_dir:="$scene_root/runtime" pedestrian_schedule:="$scene_root/episode/environment/pedestrian_schedule.json" start_pedestrians:=false start_coverage:=false >"$scene_root/mapping.launch.log" 2>&1 & launch_pid=$!
  sleep 30
  if ! kill -0 "$launch_pid" 2>/dev/null; then if wait "$launch_pid"; then rc=0; else rc=$?; fi; echo "BLOCKED: mapping launch exited before operator rc=$rc" >&2; RUNNER_EXIT_CODE=4; exit 4; fi
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "$ROOT/scripts/collect_formal_map_lifecycle_runtime.py" --mode mapping --map-root "$scene_root/runtime" --timeout "$PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC" --output "$scene_root/mapping_runtime.json" >"$scene_root/operator.log" 2>&1 & operator_pid=$!
  scene_operator_started=true
  while kill -0 "$launch_pid" 2>/dev/null && kill -0 "$collector_pid" 2>/dev/null && kill -0 "$operator_pid" 2>/dev/null && (( SECONDS < whole_deadline )); do python3 - "$PROGRESS" "$scene" "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" <<'PY' && break || true
import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception: raise SystemExit(1)
raise SystemExit(0 if max(d.get('calibration_scene_counts',{}).get(sys.argv[2],0),d.get('holdout_scene_counts',{}).get(sys.argv[2],0)) >= int(sys.argv[3]) else 1)
PY
    [[ -f "$scene_root/runtime/map_lifecycle_manifest.json" ]] && break; sleep 1
  done
  if ! kill -0 "$collector_pid" 2>/dev/null; then
    if wait "$collector_pid"; then rc=0; else rc=$?; fi
    if [[ "$rc" == 0 && -f "$DATASET/$EXPECTED_MANIFEST" ]] && python3 - "$PROGRESS" <<'PY'
import json,sys
try: raise SystemExit(0 if json.load(open(sys.argv[1])).get('collection_complete') is True else 1)
except Exception: raise SystemExit(1)
PY
    then collector_pid=""; break; fi
    echo "BLOCKED: collector exited before complete dataset rc=$rc" >&2; RUNNER_EXIT_CODE=4; exit 4
  fi
  if ! python3 - "$PROGRESS" "$scene" "$PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA" <<'PY'
import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception: raise SystemExit(1)
raise SystemExit(0 if max(d.get('calibration_scene_counts',{}).get(sys.argv[2],0),d.get('holdout_scene_counts',{}).get(sys.argv[2],0)) >= int(sys.argv[3]) else 1)
PY
  then
    [[ -f "$scene_root/runtime/map_lifecycle_manifest.json" ]] && reason=map_sealed_before_quota || reason=deadline_or_launch_exit
    python3 - "$scene_root/scene_result.json" "$reason" <<'PY'
import json,sys
open(sys.argv[1],'w').write(json.dumps({'status':'BLOCKED','reason':sys.argv[2]},sort_keys=True)+'\n')
PY
    echo "BLOCKED: $reason" >&2; RUNNER_EXIT_CODE=4; exit 4
  fi
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --deactivate-scene-selector --scene-selector "$SELECTOR" >"$scene_root/selector_inactive.json"
  if ! stop_verified; then PRIMARY_ERROR=verified_stop_failed; RUNNER_EXIT_CODE=4; exit 4; fi
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "$operator_pid" "$launch_pid"
  launch_pid=""; operator_pid=""
  if ! stop_estop_publisher; then PRIMARY_ERROR=stop_estop_cleanup_failed; RUNNER_EXIT_CODE=4; exit 4; fi
  scene_operator_started=false; scene_stop_attempted=false; scene_stop_verified=false
done < <(scene_rows)
if [[ -n "$collector_pid" ]]; then wait "$collector_pid"; collector_pid=""; fi
if [[ "$MODE" == pilot ]]; then
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --validate-pilot-manifest "$DATASET/pilot_manifest.json" >"$RUN_ROOT/pilot_manifest_validation.json"
else
  VALIDATION_SNAPSHOT="$RUN_ROOT/full_review_validation.json"
  python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py" --scene-plan "$PUBLIC_GAZEBO_CALIBRATION_PLAN" --contract "$ROOT/config/dosod_s100p_hbm_compile_contract.json" --review-receipt "$PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT" --pilot-manifest "$PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST" >"$VALIDATION_SNAPSHOT"
fi
DESIRED_STATE="$([[ "$MODE" == pilot ]] && echo NON_FORMAL_PILOT_CAPTURED || echo NON_FORMAL_CALIBRATION_FROZEN)"
