#!/usr/bin/env bash
# Source-only bounded readiness gate for the NON_FORMAL mobile collector.
public_mobile_mapping_readiness() {
  local leader="$1" log="$2" limit="${3:-60}" expected_pgid="${4:-}"
  local ros2bin="${PUBLIC_GAZEBO_CALIBRATION_ROS2_BIN:-ros2}"
  local started=$SECONDS deadline rc=0 count=0 node topic remain pgid expected file owner
  local -a required_nodes=(/formal_campus_map_lifecycle /formal_legacy_topic_adapter /formal_vehicle_training_gt_bridge)
  [[ "$limit" =~ ^[1-9][0-9]*$ && ! -e "$log" && ! -L "$log" && ! -e "$log.receipt.json" && ! -L "$log.receipt.json" ]] || return 125
  deadline=$((SECONDS + limit))
  : >"$log"

  leader_is_live() {
    local state
    kill -0 "$leader" 2>/dev/null || return 1
    state="$(awk '{print $3}' "/proc/$leader/stat" 2>/dev/null || true)"
    [[ "$state" != Z ]]
  }
  leader_pgid_is_expected() {
    pgid="$(ps -o pgid= -p "$leader" 2>/dev/null | tr -d '[:space:]')"
    if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" == 1 ]]; then
      [[ "$expected_pgid" =~ ^[0-9]+$ && "$pgid" == "$expected_pgid" ]]
    else
      [[ "$pgid" == "$leader" ]]
    fi
  }
  group_is_live() { kill -0 -- "-$1" 2>/dev/null; }
  run_probe() {
    local name="$1" begun="$(date +%s%3N)" ret=0 zero=true state grace p until
    shift
    remain=$((deadline - SECONDS))
    file="$log.$name"
    if ((remain <= 0)); then
      ret=124
    elif [[ -e "$file" || -L "$file" ]]; then
      ret=125
    else
      setsid "$@" >"$file" 2>&1 &
      p=$!
      until=$((SECONDS + remain))
      while kill -0 "$p" 2>/dev/null && ((SECONDS < until)); do
        state="$(awk '{print $3}' "/proc/$p/stat" 2>/dev/null || true)"
        [[ "$state" == Z ]] && break
        sleep 0.05
      done
      state="$(awk '{print $3}' "/proc/$p/stat" 2>/dev/null || true)"
      if ! kill -0 "$p" 2>/dev/null || [[ "$state" == Z ]]; then
        if wait "$p"; then ret=0; else ret=$?; fi
      else
        kill -TERM -- "-$p" 2>/dev/null || true
        grace=$((deadline - SECONDS)); ((grace > 10)) && grace=10
        ((grace > 0)) && sleep "$grace"
        group_is_live "$p" && kill -KILL -- "-$p" 2>/dev/null || true
        if wait "$p"; then :; else :; fi
        ret=124
      fi
      # A completed leader is insufficient: a probe child in its private PGID is a failure.
      group_is_live "$p" && { zero=false; ret=125; }
    fi
    python3 - "$file.meta.json" "$name" "$ret" "$begun" "$zero" "$file" <<'PY'
import hashlib,json,os,sys,time
p,role,rc,begun,zero,log=sys.argv[1:]
d={"role":role,"path":log,"sha256":hashlib.sha256(open(log,"rb").read()).hexdigest() if os.path.isfile(log) else None,"elapsed_ms":time.time_ns()//1_000_000-int(begun),"returncode":int(rc),"zero_survivor":zero=="true"}
q=p+".pending."+str(os.getpid())
open(q,"w",newline="\n").write(json.dumps(d,sort_keys=True)+"\n")
os.replace(q,p)
PY
    return "$ret"
  }

  leader_is_live && leader_pgid_is_expected || rc=125
  for node in "${required_nodes[@]}"; do
    ((rc == 0 && deadline - SECONDS > 0)) || { ((rc == 0)) && rc=124; break; }
    run_probe "node-${node//\//_}.list" "$ros2bin" node list || { rc=$?; break; }
    [[ "$(grep -Fxc "$node" "$file")" == 1 ]] || { rc=2; break; }
    run_probe "node-${node//\//_}.info" "$ros2bin" node info "$node" || { rc=$?; break; }
    ((count += 2))
  done
  for topic in /camera/color/image_raw /camera/color/camera_info /g2/semantic_gt/labels_map /g2/instance_gt/labels_map; do
    ((rc == 0 && deadline - SECONDS > 0)) || { ((rc == 0)) && rc=124; break; }
    run_probe "topic-${topic//\//_}.info" "$ros2bin" topic info "$topic" --verbose || { rc=$?; break; }
    case "$topic" in */camera_info) expected='sensor_msgs/msg/CameraInfo';; *) expected='sensor_msgs/msg/Image';; esac
    case "$topic" in /camera/*) owner='formal_legacy_topic_adapter';; *) owner='formal_vehicle_training_gt_bridge';; esac
    python3 "${PUBLIC_GAZEBO_CALIBRATION_PARSER:?}" "$file" "$expected" "$owner" || { rc=2; break; }
    run_probe "topic-${topic//\//_}.message" "$ros2bin" topic echo --once "$topic" "$expected" || { rc=$?; break; }
    ((count += 1))
  done
  if ((rc == 0)); then
    run_probe clock-first "$ros2bin" topic echo --once /clock rosgraph_msgs/msg/Clock || rc=$?
    local first="$file"
    ((rc == 0)) && run_probe clock-second "$ros2bin" topic echo --once /clock rosgraph_msgs/msg/Clock || rc=$?
    local second="$file"
    ((rc == 0)) && python3 - "$first" "$second" <<'PY' || rc=$?
import re,sys
def stamp(p):
 s=open(p,encoding="utf-8").read(); a=re.search(r"sec:\s*(\d+)",s); b=re.search(r"nanosec:\s*(\d+)",s)
 if not a or not b: raise SystemExit(2)
 return int(a.group(1))*1_000_000_000+int(b.group(1))
raise SystemExit(0 if stamp(sys.argv[2])>stamp(sys.argv[1]) else 2)
PY
    ((count += 2))
  fi
  leader_is_live && leader_pgid_is_expected || rc=125
  python3 - "$log.receipt.json" "$rc" "$count" "$limit" "$((SECONDS-started))" "$log" <<'PY'
import glob,json,os,sys
p,rc,count,limit,elapsed,root=sys.argv[1:]
q=p+".pending."+str(os.getpid())
rows=[]
for path in sorted(glob.glob(root+".*.meta.json")):
 with open(path,encoding="utf-8") as f: rows.append(json.load(f))
open(q,"w",newline="\n").write(json.dumps({"status":"READY" if rc=="0" else "BLOCKED","returncode":int(rc),"command_count":int(count),"deadline_seconds":int(limit),"elapsed_seconds":int(elapsed),"operations":rows},sort_keys=True)+"\n")
os.replace(q,p)
PY
  return "$rc"
}
