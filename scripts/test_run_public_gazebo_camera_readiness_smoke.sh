#!/usr/bin/env bash
# Fixture-only: fixed probe uses fake rclpy; no ROS package or Gazebo runs.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"; runner="$root/scripts/run_public_gazebo_camera_readiness_smoke.sh"
bash -n "$runner"; mkdir -p "$root/.work"; fixture="$(mktemp -d "$root/.work/public-camera-smoke.XXXXXX")"
fixture_cleanup() {
 local rc=$? file pid survivor=false
 set +e
 for file in "$fixture"/repo/.work/*-state/launch.pid; do
  [[ -f "$file" ]] || continue; pid="$(<"$file")"; kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in {1..10}; do kill -0 -- "-$pid" 2>/dev/null || break; sleep .05; done
  kill -KILL -- "-$pid" 2>/dev/null || true; kill -0 -- "-$pid" 2>/dev/null && survivor=true
 done
 [[ "$survivor" == false ]] || rc=1
 if (( rc == 0 )); then rm -rf -- "$fixture"; else echo "retained fixture: $fixture" >&2; fi
 return "$rc"
}
trap fixture_cleanup EXIT
mkdir -p "$fixture/repo/scripts" "$fixture/repo/input" "$fixture/repo/bin" "$fixture/repo/fakepy/rclpy" "$fixture/repo/fakepy/sensor_msgs" "$fixture/repo/.work/locks"
cp "$runner" "$root/scripts/public_gazebo_camera_pair_readiness.py" "$fixture/repo/scripts/"
cat >"$fixture/repo/scripts/run_formal_runtime_isolation.sh" <<'EOF'
FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE=86; FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=0; FORMAL_RUNTIME_MEMORY_WATCHDOG_PID=""
formal_runtime_install_traps(){ FORMAL_RUNTIME_CLEANUP_FUNCTION="$1"; trap 'formal_runtime_exit_trap "$?"' EXIT; trap 'exit 130' INT; trap 'exit 143' TERM; }
formal_runtime_exit_trap(){ local status="$1"; trap - EXIT INT TERM; FORMAL_RUNTIME_EXIT_STATUS="$status"; "$FORMAL_RUNTIME_CLEANUP_FUNCTION" || status=125; formal_runtime_stop_memory_watchdog || true; if formal_runtime_memory_watchdog_tripped; then status=86; elif (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT != 0 )); then status=125; fi; exit "$status"; }
formal_runtime_register_evidence_paths(){ :; }; formal_runtime_configure(){ :; }; formal_runtime_cleanup_partition(){ return 0; }
formal_runtime_cleanup_groups(){ local partition="$1"; shift; local pid failed=0; for pid in "$@"; do [[ -n "$pid" ]] || continue; kill -INT -- "-$pid" 2>/dev/null || true; kill -TERM -- "-$pid" 2>/dev/null || true; for _ in {1..10}; do kill -0 -- "-$pid" 2>/dev/null || break; sleep .05; done; kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; for _ in {1..10}; do kill -0 -- "-$pid" 2>/dev/null || break; sleep .05; done; kill -0 -- "-$pid" 2>/dev/null && failed=1; done; return "$failed"; }
formal_runtime_wait_for_setsid_pgid(){ local pgid; for _ in {1..20}; do pgid="$(ps -o pgid= -p "$1" | tr -d ' ')"; [[ "$pgid" == "$1" ]] && { printf '%s\n' "$pgid"; return 0; }; sleep .01; done; return 2; }
formal_runtime_memory_preflight(){ [[ "${FAKE_MODE:-ok}" != oom ]] || return 86; : >"$1.json"; : >"$1.log"; }
formal_runtime_start_memory_watchdog(){ : >"$2.json"; : >"$2.log"; if [[ "${FAKE_MODE:-ok}" == breach ]]; then setsid bash -c 'sleep .15; exit 86' & else setsid bash -c 'trap "exit 0" TERM; while :; do sleep 1; done' & fi; FORMAL_RUNTIME_MEMORY_WATCHDOG_PID=$!; }
formal_runtime_record_memory_watchdog_exit(){ [[ "$1" == "$FORMAL_RUNTIME_MEMORY_WATCHDOG_PID" ]]; FORMAL_RUNTIME_MEMORY_WATCHDOG_PID=""; (( $2 == 86 )) && FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=86 || (( $2 == 0 )) || FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=125; }
formal_runtime_stop_memory_watchdog(){ [[ -n "$FORMAL_RUNTIME_MEMORY_WATCHDOG_PID" ]] || return 0; kill -TERM "$FORMAL_RUNTIME_MEMORY_WATCHDOG_PID" 2>/dev/null || true; set +e; wait "$FORMAL_RUNTIME_MEMORY_WATCHDOG_PID"; local rc=$?; set -e; [[ "${FAKE_MODE:-ok}" == breach ]] || rc=0; [[ "${FAKE_MODE:-ok}" != unexpected ]] || rc=7; formal_runtime_record_memory_watchdog_exit "$FORMAL_RUNTIME_MEMORY_WATCHDOG_PID" "$rc" || true; }
formal_runtime_memory_watchdog_tripped(){ (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT == 86 )); }
EOF
cat >"$fixture/repo/bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$$" >"${FAKE_STATE}/launch.pid"; trap 'printf TERM >"${FAKE_STATE}/launch.term"' TERM; trap '' INT
while :; do sleep 1; done
EOF
cat >"$fixture/repo/bin/python3" <<'EOF'
#!/usr/bin/env bash
if [[ "${FAKE_MODE:-}" == exit0 && "$*" != *--validate-report* ]]; then exit 0; fi
exec /usr/bin/python3 "$@"
EOF
chmod +x "$fixture/repo/bin/ros2" "$fixture/repo/bin/python3"
cat >"$fixture/repo/fakepy/rclpy/__init__.py" <<'EOF'
subscriptions=[]; turn=0
class Node:
 def create_subscription(self, cls, topic, callback, qos): subscriptions.append((cls,callback))
 def get_publishers_info_by_topic(self, topic):
  import os
  gid=b'\x01'*16 if os.environ.get('FAKE_MODE') != 'gid_switch' or turn == 0 else b'\x02'*16
  return [type('I',(),{'node_name':'formal_legacy_topic_adapter','node_namespace':'/','topic_type':'sensor_msgs/msg/Image' if 'image' in topic else 'sensor_msgs/msg/CameraInfo','endpoint_gid':gid})()]
 def destroy_node(self): pass
def init(): pass
def shutdown(): pass
def create_node(name): return Node()
def spin_once(node, timeout_sec=0):
 import os
 global turn; turn += 1
 if os.environ.get('FAKE_MODE') in ('deadline','unexpected'): return
 from sensor_msgs.msg import Image, CameraInfo
 if os.environ.get('FAKE_MODE') == 'gid_switch':
  for cls, callback in subscriptions:
   if turn == 1 and cls is Image: callback(Image())
   if turn == 2 and cls is CameraInfo: callback(CameraInfo())
  return
 for cls, callback in subscriptions: callback(Image() if cls is Image else CameraInfo())
EOF
printf 'qos_profile_sensor_data=object()\n' >"$fixture/repo/fakepy/rclpy/qos.py"
printf 'from .msg import Image, CameraInfo\n' >"$fixture/repo/fakepy/sensor_msgs/__init__.py"
cat >"$fixture/repo/fakepy/sensor_msgs/msg.py" <<'EOF'
class Stamp: sec=1; nanosec=2
class Header: frame_id='camera_link'; stamp=Stamp()
class Image:
 header=Header(); width=2; height=2; encoding='rgb8'; step=6; data=b'0123456789ab'
class CameraInfo:
 header=Header(); width=2; height=2; k=[1,0,0,0,1,0,0,0,1]
EOF
printf 'export AMENT_TRACE_SETUP_FILES="$AMENT_TRACE_SETUP_FILES:fixture-stage1"\nexport PATH=%q:"$PATH"\nexport PYTHONPATH=%q${PYTHONPATH:+:$PYTHONPATH}\n' "$fixture/repo/bin" "$fixture/repo/fakepy" >"$fixture/repo/input/stage1.bash"
printf 'export ROS_DISTRO=fake\nexport ROS_DOMAIN_ID=99\nexport GZ_PARTITION=setup-must-not-win\n' >"$fixture/repo/input/ros-setup.bash"; printf 'export AMENT_PREFIX_PATH=/fixture-runtime\n' >"$fixture/repo/input/runtime.bash"; printf 'export GZ_SIM_RESOURCE_PATH=/fixture-campus\n' >"$fixture/repo/input/campus.bash"; : >"$fixture/repo/input/world.sdf"; : >"$fixture/repo/input/episode_manifest.json"
printf 'return 7\n' >"$fixture/repo/input/failing-setup.bash"
printf 'if then\n' >"$fixture/repo/input/malformed-setup.bash"
printf '.work/\n__pycache__/\n' >"$fixture/repo/.gitignore"; git -C "$fixture/repo" init -q; git -C "$fixture/repo" config user.email fixture@example.invalid; git -C "$fixture/repo" config user.name fixture; git -C "$fixture/repo" add .; git -C "$fixture/repo" commit -qm fixture
set +e; FAKE_MODE=gid_switch PYTHONPATH="$fixture/repo/fakepy" /usr/bin/python3 "$fixture/repo/scripts/public_gazebo_camera_pair_readiness.py" --image-topic /camera/color/image_raw --camera-info-topic /camera/color/camera_info --timeout .05 --output "$fixture/epoch.json"; epoch_rc=$?; set -e
[[ "$epoch_rc" == 2 ]]; python3 - "$fixture/epoch.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['status']=='BLOCKED' and d['exact_fresh_pair_count']==0
PY
run_case() {
 local name="$1" mode="$2" expected="$3" total="${4:-5}" stage1_setup="${5:-$fixture/repo/input/stage1.bash}" state rootdir rc
 state="$fixture/repo/.work/$name-state"; rootdir="$fixture/repo/.work/$name-root"
 mkdir -p "$state" "$rootdir"; set +e
 SECRET_SENTINEL=do-not-persist GZ_FUEL_PASSWORD=fuel-secret ROS_AUTH_TOKEN=ros-secret PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC=/wrong FAKE_STATE="$state" FAKE_MODE="$mode" PATH="$fixture/repo/bin:$PATH" PYTHONPATH="$fixture/repo/fakepy" PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT="$rootdir" PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK="$fixture/repo/.work/locks/$name.lock" PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC="$total" PUBLIC_GAZEBO_CAMERA_SMOKE_ROS_SETUP="$fixture/repo/input/ros-setup.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP="$stage1_setup" PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP="$fixture/repo/input/runtime.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP="$fixture/repo/input/campus.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD="$fixture/repo/input/world.sdf" PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST="$fixture/repo/input/episode_manifest.json" ROS_DOMAIN_ID=81 env -u AMENT_TRACE_SETUP_FILES timeout -k 1 12s bash "$fixture/repo/scripts/run_public_gazebo_camera_readiness_smoke.sh" >"$fixture/$name.out" 2>"$fixture/$name.err"
 rc=$?; set -e; [[ "$rc" == "$expected" ]] || { cat "$fixture/$name.err" >&2; return 1; }
 if [[ -s "$state/launch.pid" ]]; then ! kill -0 -- "-$(<"$state/launch.pid")" 2>/dev/null || { echo "surviving exact fixture group: $name" >&2; return 1; }; fi
}
run_case oom oom 86; [[ ! -e "$fixture/repo/.work/oom-state/launch.pid" ]]
run_case failing_setup ok 125 5 "$fixture/repo/input/failing-setup.bash"; [[ ! -e "$fixture/repo/.work/failing_setup-state/launch.pid" ]]
run_case malformed_setup ok 125 5 "$fixture/repo/input/malformed-setup.bash"; [[ ! -e "$fixture/repo/.work/malformed_setup-state/launch.pid" ]]
run_case clean ok 0; [[ -f "$fixture/repo/.work/clean-state/launch.term" ]]; ! grep -R -E -q 'SECRET_SENTINEL|do-not-persist|fuel-secret|ros-secret' "$fixture/repo/.work/clean-root"; grep -qx 'declare -x ROS_DISTRO="fake"' "$fixture/repo/.work/clean-root/setup_environment.sh"; grep -qx 'declare -x AMENT_PREFIX_PATH="/fixture-runtime"' "$fixture/repo/.work/clean-root/setup_environment.sh"; grep -qx 'declare -x GZ_SIM_RESOURCE_PATH="/fixture-campus"' "$fixture/repo/.work/clean-root/setup_environment.sh"
python3 - "$fixture/repo/.work/clean-root/public_gazebo_camera_readiness_smoke_receipt.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['status']=='NON_FORMAL_CAMERA_READY' and d['formal_passed'] is False and d['zero_survivor_check'] and d['readiness_report']['sha256'] and d['binding_unchanged'] and set(d['inputs'])=={'ros_setup','stage1_setup','runtime_setup','campus_setup','world','manifest'}
PY
run_case breach breach 86 8; pid=$(cat "$fixture/repo/.work/breach-state/launch.pid"); ! kill -0 -- "-$pid" 2>/dev/null
python3 - "$fixture/repo/.work/breach-root/public_gazebo_camera_readiness_smoke_receipt.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['exit_code']==86
PY
run_case deadline deadline 124 2; [[ -f "$fixture/repo/.work/deadline-state/launch.term" ]]
python3 - "$fixture/repo/.work/deadline-root/public_gazebo_camera_readiness_smoke_receipt.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['exit_code']==124
PY
run_case unexpected unexpected 125 2
python3 - "$fixture/repo/.work/unexpected-root/public_gazebo_camera_readiness_smoke_receipt.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['exit_code']==125
PY
run_case exit0 exit0 125
mkdir -p "$fixture/repo/.work/stale"; : >"$fixture/repo/.work/stale/old"
if PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT="$fixture/repo/.work/stale" PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK="$fixture/repo/.work/locks/a.lock" PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC=2 PUBLIC_GAZEBO_CAMERA_SMOKE_ROS_SETUP="$fixture/repo/input/ros-setup.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP="$fixture/repo/input/stage1.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP="$fixture/repo/input/runtime.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP="$fixture/repo/input/campus.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD="$fixture/repo/input/world.sdf" PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST="$fixture/repo/input/episode_manifest.json" ROS_DOMAIN_ID=81 bash "$fixture/repo/scripts/run_public_gazebo_camera_readiness_smoke.sh" >/dev/null 2>&1; then exit 1; fi
if FORMAL_ORCHESTRATED_STEP_SESSION=1 bash "$fixture/repo/scripts/run_public_gazebo_camera_readiness_smoke.sh" >/dev/null 2>&1; then exit 1; fi
