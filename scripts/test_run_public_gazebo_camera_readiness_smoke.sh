#!/usr/bin/env bash
# Fixture-only: no ROS setup package or Gazebo executable is invoked.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
runner="$root/scripts/run_public_gazebo_camera_readiness_smoke.sh"
bash -n "$runner"

mkdir -p "$root/.work"
fixture="$(mktemp -d "$root/.work/public-camera-smoke.XXXXXX")"
trap 'rm -rf -- "$fixture"' EXIT
mkdir -p "$fixture/repo/scripts" "$fixture/input" "$fixture/bin"
cp "$runner" "$fixture/repo/scripts/run_public_gazebo_camera_readiness_smoke.sh"
cat >"$fixture/repo/scripts/run_formal_runtime_isolation.sh" <<'EOF'
FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE=86
FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=0
FORMAL_RUNTIME_SESSION_PREFIX=(setsid)
formal_runtime_register_evidence_paths(){ :; }
formal_runtime_configure(){ :; }
formal_runtime_memory_preflight(){ [[ "${FAKE_MODE:-ok}" != oom ]] || return 86; : >"$1.json"; }
formal_runtime_start_memory_watchdog(){ : >"$2.json"; }
formal_runtime_stop_memory_watchdog(){ :; }
formal_runtime_memory_watchdog_tripped(){ return 1; }
formal_runtime_cleanup_partition(){ return 0; }
formal_runtime_cleanup_groups(){ kill -TERM -- "-$2" 2>/dev/null || true; for _ in $(seq 1 20); do kill -0 -- "-$2" 2>/dev/null || return 0; sleep .02; done; kill -KILL -- "-$2" 2>/dev/null || true; ! kill -0 -- "-$2" 2>/dev/null; }
EOF
cat >"$fixture/bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$$" >"${FAKE_STATE}/launch.pid"
trap 'printf TERM >"${FAKE_STATE}/launch.term"; exit 0' TERM
while :; do sleep 1; done
EOF
chmod +x "$fixture/bin/ros2"
cat >"$fixture/readiness.py" <<'EOF'
import argparse, json, os, time
p=argparse.ArgumentParser(); p.add_argument('--image-topic'); p.add_argument('--camera-info-topic'); p.add_argument('--timeout'); p.add_argument('--output'); a=p.parse_args()
if os.environ.get('FAKE_MODE') == 'deadline': time.sleep(5)
open(a.output, 'w', encoding='utf-8').write(json.dumps({'status':'READY','exact_fresh_pair_count':1})+'\n')
EOF
printf 'export PATH=%q:"$PATH"\n' "$fixture/bin" >"$fixture/input/stage1.bash"
: >"$fixture/input/runtime.bash"; : >"$fixture/input/campus.bash"; : >"$fixture/input/world.sdf"; : >"$fixture/input/episode_manifest.json"

run_case() {
  local name="$1" mode="$2" expected="$3" total="${4:-5}" rootdir state rc
  rootdir="$fixture/$name/root"
  state="$fixture/$name/state"
  mkdir -p "$rootdir" "$state"
  set +e
  FAKE_STATE="$state" FAKE_MODE="$mode" PATH="$fixture/bin:$PATH" \
    PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT="$rootdir" PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK="$fixture/$name/lock" \
    PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC="$total" PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP="$fixture/input/stage1.bash" \
    PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP="$fixture/input/runtime.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP="$fixture/input/campus.bash" \
    PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD="$fixture/input/world.sdf" PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST="$fixture/input/episode_manifest.json" \
    PUBLIC_GAZEBO_CAMERA_SMOKE_READINESS="$fixture/readiness.py" ROS_DOMAIN_ID=81 \
    bash "$fixture/repo/scripts/run_public_gazebo_camera_readiness_smoke.sh" >"$fixture/$name/stdout" 2>"$fixture/$name/stderr"
  rc=$?
  set -e
  [[ "$rc" == "$expected" ]] || { cat "$fixture/$name/stderr" >&2; return 1; }
}

run_case oom oom 86
[[ ! -e "$fixture/oom/state/launch.pid" ]]
python3 - "$fixture/oom/root/public_gazebo_camera_readiness_smoke_receipt.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['exit_code']==86 and d['zero_survivor_check'] is True
PY
run_case deadline deadline 124 3
[[ -f "$fixture/deadline/state/launch.term" ]]
python3 - "$fixture/deadline/root/public_gazebo_camera_readiness_smoke_receipt.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['exit_code']==124 and d['zero_survivor_check'] is True
PY
run_case clean ok 0
[[ -f "$fixture/clean/state/launch.term" ]]
python3 - "$fixture/clean/root/public_gazebo_camera_readiness_smoke_receipt.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1])); assert d['status']=='NON_FORMAL_CAMERA_READY' and d['formal_passed'] is False and d['zero_survivor_check'] is True
PY
mkdir -p "$fixture/stale/root"; : >"$fixture/stale/root/existing-evidence.json"
if FAKE_STATE="$fixture/stale/state" PATH="$fixture/bin:$PATH" PUBLIC_GAZEBO_CAMERA_SMOKE_OUTPUT="$fixture/stale/root" PUBLIC_GAZEBO_CAMERA_SMOKE_LOCK="$fixture/stale/lock" PUBLIC_GAZEBO_CAMERA_SMOKE_TOTAL_TIMEOUT_SEC=1 PUBLIC_GAZEBO_CAMERA_SMOKE_STAGE1_SETUP="$fixture/input/stage1.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_RUNTIME_SETUP="$fixture/input/runtime.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_CAMPUS_SETUP="$fixture/input/campus.bash" PUBLIC_GAZEBO_CAMERA_SMOKE_WORLD="$fixture/input/world.sdf" PUBLIC_GAZEBO_CAMERA_SMOKE_MANIFEST="$fixture/input/episode_manifest.json" ROS_DOMAIN_ID=81 bash "$fixture/repo/scripts/run_public_gazebo_camera_readiness_smoke.sh" >/dev/null 2>&1; then
  echo 'stale root unexpectedly admitted' >&2; exit 1
fi
