#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
runner="$root/scripts/run_public_gazebo_dosod_calibration.sh"

bash -n "$runner"
for required in '--split train' 'run_formal_campus_runtime.sh' 'sanitation-campus-scenario generate' '--write-scene-selector' '--deactivate-scene-selector' 'zero_survivor_check' 'wall_deadline_seconds' 'FORMAL_GAZEBO_LOCK_FILE="$PUBLIC_GAZEBO_CALIBRATION_LOCK"' 'PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC' 'PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC' 'run_formal_runtime_isolation.sh' 'formal_runtime_configure_networking' 'formal_runtime_memory_preflight' 'formal_runtime_start_memory_watchdog' 'formal_runtime_stop_memory_watchdog' 'formal_runtime_register_evidence_paths' 'FORMAL_ORCHESTRATED_STEP_SESSION=1' 'FORMAL_ORCHESTRATED_STEP_SESSION_TOKEN' 'setsid python3' 'stop_exact_group' 'wait_until_deadline'; do
  grep -Fq -- "$required" "$runner"
done
network_line="$(grep -nF -- 'formal_runtime_configure_networking' "$runner" | cut -d: -f1)"
collector_line="$(grep -nF -- '--live-output "$DATASET"' "$runner" | cut -d: -f1)"
(( network_line < collector_line ))
preflight_line="$(grep -nF -- 'formal_runtime_memory_preflight' "$runner" | head -n1 | cut -d: -f1)"
collector_line="$(grep -nF -- 'setsid python3 "$ROOT/scripts/public_gazebo_dosod_calibration.py"' "$runner" | cut -d: -f1)"
campus_line="$(grep -nF -- 'setsid bash "$ROOT/scripts/run_formal_campus_runtime.sh"' "$runner" | cut -d: -f1)"
watchdog_line="$(grep -nF -- 'formal_runtime_start_memory_watchdog "$ACTIVE_CAMPUS_PID"' "$runner" | cut -d: -f1)"
(( preflight_line < collector_line && collector_line < campus_line && campus_line < watchdog_line ))
grep -Fq -- 'for signal in TERM KILL; do' "$runner"
grep -Fq -- 'kill -0 -- "-$pgid"' "$runner"
for forbidden in 'ground_truth' 'evaluator' 'hidden' 'replay' 'cmd_vel' 'SetEntityPose' 'ros2 service'; do
  ! grep -Fq -- "$forbidden" "$runner"
done

# Missing required bindings must fail before a ROS/Gazebo command can run.
if env -i PATH="$PATH" bash "$runner" >/dev/null 2>&1; then
  echo 'runner unexpectedly accepted an unbound invocation' >&2
  exit 1
fi

# The established formal child is the sole lock owner; a concurrent child for
# that exact file must fail while the first child is live.
if command -v flock >/dev/null; then
  lock="$(mktemp)"; ready="$(mktemp)"; release="$(mktemp)"
  rm -f -- "$ready" "$release"
  trap 'rm -f -- "$lock" "$ready" "$release"' EXIT
  bash -c 'exec 9>"$1"; flock -n 9; : >"$2"; while [[ ! -e "$3" ]]; do sleep 0.02; done' _ "$lock" "$ready" "$release" &
  owner_pid=$!
  for _ in $(seq 1 100); do [[ -e "$ready" ]] && break; sleep 0.02; done
  [[ -e "$ready" ]] || { echo 'formal child lock owner never became ready' >&2; exit 1; }
  if bash -c 'exec 9>"$1"; flock -n 9' _ "$lock"; then
    echo 'independent child acquired the active formal lock' >&2; exit 1
  fi
  : >"$release"; wait "$owner_pid"
  rm -f -- "$lock" "$ready" "$release"
  trap - EXIT
fi

# Exercise the runner's resource paths in a private fake repository.  The
# stubs never source ROS or launch Gazebo; the collector is only a setsid sleep
# process so TERM-to-KILL and exact-PGID cleanup remain observable.
if command -v setsid >/dev/null; then
  resource_root="$(mktemp -d "$root/.work/public-resource-guard.XXXXXX")"
  trap 'rm -rf -- "$resource_root"' EXIT
  mkdir -p "$resource_root/repo/scripts" "$resource_root/repo/config" "$resource_root/bin" "$resource_root/input"
  cp "$runner" "$resource_root/repo/scripts/run_public_gazebo_dosod_calibration.sh"
  cat >"$resource_root/repo/scripts/run_formal_campus_runtime.sh" <<'EOF'
#!/usr/bin/env bash
if [[ "${FAKE_CAMPUS_HANG:-0}" == 1 ]]; then
  printf '%s\n' "$$" >"${FAKE_STATE}/campus.pid"
  trap 'printf TERM >"${FAKE_STATE}/campus.term"' TERM
  while true; do sleep 1; done
fi
EOF
  chmod +x "$resource_root/repo/scripts/run_formal_campus_runtime.sh"
  : >"$resource_root/repo/config/dosod_s100p_hbm_compile_contract.json"
  : >"$resource_root/input/plan.json"
  printf 'export PATH=%q:"$PATH"\n' "$resource_root/bin" >"$resource_root/input/stage1.bash"
  : >"$resource_root/input/runtime.bash"
  : >"$resource_root/input/campus.bash"

  cat >"$resource_root/repo/scripts/run_formal_runtime_isolation.sh" <<'EOF'
FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=0
formal_runtime_install_traps() { trap "$1" EXIT; trap 'exit 130' INT; trap 'exit 143' TERM; }
formal_runtime_configure_networking() { :; }
formal_runtime_memory_preflight() {
  [[ "${FAKE_PREFLIGHT:-ok}" != reject ]] || return 86
  [[ "${FAKE_PREFLIGHT:-ok}" != stale ]] || { : >"$1.json"; return 2; }
}
formal_runtime_start_memory_watchdog() {
  printf '%s\n' "$1" >"${FAKE_STATE}/watchdog.target"
  case "${FAKE_WATCHDOG:-ok}" in
    breach) sleep 0.1; return 86 ;;
    stale) : >"$2.json"; return 2 ;;
    ok) return 0 ;;
  esac
}
formal_runtime_stop_memory_watchdog() { :; }
formal_runtime_register_evidence_paths() { :; }
formal_runtime_cleanup_groups() {
  local partition="$1" pid="$2"
  printf '%s %s\n' "$partition" "$pid" >>"${FAKE_STATE}/cleanup.log"
  kill -TERM -- "-$pid" 2>/dev/null || true
  sleep 0.1
  kill -KILL -- "-$pid" 2>/dev/null || true
  ! kill -0 -- "-$pid" 2>/dev/null
}
EOF
  cat >"$resource_root/bin/python3" <<'EOF'
#!/usr/bin/env bash
set -eu
if [[ "$1" == '-' ]]; then
  if (( $# == 2 )); then printf 'calibration\tmap-0-mission-0\n'; exit 0; fi
  if (( $# == 4 )); then exit 0; fi
  printf '{"status":"%s","exit_code":%s,"wall_deadline_seconds":%s,"zero_survivor_check":%s}\n' "$3" "$4" "$PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC" "$6" >"$2"
  exit 0
fi
case " $* " in
  *' --live-output '*)
    printf '%s\n' "$$" >"${FAKE_STATE}/collector.pid"
    trap 'printf TERM >"${FAKE_STATE}/collector.term"' TERM
    while true; do sleep 1; done
    ;;
  *) exit 0 ;;
esac
EOF
cat >"$resource_root/bin/ros2" <<'EOF'
#!/usr/bin/env bash
set -Eeuo pipefail
if [[ "${1:-}" == run ]]; then
  output=""
  shift 3
  while (($#)); do
    if [[ "$1" == --output ]]; then
      output="$2"
      break
    fi
    shift
  done
  if [[ -n "$output" ]]; then
    mkdir -p "$output/public"
    : >"$output/public/episode_manifest.json"
  fi
fi
exit 0
EOF
  chmod +x "$resource_root/bin/python3" "$resource_root/bin/ros2"

  run_resource_case() {
    local name="$1" expected="$2" preflight="$3" watchdog="$4" total="$5" campus_hang="$6"
    local state run receipt rc pid
    state="$resource_root/$name"
    run="$state/run"
    receipt="$run/public_gazebo_dosod_calibration_receipt.json"
    mkdir -p "$state" "$run"
    set +e
    FAKE_STATE="$state" FAKE_PREFLIGHT="$preflight" FAKE_WATCHDOG="$watchdog" FAKE_CAMPUS_HANG="$campus_hang" PATH="$resource_root/bin:$PATH" \
      PUBLIC_GAZEBO_CALIBRATION_PLAN="$resource_root/input/plan.json" PUBLIC_GAZEBO_CALIBRATION_OUTPUT="$run" \
      PUBLIC_GAZEBO_CALIBRATION_LOCK="$state/lock" PUBLIC_GAZEBO_CALIBRATION_IMAGE_TOPIC=/camera/color/image_raw \
      PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC=/camera/color/camera_info PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC=5 \
      PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC="$total" PUBLIC_GAZEBO_CALIBRATION_PER_SCENE_QUOTA=1 \
      PUBLIC_GAZEBO_CALIBRATION_STAGE1_SETUP="$resource_root/input/stage1.bash" \
      PUBLIC_GAZEBO_CALIBRATION_RUNTIME_SETUP="$resource_root/input/runtime.bash" \
      PUBLIC_GAZEBO_CALIBRATION_CAMPUS_SETUP="$resource_root/input/campus.bash" ROS_DOMAIN_ID=81 \
      bash "$resource_root/repo/scripts/run_public_gazebo_dosod_calibration.sh" >"$state/stdout" 2>"$state/stderr"
    rc=$?
    set -e
    if [[ "$rc" != "$expected" ]]; then
      echo "resource fixture ${name}: expected rc ${expected}, got ${rc}" >&2
      cat "$state/stderr" >&2 || true
      return 1
    fi
    [[ "$rc" == "$expected" ]]
    [[ -f "$receipt" ]]
    grep -Fq '"wall_deadline_seconds":' "$receipt"
    grep -Fq '"zero_survivor_check":true' "$receipt"
    if [[ -f "$state/collector.pid" ]]; then
      pid="$(<"$state/collector.pid")"
      [[ -f "$state/collector.term" ]]
      ! kill -0 -- "-$pid" 2>/dev/null
    fi
  }

  run_resource_case preflight-reject 86 reject ok 5 0
  [[ ! -e "$resource_root/preflight-reject/collector.pid" ]]
  run_resource_case stale-preflight 2 stale ok 5 0
  [[ ! -e "$resource_root/stale-preflight/collector.pid" ]]
  run_resource_case watchdog-breach 86 ok breach 5 1
  [[ "$(<"$resource_root/watchdog-breach/watchdog.target")" == "$(<"$resource_root/watchdog-breach/campus.pid")" ]]
  [[ -f "$resource_root/watchdog-breach/campus.term" ]]
  ! kill -0 -- "-$(<"$resource_root/watchdog-breach/campus.pid")" 2>/dev/null
  run_resource_case watchdog-stale 2 ok stale 5 1
  run_resource_case collector-deadline 124 ok ok 1 0
  trap - EXIT
  rm -rf -- "$resource_root"
fi

# The supervisor configures networking without taking the Gazebo lock. A real
# collector-like child inherits that environment, while the formal child keeps
# sole ownership of the unchanged lock.
if command -v flock >/dev/null; then
  mkdir -p -- "$root/.work"
  test_root="$(mktemp -d "$root/.work/public-dds-lock-test.XXXXXX")"
  lock="$test_root/lock"; ready="$test_root/ready"; release="$test_root/release"
  owner_pid=""
  cleanup_network_lock_test() {
    if [[ -n "$owner_pid" ]] && kill -0 "$owner_pid" 2>/dev/null; then
      kill -TERM "$owner_pid" 2>/dev/null || true
      wait "$owner_pid" 2>/dev/null || true
    fi
    rm -f -- "$lock" "$ready" "$release"
    rmdir -- "$test_root" 2>/dev/null || true
  }
  trap cleanup_network_lock_test EXIT
  source "$root/scripts/run_formal_runtime_isolation.sh"
  formal_runtime_configure_networking
  bash -c '[[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp && "$ROS_AUTOMATIC_DISCOVERY_RANGE" == LOCALHOST && "$CYCLONEDDS_URI" == file://*/config/cyclonedds_localhost.xml ]]' &
  collector_pid=$!
  wait "$collector_pid"
  bash -c 'exec 8>"$1"; flock -n 8' _ "$lock"
  FORMAL_GAZEBO_LOCK_FILE="$lock" bash -c 'set -euo pipefail; source "$1"; formal_runtime_configure 81; : >"$2"; for _ in $(seq 1 250); do [[ -e "$3" ]] && exit 0; sleep 0.02; done; exit 124' _ "$root/scripts/run_formal_runtime_isolation.sh" "$ready" "$release" &
  owner_pid=$!
  for _ in $(seq 1 100); do [[ -e "$ready" ]] && break; sleep 0.02; done
  [[ -e "$ready" ]] || { echo 'formal child lock owner never became ready' >&2; exit 1; }
  if bash -c 'exec 8>"$1"; flock -n 8' _ "$lock"; then
    echo 'independent child acquired the active formal lock' >&2; exit 1
  fi
  : >"$release"; wait "$owner_pid"; owner_pid=""
  cleanup_network_lock_test
  trap - EXIT
fi
