#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
runner="$root/scripts/run_public_gazebo_dosod_calibration.sh"

bash -n "$runner"
for required in '--split train' 'run_formal_campus_runtime.sh' 'sanitation-campus-scenario generate' '--write-scene-selector' '--deactivate-scene-selector' 'zero_survivor_check' 'FORMAL_GAZEBO_LOCK_FILE="$PUBLIC_GAZEBO_CALIBRATION_LOCK"' 'PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC' 'run_formal_runtime_isolation.sh' 'formal_runtime_configure_networking'; do
  grep -Fq -- "$required" "$runner"
done
network_line="$(grep -nF -- 'formal_runtime_configure_networking' "$runner" | cut -d: -f1)"
collector_line="$(grep -nF -- '--live-output "$DATASET"' "$runner" | cut -d: -f1)"
(( network_line < collector_line ))
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
