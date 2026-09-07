#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
runner="$root/scripts/run_public_gazebo_dosod_calibration.sh"

bash -n "$runner"
for required in '--split train' 'run_formal_campus_runtime.sh' 'sanitation-campus-scenario generate' '--write-scene-selector' '--deactivate-scene-selector' 'zero_survivor_check' 'FORMAL_GAZEBO_LOCK_FILE="$PUBLIC_GAZEBO_CALIBRATION_LOCK"' 'PUBLIC_GAZEBO_CALIBRATION_CAMERA_INFO_TOPIC'; do
  grep -Fq -- "$required" "$runner"
done
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
fi
