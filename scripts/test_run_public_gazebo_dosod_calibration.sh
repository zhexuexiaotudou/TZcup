#!/usr/bin/env bash
# Route-only regression: no ROS setup, Gazebo, or collector is started.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
shim="$root/scripts/run_public_gazebo_dosod_calibration.sh"
mobile="$root/scripts/run_public_mobile_gazebo_dosod_calibration.sh"
bash -n "$shim" "$mobile"
grep -Fxq 'exec "$ROOT/scripts/run_public_mobile_gazebo_dosod_calibration.sh" "$@"' "$shim"
! grep -Eq '(^|[[:space:]])(ros2|python3|setsid)[[:space:]]' "$shim"
for required in 'PUBLIC_GAZEBO_CALIBRATION_TOTAL_TIMEOUT_SEC' 'FORMAL_MEMORY_MAX_GROUP_RSS_KIB=9437184' 'formal_runtime_start_memory_watchdog' 'wait -n -p finished' 'stop_private_group' 'binding_digest' 'validate_dosod_single_frame_preprocessing_oracle.py'; do
  grep -Fq -- "$required" "$mobile"
done
