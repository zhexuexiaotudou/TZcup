#!/usr/bin/env bash
# Compatibility entrypoint.  The mobile runner is the sole calibration owner.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec "$ROOT/scripts/run_public_mobile_gazebo_dosod_calibration.sh" "$@"
