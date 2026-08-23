#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="$repo_root/docker/compose.journey6-loopback.yaml"

# Compose interpolation is required even for down.  These placeholders are
# never mounted or executed by the down operation.
export J6_OE_BASE_IMAGE="${J6_OE_BASE_IMAGE:-scratch}"
export J6_ROS_SETUP="${J6_ROS_SETUP:-/dev/null}"
export J6_RUNTIME_BUNDLE="${J6_RUNTIME_BUNDLE:-/tmp}"
export J6_MODEL_ARTIFACTS="${J6_MODEL_ARTIFACTS:-/tmp}"
export J6_ALGORITHM_COMMAND="${J6_ALGORITHM_COMMAND:-/bin/false}"
export HIL_EVIDENCE_DIR="${HIL_EVIDENCE_DIR:-/tmp}"

docker compose -f "$compose_file" -p tzcup-j6-loopback down --remove-orphans
echo "Journey 6 loopback containers stopped; images and evidence were retained."
