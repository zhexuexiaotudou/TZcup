#!/usr/bin/env bash
set -eo pipefail

X="${1:?world x required}"
Y="${2:?world y required}"
WORLD_NAME="${3:-${WORLD_NAME:-sanitation_structured_world}}"
MODEL_NAME="${4:-${MODEL_NAME:-dynamic_pedestrian_box}}"
SERVICE_TIMEOUT_MS="${5:-${SERVICE_TIMEOUT_MS:-3000}}"
source /opt/ros/jazzy/setup.bash
set -u
service="/world/${WORLD_NAME}/set_pose"
if ! gz service -l | grep -Fxq "${service}"; then
  echo "missing Gazebo service: ${service}" >&2
  exit 4
fi
gz service \
  -s "${service}" \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout "${SERVICE_TIMEOUT_MS}" \
  --req "name: '${MODEL_NAME}', position: {x: ${X}, y: ${Y}, z: 0.55}"
