#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/tzcup/install/setup.bash
set -u
if [[ $# -eq 0 ]]; then
  set -- ros2 run sanitation_perception product_perception_node --ros-args \
    -p "pipeline_manifest:=${PERCEPTION_PIPELINE_MANIFEST:?}" \
    -p "artifact_root:=${PERCEPTION_MODEL_ROOT:?}"
fi
exec "$@"
