#!/usr/bin/env bash
set -eo pipefail

X="${1:?world x required}"
Y="${2:?world y required}"
source /opt/ros/jazzy/setup.bash
set -u
gz service \
  -s /world/sanitation_test_world/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req "name: 'dynamic_pedestrian_box', position: {x: ${X}, y: ${Y}, z: 0.55}"
