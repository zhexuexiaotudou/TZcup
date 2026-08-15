#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/tzcup/install/setup.bash
set -u
payload="$(timeout 3 ros2 topic echo --once /perception/product/health std_msgs/msg/String)"
grep -q 'perception_spot_clean_allowed' <<<"${payload}"
grep -Eq 'state.*(ACTIVE|INACTIVE)' <<<"${payload}"
