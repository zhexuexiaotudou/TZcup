#!/usr/bin/env bash
set -euo pipefail

WS="${SANITATION_WS:-$HOME/sanitation_ws}"
source /opt/ros/jazzy/setup.bash

cd "$WS"
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --event-handlers console_direct+

source "$WS/install/setup.bash"
colcon test --event-handlers console_direct+ || true
colcon test-result --verbose || true
