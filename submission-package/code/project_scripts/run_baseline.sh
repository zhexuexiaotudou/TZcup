#!/usr/bin/env bash
set -euo pipefail

WS="${SANITATION_WS:-$HOME/sanitation_ws}"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
export LINOROBOT2_BASE=4wd

exec ros2 launch sanitation_bringup sim.launch.py gui:=true
