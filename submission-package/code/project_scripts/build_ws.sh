#!/usr/bin/env bash
set -euo pipefail

WS="${SANITATION_WS:-$HOME/sanitation_ws}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "$WS"
read -r -a rosdep_skip_keys <<< "${ROSDEP_SKIP_KEYS:-micro_ros_agent}"
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "${rosdep_skip_keys[@]}"

colcon build --symlink-install \
  --event-handlers console_direct+

set +u
source "$WS/install/setup.bash"
set -u

# The pinned linorobot2_gazebo package contains no pytest tests, which makes
# pytest exit with code 5.  CMake xmllint tests fetch the ROS package schema
# over the network, so run the remaining upstream/project tests here and keep
# XML well-formedness deterministic with the offline pass below.
colcon test \
  --packages-skip linorobot2_gazebo \
  --ctest-args -E '^xmllint$' \
  --event-handlers console_direct+
colcon test-result --all --verbose

while IFS= read -r -d '' xml_file; do
  xmllint --noout "$xml_file"
done < <(
  find "$WS/src" -type f \
    \( -name 'package.xml' -o -name '*.sdf' -o -name '*.urdf' -o -name '*.xacro' \) \
    -print0
)
