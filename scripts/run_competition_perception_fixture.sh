#!/usr/bin/env bash
set -eo pipefail
: "${SOURCE:?}" "${RUNTIME:?}" "${OUTPUT:?}" "${PERCEPTION_MODEL:?}" "${PERCEPTION_OVERLAY:?}" "${PERCEPTION_DEPS:?}"
source /opt/ros/jazzy/setup.bash
source "$RUNTIME/install/setup.bash"
source "$PERCEPTION_OVERLAY/local_setup.bash"
export PYTHONPATH="$PERCEPTION_DEPS:${PYTHONPATH:-}"
source "$SOURCE/scripts/run_formal_runtime_isolation.sh"
export ROS_DOMAIN_ID=93 GZ_PARTITION=tzcup_perception_score_20260913_01
formal_runtime_configure "$ROS_DOMAIN_ID"
mkdir "$OUTPUT"
gz_pid='';bridge_pid='';perception_pid='';bag_pid=''
cleanup() {
  if [[ -n "$bag_pid" ]]; then
    kill -INT -- "-$bag_pid" 2>/dev/null || true
    for _ in {1..100}; do kill -0 "$bag_pid" 2>/dev/null || break; sleep .1; done
  fi
  formal_runtime_cleanup_groups "$GZ_PARTITION" "$perception_pid" "$bridge_pid" "$gz_pid" || true
}
trap cleanup EXIT INT TERM
setsid ros2 bag record -s mcap -o "$OUTPUT/bag" /clock /tf_static \
 /sensors/front_rgbd/depth/image_rect_raw/image /sensors/front_rgbd/depth/image_rect_raw/depth_image /sensors/front_rgbd/depth/image_rect_raw/camera_info \
 /perception/garbage/raw_detections_2d /perception/garbage/detections_2d /perception/garbage/detections_3d \
 /perception/garbage/segmentation /perception/garbage/targets /perception/garbage/diagnostics >"$OUTPUT/bag.log" 2>&1 & bag_pid=$!
setsid ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
 '/sensors/front_rgbd/depth/image_rect_raw/image@sensor_msgs/msg/Image[gz.msgs.Image' \
 '/sensors/front_rgbd/depth/image_rect_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image' \
 '/sensors/front_rgbd/depth/image_rect_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' >"$OUTPUT/bridge.log" 2>&1 & bridge_pid=$!
setsid ros2 launch sanitation_perception competition_development_perception.launch.py model_path:="$PERCEPTION_MODEL" >"$OUTPUT/perception.log" 2>&1 & perception_pid=$!
setsid env LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless gz sim -s --headless-rendering "$SOURCE/fixture/world.sdf" >"$OUTPUT/gazebo.log" 2>&1 & gz_pid=$!
python3 "$SOURCE/scripts/competition_perception_fixture_run.py" --output "$OUTPUT" >"$OUTPUT/driver.log" 2>&1
kill -INT -- "-$bag_pid" 2>/dev/null || true
for _ in {1..100}; do kill -0 "$bag_pid" 2>/dev/null || break; sleep .1; done
if kill -0 "$bag_pid" 2>/dev/null; then echo 'bag failed to seal'; exit 4; fi
bag_pid=''
timeout 15s ros2 bag info "$OUTPUT/bag" >"$OUTPUT/bag_info.txt"
