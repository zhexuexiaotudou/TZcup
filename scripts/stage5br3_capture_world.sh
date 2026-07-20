#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /work/.work/stage1_20260714_154523/install/setup.bash
source /stage5br3/.runtime_ws/install/setup.bash
set -u

ROOT=/stage5br3
DATA_ROOT="${STAGE5BR3_DATA_ROOT:-/data/g2_screening_native}"
WORLD_ID="${STAGE5BR3_WORLD_ID:?required}"
SCENE_START="${STAGE5BR3_SCENE_START:?required}"
SCENE_END="${STAGE5BR3_SCENE_END:?required}"
WORLD="${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/${WORLD_ID}.sdf"
LOG_ROOT="${DATA_ROOT}/logs/${WORLD_ID}"
mkdir -p "${LOG_ROOT}"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

XACRO=$(ros2 pkg prefix sanitation_vehicle_description)/share/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro
xacro "${XACRO}" enable_training_gt:=true > /tmp/stage5br3_vehicle.urdf
gz sim -r -s --headless-rendering "${WORLD}" >"${LOG_ROOT}/gz.log" 2>&1 & pids+=("$!")
for _ in $(seq 1 80); do gz service -l 2>/dev/null | grep -q "/world/${WORLD_ID}/create" && break; sleep .25; done
ros2 run ros_gz_sim create -world "${WORLD_ID}" -file /tmp/stage5br3_vehicle.urdf -name sanitation_vehicle -x -8 -y 0 -z .18 >"${LOG_ROOT}/spawn.log" 2>&1
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist' \
  '/ground_truth/model_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/semantic_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' '/g2/instance_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args -r /camera/camera_info:=/camera/color/camera_info -r /camera/image:=/camera/color/image_raw \
  -r /camera/depth_image:=/camera/depth/image_rect_raw -r /g2/semantic_gt/labels_map:=/ground_truth/semantic/image \
  -r /g2/instance_gt/labels_map:=/ground_truth/instance/image >"${LOG_ROOT}/bridge.log" 2>&1 & pids+=("$!")
sleep 5

for seed in $(seq "${SCENE_START}" "${SCENE_END}"); do
  scene=$(printf 'scene_%04d' "${seed}")
  out="${DATA_ROOT}/scenes/${scene}"
  mkdir -p "${out}"
  echo "${WORLD_ID} ${scene}"
  ros2 run sanitation_learning stage5br3_randomize_scene \
    --manifest "${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json" \
    --world-id "${WORLD_ID}" --scene-seed "${seed}" --output "${out}/scene_manifest.json" \
    >"${out}/randomize.log"
  ros2 run sanitation_learning stage5br3_capture_scene \
    --scene-manifest "${out}/scene_manifest.json" --output "${out}" --frame-count 10 --timeout 45 \
    >"${out}/capture.log"
done
