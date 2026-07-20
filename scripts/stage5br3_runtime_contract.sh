#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /work/.work/stage1_20260714_154523/install/setup.bash
source /stage5br3/.runtime_ws/install/setup.bash
set -u

ROOT=/stage5br3
OUT="${STAGE5BR3_RUNTIME_OUT:-${ROOT}/artifacts/stage5br3_20260720_review/runtime_contract}"
WORLD_ID="${STAGE5BR3_WORLD_ID:-world_a_asphalt_campus}"
WORLD="${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/${WORLD_ID}.sdf"
mkdir -p "${OUT}"

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
grep -q 'g2_semantic_gt' /tmp/stage5br3_vehicle.urdf
grep -q 'g2_instance_gt' /tmp/stage5br3_vehicle.urdf

gz sim -r -s --headless-rendering "${WORLD}" >"${OUT}/${WORLD_ID}.gz.log" 2>&1 &
pids+=("$!")
for _ in $(seq 1 80); do
  gz service -l 2>/dev/null | grep -q "/world/${WORLD_ID}/create" && break
  sleep 0.25
done
ros2 run ros_gz_sim create -world "${WORLD_ID}" -file /tmp/stage5br3_vehicle.urdf \
  -name sanitation_vehicle -x -8.0 -y 0.0 -z 0.18 >"${OUT}/${WORLD_ID}.spawn.log" 2>&1

ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist' \
  '/ground_truth/model_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/semantic_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/instance_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args \
  -r /camera/camera_info:=/camera/color/camera_info \
  -r /camera/image:=/camera/color/image_raw \
  -r /camera/depth_image:=/camera/depth/image_rect_raw \
  -r /g2/semantic_gt/labels_map:=/ground_truth/semantic/image \
  -r /g2/instance_gt/labels_map:=/ground_truth/instance/image \
  >"${OUT}/${WORLD_ID}.bridge.log" 2>&1 &
pids+=("$!")
ros2 run tf2_ros static_transform_publisher \
  --x 0.53 --y 0 --z 0.22 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link \
  >"${OUT}/${WORLD_ID}.tf.log" 2>&1 &
pids+=("$!")

sleep 5
gz topic -l | sort >"${OUT}/${WORLD_ID}.gz_topics.txt"
ros2 topic list | sort >"${OUT}/${WORLD_ID}.ros_topics.txt"
ros2 run sanitation_learning stage5br3_runtime_contract \
  --world-id "${WORLD_ID}" --output "${OUT}/${WORLD_ID}.json" --timeout 35

if [[ "${STAGE5BR3_SCENE_PROBE:-false}" == "true" ]]; then
  SCENE_SEED="${STAGE5BR3_SCENE_SEED:-0}"
  SCENE_ROOT="${ROOT}/artifacts/stage5br3_20260720_review/g2_screening_probe/scene_${SCENE_SEED}"
  mkdir -p "${SCENE_ROOT}"
  ros2 run sanitation_learning stage5br3_randomize_scene \
    --manifest "${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json" \
    --world-id "${WORLD_ID}" --scene-seed "${SCENE_SEED}" \
    --output "${SCENE_ROOT}/scene_manifest.json"
  ros2 run sanitation_learning stage5br3_capture_scene \
    --scene-manifest "${SCENE_ROOT}/scene_manifest.json" \
    --output "${SCENE_ROOT}" --frame-count 10 --timeout 45
fi
