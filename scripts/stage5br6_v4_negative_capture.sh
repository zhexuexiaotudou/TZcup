#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /work/.work/stage1_20260714_154523/install/setup.bash
source /runtime/install/setup.bash
set -u

ROOT=/stage5br5
OUT="${STAGE5BR6_NEGATIVE_OUT:?output root is required}"
WORLD_ID=stage5br6_v4_negative_world
WORLD="${OUT}/stage5br6_v4_negative_world.sdf"
mkdir -p "${OUT}"

python3 "${ROOT}/scripts/stage5br6_build_negative_world.py" \
  --source "${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/world_a_asphalt_campus.sdf" \
  --output "${WORLD}" >"${OUT}/world_build.log"

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

XACRO=$(ros2 pkg prefix sanitation_vehicle_description)/share/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro
xacro "${XACRO}" enable_training_gt:=true enable_self_mask_gt:=true enable_verification_camera:=true \
  verification_camera_x:=.67 verification_camera_y:=.34 verification_camera_z:=.48 \
  verification_camera_pitch_rad:=.8726646260 >"${OUT}/vehicle.urdf"

gz sim -r -s --headless-rendering "${WORLD}" >"${OUT}/gz.log" 2>&1 & pids+=("$!")
for _ in $(seq 1 160); do
  gz service -l 2>/dev/null | grep -q "/world/${WORLD_ID}/create" && break
  sleep .25
done
gz service -l 2>/dev/null | grep -q "/world/${WORLD_ID}/create" || { echo "Gazebo create service unavailable" >&2; exit 3; }
ros2 run ros_gz_sim create -world "${WORLD_ID}" -file "${OUT}/vehicle.urdf" -name sanitation_vehicle -x -8 -y 0 -z .18 >"${OUT}/spawn.log" 2>&1

ros2 run ros_gz_bridge parameter_bridge \
  '/verification_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  '/verification_camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/verification_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/verification_semantic_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/verification_instance_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args \
  -r /verification_camera/camera_info:=/verification_camera/color/camera_info \
  -r /verification_camera/image:=/verification_camera/color/image_raw \
  -r /verification_camera/depth_image:=/verification_camera/depth/image_rect_raw \
  -r /g2/verification_semantic_gt/labels_map:=/ground_truth/verification_semantic/image \
  -r /g2/verification_instance_gt/labels_map:=/ground_truth/verification_instance/image \
  >"${OUT}/bridge.log" 2>&1 & pids+=("$!")
sleep 4

python3 "${ROOT}/scripts/stage5br6_capture_v4_negatives.py" --world-id "${WORLD_ID}" --output "${OUT}/captures" >"${OUT}/capture.log"
