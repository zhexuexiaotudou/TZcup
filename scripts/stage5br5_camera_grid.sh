#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /work/.work/stage1_20260714_154523/install/setup.bash
source /runtime/install/setup.bash
set -u

ROOT=/stage5br5
OUT="${STAGE5BR5_OUT:?output root is required}"
CONFIG="${STAGE5BR5_CAMERA_CONFIG:?V1, V2, or V4 is required}"
WORLD_ID="${STAGE5BR5_WORLD_ID:?world id is required}"
SCENE_SEED="${STAGE5BR5_SCENE_SEED:-11}"
CAPTURE_MODE="${STAGE5BR5_CAPTURE_MODE:-both}"
WORLD="${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/${WORLD_ID}.sdf"
CONFIG_OUT="${OUT}/${CONFIG}/${WORLD_ID}"
mkdir -p "${CONFIG_OUT}"

case "${CONFIG}" in
  V1) verification_x=.67; verification_y=0; verification_z=.30; pitch=.6108652382 ;;
  V2) verification_x=.67; verification_y=0; verification_z=.48; pitch=.8726646260 ;;
  V4) verification_x=.67; verification_y=.34; verification_z=.48; pitch=.8726646260 ;;
  *) echo "mechanically pruned or unknown camera config ${CONFIG}" >&2; exit 2 ;;
esac

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  for _ in $(seq 1 20); do
    alive=false
    for pid in "${pids[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then alive=true; fi
    done
    if [[ "${alive}" == "false" ]]; then break; fi
    sleep .1
  done
  for pid in "${pids[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

XACRO=$(ros2 pkg prefix sanitation_vehicle_description)/share/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro
xacro "${XACRO}" enable_training_gt:=true enable_self_mask_gt:=true enable_verification_camera:=true \
  verification_camera_x:="${verification_x}" verification_camera_y:="${verification_y}" \
  verification_camera_z:="${verification_z}" verification_camera_pitch_rad:="${pitch}" \
  >"${CONFIG_OUT}/vehicle.urdf"

gz sim -r -s --headless-rendering "${WORLD}" >"${CONFIG_OUT}/gz.log" 2>&1 & pids+=("$!")
for _ in $(seq 1 120); do
  gz service -l 2>/dev/null | grep -q "/world/${WORLD_ID}/create" && break
  sleep .25
done
if ! gz service -l 2>/dev/null | grep -q "/world/${WORLD_ID}/create"; then
  echo "Gazebo create service unavailable for ${WORLD_ID}" >&2
  exit 3
fi
ros2 run ros_gz_sim create -world "${WORLD_ID}" -file "${CONFIG_OUT}/vehicle.urdf" \
  -name sanitation_vehicle -x -8 -y 0 -z .18 >"${CONFIG_OUT}/spawn.log" 2>&1

ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist' \
  '/ground_truth/model_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/semantic_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' '/g2/instance_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/verification_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  '/verification_camera/image@sensor_msgs/msg/Image[gz.msgs.Image' '/verification_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/verification_semantic_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/g2/verification_instance_gt/labels_map@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args \
  -r /camera/camera_info:=/camera/color/camera_info -r /camera/image:=/camera/color/image_raw \
  -r /camera/depth_image:=/camera/depth/image_rect_raw -r /g2/semantic_gt/labels_map:=/ground_truth/semantic/image \
  -r /g2/instance_gt/labels_map:=/ground_truth/instance/image \
  -r /verification_camera/camera_info:=/verification_camera/color/camera_info \
  -r /verification_camera/image:=/verification_camera/color/image_raw \
  -r /verification_camera/depth_image:=/verification_camera/depth/image_rect_raw \
  -r /g2/verification_semantic_gt/labels_map:=/ground_truth/verification_semantic/image \
  -r /g2/verification_instance_gt/labels_map:=/ground_truth/verification_instance/image \
  >"${CONFIG_OUT}/bridge.log" 2>&1 & pids+=("$!")
sleep 4

if [[ "${CAPTURE_MODE}" == "both" ]]; then
  ros2 run sanitation_learning stage5br3_randomize_scene \
    --manifest "${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json" \
    --world-id "${WORLD_ID}" --scene-seed "${SCENE_SEED}" --output "${CONFIG_OUT}/scene_manifest_discovery.json" \
    >"${CONFIG_OUT}/randomize_discovery.log"
  ros2 run sanitation_learning stage5br3_capture_scene \
    --scene-manifest "${CONFIG_OUT}/scene_manifest_discovery.json" --output "${CONFIG_OUT}/discovery" \
    --frame-count 10 --timeout 55 --camera-xyz .53 0 .22 --node-name "stage5br5_${CONFIG}_${WORLD_ID}_discovery" \
    >"${CONFIG_OUT}/discovery_capture.log"
elif [[ "${CAPTURE_MODE}" != "verification_only" ]]; then
  echo "unknown capture mode ${CAPTURE_MODE}" >&2
  exit 4
fi

# Reset identical object and vehicle poses before using the verification view.
ros2 run sanitation_learning stage5br3_randomize_scene \
  --manifest "${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json" \
  --world-id "${WORLD_ID}" --scene-seed "${SCENE_SEED}" --output "${CONFIG_OUT}/scene_manifest_verification.json" \
  >"${CONFIG_OUT}/randomize_verification.log"
ros2 run sanitation_learning stage5br3_capture_scene \
  --scene-manifest "${CONFIG_OUT}/scene_manifest_verification.json" --output "${CONFIG_OUT}/verification" \
  --frame-count 10 --timeout 55 \
  --rgb-topic /verification_camera/color/image_raw --depth-topic /verification_camera/depth/image_rect_raw \
  --semantic-topic /ground_truth/verification_semantic/image --instance-topic /ground_truth/verification_instance/image \
  --camera-info-topic /verification_camera/color/camera_info \
  --camera-xyz "${verification_x}" "${verification_y}" "${verification_z}" \
  --optical-frame verification_camera_depth_link --node-name "stage5br5_${CONFIG}_${WORLD_ID}_verification" \
  >"${CONFIG_OUT}/verification_capture.log"

gz topic -l | sort >"${CONFIG_OUT}/gz_topics.txt"
ros2 topic list | sort >"${CONFIG_OUT}/ros_topics.txt"
