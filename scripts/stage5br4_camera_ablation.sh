#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /work/.work/stage1_20260714_154523/install/setup.bash
source /runtime/install/setup.bash
set -u

ROOT=/stage5br4
OUT="${STAGE5BR4_OUT:-${ROOT}/artifacts/stage5br4_20260720_review/camera_ablation}"
CONFIG="${STAGE5BR4_CAMERA_CONFIG:?C0, C1, C2, or C3 is required}"
WORLD_ID="${STAGE5BR4_WORLD_ID:-world_a_asphalt_campus}"
SCENE_SEED="${STAGE5BR4_SCENE_SEED:-11}"
WORLD="${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/${WORLD_ID}.sdf"
CONFIG_OUT="${OUT}/${CONFIG}"
mkdir -p "${CONFIG_OUT}"

case "${CONFIG}" in
  C0) camera_x=.53; camera_z=.22; pitch=0; verification=false ;;
  # External negative pitch means downward; URDF camera optical +X needs the
  # opposite sign around +Y to produce that physical orientation.
  C1) camera_x=.53; camera_z=.50; pitch=.2617993878; verification=false ;;
  C2) camera_x=.53; camera_z=.70; pitch=.5235987756; verification=false ;;
  C3) camera_x=.53; camera_z=.22; pitch=0; verification=true ;;
  *) echo "unknown camera config ${CONFIG}" >&2; exit 2 ;;
esac

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

XACRO=$(ros2 pkg prefix sanitation_vehicle_description)/share/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro
xacro "${XACRO}" enable_training_gt:=true enable_self_mask_gt:=true \
  enable_verification_camera:="${verification}" camera_x:="${camera_x}" camera_z:="${camera_z}" \
  camera_pitch_rad:="${pitch}" >"${CONFIG_OUT}/vehicle.urdf"

gz sim -r -s --headless-rendering "${WORLD}" >"${CONFIG_OUT}/gz.log" 2>&1 & pids+=("$!")
for _ in $(seq 1 100); do
  gz service -l 2>/dev/null | grep -q "/world/${WORLD_ID}/create" && break
  sleep .25
done
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
sleep 5

ros2 run sanitation_learning stage5br3_randomize_scene \
  --manifest "${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json" \
  --world-id "${WORLD_ID}" --scene-seed "${SCENE_SEED}" --output "${CONFIG_OUT}/scene_manifest.json" \
  >"${CONFIG_OUT}/randomize.log"
ros2 run sanitation_learning stage5br3_capture_scene \
  --scene-manifest "${CONFIG_OUT}/scene_manifest.json" --output "${CONFIG_OUT}/discovery" \
  --frame-count 10 --timeout 50 --camera-xyz "${camera_x}" 0 "${camera_z}" --node-name "stage5br4_${CONFIG}_discovery" \
  >"${CONFIG_OUT}/discovery_capture.log"

if [[ "${CONFIG}" == C3 ]]; then
  # Reset all target and vehicle poses, then replay the identical command profile
  # through the physically separate verification camera.
  ros2 run sanitation_learning stage5br3_randomize_scene \
    --manifest "${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json" \
    --world-id "${WORLD_ID}" --scene-seed "${SCENE_SEED}" --output "${CONFIG_OUT}/scene_manifest_verification.json" \
    >"${CONFIG_OUT}/randomize_verification.log"
  ros2 run sanitation_learning stage5br3_capture_scene \
    --scene-manifest "${CONFIG_OUT}/scene_manifest_verification.json" --output "${CONFIG_OUT}/verification" \
    --frame-count 10 --timeout 50 \
    --rgb-topic /verification_camera/color/image_raw --depth-topic /verification_camera/depth/image_rect_raw \
    --semantic-topic /ground_truth/verification_semantic/image --instance-topic /ground_truth/verification_instance/image \
    --camera-info-topic /verification_camera/color/camera_info --camera-xyz .30 0 .70 \
    --optical-frame verification_camera_depth_link --node-name stage5br4_C3_verification \
    >"${CONFIG_OUT}/verification_capture.log"
fi

gz topic -l | sort >"${CONFIG_OUT}/gz_topics.txt"
ros2 topic list | sort >"${CONFIG_OUT}/ros_topics.txt"
