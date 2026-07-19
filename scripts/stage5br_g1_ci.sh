#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${STAGE5BR_G1_OUT:?STAGE5BR_G1_OUT required}"
SCENES="${STAGE5BR_G1_SCENES:-50}"
FRAMES_PER_SCENE="${STAGE5BR_G1_FRAMES_PER_SCENE:-10}"
mkdir -p "${OUT}" "${OUT}/scene_manifests" "${OUT}/scenes"
rm -rf "${OUT}/scene_manifests"/* "${OUT}/scenes"/* "${OUT}/randomizer_state.json"

set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONPATH="${PACK_ROOT}/starter_ws/src/sanitation_learning:${PYTHONPATH:-}"

registry="${PACK_ROOT}/starter_ws/src/sanitation_learning/config/asset_registry.yaml"
world="${OUT}/stage5br_g1_world.sdf"
python3 -m sanitation_learning.gazebo_g1 --registry "${registry}" --output "${world}" > "${OUT}/world_generation.log"

gz sim -r -s "${world}" > "${OUT}/gz_server.log" 2>&1 &
gz_pid=$!
ros2 run ros_gz_image image_bridge \
  /g1/rgbd/image /g1/rgbd/depth_image \
  /g1/semantic/labels_map /g1/instance/labels_map \
  > "${OUT}/image_bridge.log" 2>&1 &
image_pid=$!
ros2 run ros_gz_bridge parameter_bridge \
  "/g1/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  > "${OUT}/camera_info_bridge.log" 2>&1 &
info_pid=$!

cleanup() {
  kill -INT "${info_pid}" "${image_pid}" "${gz_pid}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "${info_pid}" 2>/dev/null && ! kill -0 "${image_pid}" 2>/dev/null && ! kill -0 "${gz_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
  kill -TERM "${info_pid}" "${image_pid}" "${gz_pid}" 2>/dev/null || true
  sleep 0.2
  kill -KILL "${info_pid}" "${image_pid}" "${gz_pid}" 2>/dev/null || true
  wait "${info_pid}" 2>/dev/null || true
  wait "${image_pid}" 2>/dev/null || true
  wait "${gz_pid}" 2>/dev/null || true
}
trap cleanup EXIT

ready=0
for _ in $(seq 1 40); do
  if gz service -l 2>/dev/null | grep -q '/world/stage5br_g1/set_pose_vector' && \
     ros2 topic list 2>/dev/null | grep -q '^/g1/rgbd/camera_info$'; then
    ready=1
    break
  fi
  sleep 1
done
test "${ready}" -eq 1
gz topic -l | sort > "${OUT}/gz_topics.txt"
ros2 topic list | sort > "${OUT}/ros_topics.txt"
gz service -l | sort > "${OUT}/gz_services.txt"

for seed in $(seq 0 $((SCENES - 1))); do
  scene_name="$(printf 'scene_%04d' "${seed}")"
  python3 -m sanitation_learning.g1_randomize \
    --world-manifest "${OUT}/stage5br_g1_world.manifest.json" \
    --scene-seed "${seed}" \
    --output "${OUT}/scene_manifests/${scene_name}.json" \
    --state "${OUT}/randomizer_state.json" \
    > "${OUT}/scene_manifests/${scene_name}.log"
  sleep 0.35
  mkdir -p "${OUT}/scenes/${scene_name}"
  python3 -m sanitation_learning.g1_collector \
    --output "${OUT}/scenes/${scene_name}" \
    --scene-seed "${seed}" \
    --frame-count "${FRAMES_PER_SCENE}" \
    --timeout-sec 35 \
    > "${OUT}/scenes/${scene_name}/collector.log"
done

python3 -m sanitation_learning.g1_finalize \
  --root "${OUT}" \
  --expected-scenes "${SCENES}" \
  --expected-frames "$((SCENES * FRAMES_PER_SCENE))" \
  > "${OUT}/finalize.log"
