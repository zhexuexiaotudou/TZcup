#!/usr/bin/env bash
set -eo pipefail
set +u

source /opt/ros/jazzy/setup.bash
if [[ -f /work/.work/stage1_20260714_154523/install/setup.bash ]]; then
  source /work/.work/stage1_20260714_154523/install/setup.bash
fi
set -u

REPO=/repo
DATA_ROOT="${AUTO05R_DATA_ROOT:-/data/g4_screening_native}"
RUNTIME_WS="${AUTO05R_RUNTIME_WS:-/data/runtime_ws_g4}"
SCENES_PER_WORLD="${AUTO05R_SCENES_PER_WORLD:-25}"
mkdir -p "${DATA_ROOT}/logs" "${DATA_ROOT}/scenes" "${RUNTIME_WS}"

colcon --log-base "${RUNTIME_WS}/log" build \
  --base-paths "${REPO}/starter_ws/src" \
  --build-base "${RUNTIME_WS}/build" \
  --install-base "${RUNTIME_WS}/install" \
  --packages-up-to sanitation_learning sanitation_vehicle_description \
  --event-handlers console_cohesion+
if [[ -d /upstream/linorobot2 ]]; then
  colcon --log-base "${RUNTIME_WS}/log" build \
    --base-paths "/upstream/linorobot2" \
    --build-base "${RUNTIME_WS}/build" \
    --install-base "${RUNTIME_WS}/install" \
    --packages-select linorobot2_description \
    --event-handlers console_cohesion+
fi
set +u
source "${RUNTIME_WS}/install/setup.bash"
set -u

ros2 run sanitation_learning auto05r_generate_g4_worlds \
  --registry "${REPO}/starter_ws/src/sanitation_learning/config/g4_asset_registry.yaml" \
  --assets-dir "${DATA_ROOT}/models" \
  --xacro "${REPO}/starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro" \
  --output-dir "${DATA_ROOT}/worlds" \
  >"${DATA_ROOT}/g4_world_generation.json"

WORLD_MANIFEST="${DATA_ROOT}/worlds/g4_world_manifest.json"
mapfile -t WORLD_IDS < <(
  python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); print("\n".join(w["world_id"] for w in m["worlds"]))' \
    "${WORLD_MANIFEST}"
)

export GZ_SIM_RESOURCE_PATH="${DATA_ROOT}/worlds:${DATA_ROOT}/models"

capture_world() {
  local world_id="$1"
  local world_index="$2"
  local seed_start=$((world_index * SCENES_PER_WORLD))
  local log_root="${DATA_ROOT}/logs/${world_id}"
  mkdir -p "${log_root}"
  local pids=()
  cleanup_world() {
    local pid
    for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
    sleep 2
    for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do
      local waited=0
      while kill -0 "${pid}" 2>/dev/null && [[ "${waited}" -lt 10 ]]; do
        sleep 1
        waited=$((waited + 1))
      done
      if kill -0 "${pid}" 2>/dev/null; then
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
    wait 2>/dev/null || true
  }
  xacro "${REPO}/starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro" \
    enable_training_gt:=true >"/tmp/auto05r_vehicle_${world_id}.urdf"
  (cd "${DATA_ROOT}/worlds" && exec gz sim -r -s --headless-rendering "${DATA_ROOT}/worlds/${world_id}.sdf") \
    >"${log_root}/gz.log" 2>&1 &
  pids+=("$!")
  local ready=false
  for _ in $(seq 1 120); do
    if gz service -l 2>/dev/null | grep -q "/world/${world_id}/create"; then
      ready=true
      break
    fi
    sleep .25
  done
  if [[ "${ready}" != true ]]; then
    echo "world service did not become ready: ${world_id}" >&2
    return 2
  fi
  /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge \
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
    >"${log_root}/bridge.log" 2>&1 &
  pids+=("$!")
  sleep 5
  ros2 run ros_gz_sim create -world "${world_id}" \
    -file "/tmp/auto05r_vehicle_${world_id}.urdf" \
    -name sanitation_vehicle -x -8 -y 0 -z .18 \
    >"${log_root}/spawn_vehicle.log" 2>&1
  sleep 3

  local index seed scene out
  for index in $(seq 0 $((SCENES_PER_WORLD - 1))); do
    seed=$((seed_start + index))
    scene=$(printf 'scene_%04d' "${seed}")
    out="${DATA_ROOT}/scenes/${scene}"
    if [[ -f "${out}/capture_report.json" ]] &&
       python3 -c 'import json,sys; raise SystemExit(0 if json.load(open(sys.argv[1]))["capture_pass"] else 1)' "${out}/capture_report.json"; then
      echo "resume-skip ${world_id} ${scene}"
      continue
    fi
    mkdir -p "${out}"
    echo "capture ${world_id} ${scene}"
    timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >"${out}/stop_vehicle.log" 2>&1 || true
    sleep 3
    mkdir -p "${out}/attempts"
    if [[ -f "${out}/capture_report.json" ]]; then
      cp "${out}/capture_report.json" \
        "${out}/attempts/capture_report_pre_resume_failed.json"
    fi
    local capture_pass=false
    local capture_attempt
    for capture_attempt in 1 2 3; do
      ros2 run sanitation_learning auto05r_randomize_g4_scene \
        --manifest "${WORLD_MANIFEST}" \
        --world-id "${world_id}" \
        --scene-seed "${seed}" \
        --scene-index "${index}" \
        --output "${out}/scene_manifest.json" \
        >"${out}/randomize.log"
      sleep 2
      if ros2 run sanitation_learning stage5br3_capture_scene \
        --scene-manifest "${out}/scene_manifest.json" \
        --output "${out}" \
        --frame-count 10 \
        --timeout 90 \
        >"${out}/capture.log"; then
        capture_pass=true
        break
      fi
      if [[ -f "${out}/capture_report.json" ]]; then
        cp "${out}/capture_report.json" \
          "${out}/attempts/capture_report_attempt_${capture_attempt}_failed.json"
      fi
      cp "${out}/capture.log" \
        "${out}/attempts/capture_attempt_${capture_attempt}_failed.log"
      timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
        >>"${out}/stop_vehicle.log" 2>&1 || true
      sleep 3
    done
    if [[ "${capture_pass}" != true ]]; then
      echo "scene failed after 3 capture attempts: ${scene}" >&2
      return 2
    fi
  done
  cleanup_world
}

for world_index in "${!WORLD_IDS[@]}"; do
  capture_world "${WORLD_IDS[$world_index]}" "${world_index}"
done

echo "G4 capture complete: worlds=${#WORLD_IDS[@]} scenes_per_world=${SCENES_PER_WORLD}"
