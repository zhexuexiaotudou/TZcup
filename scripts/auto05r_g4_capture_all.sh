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
MAX_WORLDS="${AUTO05R_MAX_WORLDS:-0}"
START_WORLD_INDEX="${AUTO05R_START_WORLD_INDEX:-0}"
CAMERA_PROFILE_ID="${AUTO05R_CAMERA_PROFILE_ID:-auto05r_v5_retracted_primary_perception_v1}"
CAMERA_X="${AUTO05R_CAMERA_X:-0.36}"
CAMERA_Y="${AUTO05R_CAMERA_Y:-0.0}"
CAMERA_Z="${AUTO05R_CAMERA_Z:-0.66}"
CAMERA_PITCH_RAD="${AUTO05R_CAMERA_PITCH_RAD:-0.872664626}"
ONLY_SCENES="${AUTO05R_ONLY_SCENES:-}"
FORCE_SCENES="${AUTO05R_FORCE_SCENES:-}"
SKIP_WORLD_GENERATION="${AUTO05R_SKIP_WORLD_GENERATION:-0}"
WORLD_MANIFEST_OVERRIDE="${AUTO05R_WORLD_MANIFEST:-}"
SCENE_SEED_OFFSET="${AUTO05R_SCENE_SEED_OFFSET:-0}"
DIAGNOSTIC_ROLE="${AUTO05R_DIAGNOSTIC_ROLE:-}"
ASSET_SOURCE_SPLIT="${AUTO05R_ASSET_SOURCE_SPLIT:-}"
NEGATIVE_SOURCE_SPLIT="${AUTO05R_NEGATIVE_SOURCE_SPLIT:-}"
FORCE_NEGATIVE_ONLY="${AUTO05R_FORCE_NEGATIVE_ONLY:-0}"
RESOURCE_ROOT="${AUTO05R_RESOURCE_ROOT:-${DATA_ROOT}}"
MODEL_RESOURCE_ROOT="${AUTO05R_MODEL_RESOURCE_ROOT:-${RESOURCE_ROOT}}"
CAPTURE_FRAME_COUNT="${AUTO05R_CAPTURE_FRAME_COUNT:-10}"
CAPTURE_TIMEOUT_SECONDS="${AUTO05R_CAPTURE_TIMEOUT_SECONDS:-90}"
CAPTURE_SPEED_MPS="${AUTO05R_CAPTURE_SPEED_MPS:-0.35}"
CAPTURE_MIN_TRANSLATION_M="${AUTO05R_CAPTURE_MIN_TRANSLATION_M:-0.25}"
CAPTURE_MIN_ROTATION_RAD="${AUTO05R_CAPTURE_MIN_ROTATION_RAD:-0.0}"
CAPTURE_MAX_ATTEMPTS="${AUTO05R_CAPTURE_MAX_ATTEMPTS:-3}"
OPRV3_COVERAGE_PROFILE="${AUTO05R_OPRV3_COVERAGE_PROFILE:-}"
DETECTOR_INSTANCES_PER_CLASS="${AUTO05R_DETECTOR_INSTANCES_PER_CLASS:-1}"
G8_AUTO_DOMAIN_MATRIX="${AUTO05R_G8_AUTO_DOMAIN_MATRIX:-0}"
G10_APPROACH_SEQUENCE="${AUTO05R_G10_APPROACH_SEQUENCE:-0}"
G10_IDENTIFIABILITY_DIAGNOSTIC="${AUTO05R_G10_IDENTIFIABILITY_DIAGNOSTIC:-0}"
if [[ "${CAPTURE_MAX_ATTEMPTS}" -lt 1 ]]; then
  echo "AUTO05R_CAPTURE_MAX_ATTEMPTS must be >= 1" >&2
  exit 2
fi
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

if [[ "${SKIP_WORLD_GENERATION}" != "1" ]]; then
  ros2 run sanitation_learning auto05r_generate_g4_worlds \
    --registry "${REPO}/starter_ws/src/sanitation_learning/config/g4_asset_registry.yaml" \
    --assets-dir "${DATA_ROOT}/models" \
    --xacro "${REPO}/starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro" \
    --output-dir "${DATA_ROOT}/worlds" \
    --camera-x "${CAMERA_X}" --camera-y "${CAMERA_Y}" \
    --camera-z "${CAMERA_Z}" --camera-pitch-rad "${CAMERA_PITCH_RAD}" \
    --camera-profile-id "${CAMERA_PROFILE_ID}" \
    >"${DATA_ROOT}/g4_world_generation.json"
fi

WORLD_MANIFEST="${WORLD_MANIFEST_OVERRIDE:-${DATA_ROOT}/worlds/g4_world_manifest.json}"
if [[ ! -f "${WORLD_MANIFEST}" ]]; then
  echo "AUTO-05R world manifest is missing: ${WORLD_MANIFEST}" >&2
  exit 2
fi
mapfile -t WORLD_IDS < <(
  python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); print("\n".join(w["world_id"] for w in m["worlds"]))' \
    "${WORLD_MANIFEST}"
)
if [[ "${START_WORLD_INDEX}" -lt 0 || "${START_WORLD_INDEX}" -ge "${#WORLD_IDS[@]}" ]]; then
  echo "AUTO05R_START_WORLD_INDEX out of range: ${START_WORLD_INDEX}" >&2
  exit 2
fi
if [[ "${MAX_WORLDS}" -gt 0 ]]; then
  WORLD_IDS=("${WORLD_IDS[@]:${START_WORLD_INDEX}:${MAX_WORLDS}}")
else
  WORLD_IDS=("${WORLD_IDS[@]:${START_WORLD_INDEX}}")
fi

if [[ ! -d "${MODEL_RESOURCE_ROOT}/models" ]]; then
  echo "AUTO-05R model resource directory is missing: ${MODEL_RESOURCE_ROOT}/models" >&2
  exit 2
fi
export GZ_SIM_RESOURCE_PATH="${RESOURCE_ROOT}/worlds:${MODEL_RESOURCE_ROOT}/models"

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
    enable_training_gt:=true camera_x:="${CAMERA_X}" camera_y:="${CAMERA_Y}" \
    camera_z:="${CAMERA_Z}" camera_pitch_rad:="${CAMERA_PITCH_RAD}" \
    >"/tmp/auto05r_vehicle_${world_id}.urdf"
  (cd "${RESOURCE_ROOT}/worlds" && exec gz sim -r -s --headless-rendering "${RESOURCE_ROOT}/worlds/${world_id}.sdf") \
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
    seed=$((SCENE_SEED_OFFSET + seed_start + index))
    scene=$(printf 'scene_%04d' "${seed}")
    out="${DATA_ROOT}/scenes/${scene}"
    if [[ -n "${ONLY_SCENES}" && ",${ONLY_SCENES}," != *",${scene},"* ]]; then
      echo "filter-skip ${world_id} ${scene}"
      continue
    fi
    local force_recap=false
    if [[ -n "${FORCE_SCENES}" && ",${FORCE_SCENES}," == *",${scene},"* ]]; then
      force_recap=true
      echo "force-recapture ${world_id} ${scene}"
    fi
    if [[ -f "${out}/capture_report.json" ]] &&
       [[ "${force_recap}" != true ]] &&
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
    for capture_attempt in $(seq 1 "${CAPTURE_MAX_ATTEMPTS}"); do
      randomize_args=(
        --manifest "${WORLD_MANIFEST}"
        --world-id "${world_id}"
        --scene-seed "${seed}"
        --scene-index "${index}"
        --output "${out}/scene_manifest.json"
        --detector-instances-per-class "${DETECTOR_INSTANCES_PER_CLASS}"
        --detector-scene-cycle "${SCENES_PER_WORLD}"
      )
      if [[ -n "${DIAGNOSTIC_ROLE}" ]]; then
        randomize_args+=(--diagnostic-role "${DIAGNOSTIC_ROLE}")
      fi
      if [[ -n "${ASSET_SOURCE_SPLIT}" ]]; then
        randomize_args+=(--asset-source-split "${ASSET_SOURCE_SPLIT}")
      fi
      if [[ -n "${NEGATIVE_SOURCE_SPLIT}" ]]; then
        randomize_args+=(--negative-source-split "${NEGATIVE_SOURCE_SPLIT}")
      fi
      if [[ "${FORCE_NEGATIVE_ONLY}" == "1" ]]; then
        randomize_args+=(--force-negative-only)
      fi
      if [[ -n "${OPRV3_COVERAGE_PROFILE}" ]]; then
        randomize_args+=(--oprv3-coverage-profile "${OPRV3_COVERAGE_PROFILE}")
      fi
      if [[ "${G8_AUTO_DOMAIN_MATRIX}" == "1" ]]; then
        randomize_args+=(--g8-auto-domain-matrix)
      fi
      if [[ "${G10_APPROACH_SEQUENCE}" == "1" ]]; then
        randomize_args+=(--g10-approach-sequence)
      fi
      if [[ "${G10_IDENTIFIABILITY_DIAGNOSTIC}" == "1" ]]; then
        randomize_args+=(--g10-identifiability-diagnostic)
      fi
      ros2 run sanitation_learning auto05r_randomize_g4_scene \
        "${randomize_args[@]}" >"${out}/randomize.log"
      sleep 2
      if ros2 run sanitation_learning stage5br3_capture_scene \
        --scene-manifest "${out}/scene_manifest.json" \
        --output "${out}" \
        --frame-count "${CAPTURE_FRAME_COUNT}" \
        --timeout "${CAPTURE_TIMEOUT_SECONDS}" \
        --linear-speed-mps "${CAPTURE_SPEED_MPS}" \
        --minimum-adjacent-translation-m "${CAPTURE_MIN_TRANSLATION_M}" \
        --minimum-adjacent-rotation-rad "${CAPTURE_MIN_ROTATION_RAD}" \
        --camera-xyz "${CAMERA_X}" "${CAMERA_Y}" "${CAMERA_Z}" \
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
      echo "scene failed after ${CAPTURE_MAX_ATTEMPTS} capture attempts: ${scene}" >&2
      return 2
    fi
  done
  cleanup_world
}

for local_world_index in "${!WORLD_IDS[@]}"; do
  world_index=$((START_WORLD_INDEX + local_world_index))
  capture_world "${WORLD_IDS[$local_world_index]}" "${world_index}"
done

echo "G4 capture complete: start_world_index=${START_WORLD_INDEX} worlds=${#WORLD_IDS[@]} scenes_per_world=${SCENES_PER_WORLD}"
