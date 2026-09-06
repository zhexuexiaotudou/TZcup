#!/usr/bin/env bash
set -eo pipefail
set +u

source /opt/ros/jazzy/setup.bash
if [[ "${AUTO05_G4_RUNTIME_BOUND:-0}" == 1 ]]; then
  : "${AUTO05_COMBINED_RUNTIME_SETUP:?G4 capture requires a fresh combined runtime setup}"
  source "${AUTO05_COMBINED_RUNTIME_SETUP}"
else
  source /work/.work/stage1_20260714_154523/install/setup.bash
fi
set -u

REPO="${AUTO05_REPO_ROOT:-/repo}"
[[ -f "${REPO}/scripts/auto05_capture_all.sh" ]] || {
  echo "AUTO05_REPO_ROOT is not a TZcup checkout: ${REPO}" >&2
  exit 64
}
DATA_ROOT="${AUTO05_DATA_ROOT:-/data/g3_screening_native}"
RUNTIME_WS="${AUTO05_RUNTIME_WS:-/data/runtime_ws}"
mkdir -p "${DATA_ROOT}/logs" "${DATA_ROOT}/scenes" "${RUNTIME_WS}"

if [[ "${AUTO05_G4_RUNTIME_BOUND:-0}" != 1 ]]; then
  colcon --log-base "${RUNTIME_WS}/log" build \
    --base-paths "${REPO}/starter_ws/src" \
    --build-base "${RUNTIME_WS}/build" \
    --install-base "${RUNTIME_WS}/install" \
    --packages-up-to sanitation_learning sanitation_vehicle_description \
    --event-handlers console_cohesion+
  set +u
  source "${RUNTIME_WS}/install/setup.bash"
  set -u
fi

ros2 run sanitation_learning auto05_generate_g3_worlds \
  --registry "${REPO}/starter_ws/src/sanitation_learning/config/asset_registry.yaml" \
  --xacro "${REPO}/starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro" \
  --output-dir "${DATA_ROOT}/worlds" \
  >"${DATA_ROOT}/world_generation.json"

WORLD_IDS=(
  world_a_asphalt_campus
  world_b_concrete_sidewalk
  world_c_wet_dark_ground
  world_g_cobblestone_arcade
  world_d_mixed_curb_vegetation
  world_h_red_brick_promenade
  world_e_tiled_plaza
  world_f_service_road
)

capture_world() {
  local world_id="$1"
  local seed_start="$2"
  local log_root="${DATA_ROOT}/logs/${world_id}"
  mkdir -p "${log_root}"
  local pids=()
  cleanup_world() {
    local pid
    for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
    sleep 2
    for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
    wait 2>/dev/null || true
  }
  xacro "${REPO}/starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro" \
    enable_training_gt:=true >"/tmp/auto05_vehicle_${world_id}.urdf"
  gz sim -r -s --headless-rendering "${DATA_ROOT}/worlds/${world_id}.sdf" \
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
  # Launch the real bridge executable so the recorded PID is the bridge
  # process itself. `ros2 run ... &` leaves parameter_bridge orphaned when
  # only its Python wrapper is terminated, which accumulates duplicate
  # /cmd_vel bridges and eventually prevents the vehicle from moving.
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
    -file "/tmp/auto05_vehicle_${world_id}.urdf" \
    -name sanitation_vehicle -x -8 -y 0 -z .18 \
    >"${log_root}/spawn_vehicle.log" 2>&1
  sleep 3

  recreate_vehicle() {
    # A stopped vehicle is not a scene reset: contacts and controller state can
    # survive a teleport.  Delete the old model and create a new one per scene.
    gz service -s "/world/${world_id}/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 5000 \
      --req 'name: "sanitation_vehicle" type: MODEL' \
      >"${log_root}/remove_vehicle.log" 2>&1 || return 1
    sleep 1
    ros2 run ros_gz_sim create -world "${world_id}" \
      -file "/tmp/auto05_vehicle_${world_id}.urdf" \
      -name sanitation_vehicle -x -8 -y 0 -z .18 \
      >>"${log_root}/spawn_vehicle.log" 2>&1 || return 1
    sleep 2
  }

  local index seed scene out
  for index in $(seq 0 14); do
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
    recreate_vehicle || {
      echo "vehicle recreation failed: ${scene}" >&2
      return 2
    }
    python3 - "${out}/vehicle_reset.json" "${world_id}" "${scene}" <<'PY'
import json, sys, time
open(sys.argv[1], "w", encoding="utf-8").write(json.dumps({
  "schema_version": 1, "world_id": sys.argv[2], "scene": sys.argv[3],
  "vehicle_reset": "gz_remove_then_ros_gz_sim_create", "epoch_ns": time.time_ns(),
}, indent=2) + "\n")
PY
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
      ros2 run sanitation_learning auto05_randomize_g3_scene \
        --manifest "${DATA_ROOT}/worlds/g3_world_manifest.json" \
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
        --timeout 45 \
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
  capture_world "${WORLD_IDS[$world_index]}" "$((world_index * 15))"
done
