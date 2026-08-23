#!/usr/bin/env bash
set -euo pipefail

REPO=/repo
DATA_ROOT=/data
RUNTIME_WS=/runtime
UPSTREAM_ROOT=/upstream/linorobot2
FRAME_COUNT=10
CAPTURE_TIMEOUT_SECONDS=60
FORBIDDEN_MARKERS=(G5 G5_V2 G5V2 VAL_NEW DEV_VAL SEALED)

reject_forbidden_value() {
  local field="$1"
  local value="$2"
  local normalized
  local padded
  local marker
  normalized="$(printf '%s' "${value}" | tr '[:lower:]' '[:upper:]' | sed -E 's/[^A-Z0-9]+/_/g; s/^_+|_+$//g')"
  padded="_${normalized}_"
  for marker in "${FORBIDDEN_MARKERS[@]}"; do
    if [[ "${padded}" == *"_${marker}_"* ]]; then
      echo "forbidden dataset marker in ${field}: ${marker}" >&2
      return 2
    fi
  done
}

for path_value in "${REPO}" "${DATA_ROOT}" "${RUNTIME_WS}" "${UPSTREAM_ROOT}"; do
  reject_forbidden_value path "${path_value}"
done
if [[ ! -f "${UPSTREAM_ROOT}/linorobot2_description/package.xml" ]]; then
  echo "upstream linorobot2_description package is missing" >&2
  exit 2
fi
if ! findmnt -T "${REPO}" -no OPTIONS | tr ',' '\n' | grep -qx ro; then
  echo "repository mount must be read-only" >&2
  exit 2
fi
mkdir -p "${DATA_ROOT}" "${RUNTIME_WS}"
if [[ ! -w "${DATA_ROOT}" || ! -w "${RUNTIME_WS}" ]]; then
  echo "external data and runtime mounts must be writable" >&2
  exit 2
fi

for mission in \
  "${DATA_ROOT}/TRAIN/world_a_asphalt_campus_scene_0001" \
  "${DATA_ROOT}/TRAIN/world_a_asphalt_campus_scene_0019" \
  "${DATA_ROOT}/HOLDOUT/world_d_mixed_curb_vegetation_scene_0003" \
  "${DATA_ROOT}/HOLDOUT/world_d_mixed_curb_vegetation_scene_0038"; do
  reject_forbidden_value mission "${mission}"
  if [[ -e "${mission}" ]]; then
    echo "fixed mission output already exists; refusing to mix captures: ${mission}" >&2
    exit 2
  fi
done

set +u
source /opt/ros/jazzy/setup.bash
set -u

mkdir -p \
  "${RUNTIME_WS}/upstream_build" "${RUNTIME_WS}/upstream_install" \
  "${RUNTIME_WS}/project_build" "${RUNTIME_WS}/project_install" \
  "${RUNTIME_WS}/log/upstream" "${RUNTIME_WS}/log/project" \
  "${RUNTIME_WS}/generated" "${DATA_ROOT}/worlds" "${DATA_ROOT}/logs"

colcon --log-base "${RUNTIME_WS}/log/upstream" build \
  --base-paths "${UPSTREAM_ROOT}" \
  --build-base "${RUNTIME_WS}/upstream_build" \
  --install-base "${RUNTIME_WS}/upstream_install" \
  --packages-select linorobot2_description \
  --event-handlers console_cohesion+
set +u
source "${RUNTIME_WS}/upstream_install/setup.bash"
set -u

colcon --log-base "${RUNTIME_WS}/log/project" build \
  --base-paths "${REPO}/starter_ws/src" \
  --build-base "${RUNTIME_WS}/project_build" \
  --install-base "${RUNTIME_WS}/project_install" \
  --packages-up-to sanitation_learning sanitation_vehicle_description \
  --event-handlers console_cohesion+
set +u
source "${RUNTIME_WS}/project_install/setup.bash"
set -u

ros2 run sanitation_learning stage5br2_generate_g2_worlds \
  --registry "${REPO}/starter_ws/src/sanitation_learning/config/asset_registry.yaml" \
  --xacro "${REPO}/starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro" \
  --output-dir "${DATA_ROOT}/worlds" \
  >"${DATA_ROOT}/logs/world_generation.json"

WORLD_MANIFEST="${DATA_ROOT}/worlds/g2_world_manifest.json"
if [[ ! -f "${WORLD_MANIFEST}" ]]; then
  echo "G2 world manifest was not generated" >&2
  exit 2
fi
export GZ_SIM_RESOURCE_PATH="${DATA_ROOT}/worlds"

validate_scene_manifest() {
  local manifest="$1"
  local expected_world="$2"
  local expected_seed="$3"
  local expected_split="$4"
  local expected_role="$5"
  python3 - "${manifest}" "${expected_world}" "${expected_seed}" "${expected_split}" "${expected_role}" <<'PY'
import json
from pathlib import Path
import sys

path, world, seed, split, role = sys.argv[1:]
payload = json.loads(Path(path).read_text(encoding="utf-8"))
assert payload["world_id"] == world
assert payload["scene_seed"] == int(seed)
assert payload["split"] == split
assert payload["trajectory_id"] == f"{world}_scene_{int(seed):04d}"
counts = payload["target_count_by_class"]
if role == "positive":
    assert payload["negative_only"] is False
    assert counts["leaf_pile"] > 0 and counts["puddle"] > 0
else:
    assert role == "negative"
    assert payload["negative_only"] is True
    assert all(int(value) == 0 for value in counts.values())
PY
}

validate_capture() {
  local mission_dir="$1"
  local expected_world="$2"
  local expected_seed="$3"
  local expected_split="$4"
  local expected_role="$5"
  python3 - "${mission_dir}" "${expected_world}" "${expected_seed}" "${expected_split}" "${expected_role}" <<'PY'
import json
from pathlib import Path
import sys

import cv2
import numpy as np

root = Path(sys.argv[1])
world, seed, split, role = sys.argv[2:]
report = json.loads((root / "capture_report.json").read_text(encoding="utf-8"))
assert report["capture_pass"] is True
assert report["world_id"] == world
assert report["scene_seed"] == int(seed)
assert report["split"] == split
assert report["captured_frames"] == report["requested_frames"] == 10
observed = set()
for record in report["records"]:
    assert record["exact_four_sensor_timestamp"] is True
    paths = record["paths"]
    rgb = cv2.imread(str(root / paths["rgb"]), cv2.IMREAD_COLOR)
    depth = np.load(root / paths["depth"], allow_pickle=False)
    semantic = np.load(root / paths["semantic"], allow_pickle=False)
    assert rgb is not None and rgb.shape[:2] == depth.shape == semantic.shape
    assert np.issubdtype(semantic.dtype, np.integer)
    labels = {int(value) for value in np.unique(semantic).tolist()}
    assert labels.issubset(set(range(6)))
    observed.update(labels)
if role == "positive":
    assert {4, 5}.issubset(observed)
else:
    assert role == "negative" and observed.issubset({0})
PY
}

capture_scene() {
  local world_id="$1"
  local scene_seed="$2"
  local source_split="$3"
  local role="$4"
  local mission_dir="$5"
  mkdir -p "${mission_dir}"
  ros2 run sanitation_learning stage5br3_randomize_scene \
    --manifest "${WORLD_MANIFEST}" \
    --world-id "${world_id}" \
    --scene-seed "${scene_seed}" \
    --output "${mission_dir}/scene_manifest.json" \
    >"${mission_dir}/randomize.log"
  validate_scene_manifest \
    "${mission_dir}/scene_manifest.json" "${world_id}" "${scene_seed}" \
    "${source_split}" "${role}"
  sleep 0.5
  ros2 run sanitation_learning stage5br3_capture_scene \
    --scene-manifest "${mission_dir}/scene_manifest.json" \
    --output "${mission_dir}" \
    --frame-count "${FRAME_COUNT}" \
    --timeout "${CAPTURE_TIMEOUT_SECONDS}" \
    >"${mission_dir}/capture.log"
  validate_capture \
    "${mission_dir}" "${world_id}" "${scene_seed}" "${source_split}" "${role}"
}

capture_world() (
  set -euo pipefail
  local world_id="$1"
  local first_seed="$2"
  local first_role="$3"
  local first_mission="$4"
  local second_seed="$5"
  local second_role="$6"
  local second_mission="$7"
  local source_split="$8"
  local log_root="${DATA_ROOT}/logs/${world_id}"
  local vehicle_urdf="${RUNTIME_WS}/generated/${world_id}_vehicle.urdf"
  local ready=0
  local process_id
  local -a process_ids=()
  mkdir -p "${log_root}"

  cleanup_world() {
    for process_id in "${process_ids[@]}"; do
      kill -INT "${process_id}" 2>/dev/null || true
    done
    sleep 1
    for process_id in "${process_ids[@]}"; do
      kill -TERM "${process_id}" 2>/dev/null || true
    done
    sleep 1
    for process_id in "${process_ids[@]}"; do
      if kill -0 "${process_id}" 2>/dev/null; then
        kill -KILL "${process_id}" 2>/dev/null || true
      fi
      wait "${process_id}" 2>/dev/null || true
    done
  }
  trap cleanup_world EXIT

  xacro "${REPO}/starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro" \
    enable_training_gt:=true enable_self_mask_gt:=false >"${vehicle_urdf}"
  gz sim -r -s --headless-rendering "${DATA_ROOT}/worlds/${world_id}.sdf" \
    >"${log_root}/gz.log" 2>&1 &
  process_ids+=("$!")
  for _ in $(seq 1 120); do
    if gz service -l 2>/dev/null | grep -q "/world/${world_id}/create"; then
      ready=1
      break
    fi
    sleep 0.25
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "world service did not become ready: ${world_id}" >&2
    exit 2
  fi

  ros2 run ros_gz_sim create \
    -world "${world_id}" -file "${vehicle_urdf}" -name sanitation_vehicle \
    -x -8 -y 0 -z 0.18 >"${log_root}/spawn.log" 2>&1
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
  process_ids+=("$!")

  ready=0
  for _ in $(seq 1 120); do
    if ros2 topic list 2>/dev/null | grep -qx '/camera/color/image_raw' && \
       ros2 topic list 2>/dev/null | grep -qx '/camera/depth/image_rect_raw' && \
       ros2 topic list 2>/dev/null | grep -qx '/camera/color/camera_info' && \
       ros2 topic list 2>/dev/null | grep -qx '/ground_truth/semantic/image' && \
       ros2 topic list 2>/dev/null | grep -qx '/ground_truth/instance/image'; then
      ready=1
      break
    fi
    sleep 0.25
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "required synchronized capture topics did not become ready: ${world_id}" >&2
    exit 2
  fi

  capture_scene "${world_id}" "${first_seed}" "${source_split}" \
    "${first_role}" "${first_mission}"
  capture_scene "${world_id}" "${second_seed}" "${source_split}" \
    "${second_role}" "${second_mission}"
)

capture_world \
  world_a_asphalt_campus 1 positive \
  "${DATA_ROOT}/TRAIN/world_a_asphalt_campus_scene_0001" \
  19 negative "${DATA_ROOT}/TRAIN/world_a_asphalt_campus_scene_0019" train
capture_world \
  world_d_mixed_curb_vegetation 3 positive \
  "${DATA_ROOT}/HOLDOUT/world_d_mixed_curb_vegetation_scene_0003" \
  38 negative "${DATA_ROOT}/HOLDOUT/world_d_mixed_curb_vegetation_scene_0038" val

python3 "${REPO}/scripts/build_emf_area_dataset.py" \
  --split-root "TRAIN=${DATA_ROOT}/TRAIN" \
  --split-root "HOLDOUT=${DATA_ROOT}/HOLDOUT" \
  --output "${DATA_ROOT}/EMF_G2_AREA_DATASET_MANIFEST.json"

python3 - "${DATA_ROOT}/EMF_G2_AREA_DATASET_MANIFEST.json" <<'PY'
import json
from pathlib import Path
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
assert payload["protocol_id"] == "EMFJ6V3"
assert payload["development_only"] is True
assert payload["sealed_access_allowed"] is False
assert payload["a4_area_dataset_ready"] is True
assert payload["semantic_audit"]["leaf_pile_positive_frame_count"] > 0
assert payload["semantic_audit"]["puddle_positive_frame_count"] > 0
assert {scene["split"] for scene in payload["scenes"]} == {"TRAIN", "HOLDOUT"}
assert len(payload["scenes"]) == 4
contract = payload["screening_dataset_contract"]
assert contract["negative_only_scene_counts_by_split"] == {"TRAIN": 1, "HOLDOUT": 1}
assert contract["negative_only_frame_counts_by_split"] == {"TRAIN": 10, "HOLDOUT": 10}
assert sum(scene["negative_only"] is True for scene in payload["scenes"]) == 2
assert sum(frame["negative_only"] is True for frame in payload["frames"]) == 20
PY

echo "EMF G2 Area capture and paired-manifest validation completed"
