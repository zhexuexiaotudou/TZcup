#!/usr/bin/env bash
# Execute real Gazebo-camera DOSOD+EdgeSAM acceptance. Accuracy failures are
# retained as BLOCKED reports; no synthetic/offline image is eligible.
set -eo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen colcon workspace}"
runtime_install="${runtime_ws}/install"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
cyclonedds_config="${repo_root}/config/cyclonedds_localhost.xml"
graph_probe="${repo_root}/scripts/wait_for_ros_graph.py"
output_root="${FORMAL_PERCEPTION_OUTPUT_ROOT:-${repo_root}/.work/formal_random_scene_perception_acceptance}"
formal_artifact="${FORMAL_PERCEPTION_FINAL_ARTIFACT:-${repo_root}/artifacts/formal_random_scene_perception_acceptance.json}"
formal_runtime_register_evidence_paths "${formal_artifact}"
episode_count="${FORMAL_PERCEPTION_EPISODE_COUNT:-30}"
base_domain="${FORMAL_PERCEPTION_BASE_DOMAIN:-70}"
formal_minimum_episode_count=30
formal_validation_map_count=8
formal_minimum_episodes_per_map=3
formal_validation_missions_per_map=100

if [[ ! "${episode_count}" =~ ^[1-9][0-9]*$ ]] || [[ ! "${base_domain}" =~ ^[0-9]+$ ]]; then
  echo "episode count and base domain must be positive integers" >&2
  exit 2
fi
if (( episode_count < formal_minimum_episode_count )); then
  echo "${episode_count} random-scene episodes are smoke-scale; formal product evidence requires at least ${formal_minimum_episode_count}" >&2
  exit 2
fi
if (( episode_count > formal_validation_map_count * formal_validation_missions_per_map )); then
  echo "episode count exceeds frozen validation split capacity" >&2
  exit 2
fi
formal_runtime_configure "${base_domain}" "${episode_count}"
if [[ -e "${output_root}" || -e "${formal_artifact}" ]]; then
  echo "Refusing stale random-scene perception evidence; use fresh output paths" >&2
  exit 2
fi
mkdir -p "$(dirname "${output_root}")" "$(dirname "${formal_artifact}")"
mkdir "${output_root}"
runtime_binding="${output_root}/runtime_gate_binding.json"
formal_runtime_register_evidence_paths "${runtime_binding}"

formal_source_bound_preflight \
  "${repo_root}" "${runtime_ws}" "${runtime_closure_manifest}" \
  "${session}" "${snapshot}" "${runtime_binding}"
mapfile -t formal_perception_roots < <(
  formal_source_bound_perception_roots "${runtime_closure_manifest}"
)
if [[ "${#formal_perception_roots[@]}" != 2 ]]; then
  echo "frozen runtime closure did not yield perception artifact and ONNX roots" >&2
  exit 2
fi
artifact_root="${formal_perception_roots[0]}"
onnx_pythonpath="${formal_perception_roots[1]}"

for required in /opt/ros/jazzy/setup.bash "${runtime_install}/setup.bash" \
  "${cyclonedds_config}" \
  "${graph_probe}" \
  "${artifact_root}/artifact_manifest.json" \
  "${repo_root}/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" \
  "${repo_root}/starter_ws/src/sanitation_perception/config/formal_random_scene_acceptance.yaml"; do
  if [[ ! -f "${required}" ]]; then
    echo "required formal perception input is missing: ${required}" >&2
    exit 2
  fi
done
if [[ ! -f "${onnx_pythonpath}/onnxruntime/__init__.py" ]]; then
  echo "frozen ONNX Runtime root is missing: ${onnx_pythonpath}" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "${runtime_install}/setup.bash"
formal_source_bound_verify_overlay "${runtime_install}"
set -u
export TZCUP_REPOSITORY_ROOT="${repo_root}"
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONPATH="${onnx_pythonpath}:${PYTHONPATH:-}"

overlay_preflight="${output_root}/overlay_preflight.log"
preflight_status=0
{
  echo "schema_version=2"
  echo "runtime_install=${runtime_install}"
  echo "runtime_gate_binding=${runtime_binding}"
  # Do not cold-start ros2 CLI once per package on DrvFS. Resolve the first
  # matching ament resource marker in AMENT_PREFIX_PATH, then verify the
  # complete installed/source file closure and hashes in one bounded process.
  if timeout 15 python3 - "${repo_root}" "${cyclonedds_config}" "${base_domain}" "${episode_count}" <<'PY'
import hashlib
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

repo = Path(sys.argv[1])
cyclonedds_config = Path(sys.argv[2])
base_domain = int(sys.argv[3])
episode_count = int(sys.argv[4])
expected_uri = f"file://{cyclonedds_config}"
active_uri = os.environ.get("CYCLONEDDS_URI", "")
print(f"CYCLONEDDS_URI {active_uri}")
active_path = Path(active_uri.removeprefix("file://")).resolve() if active_uri.startswith("file://") else None
if active_path != cyclonedds_config.resolve():
    print(f"FAIL cyclonedds_uri expected={expected_uri}")
    raise SystemExit(1)
print(f"CYCLONEDDS_CONFIG_RESOLVED {active_path}")
namespace = {"c": "https://cdds.io/config"}
config_root = ET.parse(cyclonedds_config).getroot()
interface = config_root.find(".//c:NetworkInterface", namespace)
maximum_element = config_root.find(".//c:MaxAutoParticipantIndex", namespace)
if interface is None or interface.attrib.get("name") != "lo" or maximum_element is None:
    print("FAIL cyclonedds_localhost_contract")
    raise SystemExit(1)
maximum_participant = int(maximum_element.text)
if maximum_participant != 120:
    print(f"FAIL max_auto_participant_index actual={maximum_participant}")
    raise SystemExit(1)
maximum_domain = base_domain + episode_count - 1
maximum_port = 7400 + 250 * maximum_domain + 11 + 2 * maximum_participant
print(f"CYCLONEDDS_MAX_AUTO_PARTICIPANT_INDEX {maximum_participant}")
print(f"CYCLONEDDS_MAX_UNICAST_PORT domain={maximum_domain} port={maximum_port}")
if maximum_port >= 65535:
    print("FAIL cyclonedds_udp_port_bound")
    raise SystemExit(1)
requirements = (
    ("sanitation_manipulation", "launch/formal_physical_grasp.launch.py"),
    ("sanitation_formal_campus_integration", "launch/formal_campus.launch.py"),
    ("sanitation_vehicle_description", "launch/formal_vehicle_sim.launch.py"),
    ("sanitation_gazebo_control", "package.xml"),
    ("sanitation_navigation", "package.xml"),
    ("sanitation_localization", "package.xml"),
    ("sanitation_perception_interfaces", "package.xml"),
    ("sanitation_perception", "package.xml"),
)
prefixes = [Path(value) for value in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if value]
failed = False
for package, relative in requirements:
    prefix = next(
        (
            candidate
            for candidate in prefixes
            if (candidate / "share/ament_index/resource_index/packages" / package).is_file()
        ),
        None,
    )
    if prefix is None:
        print(f"FAIL ament_resource {package}")
        failed = True
        continue
    package_xml = prefix / "share" / package / "package.xml"
    installed = prefix / "share" / package / relative
    source = repo / "starter_ws/src" / package / relative
    print(f"PACKAGE {package} {prefix}")
    for label, path in (("AMENT_RESOURCE", prefix / "share/ament_index/resource_index/packages" / package),
                        ("PACKAGE_XML", package_xml), ("INSTALLED", installed), ("SOURCE", source)):
        if not path.is_file():
            print(f"FAIL {label.lower()} {path}")
            failed = True
        else:
            print(f"{label} {path}")
    if installed.is_file() and source.is_file():
        installed_hash = hashlib.sha256(installed.read_bytes()).hexdigest()
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        print(f"HASH {package} installed={installed_hash} source={source_hash}")
        if installed_hash != source_hash:
            print(f"FAIL source_install_hash {package}")
            failed = True
if failed:
    raise SystemExit(1)
print("FILE_CLOSURE OK")
PY
  then
    :
  else
    echo "FAIL file_closure"
    preflight_status=1
  fi

  # One ament-index process confirms that the Python runtime sees the same
  # first-match prefixes. A DrvFS timeout is retained as evidence but does not
  # override a complete file closure plus the runtime import gate below.
  set +e
  timeout 15 python3 - <<'PY'
from ament_index_python.packages import get_package_prefix, get_package_share_directory

packages = (
    "sanitation_manipulation",
    "sanitation_formal_campus_integration",
    "sanitation_vehicle_description",
    "sanitation_gazebo_control",
    "sanitation_navigation",
    "sanitation_localization",
    "sanitation_perception_interfaces",
    "sanitation_perception",
)
for package in packages:
    print(f"AMENT_PYTHON {package} {get_package_prefix(package)} {get_package_share_directory(package)}")
print("AMENT_PYTHON OK")
PY
  ament_python_status=$?
  set -e
  if [[ "${ament_python_status}" == "124" ]]; then
    echo "WARN ament_python timeout_drvfs_file_closure_authoritative"
  elif [[ "${ament_python_status}" != "0" ]]; then
    echo "FAIL ament_python status=${ament_python_status}"
    preflight_status=1
  fi

  if timeout 15 python3 -c \
    'from sanitation_perception_interfaces.msg import GarbageTargetArray; from sanitation_perception.formal_random_scene_evaluator import _project_cube; print("IMPORT sanitation_perception_interfaces OK"); print("IMPORT sanitation_perception OK")'; then
    :
  else
    echo "FAIL perception_runtime_imports"
    preflight_status=1
  fi
  if [[ "${preflight_status}" == "0" ]]; then
    echo "status=READY"
  else
    echo "status=BLOCKED"
  fi
} >"${overlay_preflight}" 2>&1
if [[ "${preflight_status}" != "0" ]]; then
  cat "${overlay_preflight}" >&2
  exit 2
fi

launch_pid=""
product_pid=""
cleanup_episode() {
  local cleanup_status=0
  formal_runtime_cleanup_groups "${GZ_PARTITION:-}" \
    "${product_pid}" "${launch_pid}" || cleanup_status=1
  launch_pid=""
  product_pid=""
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup_episode

for ((index=0; index<episode_count; index++)); do
  episode_root="${output_root}/episode-${index}"
  mkdir -p "${episode_root}"
  # The frozen validation split has eight layouts.  Round-robin maps before
  # advancing the mission index so the formal matrix cannot silently collapse
  # to three nearby samples or overflow the split's map range.
  map_index=$((index % formal_validation_map_count))
  mission_index=$((index / formal_validation_map_count))
  ros2 run sanitation_campus_scenario sanitation-campus-scenario generate \
    --config "${repo_root}/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" \
    --profile formal --split val --map-index "${map_index}" --mission-index "${mission_index}" \
    --output "${episode_root}/scenario"
  export ROS_DOMAIN_ID=$((base_domain + index))
  export GZ_PARTITION="tzcup_formal_perception_${ROS_DOMAIN_ID}_$$"
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus.launch.py \
    gui:=false \
    world:="${episode_root}/scenario/public/world.sdf" \
    episode_manifest:="${episode_root}/scenario/public/episode_manifest.json" \
    world_name:=campus_formal \
    runtime_artifact_dir:="${episode_root}/materialized" \
    start_navigation:=true start_coverage:=false start_pedestrians:=true \
    simulation_initial_estop_active:=true \
    >"${episode_root}/formal_campus.launch.log" 2>&1 &
  launch_pid=$!

  # Product startup is independent from evaluator truth. It receives only the
  # public map, TF, camera, depth and CameraInfo topics.
  if ! timeout 125 python3 "${graph_probe}" \
    --timeout 120 --pid "${launch_pid}" \
    --output "${episode_root}/campus_graph_readiness.json" \
    --topic '/map=nav_msgs/msg/OccupancyGrid' \
    --topic '/sensors/front_rgbd/depth/image_rect_raw/image=sensor_msgs/msg/Image'; then
    echo "formal campus failed bounded map/front-depth readiness: ${episode_root}/campus_graph_readiness.json" >&2
    cleanup_episode
    continue
  fi
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run sanitation_perception pc_open_vocab_product_adapter --ros-args \
    -p use_sim_time:=true -p artifact_root:="${artifact_root}" \
    -p score_threshold:=0.005 \
    -p fallen_leaves_score_threshold:=0.0025 \
    -p dust_or_soil_score_threshold:=0.002 \
    -p puddle_score_threshold:=0.003 \
    -p intermediate_capture_root:="${episode_root}/product_intermediates" \
    -p intermediate_capture_max_frames:=12 \
    -p intermediate_capture_interval_s:=1.0 \
    -p intermediate_capture_max_bytes:=268435456 \
    >"${episode_root}/product_perception.log" 2>&1 &
  product_pid=$!

  if ! timeout 35 python3 "${graph_probe}" \
    --timeout 30 --pid "${product_pid}" \
    --output "${episode_root}/product_graph_readiness.json" \
    --node '/pc_open_vocab_product_adapter'; then
    echo "formal product perception node failed bounded direct discovery: ${episode_root}/product_graph_readiness.json" >&2
    cleanup_episode
    continue
  fi
  if ! timeout 15 ros2 topic echo --once --qos-durability transient_local \
    /perception/open_vocab/diagnostics diagnostic_msgs/msg/DiagnosticArray \
    >"${episode_root}/product_startup_diagnostic.yaml" \
    2>"${episode_root}/product_startup_diagnostic.err"; then
    echo "formal product perception emitted no startup liveness diagnostic" >&2
    cleanup_episode
    continue
  fi
  ros2 node info /pc_open_vocab_product_adapter \
    >"${episode_root}/product_node_info.txt" 2>&1 || true
  # One bounded parameter service request avoids serially delaying the
  # evaluator behind four timeouts when the single-threaded inference executor
  # is busy. The dump still records every frozen class threshold.
  timeout 5 ros2 param dump /pc_open_vocab_product_adapter \
    >"${episode_root}/product_score_threshold.txt" 2>&1 || true
  sha256sum "${artifact_root}/artifact_manifest.json" \
    >"${episode_root}/artifact_manifest.sha256"
  (
    cd "${repo_root}"
    sha256sum \
      starter_ws/src/sanitation_perception/sanitation_perception/pc_open_vocab_adapter.py \
      starter_ws/src/sanitation_perception/sanitation_perception/product_intermediate_capture.py \
      starter_ws/src/sanitation_perception/sanitation_perception/product_projection.py \
      starter_ws/src/sanitation_perception/sanitation_perception/dosod_ros_adapter.py \
      starter_ws/src/sanitation_perception/sanitation_perception/edgesam_ros_adapter.py \
      starter_ws/src/sanitation_perception/sanitation_perception/formal_random_scene_evaluator.py \
      starter_ws/src/sanitation_perception/config/formal_random_scene_acceptance.yaml \
      starter_ws/src/sanitation_campus_scenario/sanitation_campus_scenario/generator.py \
      scripts/wait_for_ros_graph.py \
      scripts/run_formal_random_scene_perception.sh
  ) >"${episode_root}/product_source_manifest.sha256"
  timeout 15 ros2 topic echo --once --qos-durability transient_local /tf_static \
    >"${episode_root}/tf_static_once.yaml" 2>"${episode_root}/tf_static_once.err" || true
  timeout 15 ros2 topic echo --once /tf \
    >"${episode_root}/tf_once.yaml" 2>"${episode_root}/tf_once.err" || true
  timeout 15 ros2 topic echo --once /odom \
    >"${episode_root}/odom_once.yaml" 2>"${episode_root}/odom_once.err" || true
  timeout 15 ros2 topic echo --once /joint_states \
    >"${episode_root}/joint_states_once.yaml" 2>"${episode_root}/joint_states_once.err" || true
  for chain in "map base_link" "base_link lidar_2d_link" "base_link front_rgbd_depth_optical_frame"; do
    read -r parent_frame child_frame <<<"${chain}"
    timeout 8 ros2 run tf2_ros tf2_echo "${parent_frame}" "${child_frame}" \
      >"${episode_root}/tf2_echo_${parent_frame}_${child_frame}.txt" 2>&1 || true
  done

  set +e
  ros2 run sanitation_perception formal_random_scene_perception_evaluator --ros-args \
    -p use_sim_time:=true \
    -p truth_path:="${episode_root}/scenario/evaluator/ground_truth.json" \
    -p public_manifest_path:="${episode_root}/scenario/public/episode_manifest.json" \
    -p acceptance_config:="${repo_root}/starter_ws/src/sanitation_perception/config/formal_random_scene_acceptance.yaml" \
    -p output_path:="${episode_root}/perception_acceptance.json" \
    -p diagnostic_frame_path:="${episode_root}/best_front_frame.png" \
    -p world_name:=campus_formal \
    >"${episode_root}/evaluator.log" 2>&1
  evaluator_status=$?
  set -e
  printf '%s\n' "${evaluator_status}" >"${episode_root}/evaluator_exit_code.txt"
  cleanup_episode
  if [[ -f "${episode_root}/best_front_frame.png" ]]; then
    set +e
    python3 "${repo_root}/scripts/diagnose_formal_dosod_frame.py" \
      --image "${episode_root}/best_front_frame.png" \
      --metadata "${episode_root}/best_front_frame.json" \
      --model "${artifact_root}/dosod/dosod_mlp3x_s_tzcup_rep.onnx" \
      --output "${episode_root}/dosod_raw_diagnostic.json" \
      >"${episode_root}/dosod_raw_diagnostic.log" 2>&1
    diagnostic_status=$?
    set -e
    printf '%s\n' "${diagnostic_status}" >"${episode_root}/dosod_raw_diagnostic_exit_code.txt"
  fi
done

set +e
python3 "${repo_root}/scripts/aggregate_formal_random_scene_perception.py" \
  --input-root "${output_root}" \
  --minimum-episodes "${formal_minimum_episode_count}" \
  --runtime-binding "${runtime_binding}" \
  --output "${formal_artifact}"
aggregate_status=$?
set -e
cp "${formal_artifact}" "${output_root}/formal_random_scene_perception_matrix.json"
exit "${aggregate_status}"
