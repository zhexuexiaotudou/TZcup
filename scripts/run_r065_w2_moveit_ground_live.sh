#!/usr/bin/env bash
# Run R065 W2 on the physical formal campus, real localization, and MoveIt.
#
# The runner never injects TF or shared joint states and never sends an arm,
# gripper, controller, or actuator command.  The installed gate only exercises
# planning-scene services and emits its JSON result on this runner's stdout.
set -Eeuo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)}"
[[ "$#" -eq 1 ]] || { echo "usage: run_r065_w2_moveit_ground_live.sh RUN_ROOT" >&2; exit 2; }
run_root_arg="$1"
[[ ! -L "${run_root_arg}" ]] || { echo "R065 W2 run-root argument must not be a symlink" >&2; exit 2; }
raw_run_root="$(realpath --no-symlinks -e "${run_root_arg}")"
run_root="$(realpath -e "${run_root_arg}")"
[[ "${raw_run_root}" == "${run_root}" ]] || { echo "R065 W2 run-root path must not traverse a symlink" >&2; exit 2; }
runtime_ws="${R065_RUNTIME_WS:?R065_RUNTIME_WS is required}"
closure_manifest="${R065_CLOSURE_MANIFEST:?R065_CLOSURE_MANIFEST is required}"
session="${R065_SESSION:?R065_SESSION is required}"
snapshot="${R065_SNAPSHOT:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
episode_root="${R065_EPISODE_ROOT:-${run_root}/episode}"
domain="${R065_W2_ROS_DOMAIN_ID:-92}"

[[ ! -L "${run_root}" && -d "${run_root}" ]] || {
  echo "R065 W2 run root must be an existing non-symlink directory" >&2
  exit 2
}
for required in \
  "${episode_root}/public/episode_manifest.json" \
  "${episode_root}/public/world.sdf" \
  "${session}" "${snapshot}" "${runtime_ws}/install/setup.bash" \
  "${closure_manifest}"; do
  [[ -f "${required}" && ! -L "${required}" ]] || {
    echo "R065 W2 required regular input is missing: ${required}" >&2
    exit 2
  }
done

runtime_root="${run_root}/w2_runtime"
runtime_binding="${run_root}/w2.runtime_binding.json"
launch_log="${runtime_root}/formal_campus_moveit.launch.log"
gate_json="${runtime_root}/moveit_ground_runtime_gate.json"
request_json_path="${run_root}/w2_request.json"
request_provenance="${run_root}/w2_request_provenance.json"
cleanup_evidence="${runtime_root}/cleanup_evidence.txt"
[[ ! -e "${runtime_root}" && ! -e "${run_root}/w2.json" && ! -e "${runtime_binding}" && \
   ! -e "${request_json_path}" && ! -e "${request_provenance}" ]] || {
  echo "R065 W2 refuses retained runtime evidence or output" >&2
  exit 2
}
mkdir -p "${runtime_root}"

set +u
source /opt/ros/jazzy/setup.bash
set -u
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
formal_runtime_register_evidence_paths \
  "${runtime_root}" "${runtime_binding}" "${request_json_path}" \
  "${request_provenance}" "${cleanup_evidence}"

primary_error=""
trap '[[ -n "${primary_error}" ]] || primary_error="${BASH_COMMAND}"' ERR

formal_source_bound_preflight \
  "${repo_root}" "${runtime_ws}" "${closure_manifest}" \
  "${session}" "${snapshot}" "${runtime_binding}" \
  >"${runtime_root}/source_bound_preflight.stdout"
set +u
source "${runtime_ws}/install/setup.bash"
set -u
formal_source_bound_verify_overlay "${runtime_ws}/install" \
  >"${runtime_root}/source_bound_overlay.stdout"

export TZCUP_REPOSITORY_ROOT="${repo_root}"
export ROS_DOMAIN_ID="${domain}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
[[ -z "${GZ_PARTITION:-}" ]] || {
  echo "R065 W2 refuses an inherited Gazebo partition" >&2
  exit 2
}
export GZ_PARTITION="tzcup_r065_w2_${ROS_DOMAIN_ID}_$$"

mapfile -t perception_roots < <(/usr/bin/python3 - "${closure_manifest}" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
closure = manifest.get("closure")
if not isinstance(closure, dict):
    raise SystemExit("frozen closure has no closure object")
for key in ("perception_artifact_root", "onnx_pythonpath"):
    value = closure.get(key)
    if not isinstance(value, str) or not value:
        raise SystemExit(f"frozen closure lacks {key}")
    candidate = Path(value)
    if candidate.is_symlink() or not candidate.is_dir():
        raise SystemExit(f"frozen closure {key} is not a regular directory")
    root = candidate.resolve(strict=True)
    if key == "onnx_pythonpath":
        marker = root / "onnxruntime" / "__init__.py"
        if marker.is_symlink() or not marker.is_file():
            raise SystemExit("frozen closure onnx_pythonpath lacks regular onnxruntime/__init__.py")
    print(root)
PY
)
[[ "${#perception_roots[@]}" -eq 2 ]] || {
  echo "frozen closure perception roots are incomplete" >&2
  exit 2
}
perception_artifact_root="${perception_roots[0]}"
onnx_pythonpath="${perception_roots[1]}"
export PYTHONPATH="${onnx_pythonpath}:${PYTHONPATH:-}"

launch_pid=""
perception_pid=""
cleanup() {
  local cleanup_status=0
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${perception_pid}" "${launch_pid}" || cleanup_status=$?
  printf 'primary_error=%s\ncleanup_status=%s\nros_domain_id=%s\ngz_partition=%s\n' \
    "${primary_error:-none}" "${cleanup_status}" "${ROS_DOMAIN_ID}" "${GZ_PARTITION}" \
    >"${cleanup_evidence}"
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup

# formal_campus owns the live global-EKF localization chain and includes the
# production formal_physical_grasp launch (MoveIt plus its scene bootstrap).
# Navigation and coverage stay off. This runner starts neither active-cleaning
# nor a control command; the production grasp node stays idle during the
# planning-scene-only gate.
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration \
  formal_campus.launch.py \
  gui:=false \
  world:="${episode_root}/public/world.sdf" \
  episode_manifest:="${episode_root}/public/episode_manifest.json" \
  pedestrian_schedule:="${episode_root}/environment/pedestrian_schedule.json" \
  start_pedestrians:=false start_navigation:=false start_coverage:=false \
  high_bandwidth_sensor_runtime:=true \
  localization_backend:=amcl runtime_artifact_dir:="${runtime_root}/campus_artifacts" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

# A direct live TF query only observes the production global-EKF output.  It
# is deliberately not a static-transform publisher or a shared-state source.
ready="false"
for _ in $(seq 1 "${R065_W2_STARTUP_POLLS:-180}"); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "R065 W2 formal campus launch exited early: ${launch_log}" >&2
    exit 3
  fi
  if ros2 node list 2>/dev/null | grep -Fxq /move_group && \
      ros2 node list 2>/dev/null | grep -Fxq /global_ekf && \
      timeout 3s ros2 run tf2_ros tf2_echo map base_footprint \
        >"${runtime_root}/map_to_base_footprint.tf.log" 2>&1; then
    ready="true"
    break
  fi
  sleep 1
done
[[ "${ready}" == "true" ]] || {
  echo "R065 W2 timed out waiting for live map->base_footprint and MoveIt" >&2
  exit 3
}

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_perception \
  formal_pc_open_vocab.launch.py artifact_root:="${perception_artifact_root}" \
  >"${runtime_root}/formal_pc_open_vocab.launch.log" 2>&1 &
perception_pid=$!

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" /usr/bin/python3 \
  "${repo_root}/scripts/collect_r065_w2_live_grasp_request.py" \
  --run-root "${run_root}" --request-output "${request_json_path}" \
  --provenance-output "${request_provenance}" --session "${session}" \
  --runtime-binding "${runtime_binding}" --closure-manifest "${closure_manifest}" \
  --timeout-sec "${R065_W2_PERCEPTION_TIMEOUT_S:-90}" \
  >"${runtime_root}/w2_request_collector.log" 2>&1

for captured in "${request_json_path}" "${request_provenance}"; do
  [[ -f "${captured}" && ! -L "${captured}" ]] || {
    echo "R065 W2 fresh product-perception capture is missing: ${captured}" >&2
    exit 3
  }
done

request_parameter="$(/usr/bin/python3 - "${request_json_path}" <<'PY'
import json
import sys
from pathlib import Path

raw = Path(sys.argv[1]).read_text(encoding="utf-8")
# The CLI must receive a YAML string scalar, not an object parameter.  JSON
# encoding the whole verified request supplies that scalar without shell
# quoting or content interpolation.
json.loads(raw)
print(json.dumps(raw))
PY
)"
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run sanitation_manipulation \
  moveit_ground_runtime_gate --ros-args \
  -p config_file:="$(ros2 pkg prefix --share sanitation_manipulation)/config/bin_and_scene.yaml" \
  -p "request_json:=${request_parameter}" \
  -p allow_ground_removal_test:=true \
  -p timeout_sec:="${R065_W2_GATE_TIMEOUT_S:-20}" \
  >"${gate_json}"

/usr/bin/python3 - "${gate_json}" <<'PY'
import json
import sys
from pathlib import Path

value = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
required = {
    "runtime_gate": "moveit_ground_collision",
    "passed": True,
    "executor_or_controller_commands_sent": False,
    "truth_used_for_control": False,
    "ground_removal_preserved_non_ground_world_and_acm": True,
    "ground_removal_used_robot_state_diff_only": True,
}
if not isinstance(value, dict) or any(value.get(key) != expected for key, expected in required.items()):
    raise SystemExit("R065 W2 gate JSON has no passing no-controller-command contract")
PY

# The wrapper seals this sole stdout JSON object into RUN_ROOT/w2.json.  Keep
# all preparatory output in runtime_root so stdout remains machine-readable.
cat "${gate_json}"
