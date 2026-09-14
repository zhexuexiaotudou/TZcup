#!/usr/bin/env bash
# Run one predeclared dynamic-avoidance trial, then evaluate it offline.
set -euo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
run_root="${FORMAL_DYNAMIC_SINGLE_RUN_ROOT:?set a fresh FORMAL_DYNAMIC_SINGLE_RUN_ROOT}"
runtime_ws="${FORMAL_DYNAMIC_SINGLE_RUN_RUNTIME_WS:?set FORMAL_DYNAMIC_SINGLE_RUN_RUNTIME_WS}"
closure="${FORMAL_DYNAMIC_SINGLE_RUN_CLOSURE_MANIFEST:?set FORMAL_DYNAMIC_SINGLE_RUN_CLOSURE_MANIFEST}"
episode_root="${FORMAL_DYNAMIC_SINGLE_RUN_EPISODE_ROOT:?set FORMAL_DYNAMIC_SINGLE_RUN_EPISODE_ROOT}"
saved_map_root="${FORMAL_DYNAMIC_SINGLE_RUN_SAVED_MAP_ROOT:?set FORMAL_DYNAMIC_SINGLE_RUN_SAVED_MAP_ROOT}"
protocol="${FORMAL_DYNAMIC_SINGLE_RUN_PROTOCOL:-${repo_root}/config/dynamic_avoidance_single_run_protocol.json}"
runtime_install="${runtime_ws}/install"
episode_manifest="${episode_root}/public/episode_manifest.json"
public_world="${episode_root}/public/world.sdf"
base_schedule="${episode_root}/environment/pedestrian_schedule.json"
route_manifest="${run_root}/predeclared_obstacle_route.json"
timeline="${run_root}/single_run_timeline.json"
formal_report="${run_root}/dynamic_obstacle_acceptance.json"
runtime_root="${run_root}/runtime"
product_telemetry="${runtime_root}/runtime_telemetry.json"
environment_telemetry="${runtime_root}/environment_truth_telemetry.json"
runtime_binding="${run_root}/dynamic.runtime_binding.json"
single_run_report="${run_root}/single_run_evaluation.json"

for required in \
  "${protocol}" \
  "${runtime_install}/setup.bash" \
  "${closure}" \
  "${episode_manifest}" \
  "${public_world}" \
  "${base_schedule}"; do
  [[ -f "${required}" && ! -L "${required}" ]] || {
    echo "single dynamic trial prerequisite is not a regular file: ${required}" >&2
    exit 2
  }
done
[[ -d "${saved_map_root}" && ! -L "${saved_map_root}" ]] || {
  echo "single dynamic trial saved map root is missing: ${saved_map_root}" >&2
  exit 2
}
if [[ -e "${run_root}" ]]; then
  echo "refusing stale single dynamic trial root: ${run_root}" >&2
  exit 2
fi
mkdir -p "${run_root}"

set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_install}/setup.bash"
set -u

ros2_executable="$(realpath /opt/ros/jazzy/bin/ros2)"
if ! "${ros2_executable}" --help >/dev/null 2>&1; then
  echo "dynamic single-run wrapper ros2 executable failed validation: ${ros2_executable}" >&2
  exit 2
fi
export FORMAL_ROS2_EXECUTABLE="${ros2_executable}"

export PYTHONPATH="${repo_root}/scripts${PYTHONPATH:+:${PYTHONPATH}}"
python3 "${repo_root}/scripts/prepare_dynamic_avoidance_single_run.py" \
  --protocol "${protocol}" \
  --episode-manifest "${episode_manifest}" \
  --public-world "${public_world}" \
  --base-schedule "${base_schedule}" \
  --output "${route_manifest}"

seed="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["schedule"]["seed"])' "${protocol}")"
nominal_leg_m="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["schedule"]["nominal_leg_m"])' "${protocol}")"
task_timeout_s="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["task"]["timeout_s"])' "${protocol}")"
repository_revision="$(git -C "${repo_root}" rev-parse HEAD)"
run_start_epoch_ns="$(python3 -c 'import time; print(time.time_ns())')"
python3 - "${timeline}" "${run_root}" "${run_start_epoch_ns}" "${seed}" \
  "${repository_revision}" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
value = {
    "schema_version": 1,
    "run_id": pathlib.Path(sys.argv[2]).name,
    "status": "STARTED",
    "run_start_epoch_ns": int(sys.argv[3]),
    "run_end_epoch_ns": None,
    "schedule_seed": int(sys.argv[4]),
    "repository_revision": sys.argv[5],
    "runner_exit_code": None,
    "evaluator_exit_code": None,
}
path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

export FORMAL_VEHICLE_RUNTIME_WS="${runtime_ws}"
export FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST="${closure}"
export FORMAL_DYNAMIC_EPISODE_ROOT="${episode_root}"
export FORMAL_DYNAMIC_SAVED_MAP_ROOT="${saved_map_root}"
export FORMAL_DYNAMIC_OUTPUT="${formal_report}"
export FORMAL_DYNAMIC_TELEMETRY="${product_telemetry}"
export FORMAL_DYNAMIC_RUNTIME_BINDING="${runtime_binding}"
export FORMAL_DYNAMIC_SEED="${seed}"
export FORMAL_DYNAMIC_NOMINAL_LEG_M="${nominal_leg_m}"
export FORMAL_DYNAMIC_TIMEOUT_S="${task_timeout_s}"

set +e
bash "${repo_root}/scripts/run_formal_dynamic_obstacle_avoidance.sh"
runner_exit_code=$?
set -e
run_end_epoch_ns="$(python3 -c 'import time; print(time.time_ns())')"

python3 - "${timeline}" "${runner_exit_code}" "${run_end_epoch_ns}" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["status"] = "RUNNER_FINISHED"
value["runner_exit_code"] = int(sys.argv[2])
value["run_end_epoch_ns"] = int(sys.argv[3])
temporary = path.with_suffix(path.suffix + ".pending")
temporary.write_text(
    json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
temporary.replace(path)
PY

set +e
python3 "${repo_root}/scripts/evaluate_dynamic_avoidance_single_run.py" \
  --protocol "${protocol}" \
  --run-root "${run_root}" \
  --route-manifest "${route_manifest}" \
  --run-timeline "${timeline}" \
  --output "${single_run_report}"
evaluator_exit_code=$?
set -e

if (( runner_exit_code != 0 )) && (( evaluator_exit_code == 0 )); then
  echo "evaluator passed while the live runner failed; refusing contradictory result" >&2
  exit 3
fi
echo "single dynamic trial complete: report=${single_run_report}"
exit "${evaluator_exit_code}"
