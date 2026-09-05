#!/usr/bin/env bash
# Fail-closed entry point for the saved-map/Nav2 dynamic-obstacle acceptance.
set -eo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
episode_root="${FORMAL_DYNAMIC_EPISODE_ROOT:-${repo_root}/.work/formal_campus_episode_runtime}"
saved_map_root="${FORMAL_DYNAMIC_SAVED_MAP_ROOT:-${repo_root}/.work/formal_first_map_acceptance}"
output="${FORMAL_DYNAMIC_OUTPUT:-${repo_root}/artifacts/formal_dynamic_obstacle_avoidance_acceptance.json}"
telemetry="${FORMAL_DYNAMIC_TELEMETRY:-${repo_root}/.work/formal_dynamic_obstacle_avoidance/runtime_telemetry.json}"
runtime_root="$(dirname "${telemetry}")"
environment_telemetry="${runtime_root}/environment_truth_telemetry.json"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen colcon workspace}"
runtime_install="${runtime_ws}/install"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
runtime_binding="${FORMAL_DYNAMIC_RUNTIME_BINDING:-${output}.runtime_binding.json}"
formal_runtime_register_evidence_paths "${output}" "${telemetry}" "${environment_telemetry}" "${runtime_binding}"
snapshot_manifest="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
session_status="${FORMAL_ACCEPTANCE_SESSION_STATUS:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
domain="${ROS_DOMAIN_ID:-73}"
dynamic_seed="${FORMAL_DYNAMIC_SEED:-$(date +%s%N)}"

export PYTHONPATH="${repo_root}/starter_ws/src/sanitation_formal_campus_integration${PYTHONPATH:+:${PYTHONPATH}}"
formal_runtime_configure "${domain}"
for required in \
  "${runtime_install}/setup.bash" \
  "${snapshot_manifest}" \
  "${session_status}"; do
  if [[ ! -f "${required}" ]]; then
    echo "formal dynamic prerequisite is missing: ${required}" >&2
    exit 2
  fi
done
if [[ -e "${output}" || -e "${runtime_root}" || -e "${runtime_binding}" ]]; then
  echo "Refusing stale dynamic-obstacle evidence; archive ${output}, ${runtime_root}, and ${runtime_binding} before a fresh run" >&2
  exit 2
fi
mkdir -p "${runtime_root}" "$(dirname "${output}")"

/usr/bin/python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${repo_root}" --install-root "${runtime_install}" \
  --closure-manifest "${runtime_closure_manifest}" \
  --session "${session_status}" --snapshot "${snapshot_manifest}" \
  --output "${runtime_binding}"

# The vehicle snapshot must still describe the checked-out authoritative
# sources, and the acceptance session must have been started from those exact
# bytes.  This check is repeated after the live run so source drift during a
# long Gazebo mission cannot be published as current evidence.
/usr/bin/python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" --check \
  >"${runtime_root}/snapshot_preflight.json"
session_started_epoch_ns="$(/usr/bin/python3 - "${snapshot_manifest}" "${session_status}" <<'PY'
import hashlib, json, sys
from pathlib import Path

snapshot_path, session_path = map(Path, sys.argv[1:])
snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
session = json.loads(session_path.read_text(encoding="utf-8"))
outputs = snapshot.get("outputs", {})
urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
identity = {
    "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
    "source_inventory_sha256": snapshot.get("source_inventory_sha256"),
    "expanded_urdf_sha256": urdf.get("sha256"),
}
if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
    raise SystemExit("formal acceptance session is not RUNNING")
if session.get("snapshot") != identity:
    raise SystemExit("formal acceptance session does not match the frozen vehicle snapshot")
started = session.get("started_epoch_ns")
if not isinstance(started, int) or started <= 0:
    raise SystemExit("formal acceptance session has no valid start time")
print(started)
PY
)"

# Validate the first-task lifecycle artifact before ROS or Gazebo starts. This
# prevents a materialized world map, a partial SLAM map, a different map ID or
# a hand-written observed_fraction from entering the product localization path.
set +e
/usr/bin/python3 "${repo_root}/scripts/validate_formal_dynamic_obstacle_avoidance.py" \
  --episode-manifest "${episode_root}/public/episode_manifest.json" \
  --saved-map-artifact-dir "${saved_map_root}" \
  --snapshot-manifest "${snapshot_manifest}" \
  --session-status "${session_status}" \
  --runtime-binding "${runtime_binding}" \
  --preflight-only \
  --output "${output}"
status=$?
set -e
if (( status != 0 )); then
  echo "dynamic avoidance blocked before launch: no qualified saved-map lifecycle artifact; report=${output}" >&2
  exit "${status}"
fi
# A valid saved map is only admission to the runtime, not acceptance. Write a
# fresh BLOCKED report immediately so an old PASS cannot survive an environment,
# build or launch failure later in this script.
/usr/bin/python3 "${repo_root}/scripts/validate_formal_dynamic_obstacle_avoidance.py" \
  --episode-manifest "${episode_root}/public/episode_manifest.json" \
  --saved-map-artifact-dir "${saved_map_root}" \
  --snapshot-manifest "${snapshot_manifest}" \
  --session-status "${session_status}" \
  --runtime-binding "${runtime_binding}" \
  --output "${output}" >/dev/null 2>&1 || true

source /opt/ros/jazzy/setup.bash
set +u
source "${runtime_install}/setup.bash"
set -u
/usr/bin/python3 -c 'import action_msgs, diagnostic_msgs, geometry_msgs, nav2_msgs, nav_msgs, rclpy, ros_gz_interfaces, sensor_msgs, std_msgs, tf2_msgs' || {
  echo "dynamic avoidance ROS Python environment is incomplete" >&2
  exit 3
}

# A formal run sources exactly one project overlay above Jazzy.  The manifest
# below proves that this already frozen install matches the current checkout;
# the runner never builds or sources historical evidence workspaces itself.
for required_package in \
  sanitation_formal_campus_integration \
  sanitation_vehicle_description \
  sanitation_gazebo_control \
  sanitation_gazebo_auxiliary \
  sanitation_navigation \
  sanitation_localization \
  sanitation_power_system \
  sanitation_safety \
  sanitation_campus_scenario \
  sanitation_manipulation \
  sanitation_coverage; do
  resolved_prefix="$(ros2 pkg prefix "${required_package}" 2>/dev/null)" || {
    echo "current runtime package missing: ${required_package}" >&2
    exit 3
  }
  if [[ "$(realpath "${resolved_prefix}")" != "$(realpath "${runtime_install}")" ]]; then
    echo "project package resolved outside the one frozen overlay: ${required_package} -> ${resolved_prefix}" >&2
    exit 3
  fi
done
build_manifest="${runtime_root}/current_source_build_manifest.json"
/usr/bin/python3 "${repo_root}/scripts/generate_formal_dynamic_runtime_build_manifest.py" \
  --repository-root "${repo_root}" \
  --install-root "${runtime_install}" \
  --output "${build_manifest}" || {
    echo "current runtime install does not match checkout sources; report remains BLOCKED" >&2
    exit 3
  }
export TZCUP_REPOSITORY_ROOT="${repo_root}"
export ROS_DOMAIN_ID="${domain}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_dynamic_${domain}_$$}"
runtime_schedule="${runtime_root}/pedestrian_schedule.seed_${dynamic_seed}.json"
/usr/bin/python3 "${repo_root}/scripts/prepare_formal_dynamic_obstacle_schedule.py" \
  --episode-manifest "${episode_root}/public/episode_manifest.json" \
  --public-world "${episode_root}/public/world.sdf" \
  --base-schedule "${episode_root}/environment/pedestrian_schedule.json" \
  --seed "${dynamic_seed}" \
  --nominal-leg "${FORMAL_DYNAMIC_NOMINAL_LEG_M:-30.0}" \
  --output "${runtime_schedule}"
runtime_world="${runtime_root}/cleaning_world.with_contact_system.sdf"
runtime_world_manifest="${runtime_root}/cleaning_world_manifest.json"
/usr/bin/python3 "${repo_root}/scripts/prepare_formal_dynamic_runtime_world.py" \
  --source "${episode_root}/public/world.sdf" \
  --output "${runtime_world}" \
  --manifest "${runtime_world_manifest}"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration \
  formal_campus_map_lifecycle.launch.py \
  mission_mode:=cleaning gui:=false \
  world:="${runtime_world}" \
  episode_manifest:="${episode_root}/public/episode_manifest.json" \
  map_artifact_dir:="${saved_map_root}" \
  pedestrian_schedule:="${runtime_schedule}" \
  start_pedestrians:=true start_coverage:=false operation_speed_profile:=mapping_safe \
  >"${runtime_root}/dynamic.launch.log" 2>&1 &
launch_pid=$!
collector_pid=""
environment_collector_pid=""
cleanup() {
  formal_runtime_cleanup_groups \
    "${GZ_PARTITION}" \
    "${collector_pid}" \
    "${environment_collector_pid}" \
    "${launch_pid}"
}
formal_runtime_install_traps cleanup

# Join DDS before the delayed Nav2 group starts. The fixed map-frame goal is a
# public mission input; it is not selected from the pedestrian schedule.
sleep 2
collector_args=(
  "${repo_root}/scripts/collect_formal_dynamic_obstacle_avoidance_runtime.py"
  --episode-manifest "${episode_root}/public/episode_manifest.json"
  --runtime-build-manifest "${build_manifest}"
  --runtime-world-manifest "${runtime_world_manifest}"
  --nominal-leg "${FORMAL_DYNAMIC_NOMINAL_LEG_M:-30.0}"
  --timeout "${FORMAL_DYNAMIC_TIMEOUT_S:-300}"
  --output "${telemetry}"
)
if [[ -n "${FORMAL_DYNAMIC_GOAL_X:-}" ]]; then
  collector_args+=(--goal-x "${FORMAL_DYNAMIC_GOAL_X}")
fi
if [[ -n "${FORMAL_DYNAMIC_GOAL_Y:-}" ]]; then
  collector_args+=(--goal-y "${FORMAL_DYNAMIC_GOAL_Y}")
fi
environment_collector_args=(
  "${repo_root}/scripts/collect_formal_dynamic_environment_runtime.py"
  --timeout "$(( ${FORMAL_DYNAMIC_TIMEOUT_S:-300} + 30 ))"
  --output "${environment_telemetry}"
)
set +e
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" /usr/bin/python3 "${environment_collector_args[@]}" \
  >"${runtime_root}/environment_collector.log" 2>&1 &
environment_collector_pid=$!
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" /usr/bin/python3 "${collector_args[@]}" >"${runtime_root}/collector.log" 2>&1 &
collector_pid=$!
collector_status=0
while kill -0 "${collector_pid}" 2>/dev/null; do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "formal dynamic launch exited early; see ${runtime_root}/dynamic.launch.log" >&2
    kill -TERM "${collector_pid}" 2>/dev/null || true
    wait "${collector_pid}" 2>/dev/null
    collector_status=3
    break
  fi
  if ! kill -0 "${environment_collector_pid}" 2>/dev/null; then
    echo "evaluator-only environment collector exited early" >&2
    kill -TERM "${collector_pid}" 2>/dev/null || true
    wait "${collector_pid}" 2>/dev/null
    collector_status=3
    break
  fi
  sleep 1
done
if (( collector_status == 0 )); then
  wait "${collector_pid}"
  collector_status=$?
fi
environment_collector_status=0
if kill -0 "${environment_collector_pid}" 2>/dev/null; then
  kill -INT "${environment_collector_pid}" 2>/dev/null || true
  environment_stop_deadline=$((SECONDS + 10))
  while kill -0 "${environment_collector_pid}" 2>/dev/null \
      && (( SECONDS < environment_stop_deadline )); do
    sleep 1
  done
fi
if kill -0 "${environment_collector_pid}" 2>/dev/null; then
  kill -TERM "${environment_collector_pid}" 2>/dev/null || true
  environment_collector_status=3
fi
wait "${environment_collector_pid}" 2>/dev/null || environment_collector_status=$?
set -e

set +e
/usr/bin/python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" --check \
  >"${runtime_root}/snapshot_postflight.json"
snapshot_status=$?
set -e
if (( collector_status != 0 || environment_collector_status != 0 || snapshot_status != 0 )) \
    || [[ ! -f "${telemetry}" || ! -f "${environment_telemetry}" ]]; then
  echo "dynamic avoidance runtime failed closed: report=${output}" >&2
  exit 2
fi
telemetry_mtime_ns="$(/usr/bin/python3 -c 'import os,sys; print(os.stat(sys.argv[1]).st_mtime_ns)' "${telemetry}")"
if (( telemetry_mtime_ns < session_started_epoch_ns )); then
  echo "dynamic avoidance telemetry predates the frozen session: ${telemetry}" >&2
  exit 2
fi

set +e
/usr/bin/python3 "${repo_root}/scripts/validate_formal_dynamic_obstacle_avoidance.py" \
  --episode-manifest "${episode_root}/public/episode_manifest.json" \
  --saved-map-artifact-dir "${saved_map_root}" \
  --telemetry "${telemetry}" \
  --pedestrian-schedule "${runtime_schedule}" \
  --environment-telemetry "${environment_telemetry}" \
  --snapshot-manifest "${snapshot_manifest}" \
  --session-status "${session_status}" \
  --runtime-binding "${runtime_binding}" \
  --output "${output}"
status=$?
set -e
if (( status != 0 )); then
  echo "dynamic avoidance runtime failed closed: report=${output}" >&2
  exit 2
fi
output_mtime_ns="$(/usr/bin/python3 -c 'import os,sys; print(os.stat(sys.argv[1]).st_mtime_ns)' "${output}")"
if (( output_mtime_ns < session_started_epoch_ns )); then
  echo "dynamic avoidance report predates the frozen session: ${output}" >&2
  exit 2
fi

if ! cleanup; then
  /usr/bin/python3 - "${output}" <<'PY'
import json, os, pathlib, sys

path = pathlib.Path(sys.argv[1])
if path.is_file():
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["status"] = "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_BLOCKED"
    payload["passed"] = False
    payload.setdefault("checks", {})["runtime_partition_cleanup_complete"] = False
    blockers = payload.setdefault("blockers", [])
    if "runtime_partition_cleanup_complete" not in blockers:
        blockers.append("runtime_partition_cleanup_complete")
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)
PY
  exit 3
fi

echo "formal dynamic-obstacle avoidance passed: ${output}"
