#!/usr/bin/env bash
# Run R065 W1 against an isolated, real formal map-lifecycle ROS graph.
#
# This runner intentionally has no actuator, action, or joint-state writer.
# The installed runtime gate is the only test participant that publishes, and
# it can only assert base-motion inhibition plus the manager's opt-in endpoint.
set -Eeuo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)}"
run_root_arg="${1:?usage: run_r065_w1_dynamic_footprint_live.sh RUN_ROOT}"
[[ ! -L "${run_root_arg}" ]] || { echo "R065 W1 run-root argument must not be a symlink" >&2; exit 2; }
raw_run_root="$(realpath --no-symlinks -e "${run_root_arg}")"
run_root="$(realpath -e "${run_root_arg}")"
[[ "${raw_run_root}" == "${run_root}" ]] || { echo "R065 W1 run-root path must not traverse a symlink" >&2; exit 2; }
runtime_ws="${R065_RUNTIME_WS:?R065_RUNTIME_WS is required}"
closure_manifest="${R065_CLOSURE_MANIFEST:?R065_CLOSURE_MANIFEST is required}"
session="${R065_SESSION:?R065_SESSION is required}"
snapshot="${R065_SNAPSHOT:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
episode_root="${R065_EPISODE_ROOT:-${run_root}/episode}"
domain="${R065_W1_ROS_DOMAIN_ID:-91}"

[[ ! -L "${run_root}" && -d "${run_root}" ]] || {
  echo "R065 W1 run root must be an existing non-symlink directory" >&2
  exit 2
}
for required in \
  "${episode_root}/public/episode_manifest.json" \
  "${episode_root}/public/world.sdf" \
  "${session}" "${snapshot}" "${runtime_ws}/install/setup.bash" \
  "${closure_manifest}"; do
  [[ -f "${required}" && ! -L "${required}" ]] || {
    echo "R065 W1 required regular input is missing: ${required}" >&2
    exit 2
  }
done

runtime_root="${run_root}/w1_runtime"
output="${run_root}/w1.json"
runtime_binding="${run_root}/w1.runtime_binding.json"
launch_log="${runtime_root}/formal_map_lifecycle.launch.log"
gate_log="${runtime_root}/dynamic_footprint_gate.stdout"
cleanup_evidence="${runtime_root}/cleanup_evidence.txt"
[[ ! -e "${runtime_root}" && ! -e "${output}" && ! -e "${runtime_binding}" ]] || {
  echo "R065 W1 refuses retained runtime evidence or output" >&2
  exit 2
}
mkdir -p "${runtime_root}"

set +u
source /opt/ros/jazzy/setup.bash
set -u
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
formal_runtime_register_evidence_paths \
  "${runtime_root}" "${output}" "${runtime_binding}" "${cleanup_evidence}"

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
  echo "R065 W1 refuses an inherited Gazebo partition" >&2
  exit 2
}
export GZ_PARTITION="tzcup_r065_w1_${ROS_DOMAIN_ID}_$$"

launch_pid=""
cleanup() {
  local cleanup_status=0
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}" || cleanup_status=$?
  printf 'primary_error=%s\ncleanup_status=%s\nros_domain_id=%s\ngz_partition=%s\n' \
    "${primary_error:-none}" "${cleanup_status}" "${ROS_DOMAIN_ID}" "${GZ_PARTITION}" \
    >"${cleanup_evidence}"
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup

# Mapping mode ensures the formal lifecycle graph, real Nav2 costmaps and the
# formal localization/safety stack are present.  The test-only override is
# explicit and stays fail-safe because the gate permanently asserts inhibition.
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration \
  formal_campus_map_lifecycle.launch.py \
  mission_mode:=mapping gui:=false \
  world:="${episode_root}/public/world.sdf" \
  episode_manifest:="${episode_root}/public/episode_manifest.json" \
  map_artifact_dir:="${runtime_root}/map_lifecycle" \
  pedestrian_schedule:="${episode_root}/environment/pedestrian_schedule.json" \
  start_pedestrians:=false start_coverage:=false \
  enable_dynamic_footprint_runtime_test_override:=true \
  operation_speed_profile:=mapping_safe \
  >"${launch_log}" 2>&1 &
launch_pid=$!

# The lifecycle launch stages Nav2 after the vehicle/safety graph.  Do not use
# an injected footprint, static TF, joint state, or command publisher as a
# shortcut; the installed gate checks the named production graph itself.
ready="false"
for _ in $(seq 1 "${R065_W1_STARTUP_POLLS:-180}"); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "R065 W1 formal map-lifecycle launch exited early: ${launch_log}" >&2
    exit 3
  fi
  if ros2 node list 2>/dev/null | grep -Fxq /formal_dynamic_footprint_manager && \
      ros2 node list 2>/dev/null | grep -Fxq /whole_vehicle_safety_manager; then
    ready="true"
    break
  fi
  sleep 1
done
[[ "${ready}" == "true" ]] || {
  echo "R065 W1 timed out waiting for footprint and safety production nodes" >&2
  exit 3
}

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run sanitation_formal_campus_integration \
  formal-dynamic-footprint-runtime-gate \
  --motion-profile-file "${repo_root}/config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml" \
  --timeout-sec "${R065_W1_GATE_TIMEOUT_S:-90}" \
  --output "${output}" >"${gate_log}" 2>&1

[[ -f "${output}" && ! -L "${output}" ]] || {
  echo "R065 W1 gate did not emit its required output" >&2
  exit 3
}
python3 - "${repo_root}" "${output}" <<'PY'
import json
import pathlib
import sys

repo_root = pathlib.Path(sys.argv[1])
sys.path.insert(0, str(repo_root / "scripts"))
from publish_r065_public_modeling_receipt import _w1_passed

path = pathlib.Path(sys.argv[2])
payload = json.loads(path.read_text(encoding="utf-8"))
if not isinstance(payload, dict) or not _w1_passed(payload):
    raise SystemExit("R065 W1 gate output is not the exact PASS schema")
PY
echo "R065 W1 dynamic-footprint live gate passed: ${output}"
