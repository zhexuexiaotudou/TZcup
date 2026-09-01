#!/usr/bin/env bash
# ROS setup hooks read optional variables that are legitimately unset. Enable
# nounset only after all three overlays have been sourced.
set -eo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
stage1_setup="${FORMAL_CAMPUS_STAGE1_SETUP:-${repo_root}/.work/stage1_20260826_023716/install/setup.bash}"
runtime_setup="${FORMAL_CAMPUS_RUNTIME_SETUP:-/home/zhexu/tzcup_integrated_build_20260826_v3/install/setup.bash}"
campus_setup="${FORMAL_CAMPUS_AGENT_SETUP:-/home/zhexu/tzcup_integrated_build_20260826_v3/install_formal_campus_agent/setup.bash}"
episode_root="${FORMAL_CAMPUS_EPISODE_ROOT:-${repo_root}/.work/formal_campus_episode_runtime}"
output_root="${FORMAL_CAMPUS_OUTPUT_ROOT:-${repo_root}/.work/formal_campus_runtime_acceptance}"
domain="${ROS_DOMAIN_ID:-89}"
for required in \
  /opt/ros/jazzy/setup.bash \
  "${stage1_setup}" \
  "${runtime_setup}" \
  "${campus_setup}" \
  "${episode_root}/public/world.sdf" \
  "${episode_root}/public/episode_manifest.json"; do
  if [[ ! -f "${required}" ]]; then
    echo "required formal-campus input is missing: ${required}" >&2
    exit 2
  fi
done

# Order is contractual: stage1 supplies OpenNav Coverage, v3 supplies the
# formal vehicle, and the campus overlay supplies the integration package.
source /opt/ros/jazzy/setup.bash
source "${stage1_setup}"
source "${runtime_setup}"
source "${campus_setup}"
set -u

export TZCUP_REPOSITORY_ROOT="${repo_root}"
export ROS_DOMAIN_ID="${domain}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
# This acceptance graph is deliberately single-host. LOCALHOST avoids WSL
# multicast/NAT discovery variance and prevents unrelated LAN participants
# from entering the safety-critical readiness decision.
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_campus_${domain}_$$}"
export RCUTILS_COLORIZED_OUTPUT=0

mkdir -p "${output_root}"
launch_log="${output_root}/formal_campus.launch.log"
report="${output_root}/formal_campus_runtime_readiness.json"
formal_runtime_register_evidence_paths "${report}"
materialized="${output_root}/materialized"
rm -f -- "${launch_log}" "${report}"

launch_pid=""
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_formal_campus_integration formal_campus.launch.py \
  gui:=false \
  world:="${episode_root}/public/world.sdf" \
  episode_manifest:="${episode_root}/public/episode_manifest.json" \
  world_name:=campus_formal \
  runtime_artifact_dir:="${materialized}" \
  simulation_initial_estop_active:=true \
  >"${launch_log}" 2>&1 &
launch_pid=$!

# Start the probe before the delayed Nav2 and coverage phases. Its DDS
# participant therefore observes the graph from inception instead of joining
# after the 30-process discovery burst that caused false empty-graph results.
sleep 2
set +e
python3 "${repo_root}/scripts/validate_formal_campus_runtime.py" \
  --timeout "${FORMAL_CAMPUS_READINESS_TIMEOUT_S:-240}" \
  --output "${report}"
status=$?
set -e

if ! kill -0 "${launch_pid}" 2>/dev/null; then
  echo "formal campus launch exited before readiness; see ${launch_log}" >&2
  exit 1
fi
if (( status != 0 )); then
  echo "formal campus readiness failed; report=${report} log=${launch_log}" >&2
  exit "${status}"
fi

echo "formal campus readiness passed: ${report}"
