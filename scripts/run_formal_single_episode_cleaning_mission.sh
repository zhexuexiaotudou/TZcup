#!/usr/bin/env bash
# One Gazebo process, one frozen episode, one collector and one aggregate.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/scripts/run_formal_runtime_isolation.sh"
OPERATION_SPEED_PROFILE="${FORMAL_OPERATION_SPEED_PROFILE:-dry_cleaning_competition_candidate}"
REQUALIFIED_DRY_SPEED_ENABLEMENT="${FORMAL_REQUALIFIED_DRY_SPEED_ENABLEMENT:-0}"
REQUALIFICATION_RECEIPT="${FORMAL_DRY_SPEED_REQUALIFICATION_RECEIPT:-}"
WHOLE_VEHICLE_SAFETY_CAP="0.45"
EPISODE_ROOT=""
SESSION_STATUS=""
SAVED_MAP=""
PERCEPTION_ARTIFACTS=""
POLICY_CHECKPOINT=""
BASELINE=""
OUTPUT=""
RUNTIME_OVERLAY="${FORMAL_E2E_RUNTIME_OVERLAY:-}"
FORMAL_OUTPUT="${FORMAL_E2E_FINAL_ARTIFACT:-}"
ROS_DOMAIN="62"
TIMEOUT="21600"
MULTISITE_SITE_EVIDENCE=""
MULTISITE_SPLIT=""
MULTISITE_MAP_INDEX=""
MULTISITE_MAP_ID=""
MULTISITE_MISSION_INDEX=""

while (($#)); do
  case "$1" in
    --episode-root) EPISODE_ROOT="$2"; shift 2 ;;
    --session-status) SESSION_STATUS="$2"; shift 2 ;;
    --saved-map) SAVED_MAP="$2"; shift 2 ;;
    --perception-artifacts) PERCEPTION_ARTIFACTS="$2"; shift 2 ;;
    --policy-checkpoint) POLICY_CHECKPOINT="$2"; shift 2 ;;
    --same-map-baseline) BASELINE="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    --runtime-overlay) RUNTIME_OVERLAY="$2"; shift 2 ;;
    --formal-output) FORMAL_OUTPUT="$2"; shift 2 ;;
    --ros-domain-id) ROS_DOMAIN="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --multisite-site-evidence) MULTISITE_SITE_EVIDENCE="$2"; shift 2 ;;
    --multisite-split) MULTISITE_SPLIT="$2"; shift 2 ;;
    --multisite-map-index) MULTISITE_MAP_INDEX="$2"; shift 2 ;;
    --multisite-map-id) MULTISITE_MAP_ID="$2"; shift 2 ;;
    --multisite-mission-index) MULTISITE_MISSION_INDEX="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

multisite_args=("${MULTISITE_SITE_EVIDENCE}" "${MULTISITE_SPLIT}" "${MULTISITE_MAP_INDEX}" "${MULTISITE_MAP_ID}" "${MULTISITE_MISSION_INDEX}")
multisite_enabled=0
for value in "${multisite_args[@]}"; do
  [[ -n "${value}" ]] && multisite_enabled=1
done
if (( multisite_enabled )); then
  for value in "${multisite_args[@]}"; do
    [[ -n "${value}" ]] || { echo "all multi-site site-evidence arguments are required together" >&2; exit 2; }
  done
  [[ "${MULTISITE_SPLIT}" == "validation" || "${MULTISITE_SPLIT}" == "hidden" ]] || { echo "invalid multi-site split" >&2; exit 2; }
  [[ "${MULTISITE_MAP_INDEX}" =~ ^[0-9]+$ && "${MULTISITE_MISSION_INDEX}" =~ ^[0-9]+$ ]] || { echo "multi-site indices must be nonnegative integers" >&2; exit 2; }
  [[ ! -e "${MULTISITE_SITE_EVIDENCE}" ]] || { echo "refusing to overwrite multi-site evidence: ${MULTISITE_SITE_EVIDENCE}" >&2; exit 3; }
fi
COLLECTOR_MULTISITE_ARGS=()
if (( multisite_enabled )); then
  COLLECTOR_MULTISITE_ARGS=(--multisite-topic-observations "${OUTPUT}/multisite_live_observations.json")
fi

[[ -n "${FORMAL_OUTPUT}" ]] || FORMAL_OUTPUT="${ROOT}/artifacts/formal_end_to_end_cleaning_mission_acceptance.json"
formal_runtime_register_evidence_paths "${FORMAL_OUTPUT}"
for value in EPISODE_ROOT SESSION_STATUS SAVED_MAP PERCEPTION_ARTIFACTS POLICY_CHECKPOINT BASELINE OUTPUT RUNTIME_OVERLAY; do
  [[ -n "${!value}" ]] || { echo "missing required argument for ${value}" >&2; exit 2; }
done
formal_runtime_configure "${ROS_DOMAIN}"
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "missing ROS Jazzy setup" >&2; exit 2; }
[[ -f "${RUNTIME_OVERLAY}/setup.bash" ]] || {
  echo "missing frozen E2E runtime overlay setup: ${RUNTIME_OVERLAY}/setup.bash" >&2
  exit 2
}
[[ ! -e "${OUTPUT}" ]] || { echo "refusing to overwrite retained evidence: ${OUTPUT}" >&2; exit 3; }
[[ ! -e "${FORMAL_OUTPUT}" ]] || { echo "refusing to overwrite formal E2E evidence: ${FORMAL_OUTPUT}" >&2; exit 3; }
RUNTIME_BINDING="${FORMAL_E2E_RUNTIME_BINDING:-${FORMAL_OUTPUT}.runtime_binding.json}"
[[ ! -e "${RUNTIME_BINDING}" ]] || { echo "refusing to overwrite formal E2E runtime binding: ${RUNTIME_BINDING}" >&2; exit 3; }
mkdir -p "${OUTPUT}"
RUNTIME_CLOSURE="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${RUNTIME_OVERLAY}")/final_runtime_closure_manifest.json}"
formal_runtime_register_evidence_paths "${FORMAL_OUTPUT}" "${RUNTIME_BINDING}"
python3 "${ROOT}/scripts/formal_runtime_gate_binding.py" \
  --repository-root "${ROOT}" --install-root "${RUNTIME_OVERLAY}" \
  --closure-manifest "${RUNTIME_CLOSURE}" --session "${SESSION_STATUS}" \
  --snapshot "${ROOT}/reports/engineering/formal_vehicle_snapshot_manifest.json" \
  --output "${RUNTIME_BINDING}"

# The single episode remains capped at 0.45 m/s unless an operator explicitly
# opts into the retained, source/session/runtime-bound four-stage receipt.
# Neither this receipt nor this switch changes wet, mapping, transport, or
# hardware authority; measured same-map efficiency is still validated later.
case "${REQUALIFIED_DRY_SPEED_ENABLEMENT}" in
  0) ;;
  1)
    # product_demo co-hosts manipulation and lacks a transition-bound
    # dry-cleaning qualification state.  A global manager cap would also
    # widen non-dry phases, so keep high-speed use fail-closed.
    echo "requalified 1.0 m/s single-episode use is BLOCKED: product_demo lacks a dry-only safety-manager state and live effective-cap receipt" >&2
    exit 2
    ;;
  *) echo "FORMAL_REQUALIFIED_DRY_SPEED_ENABLEMENT must be 0 or 1" >&2; exit 2 ;;
esac

# A formal mission sources exactly one source-frozen overlay above the base ROS
# distribution. Historical developer overlays must never leak into this graph.
source /opt/ros/jazzy/setup.bash
source "${RUNTIME_OVERLAY}/setup.bash"

EPISODE_MANIFEST="${EPISODE_ROOT}/public/episode_manifest.json"
EVALUATOR_EPISODE_MANIFEST="${EPISODE_ROOT}/evaluator/episode_manifest.json"
EVALUATOR_GROUND_TRUTH="${EPISODE_ROOT}/evaluator/ground_truth.json"
WORLD="${EPISODE_ROOT}/public/world.sdf"
SCHEDULE="${EPISODE_ROOT}/environment/pedestrian_schedule.json"
for file in "${EPISODE_MANIFEST}" "${EVALUATOR_EPISODE_MANIFEST}" "${EVALUATOR_GROUND_TRUTH}" "${WORLD}" "${SCHEDULE}" "${SESSION_STATUS}" "${POLICY_CHECKPOINT}" "${BASELINE}"; do
  [[ -f "${file}" ]] || { echo "missing required file: ${file}" >&2; exit 3; }
done
for directory in "${SAVED_MAP}" "${PERCEPTION_ARTIFACTS}"; do
  [[ -d "${directory}" ]] || { echo "missing required directory: ${directory}" >&2; exit 3; }
done

# Recompute the FullCoverage baseline from its source evidence before any
# Gazebo process is started.  This rejects a copied status JSON, a different
# formal session/snapshot, changed saved-map inputs or a forged distance.
python3 "${ROOT}/scripts/generate_formal_same_map_baseline.py" validate \
  --input "${BASELINE}" --session "${SESSION_STATUS}" \
  --snapshot "${ROOT}/reports/engineering/formal_vehicle_snapshot_manifest.json"

readarray -t IDENTITY < <(python3 - "${EPISODE_MANIFEST}" "${EVALUATOR_EPISODE_MANIFEST}" "${EVALUATOR_GROUND_TRUTH}" "${SESSION_STATUS}" "${BASELINE}" "${WORLD}" "${POLICY_CHECKPOINT}" "${SCHEDULE}" <<'PY'
import json, sys
import hashlib
episode=json.load(open(sys.argv[1], encoding='utf-8'))
evaluator=json.load(open(sys.argv[2], encoding='utf-8'))
truth=json.load(open(sys.argv[3], encoding='utf-8'))
session=json.load(open(sys.argv[4], encoding='utf-8'))
baseline=json.load(open(sys.argv[5], encoding='utf-8'))
snapshot=session.get('snapshot',{}).get('snapshot_manifest_sha256')
started=session.get('started_epoch_ns')
if not snapshot or not isinstance(started,int): raise SystemExit('invalid frozen session')
if evaluator.get('episode_id') != episode.get('episode_id') or truth.get('episode_id') != episode.get('episode_id'):
    raise SystemExit('episode identity mismatch across public/evaluator inputs')
if evaluator.get('map_id') != episode.get('map_id') or baseline.get('map_id') != episode.get('map_id'):
    raise SystemExit('map identity mismatch')
baseline_episode=baseline.get('evidence',{}).get('episode_manifest',{})
if baseline.get('episode_id') != episode.get('episode_id') or baseline_episode.get('sha256') != hashlib.sha256(open(sys.argv[1], 'rb').read()).hexdigest():
    raise SystemExit('baseline was not generated from this exact episode manifest')
if evaluator.get('truth_boundary',{}).get('control_use_prohibited') is not True or truth.get('control_use_prohibited') is not True:
    raise SystemExit('evaluator truth boundary missing')
if hashlib.sha256(open(sys.argv[6], 'rb').read()).hexdigest() != evaluator.get('world_sha256'):
    raise SystemExit('world hash differs from evaluator episode manifest')
if episode.get('profile') != 'formal' or episode.get('counts',{}).get('discrete_cubes') != 20:
    raise SystemExit('episode is not the formal 20-cube mission')
if episode.get('cube_contract',{}).get('edge_m') != 0.03:
    raise SystemExit('episode does not explicitly declare 3 cm cubes')
if episode.get('dynamic_pedestrians_present') is not True or episode.get('counts',{}).get('pedestrians',0) <= 0:
    raise SystemExit('episode has no explicitly randomized moving pedestrians')
schedule=json.load(open(sys.argv[8], encoding='utf-8'))
if len(schedule.get('pedestrians',[])) != episode['counts']['pedestrians']:
    raise SystemExit('pedestrian schedule count differs from episode manifest')
if baseline.get('report_id') != 'tzcup_formal_same_map_full_coverage_baseline_v1' or baseline.get('session_bound') is not True:
    raise SystemExit('baseline is not a session-bound formal same-map report')
for key in ('fixed_start_verified','first_map_ignored_dirt','saved_map_hard_restart_verified'):
    if baseline.get(key) is not True: raise SystemExit(f'baseline missing required true fact: {key}')
if baseline.get('status') != 'FORMAL_FULL_COVERAGE_BASELINE_PASSED':
    raise SystemExit('baseline does not declare a successful status')
if baseline.get('planner') != 'full_coverage' or baseline.get('truth_used_for_control') is not False:
    raise SystemExit('baseline is not the truth-free FullCoverage planner')
if baseline.get('return_distance_included') is not False:
    raise SystemExit('baseline distance includes return-home travel')
policy=json.load(open(sys.argv[7], encoding='utf-8'))
if policy.get('policy') != 'q_learning' or policy.get('truth_access_used') is not False or not policy.get('q_table'):
    raise SystemExit('policy checkpoint is not a trained truth-free Q policy')
seeds=evaluator.get('seeds')
if not isinstance(seeds,dict) or set(seeds) != {'layout','dirt','cubes','pedestrians','sensor'}:
    raise SystemExit('evaluator seed ledger missing or malformed')
seed=seeds.get('dirt')
if not isinstance(seed,int) or seed <= 0: raise SystemExit('mission seed missing')
print(episode['episode_id']); print(seed); print(f'{snapshot}:{started}'); print(started)
print(float(baseline['successful_distance_m']))
PY
)
EPISODE_ID="${IDENTITY[0]}"
EPISODE_SEED="${IDENTITY[1]}"
SESSION_ID="${IDENTITY[2]}"
SESSION_START="${IDENTITY[3]}"
MAX_DISTANCE="${IDENTITY[4]}"
RUNTIME_ID="${EPISODE_ID}-$(date -u +%Y%m%dT%H%M%SZ)-$$"
export ROS_DOMAIN_ID="${ROS_DOMAIN}"
export GZ_PARTITION="tzcup-single-episode-${ROS_DOMAIN}-$$"

# Freeze every launch input before Gazebo is allowed to read it. Directories
# use a canonical relative-path / size / SHA-256 tree manifest, so adding,
# removing, renaming or mutating a file invalidates the episode.
python3 "${ROOT}/scripts/collect_formal_single_episode_cleaning_mission.py" \
  --prepare-input-binding "${OUTPUT}/input_binding.json" \
  --episode-manifest "${EPISODE_MANIFEST}" \
  --evaluator-episode-manifest "${EVALUATOR_EPISODE_MANIFEST}" \
  --evaluator-ground-truth "${EVALUATOR_GROUND_TRUTH}" \
  --world "${WORLD}" --pedestrian-schedule "${SCHEDULE}" \
  --session-status "${SESSION_STATUS}" --same-map-baseline "${BASELINE}" \
  --policy-checkpoint "${POLICY_CHECKPOINT}" \
  --runtime-binding "${RUNTIME_BINDING}" \
  --saved-map "${SAVED_MAP}" --perception-artifacts "${PERCEPTION_ARTIFACTS}"

PIDS=()
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${PIDS[@]:-}"
}
formal_runtime_install_traps cleanup

# Exactly one launch owns Gazebo and all product nodes for the whole episode.
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_product_demo_integration product_demo.launch.py \
  gui:=false world:="${WORLD}" episode_manifest:="${EPISODE_MANIFEST}" \
  pedestrian_schedule:="${SCHEDULE}" start_pedestrians:=true \
  saved_map_artifact_dir:="${SAVED_MAP}" perception_artifact_root:="${PERCEPTION_ARTIFACTS}" \
  policy_checkpoint:="${POLICY_CHECKPOINT}" maximum_task_distance_m:="${MAX_DISTANCE}" \
  episode_seed:="${EPISODE_SEED}" operation_speed_profile:="${OPERATION_SPEED_PROFILE}" \
  max_linear_velocity:="${WHOLE_VEHICLE_SAFETY_CAP}" >"${OUTPUT}/product_demo.log" 2>&1 &
GAZEBO_LAUNCH_PID=$!
PIDS+=("${GAZEBO_LAUNCH_PID}")

# These are one-way evaluator bridges into the collector namespace.  Their ROS
# outputs are not names accepted by any product node.
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  /model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json@std_msgs/msg/String[gz.msgs.StringMsg \
  --ros-args -r /model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json:=/evaluation/single_episode/ground_dirt/status_json \
  >"${OUTPUT}/ground_dirt_evaluator_bridge.log" 2>&1 & PIDS+=("$!")
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  /model/tzcup_formal_sanitation_vehicle/water_recovery/status_json@std_msgs/msg/String[gz.msgs.StringMsg \
  --ros-args -r /model/tzcup_formal_sanitation_vehicle/water_recovery/status_json:=/evaluation/single_episode/water_recovery/status_json \
  >"${OUTPUT}/water_evaluator_bridge.log" 2>&1 & PIDS+=("$!")
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 run ros_gz_bridge parameter_bridge \
  /model/tzcup_formal_sanitation_vehicle/dry_bin/status_json@std_msgs/msg/String[gz.msgs.StringMsg \
  --ros-args -r /model/tzcup_formal_sanitation_vehicle/dry_bin/status_json:=/evaluation/single_episode/dry_bin/status_json \
  >"${OUTPUT}/dry_bin_evaluator_bridge.log" 2>&1 & PIDS+=("$!")

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" python3 "${ROOT}/scripts/collect_formal_single_episode_cleaning_mission.py" \
  --session-id "${SESSION_ID}" --episode-id "${EPISODE_ID}" --episode-seed "${EPISODE_SEED}" \
  --runtime-id "${RUNTIME_ID}" --gazebo-process-id "${GAZEBO_LAUNCH_PID}" \
  --session-start-epoch-ns "${SESSION_START}" --episode-manifest "${EPISODE_MANIFEST}" \
  --evaluator-episode-manifest "${EVALUATOR_EPISODE_MANIFEST}" \
  --evaluator-ground-truth "${EVALUATOR_GROUND_TRUTH}" \
  --session-status "${SESSION_STATUS}" --same-map-baseline "${BASELINE}" \
  --world "${WORLD}" --pedestrian-schedule "${SCHEDULE}" \
  --policy-checkpoint "${POLICY_CHECKPOINT}" \
  --runtime-binding "${RUNTIME_BINDING}" \
  --saved-map "${SAVED_MAP}" --perception-artifacts "${PERCEPTION_ARTIFACTS}" \
  --input-binding "${OUTPUT}/input_binding.json" \
  "${COLLECTOR_MULTISITE_ARGS[@]}" \
  --ready-file "${OUTPUT}/collector_ready.json" \
  --timeout "${TIMEOUT}" --output "${OUTPUT}/raw_collection.json" &
COLLECTOR_PID=$!
PIDS+=("${COLLECTOR_PID}")

# Do not start the operator gate until the collector has frozen initial
# evaluator states, resolved runtime parameters and proved from the executable
# ROS graph that no product node subscribes to evaluator truth.
for ((attempt=0; attempt<240; attempt++)); do
  [[ -f "${OUTPUT}/collector_ready.json" ]] && break
  kill -0 "${GAZEBO_LAUNCH_PID}" 2>/dev/null || { echo "product launch exited before readiness" >&2; exit 4; }
  kill -0 "${COLLECTOR_PID}" 2>/dev/null || { echo "collector exited before readiness" >&2; exit 4; }
  sleep 1
done
[[ -f "${OUTPUT}/collector_ready.json" ]] || { echo "collector did not establish fail-closed pre-start state" >&2; exit 4; }

# The evaluator never commands actuators or teleports the robot. This public
# product operator gate is the only mission-start write.
ros2 topic pub --once /product_demo/operator_start std_msgs/msg/Bool '{data: true}'
wait "${COLLECTOR_PID}"

python3 "${ROOT}/scripts/aggregate_formal_single_episode_cleaning_mission.py" \
  --raw "${OUTPUT}/raw_collection.json" --output "${OUTPUT}/aggregate.json"
python3 "${ROOT}/scripts/validate_formal_end_to_end_cleaning_mission.py" \
  --input "${OUTPUT}/aggregate.json" \
  --snapshot-manifest "${ROOT}/reports/engineering/formal_vehicle_snapshot_manifest.json" \
  --session-status "${SESSION_STATUS}" --runtime-binding "${RUNTIME_BINDING}" \
  --output "${OUTPUT}/validation.json"
if (( multisite_enabled )); then
  python3 "${ROOT}/scripts/formal_multisite_product_acceptance.py" \
    --emit-site-evidence "${MULTISITE_SITE_EVIDENCE}" \
    --validation "${OUTPUT}/validation.json" --raw "${OUTPUT}/raw_collection.json" \
    --topic-observations "${OUTPUT}/multisite_live_observations.json" \
    --episode-manifest "${EPISODE_MANIFEST}" \
    --snapshot "${ROOT}/reports/engineering/formal_vehicle_snapshot_manifest.json" \
    --session "${SESSION_STATUS}" --runtime-closure "${RUNTIME_CLOSURE}" \
    --split "${MULTISITE_SPLIT}" --map-index "${MULTISITE_MAP_INDEX}" \
    --map-id "${MULTISITE_MAP_ID}" --mission-index "${MULTISITE_MISSION_INDEX}"
fi
mkdir -p "$(dirname "${FORMAL_OUTPUT}")"
pending="${FORMAL_OUTPUT}.pending.$$"
[[ ! -e "${pending}" ]] || { echo "refusing stale formal publish path: ${pending}" >&2; exit 3; }
cp -- "${OUTPUT}/validation.json" "${pending}"
mv -- "${pending}" "${FORMAL_OUTPUT}"
echo "Published formal E2E acceptance: ${FORMAL_OUTPUT}"
