#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
source "${repo_root}/scripts/formal_source_bound_preflight.sh"
runtime_setup="${FORMAL_VEHICLE_VISUAL_RUNTIME_SETUP:-${repo_root}/.work/final_functional_build/install/setup.bash}"
if [[ ! -f "${runtime_setup}" ]]; then
  echo "Missing built ROS workspace setup: ${runtime_setup}" >&2
  exit 2
fi
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-$(cd "$(dirname "${runtime_setup}")/.." && pwd)}"
runtime_install="${runtime_ws}/install"
if [[ "${runtime_setup}" != "${runtime_install}/setup.bash" ]]; then
  echo "visual runtime setup must be the frozen runtime install setup: ${runtime_setup}" >&2
  exit 2
fi
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"

base_domain="${ROS_DOMAIN_ID:-100}"
base_partition="${GZ_PARTITION:-tzcup_formal_visual_acceptance}"
run_id="${FORMAL_VEHICLE_VISUAL_RUN_ID:-$(date -u +%Y%m%dT%H%M%S)_$$_${RANDOM}}"
run_root="${FORMAL_VEHICLE_VISUAL_RUN_ROOT:-${repo_root}/.work/formal_vehicle_visual_acceptance/${run_id}}"
publish_root="${FORMAL_VEHICLE_VISUAL_PUBLISH_ROOT:-${repo_root}/reports/engineering}"
formal_runtime_configure "${base_domain}" 2
if ! mkdir -p "$(dirname "${run_root}")" || ! mkdir "${run_root}" 2>/dev/null; then
  echo "Refusing to reuse visual-acceptance run directory: ${run_root}" >&2
  exit 2
fi
runtime_binding="${run_root}/runtime_gate_binding.json"
formal_runtime_register_evidence_paths "${run_root}" "${runtime_binding}"

# Bind the one product/service run to the frozen runtime before sourcing its
# overlay or admitting either Gazebo profile. The capture finalizer copies
# this exact object into both published profile manifests and their sidecars.
formal_source_bound_preflight \
  "${repo_root}" "${runtime_ws}" "${runtime_closure_manifest}" \
  "${session}" "${snapshot}" "${runtime_binding}"

source /opt/ros/jazzy/setup.bash
source "${runtime_install}/setup.bash"
formal_source_bound_verify_overlay "${runtime_install}"
set -u
installed_visual_world="$(ros2 pkg prefix --share sanitation_vehicle_description)/worlds/formal_vehicle_visual_acceptance.sdf"
triggered_visual_world="${run_root}/formal_vehicle_visual_acceptance.triggered.sdf"
python3 "${repo_root}/scripts/prepare_formal_triggered_visual_world.py" \
  --source-world "${installed_visual_world}" \
  --output-world "${triggered_visual_world}" \
  --report "${run_root}/triggered_visual_world_report.json"

active_launch_pid=""
active_partition=""
FORMAL_VISUAL_LAST_COMPLETED_PARTITION=""
cleanup_active() {
  local result=0
  if [[ -n "${active_launch_pid}" ]]; then
    formal_runtime_cleanup_groups "${active_partition}" "${active_launch_pid}" || result=$?
    active_launch_pid=""
    active_partition=""
  fi
  return "${result}"
}
formal_runtime_install_traps cleanup_active

prepare_service_profile_after_product() {
  local product_dir="$1"
  local handoff_prefix="${run_root}/product_to_service.windows_memory_preflight"
  local cache_evidence="${run_root}/product_to_service.cache_reclaim.json"

  [[ -n "${FORMAL_VISUAL_LAST_COMPLETED_PARTITION}" ]] || {
    echo "visual service profile has no completed product partition to verify" >&2
    return 125
  }
  # capture_profile already reaped the launch leader and its partition. Repeat
  # the exact partition scan as a handoff assertion: service must never launch
  # while a product-profile process still owns its Gazebo partition.
  formal_runtime_cleanup_partition "${FORMAL_VISUAL_LAST_COMPLETED_PARTITION}" || {
    echo "product visual profile still has partition processes at service handoff" >&2
    return 125
  }
  [[ ! -e "${cache_evidence}" ]] || {
    echo "refusing stale product-to-service cache-reclaim evidence: ${cache_evidence}" >&2
    return 125
  }
  python3 - "${product_dir}" "${cache_evidence}" <<'PY'
import json
import os
import sys
from pathlib import Path

root = Path(sys.argv[1])
evidence = Path(sys.argv[2])
if not root.is_dir() or evidence.exists():
    raise SystemExit(2)
fadvise = getattr(os, "posix_fadvise", None)
dontneed = getattr(os, "POSIX_FADV_DONTNEED", None)
advised = 0
skipped = 0
for path in sorted(root.rglob("*")):
    if not path.is_file() or path.is_symlink():
        continue
    if fadvise is None or dontneed is None:
        skipped += 1
        continue
    try:
        with path.open("rb") as stream:
            fadvise(stream.fileno(), 0, 0, dontneed)
        advised += 1
    except (OSError, ValueError):
        skipped += 1
payload = json.dumps({
    "schema_version": 1,
    "status": "FORMAL_VISUAL_PROFILE_FILE_CACHE_RECLAIM_ATTEMPTED",
    "passed": True,
    "source_profile": "product",
    "target_profile": "service",
    "fadvise_dontneed_supported": fadvise is not None and dontneed is not None,
    "advised_file_count": advised,
    "skipped_file_count": skipped,
}, sort_keys=True) + "\n"
flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
fd = os.open(evidence, flags, 0o600)
with os.fdopen(fd, "w", encoding="utf-8") as stream:
    stream.write(payload)
PY
  # The unchanged formal Windows gate owns the bounded 60-second / 5-second
  # recovery wait and refuses the service launch if recovery is insufficient.
  formal_runtime_memory_preflight "${handoff_prefix}" || return "$?"
}

capture_profile() {
  local profile="$1"
  local bodywork_visible="$2"
  local domain_id="$3"
  local output_dir="$4"
  local launch_log="${output_dir}/launch.log"

  if ! mkdir "${output_dir}" 2>/dev/null; then
    echo "Refusing stale visual profile directory: ${output_dir}" >&2
    return 2
  fi
  export ROS_DOMAIN_ID="${domain_id}"
  export GZ_PARTITION="${base_partition}_${run_id}_${profile}_${domain_id}"

  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_visual_acceptance.launch.py \
    world:="${triggered_visual_world}" \
    bodywork_visible:="${bodywork_visible}" >"${launch_log}" 2>&1 &
  local launch_pid=$!
  active_launch_pid="${launch_pid}"
  active_partition="${GZ_PARTITION}"

  local result=0
  python3 "${repo_root}/scripts/capture_formal_vehicle_visual_acceptance.py" \
    --output "${output_dir}" --bodywork-profile "${profile}" \
    --runtime-binding "${runtime_binding}" \
    --renderer-log "${launch_log}" --timeout 150 --settle-seconds 18 \
    --trigger-cameras-sequentially \
    --triggered-world-report "${run_root}/triggered_visual_world_report.json" \
    --camera-contract-world "${triggered_visual_world}" || result=$?
  if ! formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"; then
    return 125
  fi
  FORMAL_VISUAL_LAST_COMPLETED_PARTITION="${GZ_PARTITION}"
  active_launch_pid=""
  active_partition=""
  return "${result}"
}

capture_profile product true "${base_domain}" \
  "${run_root}/formal_vehicle_visual_acceptance"
prepare_service_profile_after_product "${run_root}/formal_vehicle_visual_acceptance"
capture_profile service false "$((base_domain + 1))" \
  "${run_root}/formal_vehicle_service_visual_acceptance"

publish_profile() {
  local source_dir="$1"
  local target_dir="$2"
  local pending="${target_dir}.pending.$$"
  local previous="${target_dir}.previous.$$"
  [[ ! -e "${pending}" && ! -e "${previous}" ]] || {
    echo "Refusing stale visual publish staging path for ${target_dir}" >&2
    return 2
  }
  mkdir -p "$(dirname "${target_dir}")"
  cp -a -- "${source_dir}" "${pending}"
  if [[ -e "${target_dir}" ]]; then
    mv -- "${target_dir}" "${previous}"
  fi
  if ! mv -- "${pending}" "${target_dir}"; then
    [[ ! -e "${previous}" ]] || mv -- "${previous}" "${target_dir}"
    return 2
  fi
  rm -rf -- "${previous}"
}

# Publish only after both fresh Gazebo captures and their local validator pass.
publish_profile "${run_root}/formal_vehicle_visual_acceptance" \
  "${publish_root}/formal_vehicle_visual_acceptance"
publish_profile "${run_root}/formal_vehicle_service_visual_acceptance" \
  "${publish_root}/formal_vehicle_service_visual_acceptance"
echo "Published fresh visual acceptance from ${run_root}"
