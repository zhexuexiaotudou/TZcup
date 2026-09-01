#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh merged non-symlink runtime workspace}"
output_dir="${FORMAL_WATER_TYPED_OUTPUT_DIR:?set FORMAL_WATER_TYPED_OUTPUT_DIR to a fresh output directory}"
critical_manifest="${FORMAL_WATER_CRITICAL_SOURCE_MANIFEST:?set FORMAL_WATER_CRITICAL_SOURCE_MANIFEST to the fresh runtime/source manifest}"
duration_s="${FORMAL_WATER_TYPED_DURATION_S:-10}"
typed_topic=/model/tzcup_formal_sanitation_vehicle/cleaning_motors/telemetry_snapshot

[[ ! -e "${output_dir}" ]] || {
  echo "Refusing stale typed diagnostic output: ${output_dir}" >&2
  exit 2
}
[[ ! -L "${runtime_ws}" && -f "${runtime_ws}/install/setup.bash" && ! -L "${runtime_ws}/install/setup.bash" ]] || {
  echo "Typed diagnostic requires a regular merged runtime workspace: ${runtime_ws}" >&2
  exit 2
}
[[ -f "${runtime_ws}/INSTALL_SYMLINKS.txt" && ! -L "${runtime_ws}/INSTALL_SYMLINKS.txt" ]] || {
  echo "Runtime has no regular INSTALL_SYMLINKS.txt audit" >&2
  exit 2
}
[[ ! -s "${runtime_ws}/INSTALL_SYMLINKS.txt" ]] || {
  echo "Runtime install audit reports symbolic links" >&2
  exit 2
}
[[ -z "$(find "${runtime_ws}/install" -type l -print -quit)" ]] || {
  echo "Runtime install contains a symbolic link" >&2
  exit 2
}
[[ -f "${runtime_ws}/install/share/ament_index/resource_index/packages/sanitation_gazebo_control" ]] || {
  echo "Runtime has no merged ament package marker" >&2
  exit 2
}
[[ ! -e "${runtime_ws}/install/sanitation_gazebo_control" ]] || {
  echo "Runtime is isolated or mixed rather than one merged prefix" >&2
  exit 2
}
[[ -f "${critical_manifest}" && ! -L "${critical_manifest}" ]] || {
  echo "Critical source manifest must be a regular file" >&2
  exit 2
}

mkdir "${output_dir}"
formal_runtime_register_evidence_paths "${output_dir}"

python3 - "${repo_root}" "${runtime_ws}" "${critical_manifest}" \
  "${output_dir}/runtime_binding.json" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

repo, workspace, manifest_path, output = map(Path, sys.argv[1:])
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
assert manifest.get("schema_version") == 1
assert Path(manifest.get("workspace", "")).resolve() == workspace.resolve()
assert manifest.get("source_package_files_match_frozen_copy") is True
assert manifest.get("install_symlink_count") == 0
rows = manifest.get("critical_files")
assert isinstance(rows, list) and rows
for row in rows:
    relative = row["path"]
    source = repo / relative
    assert source.is_file() and not source.is_symlink()
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert digest == row["source_sha256"]
    if relative.startswith("starter_ws/"):
        frozen = workspace / relative.removeprefix("starter_ws/")
        assert frozen.is_file() and not frozen.is_symlink()
        assert hashlib.sha256(frozen.read_bytes()).hexdigest() == digest
        assert row.get("source_matches_frozen_copy") is True
payload = {
    "schema_version": 1,
    "status": "FORMAL_TYPED_RUNTIME_BINDING_VERIFIED",
    "passed": True,
    "runtime_workspace": str(workspace.resolve()),
    "critical_source_manifest": str(manifest_path.resolve()),
    "critical_source_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
    "install_mode": "merged_copy_install",
    "install_symlink_count": 0,
    "critical_file_count": len(rows),
}
output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

source "${runtime_ws}/install/setup.bash"
set -u
export PYTHONPATH="${repo_root}/scripts:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="tzcup_formal_typed_${ROS_DOMAIN_ID}_$$"
launch_pid=""

cleanup_launch() {
  local status=0
  if [[ -n "${launch_pid}" ]]; then
    formal_runtime_kill_group "${launch_pid}" || status=1
    launch_pid=""
  fi
  formal_runtime_stop_memory_watchdog || status=1
  formal_runtime_cleanup_partition "${GZ_PARTITION}" || status=1
  return "${status}"
}
formal_runtime_install_traps cleanup_launch

formal_runtime_memory_preflight "${output_dir}/windows_memory_preflight"
"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
  gui:=false bodywork_visible:=true high_bandwidth_sensor_runtime:=false \
  start_controllers:=true \
  enable_safety_manager:=true simulation_initial_estop_active:=true \
  start_simulation_safety_inputs:=true start_power_system_simulators:=true \
  water_evaluation_interfaces:=true \
  cleaning_realtime_telemetry_enabled:=true \
  cleaning_status_json_enabled:=true \
  cleaning_status_json_publish_rate_hz:=1.0 \
  >"${output_dir}/launch.log" 2>&1 &
launch_pid=$!
formal_runtime_start_memory_watchdog "${launch_pid}" \
  "${output_dir}/memory_watchdog"

python3 "${repo_root}/scripts/check_formal_water_preoperational_readiness.py" \
  --output "${output_dir}/preoperational_readiness.json" \
  >"${output_dir}/preoperational_readiness.log" 2>&1
gz topic -i -t "${typed_topic}" >"${output_dir}/gz_typed_topic_info.txt" 2>&1
ros2 topic info -v "${typed_topic}" >"${output_dir}/ros_typed_topic_info.txt" 2>&1

python3 "${repo_root}/scripts/collect_formal_typed_cleaning_motor_diagnostic.py" \
  --collect --duration-s "${duration_s}" \
  --trace "${output_dir}/raw_frames.jsonl" \
  --output "${output_dir}/typed_diag.collector.json" \
  >"${output_dir}/typed_diag.collector.log" 2>&1

cleanup_launch
python3 "${repo_root}/scripts/audit_formal_water_launch_log.py" \
  --log "${output_dir}/launch.log" \
  --output "${output_dir}/launch_audit.json" \
  >"${output_dir}/launch_audit.log" 2>&1
python3 "${repo_root}/scripts/collect_formal_typed_cleaning_motor_diagnostic.py" \
  --finalize \
  --collector-report "${output_dir}/typed_diag.collector.json" \
  --launch-log "${output_dir}/launch.log" \
  --launch-audit "${output_dir}/launch_audit.json" \
  --gz-topic-info "${output_dir}/gz_typed_topic_info.txt" \
  --ros-topic-info "${output_dir}/ros_typed_topic_info.txt" \
  --output "${output_dir}/typed_diag.json" \
  >"${output_dir}/typed_diag.log" 2>&1

sha256sum "${output_dir}"/* \
  "${repo_root}/scripts/run_formal_typed_cleaning_motor_diagnostic.sh" \
  "${repo_root}/scripts/collect_formal_typed_cleaning_motor_diagnostic.py" \
  "${critical_manifest}" >"${output_dir}/SHA256SUMS"
cat "${output_dir}/typed_diag.json"
