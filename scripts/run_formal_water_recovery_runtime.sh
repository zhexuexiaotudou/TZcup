#!/usr/bin/env bash
set -eo pipefail

default_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${FORMAL_WATER_REPOSITORY_ROOT:-}" && "${FORMAL_TASK_ONLY_DIAGNOSTIC:-0}" != "1" ]]; then
  echo "FORMAL_WATER_REPOSITORY_ROOT is restricted to task-only diagnostics" >&2
  exit 2
fi
repo_root="${FORMAL_WATER_REPOSITORY_ROOT:-${default_repo_root}}"
[[ -d "${repo_root}" ]] || { echo "Invalid formal water repository root: ${repo_root}" >&2; exit 2; }
repo_root="$(cd "${repo_root}" && pwd -P)"
[[ -f "${repo_root}/scripts/run_formal_runtime_isolation.sh" ]] || {
  echo "Missing formal runtime isolation helper under repository root: ${repo_root}" >&2
  exit 2
}
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime/install}"
runtime_closure_manifest="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-$(dirname "${runtime_ws}")/final_runtime_closure_manifest.json}"
output_dir="${FORMAL_WATER_OUTPUT_DIR:-${repo_root}/artifacts/formal_water_recovery}"
formal_output="${FORMAL_WATER_FINAL_ARTIFACT:-${repo_root}/artifacts/formal_water_recovery_acceptance.json}"
runtime_binding="${FORMAL_WATER_RUNTIME_BINDING:-${formal_output}.runtime_binding.json}"
session="${FORMAL_ACCEPTANCE_SESSION:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
launch_settle_s="${FORMAL_WATER_LAUNCH_SETTLE_S:-0}"
preembedded_model_pose="${FORMAL_WATER_PREEMBEDDED_MODEL_POSE:-0 0 0.005 0 0 0}"
scenario="all"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      scenario="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ "${scenario}" != "normal" && "${scenario}" != "full" && "${scenario}" != "diagnostic" && "${scenario}" != "all" ]]; then
  echo "--scenario must be normal, full, diagnostic, or all" >&2
  exit 2
fi
if [[ "${scenario}" == "all" && -n "${FORMAL_WATER_REPOSITORY_ROOT:-}" ]]; then
  echo "Repository-root override is forbidden for formal all-scenarios acceptance" >&2
  exit 2
fi

# An all-scenarios invocation is the only mode that may publish the canonical
# water-recovery acceptance.  Retire any earlier canonical result before ROS
# setup or any other preflight so a failed fresh attempt can never leave an
# older PASS looking current.  The raw scenario directory remains immutable
# evidence and is still rejected below rather than overwritten.
if [[ "${scenario}" == "all" ]]; then
  for retained in "${formal_output}" "${runtime_binding}"; do
    if [[ -e "${retained}" || -L "${retained}" ]]; then
      superseded="${retained}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"
      [[ ! -e "${superseded}" && ! -L "${superseded}" ]] || {
        echo "Refusing to overwrite retained superseded water-recovery evidence: ${superseded}" >&2
        exit 2
      }
      mv -- "${retained}" "${superseded}"
    fi
  done
fi

# Treat the final report and its frozen-runtime binding as one evidence unit.
# If targeted teardown fails, leaving the binding at its canonical path would
# make it look as though a later report could still be attached to this
# contaminated Gazebo session.  The other final-runtime runners quarantine
# both files, so retain that fail-closed contract here as well.
formal_runtime_register_evidence_paths "${formal_output}" "${runtime_binding}"
for preembedded_scenario in normal full diagnostic; do
  formal_runtime_register_evidence_paths \
    "${output_dir}/water_${preembedded_scenario}_preembedded_sensor_world.sdf" \
    "${output_dir}/water_${preembedded_scenario}_preembedded_sensor_world.json"
done
if [[ -f "${runtime_ws}/install/setup.bash" ]]; then
  runtime_setup="${runtime_ws}/install/setup.bash"
  runtime_install_root="${runtime_ws}/install"
elif [[ -f "${runtime_ws}/setup.bash" ]]; then
  runtime_setup="${runtime_ws}/setup.bash"
  runtime_install_root="${runtime_ws}"
else
  echo "Missing built ROS workspace setup under: ${runtime_ws}" >&2
  exit 2
fi

if [[ "${scenario}" == "all" ]]; then
  [[ -f "${runtime_closure_manifest}" ]] || {
    echo "Missing frozen final runtime closure: ${runtime_closure_manifest}" >&2
    exit 2
  }
  [[ -f "${session}" ]] || {
    echo "Missing running formal acceptance session: ${session}" >&2
    exit 2
  }
  python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \
    --check --output "${snapshot}"
  python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
    --repository-root "${repo_root}" --install-root "${runtime_ws}" \
    --closure-manifest "${runtime_closure_manifest}" --session "${session}" \
    --snapshot "${snapshot}" --output "${runtime_binding}"
fi

source /opt/ros/jazzy/setup.bash
source "${runtime_setup}"
set -u

installed_package_share="$(ros2 pkg prefix --share sanitation_vehicle_description)"
expected_package_share="${runtime_install_root}/share/sanitation_vehicle_description"
if [[ ! -d "${expected_package_share}" ]] || [[ "$(cd "${installed_package_share}" && pwd -P)" != "$(cd "${expected_package_share}" && pwd -P)" ]]; then
  echo "sanitation_vehicle_description resolved outside frozen runtime: ${installed_package_share}" >&2
  exit 2
fi
vehicle_xacro="${installed_package_share}/urdf/formal_competition_vehicle.urdf.xacro"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
if [[ "${scenario}" == "all" && ( -e "${output_dir}" || -e "${formal_output}" ) ]]; then
  echo "Refusing stale water-recovery evidence; use fresh raw and formal output paths" >&2
  exit 2
fi
mkdir -p "$(dirname "${output_dir}")" "$(dirname "${formal_output}")"
[[ -d "${output_dir}" ]] || mkdir "${output_dir}"

launch_pid=""
active_partition=""
cleanup_launch() {
  local cleanup_status=0
  if [[ -n "${launch_pid}" ]]; then
    formal_runtime_kill_group "${launch_pid}" || cleanup_status=1
    launch_pid=""
  fi
  formal_runtime_stop_memory_watchdog || cleanup_status=1
  if [[ -n "${active_partition}" ]]; then
    formal_runtime_cleanup_partition "${active_partition}" || cleanup_status=1
    active_partition=""
  fi
  return "${cleanup_status}"
}
formal_runtime_install_traps cleanup_launch

formal_water_stop_active_bridges() {
  local timeline="$1"
  python3 - "${GZ_PARTITION}" "${timeline}" <<'PY'
import json
import os
from pathlib import Path
import signal
import sys
import time


partition, timeline_arg = sys.argv[1:]
timeline = Path(timeline_arg)
ordered_targets = (
    ("native", "water_evaluation_bridge", "water_evaluation_bridge"),
    ("native", "formal_vehicle_product_native_bridge", "formal_vehicle_product_bridge"),
    ("native", "cleaning_actuator_scalar_native_bridge", "cleaning_actuator_scalar_bridge"),
    ("native", "a300_drivetrain_native_bridge", "a300_drivetrain_bridge"),
    ("native", "formal_contact_evaluation_native_bridge", "formal_squeegee_evaluation_bridge"),
    ("native", "formal_contact_evaluation_native_bridge", "formal_brush_contact_evaluation_bridge"),
    ("native", "formal_contact_evaluation_native_bridge", "charge_receptacle_contact_bridge"),
    ("native", "formal_contact_evaluation_native_bridge", "wastewater_drain_contact_bridge"),
    ("native", "formal_contact_evaluation_native_bridge", "front_bumper_contact_bridge"),
    ("native", "formal_contact_evaluation_native_bridge", "rear_bumper_contact_bridge"),
    ("native", "formal_auxiliary_native_bridge", "formal_auxiliary_bridge"),
    # This native bridge is the sole /clock writer.  Keep it alive until every
    # other managed bridge has drained, then prove that it also exits cleanly.
    ("native", "cleaning_actuator_vector_bridge", "cleaning_actuator_motor_bridge"),
)
native_targets = tuple(
    (executable, node) for kind, executable, node in ordered_targets if kind == "native"
)
native_nodes_by_executable = {}
for executable, node in native_targets:
    native_nodes_by_executable.setdefault(executable, set()).add(node)


def record(event, **fields):
    entry = {
        "event": event,
        "monotonic_s": time.monotonic(),
        "partition": partition,
        **fields,
    }
    with timeline.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(entry, sort_keys=True) + "\n")


def read_proc(pid, name):
    try:
        return (Path("/proc") / str(pid) / name).read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None


def parameter_bridge_census():
    nodes = {}
    malformed = []
    for proc_dir in Path("/proc").iterdir():
        if not proc_dir.name.isdigit():
            continue
        pid = int(proc_dir.name)
        environ_raw = read_proc(pid, "environ")
        cmdline_raw = read_proc(pid, "cmdline")
        if environ_raw is None or cmdline_raw is None:
            continue
        environment = environ_raw.split(b"\0")
        if f"GZ_PARTITION={partition}".encode() not in environment:
            continue
        argv = [part.decode("utf-8", "replace") for part in cmdline_raw.split(b"\0") if part]
        if not argv:
            continue
        executable = Path(argv[0])
        if executable.name != "parameter_bridge" or "ros_gz_bridge" not in executable.parts:
            continue
        node_args = [argument for argument in argv if argument.startswith("__node:=")]
        if len(node_args) != 1 or not node_args[0].removeprefix("__node:="):
            malformed.append(pid)
            continue
        node_name = node_args[0].removeprefix("__node:=")
        nodes.setdefault(node_name, []).append(pid)
    return ({node: sorted(pids) for node, pids in sorted(nodes.items())}, sorted(malformed))


def native_bridge_census():
    nodes = {}
    malformed = []
    unknown = []
    for proc_dir in Path("/proc").iterdir():
        if not proc_dir.name.isdigit():
            continue
        pid = int(proc_dir.name)
        environ_raw = read_proc(pid, "environ")
        cmdline_raw = read_proc(pid, "cmdline")
        if environ_raw is None or cmdline_raw is None:
            continue
        environment = environ_raw.split(b"\0")
        if f"GZ_PARTITION={partition}".encode() not in environment:
            continue
        argv = [part.decode("utf-8", "replace") for part in cmdline_raw.split(b"\0") if part]
        if not argv:
            continue
        executable = Path(argv[0])
        expected_nodes = native_nodes_by_executable.get(executable.name)
        if "sanitation_gazebo_control" not in executable.parts:
            continue
        if expected_nodes is None:
            # Water disables manipulation, dry-bin and service-door evaluator
            # interfaces.  A same-partition control executable whose basename
            # is a bridge is therefore neither a benign adapter/mirror nor an
            # accepted water process; retain its identity and fail closed.
            if "bridge" in executable.name.lower():
                unknown.append({"pid": pid, "executable": executable.name, "argv": argv})
            continue
        node_args = [argument for argument in argv if argument.startswith("__node:=")]
        node_name = node_args[0].removeprefix("__node:=") if len(node_args) == 1 else ""
        if not node_name or node_name not in expected_nodes:
            malformed.append(pid)
            continue
        nodes.setdefault(node_name, []).append(pid)
    return (
        {node: sorted(pids) for node, pids in sorted(nodes.items())},
        sorted(malformed),
        sorted(unknown, key=lambda item: (item["executable"], item["pid"])),
    )


def process_exit_state(pid):
    status = read_proc(pid, "stat")
    if status is None:
        return "missing"
    fields = status.decode("utf-8", "replace").split()
    if len(fields) >= 3 and fields[2] == "Z":
        return "zombie"
    return None


initial_nodes, malformed = parameter_bridge_census()
initial_native_nodes, native_malformed, native_unknown = native_bridge_census()
unexpected = sorted(initial_nodes)
duplicates = {node: pids for node, pids in initial_nodes.items() if len(pids) != 1}
expected_native_nodes = {node for _, node in native_targets}
native_missing = sorted(expected_native_nodes - set(initial_native_nodes))
native_unexpected = sorted(set(initial_native_nodes) - expected_native_nodes)
native_duplicates = {
    node: pids for node, pids in initial_native_nodes.items() if len(pids) != 1
}
record(
    "pre_shutdown_census",
    nodes=initial_nodes,
    malformed_pids=malformed,
    expected_nodes=[],
    unexpected=unexpected,
    duplicates=duplicates,
)
for executable, target in native_targets:
    record(
        "native_bridge_pre_shutdown",
        executable=executable,
        target=target,
        pids=initial_native_nodes.get(target, []),
        malformed_pids=native_malformed,
        unknown_bridge_processes=native_unknown,
    )
if (
    malformed
    or native_malformed
    or native_unknown
    or unexpected
    or duplicates
    or native_missing
    or native_unexpected
    or native_duplicates
):
    record("pre_shutdown_census_failed")
    raise SystemExit(1)

def stop_native_bridge(executable, native_target):
    native_pid = initial_native_nodes[native_target][0]
    try:
        os.kill(native_pid, signal.SIGINT)
    except ProcessLookupError:
        record(
            "native_bridge_signal_failed",
            executable=executable,
            target=native_target,
            pid=native_pid,
        )
        raise SystemExit(1)
    record(
        "native_bridge_sigint_sent",
        executable=executable,
        target=native_target,
        pid=native_pid,
    )
    native_deadline = time.monotonic() + 8.0
    native_saw_zombie = False
    while time.monotonic() < native_deadline:
        native_state = process_exit_state(native_pid)
        if native_state == "missing":
            record(
                "native_bridge_reaped",
                executable=executable,
                target=native_target,
                pid=native_pid,
                saw_zombie=native_saw_zombie,
            )
            break
        native_saw_zombie = native_saw_zombie or native_state == "zombie"
        time.sleep(0.05)
    else:
        record(
            "native_bridge_exit_not_clean",
            executable=executable,
            target=native_target,
            pid=native_pid,
            exit_state=process_exit_state(native_pid),
            saw_zombie=native_saw_zombie,
        )
        raise SystemExit(1)


first_kind, first_executable, first_target = ordered_targets[0]
if first_kind != "native":
    record("shutdown_order_invalid", first_kind=first_kind)
    raise SystemExit(1)
stop_native_bridge(first_executable, first_target)
record("ordered_shutdown_started", targets=[node for _, _, node in ordered_targets])
for kind, executable, target in ordered_targets[1:]:
    if kind == "native":
        stop_native_bridge(executable, target)
    else:
        record("shutdown_order_invalid", kind=kind, target=target)
        raise SystemExit(1)
remaining_nodes, malformed = parameter_bridge_census()
remaining_native_nodes, native_malformed, native_unknown = native_bridge_census()
record(
    "post_shutdown_census",
    nodes=remaining_nodes,
    native_nodes=remaining_native_nodes,
    malformed_pids=malformed,
    native_malformed_pids=native_malformed,
    native_unknown_bridge_processes=native_unknown,
)
if remaining_nodes or remaining_native_nodes or malformed or native_malformed or native_unknown:
    record("post_shutdown_census_failed")
    raise SystemExit(1)
record("ordered_shutdown_completed")
PY
}

run_scenario() {
  local selected="$1"
  local expected_stable_marker_count=1
  local preembedded_world="${output_dir}/water_${selected}_preembedded_sensor_world.sdf"
  local preembedded_report="${output_dir}/water_${selected}_preembedded_sensor_world.json"
  for stale in "${output_dir}/water_${selected}.json" "${output_dir}/water_${selected}_launch.log" "${output_dir}/water_${selected}_launch_audit.json" "${output_dir}/water_${selected}_bridge_shutdown.jsonl" "${output_dir}/water_${selected}_probe.log" "${output_dir}/water_${selected}_safety_preflight.json" "${output_dir}/water_${selected}_safety_preflight.log" "${output_dir}/water_${selected}_side_brush_sdf_surface.json" "${output_dir}/water_${selected}_side_brush_sdf_surface.log" "${output_dir}/water_${selected}_preoperational_readiness.json" "${output_dir}/water_${selected}_preoperational_readiness.log" "${output_dir}/water_${selected}_memory_watchdog.json" "${output_dir}/water_${selected}_memory_watchdog.log" "${output_dir}/water_${selected}_windows_memory_preflight.json" "${output_dir}/water_${selected}_windows_memory_preflight.log" "${preembedded_world}" "${preembedded_report}"; do
    [[ ! -e "${stale}" ]] || { echo "Refusing stale scenario evidence: ${stale}" >&2; return 2; }
  done
  export GZ_PARTITION="tzcup_formal_water_${selected}_${ROS_DOMAIN_ID}_$$"
  active_partition="${GZ_PARTITION}"
  python3 "${repo_root}/scripts/validate_formal_side_brush_sdf_surface.py" \
    --vehicle-xacro "${vehicle_xacro}" \
    --output "${output_dir}/water_${selected}_side_brush_sdf_surface.json" \
    >"${output_dir}/water_${selected}_side_brush_sdf_surface.log" 2>&1
  formal_runtime_memory_preflight \
    "${output_dir}/water_${selected}_windows_memory_preflight"
  python3 "${repo_root}/scripts/prepare_formal_preembedded_sensor_world.py" \
    --source-world "${installed_package_share}/worlds/formal_vehicle_validation.sdf" \
    --vehicle-urdf "${repo_root}/reports/engineering/formal_competition_vehicle.urdf" \
    --controller-config "${installed_package_share}/config/formal_vehicle_controllers.yaml" \
    --runtime-install-root "${runtime_install_root}" \
    --output-world "${preembedded_world}" \
    --report "${preembedded_report}" \
    --model-pose "${preembedded_model_pose}"
  "${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py \
    world:="${preembedded_world}" spawn_robot:=false \
    gui:=false bodywork_visible:=true high_bandwidth_sensor_runtime:=false \
    start_controllers:=true \
    enable_safety_manager:=true simulation_initial_estop_active:=true \
    start_service_drain_safety_manager:=false \
    start_simulation_safety_inputs:=true start_power_system_simulators:=true \
    water_evaluation_interfaces:=true squeegee_evaluation_interfaces:=true \
    >"${output_dir}/water_${selected}_launch.log" 2>&1 &
  launch_pid=$!
  formal_runtime_start_memory_watchdog "${launch_pid}" \
    "${output_dir}/water_${selected}_memory_watchdog"

  sleep "${launch_settle_s}"
  python3 "${repo_root}/scripts/check_formal_water_preoperational_readiness.py" \
    --service-drain-manager absent \
    --output "${output_dir}/water_${selected}_preoperational_readiness.json" \
    >"${output_dir}/water_${selected}_preoperational_readiness.log" 2>&1
  python3 "${repo_root}/scripts/collect_formal_water_safety_preflight.py" \
    --stable-duration-s 5.0 \
    --output "${output_dir}/water_${selected}_safety_preflight.json" \
    >"${output_dir}/water_${selected}_safety_preflight.log" 2>&1
  validator_args=(
    --scenario "${selected}"
    --output "${output_dir}/water_${selected}.json"
  )
  if [[ "${scenario}" == "all" ]]; then
    validator_args+=(
      --snapshot "${snapshot}"
      --session "${session}"
      --runtime-binding "${runtime_binding}"
      --preembedded-report "${preembedded_report}"
      --preembedded-world "${preembedded_world}"
      --preembedded-model-pose "${preembedded_model_pose}"
    )
  fi
  local validator_rc=0
  local bridge_shutdown_rc=0
  local launch_cleanup_rc=0
  local launch_audit_rc=0
  set +e
  python3 "${repo_root}/scripts/validate_formal_water_recovery_runtime.py" \
    "${validator_args[@]}" \
    >"${output_dir}/water_${selected}_probe.log" 2>&1
  validator_rc=$?
  formal_water_stop_active_bridges \
    "${output_dir}/water_${selected}_bridge_shutdown.jsonl"
  bridge_shutdown_rc=$?
  cleanup_launch
  launch_cleanup_rc=$?
  python3 "${repo_root}/scripts/audit_formal_water_launch_log.py" \
    --log "${output_dir}/water_${selected}_launch.log" \
    --expected-stable-marker-count "${expected_stable_marker_count}" \
    --required-clean-exit-process water_evaluation_bridge \
    --required-clean-exit-process formal_vehicle_product_native_bridge \
    --required-clean-exit-process cleaning_actuator_scalar_native_bridge \
    --required-clean-exit-process a300_drivetrain_native_bridge \
    --required-clean-exit-process-count formal_contact_evaluation_native_bridge=6 \
    --required-clean-exit-process formal_auxiliary_native_bridge \
    --required-clean-exit-process cleaning_actuator_vector_bridge \
    --output "${output_dir}/water_${selected}_launch_audit.json"
  launch_audit_rc=$?
  set -e
  if [[ "${bridge_shutdown_rc}" -ne 0 || "${launch_cleanup_rc}" -ne 0 || "${launch_audit_rc}" -ne 0 ]]; then
    printf 'scenario=%s validator_rc=%s bridge_shutdown_rc=%s launch_cleanup_rc=%s launch_audit_rc=%s\n' \
      "${selected}" "${validator_rc}" "${bridge_shutdown_rc}" "${launch_cleanup_rc}" "${launch_audit_rc}" \
      >>"${output_dir}/water_${selected}_probe.log"
    return 125
  fi
  if [[ "${validator_rc}" -ne 0 ]]; then
    return "${validator_rc}"
  fi
}

if [[ "${scenario}" == "normal" || "${scenario}" == "all" ]]; then
  run_scenario normal
fi
if [[ "${scenario}" == "full" || "${scenario}" == "all" ]]; then
  run_scenario full
fi
if [[ "${scenario}" == "diagnostic" ]]; then
  run_scenario diagnostic
fi
if [[ "${scenario}" == "all" ]]; then
  typed_diag="${FORMAL_WATER_TYPED_DIAG_JSON:?set FORMAL_WATER_TYPED_DIAG_JSON}"
  typed_trace="${FORMAL_WATER_TYPED_RAW_TRACE:?set FORMAL_WATER_TYPED_RAW_TRACE}"
  typed_runner="${FORMAL_WATER_TYPED_RUNNER:?set FORMAL_WATER_TYPED_RUNNER}"
  typed_collector="${FORMAL_WATER_TYPED_COLLECTOR:?set FORMAL_WATER_TYPED_COLLECTOR}"
  critical_manifest="${FORMAL_WATER_CRITICAL_SOURCE_MANIFEST:?set FORMAL_WATER_CRITICAL_SOURCE_MANIFEST}"
  typed_source_digest="${FORMAL_WATER_TYPED_SUBCLOSURE_SHA256:?set FORMAL_WATER_TYPED_SUBCLOSURE_SHA256}"
  combined="${output_dir}/water_recovery_acceptance.json"
  python3 "${repo_root}/scripts/finalize_formal_water_recovery_acceptance.py" \
    --normal "${output_dir}/water_normal.json" \
    --full "${output_dir}/water_full.json" \
    --normal-side-brush-surface "${output_dir}/water_normal_side_brush_sdf_surface.json" \
    --full-side-brush-surface "${output_dir}/water_full_side_brush_sdf_surface.json" \
    --typed-diag "${typed_diag}" \
    --typed-raw-trace "${typed_trace}" \
    --typed-runner "${typed_runner}" \
    --typed-collector "${typed_collector}" \
    --critical-source-manifest "${critical_manifest}" \
    --typed-cleaning-telemetry-source-sha256 "${typed_source_digest}" \
    --runtime-binding "${runtime_binding}" \
    --output "${combined}"
  pending="${formal_output}.pending.$$"
  [[ ! -e "${pending}" ]] || { echo "Refusing stale publish path: ${pending}" >&2; exit 2; }
  cp -- "${combined}" "${pending}"
  mv -- "${pending}" "${formal_output}"
  echo "Published formal water-recovery acceptance: ${formal_output}"
fi
