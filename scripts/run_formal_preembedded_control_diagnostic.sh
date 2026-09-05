#!/usr/bin/env bash
# Diagnostic-only bisection for the preembedded gz_ros2_control first-write
# crash. This runner never writes formal acceptance reports. Its raw mode
# deliberately keeps sdformat's converted sensor layout so the reconstructed
# fixed sensor attachment joints are absent. Full mode retains the production
# preparation path and dynamic mode uses normal UserCommands spawning for an
# A/B/C comparison.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)}"
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"

runtime_setup="${FORMAL_DIAGNOSTIC_RUNTIME_SETUP:-}"
output_dir="${FORMAL_DIAGNOSTIC_OUTPUT_DIR:-}"
mode="${FORMAL_DIAGNOSTIC_ATTACHMENT_MODE:-raw}"
duration_s="${FORMAL_DIAGNOSTIC_DURATION_S:-30}"
domain_id="${ROS_DOMAIN_ID:-94}"

[[ -n "${output_dir}" && "${output_dir}" = /* ]] || {
  echo "FORMAL_DIAGNOSTIC_OUTPUT_DIR must be a new absolute directory" >&2
  exit 2
}
[[ ! -e "${output_dir}" ]] || {
  echo "refusing existing diagnostic output directory: ${output_dir}" >&2
  exit 2
}
[[ "${mode}" == "raw" || "${mode}" == "full" || "${mode}" == "dynamic" ]] || {
  echo "FORMAL_DIAGNOSTIC_ATTACHMENT_MODE must be raw, full, or dynamic" >&2
  exit 2
}
[[ "${duration_s}" =~ ^[1-9][0-9]*$ ]] && (( duration_s <= 120 )) || {
  echo "FORMAL_DIAGNOSTIC_DURATION_S must be an integer in 1..120" >&2
  exit 2
}
[[ -n "${runtime_setup}" ]] || {
  echo "FORMAL_DIAGNOSTIC_RUNTIME_SETUP must explicitly name a frozen runtime setup.bash" >&2
  exit 2
}
[[ -f "${runtime_setup}" ]] || {
  echo "missing frozen diagnostic runtime: ${runtime_setup}" >&2
  exit 2
}
runtime_setup="$(readlink -f -- "${runtime_setup}")"
[[ "$(basename -- "${runtime_setup}")" == "setup.bash" ]] || {
  echo "FORMAL_DIAGNOSTIC_RUNTIME_SETUP must resolve to an install setup.bash: ${runtime_setup}" >&2
  exit 2
}

mkdir -p "${output_dir}"
install_root="$(cd "$(dirname -- "${runtime_setup}")" && pwd -P)"
[[ -d "${install_root}/share" ]] || {
  echo "frozen diagnostic runtime has no canonical install/share directory: ${install_root}" >&2
  exit 2
}
set +u
source "${runtime_setup}"
set -u
package_share_raw="$(ros2 pkg prefix --share sanitation_vehicle_description)"
package_share="$(readlink -f -- "${package_share_raw}")"
expected_package_share="$(readlink -f -- "${install_root}/share/sanitation_vehicle_description")"
case "${expected_package_share}" in
  "${install_root}"/share/sanitation_vehicle_description) ;;
  *)
    echo "canonical install/share escapes requested frozen runtime: ${expected_package_share}" >&2
    exit 2
    ;;
esac
[[ "${package_share}" == "${expected_package_share}" ]] || {
  echo "vehicle package resolves outside the requested frozen runtime: ${package_share}" >&2
  exit 2
}

binding_manifest="${output_dir}/runtime_source_install_bindings.tsv"
declare -a binding_rows=()
bind_source_install() {
  local name="$1"
  local source_path="$2"
  local installed_relative="$3"
  local installed_path="${package_share}/${installed_relative}"
  local source_sha installed_sha
  [[ -f "${source_path}" && -f "${installed_path}" ]] || {
    echo "frozen diagnostic runtime is missing source/install binding for ${name}" >&2
    exit 2
  }
  installed_path="$(readlink -f -- "${installed_path}")"
  case "${installed_path}" in
    "${expected_package_share}"/*) ;;
    *)
      echo "installed binding escapes requested frozen runtime for ${name}: ${installed_path}" >&2
      exit 2
      ;;
  esac
  source_sha="$(sha256sum -- "${source_path}" | awk '{print $1}')"
  installed_sha="$(sha256sum -- "${installed_path}" | awk '{print $1}')"
  [[ "${source_sha}" == "${installed_sha}" ]] || {
    echo "frozen diagnostic runtime is stale for ${name}" >&2
    exit 2
  }
  binding_rows+=("${name}"$'\t'"${source_path}"$'\t'"${installed_path}"$'\t'"${source_sha}")
}

bind_source_install root_xacro \
  "${repo_root}/starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro" \
  "urdf/formal_competition_vehicle.urdf.xacro"
bind_source_install manipulator_stack \
  "${repo_root}/starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/manipulator_stack.xacro" \
  "urdf/high_fidelity/manipulator_stack.xacro"
bind_source_install launch \
  "${repo_root}/starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py" \
  "launch/formal_vehicle_sim.launch.py"
bind_source_install controller_config \
  "${repo_root}/starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml" \
  "config/formal_vehicle_controllers.yaml"
bind_source_install source_world \
  "${repo_root}/starter_ws/src/sanitation_vehicle_description/worlds/formal_vehicle_validation.sdf" \
  "worlds/formal_vehicle_validation.sdf"

prepare_script="${repo_root}/scripts/prepare_formal_preembedded_sensor_world.py"
[[ -f "${prepare_script}" ]] || {
  echo "missing diagnostic preparation script: ${prepare_script}" >&2
  exit 2
}
prepare_script="$(readlink -f -- "${prepare_script}")"
prepare_script_sha="$(sha256sum -- "${prepare_script}" | awk '{print $1}')"
{
  printf 'diagnostic=true\nformal_eligible=false\n'
  printf 'runtime_setup=%s\ninstall_root=%s\npackage_share=%s\n' \
    "${runtime_setup}" "${install_root}" "${package_share}"
  printf 'name\tsource_path\tinstalled_path\tsha256\n'
  printf '%s\n' "${binding_rows[@]}"
  printf 'prepare_script\t%s\t%s\n' "${prepare_script}" "${prepare_script_sha}"
} >"${binding_manifest}"

export ROS_DOMAIN_ID="${domain_id}"
formal_runtime_configure "${ROS_DOMAIN_ID}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_preembedded_control_diag_${mode}_${ROS_DOMAIN_ID}_$$}"

world="${output_dir}/${mode}.sdf"
report="${output_dir}/${mode}.json"
log="${output_dir}/launch.log"
memory_preflight="${output_dir}/memory_preflight"
memory_watchdog="${output_dir}/memory_watchdog"
spawn_robot=false
if [[ "${mode}" == "dynamic" ]]; then
  world="${package_share}/worlds/formal_vehicle_validation.sdf"
  spawn_robot=true
else
  prepare_args=()
  if [[ "${mode}" == "raw" ]]; then
    prepare_args+=(--diagnostic-skip-attachment-restoration)
  fi

  python3 "${repo_root}/scripts/prepare_formal_preembedded_sensor_world.py" \
    --source-world "${package_share}/worlds/formal_vehicle_validation.sdf" \
    --vehicle-urdf "${repo_root}/reports/engineering/formal_competition_vehicle.urdf" \
    --controller-config "${package_share}/config/formal_vehicle_controllers.yaml" \
    --runtime-install-root "${install_root}" \
    --output-world "${world}" --report "${report}" \
    "${prepare_args[@]}"
  python3 - "${report}" "${mode}" "${binding_manifest}" <<'PY'
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1])
report = json.loads(report_path.read_text(encoding="utf-8"))
report.update(
    {
        "status": "PREEMBEDDED_CONTROL_DIAGNOSTIC_WORLD_READY",
        "formal_eligible": False,
        "claim_boundary": (
            "Diagnostic-only preembedded-control bisection. This evidence is not "
            "formal acceptance and cannot satisfy a formal runtime gate."
        ),
        "diagnostic_mode": sys.argv[2],
        "runtime_source_install_bindings": sys.argv[3],
    }
)
report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
fi

launch_pid=""
cleanup() {
  formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"
}
formal_runtime_install_traps cleanup
formal_runtime_memory_preflight "${memory_preflight}"

"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch \
  sanitation_vehicle_description formal_vehicle_sim.launch.py \
  world:="${world}" spawn_robot:="${spawn_robot}" gui:=false headless_rendering:=true \
  bodywork_visible:=true start_controllers:=true enable_safety_manager:=true \
  simulation_initial_estop_active:=true high_bandwidth_sensor_runtime:=false \
  >"${log}" 2>&1 &
launch_pid=$!
formal_runtime_start_memory_watchdog "${launch_pid}" "${memory_watchdog}"

for ((second=0; second<duration_s; second++)); do
  kill -0 "${launch_pid}" 2>/dev/null || break
  sleep 1
done

activation_markers="$(grep -c 'Activating controllers:' "${log}" || true)"
hardware_active_markers="$(grep -c "Successful 'activate' of hardware 'formal_vehicle_system'" "${log}" || true)"
controller_load_markers="$(grep -c 'Loaded joint_state_broadcaster' "${log}" || true)"
switch_markers="$(grep -c 'Successfully switched controllers' "${log}" || true)"
write_markers="$(grep -c 'GazeboSimSystem::write' "${log}" || true)"
segfault_markers="$(grep -Eic 'Segmentation fault|exit code 139' "${log}" || true)"
alive=false
kill -0 "${launch_pid}" 2>/dev/null && alive=true
formal_runtime_stop_memory_watchdog
watchdog_result="${FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT}"

if (( watchdog_result != 0 )) && [[ -f "${report}" ]]; then
  python3 - "${report}" "${watchdog_result}" <<'PY'
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1])
report = json.loads(report_path.read_text(encoding="utf-8"))
report.update(
    {
        "status": "PREEMBEDDED_CONTROL_DIAGNOSTIC_MEMORY_WATCHDOG_FAILED",
        "passed": False,
        "formal_eligible": False,
        "memory_watchdog_result": int(sys.argv[2]),
    }
)
report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
fi

printf 'diagnostic=true\n'
printf 'formal_eligible=false\n'
printf 'diagnostic_mode=%s\n' "${mode}"
printf 'process_alive_after_%ss=%s\n' "${duration_s}" "${alive}"
printf 'activation_markers=%s\n' "${activation_markers}"
printf 'hardware_active_markers=%s\n' "${hardware_active_markers}"
printf 'controller_load_markers=%s\n' "${controller_load_markers}"
printf 'switch_success_markers=%s\n' "${switch_markers}"
printf 'write_stack_markers=%s\n' "${write_markers}"
printf 'segfault_markers=%s\n' "${segfault_markers}"
printf 'memory_watchdog_result=%s\n' "${watchdog_result}"

if (( watchdog_result != 0 )); then
  echo "PREEMBEDDED_CONTROL_DIAGNOSTIC_MEMORY_WATCHDOG_FAILED" >&2
  exit "${watchdog_result}"
fi

if (( hardware_active_markers > 0 && controller_load_markers > 0 && write_markers > 0 && segfault_markers > 0 )); then
  echo "PREEMBEDDED_CONTROL_DIAGNOSTIC_CONFIRMED_139"
  exit 139
fi
if [[ "${alive}" == true ]] && (( switch_markers > 0 )); then
  echo "PREEMBEDDED_CONTROL_DIAGNOSTIC_SURVIVED"
  exit 0
fi
echo "PREEMBEDDED_CONTROL_DIAGNOSTIC_INCONCLUSIVE"
exit 2
