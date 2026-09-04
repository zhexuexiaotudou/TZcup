#!/usr/bin/env bash
# Build the one fresh, merged, non-symlink runtime used by final acceptance.
# ROS environment hooks are not nounset-safe.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# MoveIt loads this plugin from the system ament index at launch time; the
# ament_python package itself does not make colcon discover a missing plugin.
ros2 pkg prefix moveit_simple_controller_manager >/dev/null 2>&1 || {
  echo "Missing system ROS package moveit_simple_controller_manager; install its rosdep before the formal build" >&2
  exit 2
}
cold_gate_evidence=""
if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
  cold_gate_evidence="${FORMAL_WINDOWS_COLD_GATE_EVIDENCE:-}"
  [[ "${cold_gate_evidence}" = /* ]] || {
    echo "WSL final builds require the Windows cold-start wrapper evidence" >&2
    exit 86
  }
  python3 "${repo_root}/scripts/validate_formal_windows_cold_gate_evidence.py" \
    --evidence "${cold_gate_evidence}" \
    --max-age-s 300 || exit $?
fi
source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
runtime_ws="${FORMAL_FINAL_RUNTIME_WS:-${repo_root}/.work/final_frozen_runtime}"
parallel_workers="${FORMAL_COLCON_PARALLEL_WORKERS:-1}"
min_linux_mem_available_kib="${FORMAL_FINAL_BUILD_MIN_MEM_AVAILABLE_KIB:-4194304}"
max_linux_swap_used_kib="${FORMAL_FINAL_BUILD_MAX_SWAP_USED_KIB:-1048576}"
build_pid=""
[[ "${parallel_workers}" = "1" ]] || {
  echo "FORMAL_COLCON_PARALLEL_WORKERS must be exactly 1 for formal serial recovery" >&2
  exit 2
}
for value in "${min_linux_mem_available_kib}" "${max_linux_swap_used_kib}"; do
  [[ "${value}" =~ ^[0-9]+$ ]] || {
    echo "formal final build Linux memory thresholds must be unsigned KiB integers" >&2
    exit 2
  }
done
[[ "${runtime_ws}" = /* && "${runtime_ws}" != "/" ]] || {
  echo "FORMAL_FINAL_RUNTIME_WS must be an absolute path other than /" >&2
  exit 2
}
export CMAKE_BUILD_PARALLEL_LEVEL="${parallel_workers}"
export MAKEFLAGS="-j${parallel_workers}"
vendor_work_root="${runtime_ws}/vendor/gz_transport13_eintr_build"
vendor_build_report="${runtime_ws}/gz_transport13_eintr_vendor_build_report.json"
vendor_runtime_report="${runtime_ws}/gz_transport13_eintr_runtime_binding_report.json"
frozen_source_root="${runtime_ws}/src"
install_symlinks_report="${runtime_ws}/INSTALL_SYMLINKS.txt"
side_brush_surface_preflight="${runtime_ws}/side_brush_sdf_surface_preflight.json"
integrated_build_manifest="${runtime_ws}/integrated_build_manifest.json"
proot_compat_source="${repo_root}/scripts/proot_glibc_compat.c"
proot_compat_install="${runtime_ws}/install/lib/libtzcup_proot_glibc_compat.so"
windows_preflight_prefix="${runtime_ws}/formal_final_build_windows_memory_preflight"
windows_cold_gate_bound_json="${runtime_ws}/formal_windows_cold_start_evidence.json"
linux_preflight_json="${runtime_ws}/formal_final_build_linux_memory_preflight.json"
watchdog_prefix="${runtime_ws}/formal_final_build_memory_watchdog"
[[ ! -e "${runtime_ws}" && ! -L "${runtime_ws}" ]] || {
  echo "refusing non-fresh final runtime workspace: ${runtime_ws}" >&2
  exit 2
}
for path in \
  "${runtime_ws}/build" \
  "${runtime_ws}/install" \
  "${runtime_ws}/log" \
  "${frozen_source_root}" \
  "${install_symlinks_report}" \
  "${runtime_ws}/vendor" \
  "${vendor_build_report}" \
  "${vendor_runtime_report}" \
  "${side_brush_surface_preflight}" \
  "${integrated_build_manifest}" \
  "${windows_preflight_prefix}.json" \
  "${windows_preflight_prefix}.log" \
  "${windows_cold_gate_bound_json}" \
  "${linux_preflight_json}" \
  "${watchdog_prefix}.json" \
  "${watchdog_prefix}.log"; do
  [[ ! -e "${path}" && ! -L "${path}" ]] || {
    echo "refusing non-fresh final runtime path: ${path}" >&2
    exit 2
  }
done
mkdir -p "${runtime_ws}"
if [[ -n "${cold_gate_evidence}" ]]; then
  install -m 0444 -- "${cold_gate_evidence}" "${windows_cold_gate_bound_json}"
  cmp -s -- "${cold_gate_evidence}" "${windows_cold_gate_bound_json}" || {
    echo "bound Windows cold-start evidence differs from the validated source" >&2
    exit 125
  }
fi

formal_final_build_linux_memory_preflight() {
  local mem_available_kib=0 swap_total_kib=0 swap_free_kib=0 swap_used_kib=0
  local mem_ok=false swap_ok=false passed=false
  local status="FORMAL_FINAL_BUILD_LINUX_MEMORY_START_REFUSED"
  local pending="${linux_preflight_json}.pending.$$"
  while read -r key value _; do
    case "${key}" in
      MemAvailable:) mem_available_kib="${value}" ;;
      SwapTotal:) swap_total_kib="${value}" ;;
      SwapFree:) swap_free_kib="${value}" ;;
    esac
  done </proc/meminfo
  for value in "${mem_available_kib}" "${swap_total_kib}" "${swap_free_kib}"; do
    [[ "${value}" =~ ^[0-9]+$ ]] || return 125
  done
  (( swap_free_kib <= swap_total_kib )) || return 125
  swap_used_kib=$((swap_total_kib - swap_free_kib))
  (( mem_available_kib >= min_linux_mem_available_kib )) && mem_ok=true
  (( swap_used_kib <= max_linux_swap_used_kib )) && swap_ok=true
  if [[ "${mem_ok}" == true && "${swap_ok}" == true ]]; then
    passed=true
    status="FORMAL_FINAL_BUILD_LINUX_MEMORY_START_PASSED"
  fi
  printf '{\n  "report_id": "tzcup_formal_final_build_linux_memory_start_gate_v1",\n  "status": "%s",\n  "passed": %s,\n  "sample_epoch_ns": %s,\n  "thresholds_kib": {"min_mem_available": %d, "max_swap_used": %d},\n  "observed_kib": {"mem_available": %d, "swap_total": %d, "swap_free": %d, "swap_used": %d},\n  "checks": {"mem_available_at_least_configured_minimum": %s, "swap_used_at_most_configured_maximum": %s},\n  "signals": {"exact_pgid_only": true, "docker_signalled_or_stopped": false}\n}\n' \
    "${status}" "${passed}" "$(date +%s%N)" \
    "${min_linux_mem_available_kib}" "${max_linux_swap_used_kib}" \
    "${mem_available_kib}" "${swap_total_kib}" "${swap_free_kib}" "${swap_used_kib}" \
    "${mem_ok}" "${swap_ok}" >"${pending}"
  mv -- "${pending}" "${linux_preflight_json}"
  [[ "${passed}" == true ]] || return "${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
}

formal_final_build_cleanup() {
  local result=0
  if [[ -n "${build_pid}" ]]; then
    formal_runtime_kill_group "${build_pid}" || result=1
    build_pid=""
  fi
  return "${result}"
}

formal_runtime_install_traps formal_final_build_cleanup
formal_runtime_memory_preflight "${windows_preflight_prefix}"
formal_final_build_linux_memory_preflight || {
  result=$?
  if (( result == FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE )); then
    echo "formal final build start refused by Linux memory gate" >&2
  else
    FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=125
    echo "formal final build Linux memory gate failed closed" >&2
  fi
  exit "${result}"
}
if [[ -z "${cold_gate_evidence}" ]]; then
  python3 "${repo_root}/scripts/formal_native_linux_cold_start_evidence.py" \
    --runtime-ws "${runtime_ws}"
fi

# Qt's resource compiler probes payloads with statx().  PRoot can translate
# ordinary open/fstatat calls on the bound workspace while returning ENOENT
# for that statx call, so the compatibility layer must already be active
# during the build rather than being installed only after colcon succeeds.
[[ -f "${proot_compat_source}" && ! -L "${proot_compat_source}" ]] || {
  echo "formal PRoot/glibc compatibility source is missing or not regular" >&2
  exit 125
}
[[ ! -e "${proot_compat_install}" && ! -L "${proot_compat_install}" ]] || {
  echo "refusing stale formal PRoot/glibc compatibility library" >&2
  exit 125
}
mkdir -p -- "$(dirname -- "${proot_compat_install}")"
proot_compat_pending="${proot_compat_install}.pending.$$"
cc -shared -fPIC -O2 -Wall -Wextra \
  -o "${proot_compat_pending}" "${proot_compat_source}"
chmod 0555 -- "${proot_compat_pending}"
mv -- "${proot_compat_pending}" "${proot_compat_install}"
[[ -f "${proot_compat_install}" && ! -L "${proot_compat_install}" ]] || {
  echo "formal PRoot/glibc compatibility library was not installed regularly" >&2
  exit 125
}

build_started_epoch_ns="$(date +%s%N)"
LD_PRELOAD="${proot_compat_install}${LD_PRELOAD:+:${LD_PRELOAD}}" setsid bash -c '
set -euo pipefail
repo_root="$1"
runtime_ws="$2"
vendor_work_root="$3"
vendor_build_report="$4"
parallel_workers="$5"
frozen_source_root="$6"
python3 - "${repo_root}/starter_ws/src" "${frozen_source_root}" <<'PY'
import hashlib
import os
import shutil
import sys
from pathlib import Path

source, frozen = map(Path, sys.argv[1:])


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def regular_inventory(root: Path, label: str) -> dict[str, str]:
    if root.is_symlink() or not root.is_dir():
        raise SystemExit(f"{label} must be a real directory: {root}")
    rows: dict[str, str] = {}
    stack = [root]
    while stack:
        directory = stack.pop()
        for entry in os.scandir(directory):
            path = Path(entry.path)
            if entry.name in {"__pycache__", ".pytest_cache"}:
                raise SystemExit(f"{label} contains a generated test/cache directory: {path}")
            if entry.is_symlink():
                raise SystemExit(f"{label} contains a symbolic link: {path}")
            if entry.is_dir(follow_symlinks=False):
                stack.append(path)
            elif entry.is_file(follow_symlinks=False):
                if path.suffix == ".pyc":
                    raise SystemExit(f"{label} contains generated Python bytecode: {path}")
                if path.suffix.lower() in {".o", ".a", ".so", ".dll", ".exe"}:
                    raise SystemExit(
                        f"{label} contains a compiled build artifact: {path}"
                    )
                with path.open("rb") as stream:
                    magic = stream.read(4)
                if magic == b"\x7fELF" or magic[:2] == b"MZ":
                    raise SystemExit(
                        f"{label} contains an unlabelled executable artifact: {path}"
                    )
                relative = path.relative_to(root).as_posix()
                rows[relative] = sha256(path)
            else:
                raise SystemExit(f"{label} contains a non-regular entry: {path}")
    if not rows:
        raise SystemExit(f"{label} is empty: {root}")
    return rows


source_inventory = regular_inventory(source, "starter source tree")
shutil.copytree(source, frozen, copy_function=shutil.copy2)
frozen_inventory = regular_inventory(frozen, "frozen runtime source tree")
if frozen_inventory != source_inventory:
    raise SystemExit("frozen runtime source tree differs from starter_ws/src")
PY
bash "${repo_root}/scripts/build_gz_transport13_eintr_vendor.sh" \
  --work-root "${vendor_work_root}" \
  --install-prefix "${runtime_ws}/install" \
  --report "${vendor_build_report}" \
  --parallel-workers "${parallel_workers}"
source "${vendor_work_root}/activate_patched_runtime.sh"
system_multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
system_protobuf_libdir="/usr/lib/${system_multiarch}"
exec colcon --log-base "${runtime_ws}/log" build --merge-install \
  --executor parallel --parallel-workers "${parallel_workers}" \
  --build-base "${runtime_ws}/build" \
  --install-base "${runtime_ws}/install" \
  --base-paths "${frozen_source_root}" \
  --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_IGNORE_PREFIX_PATH=/opt/ros/jazzy/opt/ortools_vendor \
    -DProtobuf_DIR:PATH=Protobuf_DIR-NOTFOUND \
    -DProtobuf_INCLUDE_DIR:PATH=/usr/include \
    -DProtobuf_LIBRARY:FILEPATH="${system_protobuf_libdir}/libprotobuf.so" \
    -DProtobuf_LIBRARY_RELEASE:FILEPATH="${system_protobuf_libdir}/libprotobuf.so" \
    -DProtobuf_LITE_LIBRARY:FILEPATH="${system_protobuf_libdir}/libprotobuf-lite.so" \
    -DProtobuf_LITE_LIBRARY_RELEASE:FILEPATH="${system_protobuf_libdir}/libprotobuf-lite.so" \
    -DProtobuf_PROTOC_LIBRARY:FILEPATH="${system_protobuf_libdir}/libprotoc.so" \
    -DProtobuf_PROTOC_LIBRARY_RELEASE:FILEPATH="${system_protobuf_libdir}/libprotoc.so" \
    -DProtobuf_PROTOC_EXECUTABLE:FILEPATH=/usr/bin/protoc \
    -DProtobuf_USE_STATIC_LIBS:BOOL=OFF \
  --packages-up-to \
    sanitation_active_cleaning sanitation_campus_scenario sanitation_coverage \
    sanitation_formal_campus_integration sanitation_gazebo_auxiliary \
    sanitation_gazebo_control sanitation_localization sanitation_manipulation \
    sanitation_navigation sanitation_perception sanitation_perception_interfaces \
    sanitation_power_system sanitation_product_demo_integration sanitation_safety \
    sanitation_service_acceptance sanitation_vehicle_description
' formal-final-build "${repo_root}" "${runtime_ws}" "${vendor_work_root}" "${vendor_build_report}" "${parallel_workers}" "${frozen_source_root}" &
build_pid=$!

set +e
formal_runtime_start_memory_watchdog "${build_pid}" "${watchdog_prefix}"
watchdog_start_status=$?
set -e
(( watchdog_start_status == 0 )) || exit "${watchdog_start_status}"

set +e
wait "${build_pid}"
build_status=$?
set -e
formal_runtime_stop_memory_watchdog
formal_final_build_cleanup
if formal_runtime_memory_watchdog_tripped; then
  echo "formal final runtime build was stopped by the memory watchdog" >&2
  exit "${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
fi
if (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT != 0 )); then
  echo "formal final runtime build memory watchdog failed closed" >&2
  exit 125
fi
if (( build_status != 0 )); then
  echo "formal final runtime build failed: rc=${build_status}" >&2
  exit "${build_status}"
fi

# The preloaded compatibility layer is part of the immutable runtime, not a
# post-build host injection.  Recheck it before the install-tree inventory and
# integrated build snapshot so every later closure check sees identical bytes.
[[ -f "${proot_compat_install}" && ! -L "${proot_compat_install}" ]] || {
  echo "formal PRoot/glibc compatibility library was not installed regularly" >&2
  exit 125
}

# Every downstream CMake package is configured again by colcon.  Prove that
# none of those configure steps reintroduced the OR-Tools Protobuf config or
# headers after the audited vendor library itself was built.
python3 - "${runtime_ws}/build" <<'PY'
import json
import os
import sys
from pathlib import Path

build_root = Path(sys.argv[1])
forbidden = "/opt/ros/jazzy/opt/ortools_vendor"
expected = {
    "CMAKE_IGNORE_PREFIX_PATH": forbidden,
    "Protobuf_DIR": "Protobuf_DIR-NOTFOUND",
    "Protobuf_INCLUDE_DIR": "/usr/include",
    "Protobuf_PROTOC_EXECUTABLE": "/usr/bin/protoc",
}
caches = sorted(build_root.glob("*/CMakeCache.txt"))
if not caches:
    raise SystemExit("final Protobuf binding validation failed: no package CMake caches")
for cache_path in caches:
    cache: dict[str, str] = {}
    for line in cache_path.read_text(encoding="utf-8", errors="strict").splitlines():
        if not line or line.startswith(("//", "#")) or "=" not in line or ":" not in line:
            continue
        key_and_type, value = line.split("=", 1)
        key, _type = key_and_type.split(":", 1)
        cache[key] = value
    for key, wanted in expected.items():
        actual = cache.get(key)
        if actual != wanted:
            raise SystemExit(
                f"final Protobuf binding validation failed: {cache_path}: "
                f"{key}={actual!r}, expected {wanted!r}"
            )

compile_databases = sorted(build_root.glob("*/compile_commands.json"))
if not compile_databases:
    raise SystemExit("final Protobuf binding validation failed: no compile databases")
command_count = 0
for commands_path in compile_databases:
    rows = json.loads(commands_path.read_text(encoding="utf-8", errors="strict"))
    if not isinstance(rows, list):
        raise SystemExit(
            f"final Protobuf binding validation failed: invalid {commands_path}"
        )
    for row in rows:
        command_count += 1
        command = row.get("command") if isinstance(row, dict) else None
        arguments = row.get("arguments") if isinstance(row, dict) else None
        rendered = command if isinstance(command, str) else "\0".join(arguments or [])
        if forbidden in rendered:
            raise SystemExit(
                f"final Protobuf binding validation failed: {commands_path} "
                "references OR-Tools vendor headers"
            )
if command_count == 0:
    raise SystemExit("final Protobuf binding validation failed: empty compile databases")
PY

# Record the exact install-tree link inventory.  Final acceptance is a copy
# install, so any reported link is a hard failure rather than an allowed mode.
python3 - "${runtime_ws}/install" "${install_symlinks_report}" <<'PY'
import os
import sys
from pathlib import Path

install_root, output = map(Path, sys.argv[1:])
if install_root.is_symlink() or not install_root.is_dir():
    raise SystemExit(f"merged install must be a real directory: {install_root}")
links: list[str] = []
stack = [install_root]
while stack:
    directory = stack.pop()
    for entry in os.scandir(directory):
        path = Path(entry.path)
        if entry.is_symlink():
            links.append(path.relative_to(install_root).as_posix())
        elif entry.is_dir(follow_symlinks=False):
            stack.append(path)
        elif not entry.is_file(follow_symlinks=False):
            raise SystemExit(f"install contains a non-regular entry: {path}")
links.sort()
pending = output.with_name(f"{output.name}.pending.{os.getpid()}")
if pending.exists() or pending.is_symlink():
    raise SystemExit(f"refusing stale install link report pending path: {pending}")
pending.write_text("".join(f"{relative}\n" for relative in links), encoding="utf-8")
pending.replace(output)
if links:
    raise SystemExit(
        "formal final install contains symbolic links; see "
        f"{output}: {len(links)}"
    )
PY

# Colcon-generated setup files read optional variables such as COLCON_TRACE.
# Source them with nounset disabled, matching the guarded ROS setup above,
# then restore strict mode for all validation that follows.
set +u
source "${vendor_work_root}/activate_patched_runtime.sh"
source "${runtime_ws}/install/setup.bash"
set -u

# Prove that a clean sourced merged overlay resolves the Gazebo plugins and
# bridge executable to the patched library inside this same runtime prefix.
python3 "${repo_root}/scripts/validate_gz_transport13_eintr_vendor.py" \
  --source-dir "${vendor_work_root}/source" \
  --install-prefix "${runtime_ws}/install" \
  --protobuf-binding "${vendor_work_root}/protobuf_binding.json" \
  --memory-preflight "${vendor_work_root}/windows_memory_preflight.json" \
  --parallel-workers "${parallel_workers}" \
  --runtime-plugin "${runtime_ws}/install/lib/libWaterRecoverySystem.so" \
  --runtime-plugin "${runtime_ws}/install/lib/libCleaningActuatorMotorSystem.so" \
  --runtime-plugin "${runtime_ws}/install/lib/sanitation_gazebo_control/cleaning_actuator_vector_bridge" \
  --require-active-runtime \
  --output "${vendor_runtime_report}"

installed_vehicle_xacro="${runtime_ws}/install/share/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
python3 "${repo_root}/scripts/validate_formal_side_brush_sdf_surface.py" \
  --vehicle-xacro "${installed_vehicle_xacro}" \
  --output "${side_brush_surface_preflight}"
[[ -f "${side_brush_surface_preflight}" && ! -L "${side_brush_surface_preflight}" ]] || {
  echo "formal final side-brush preflight was not materialized as a regular file" >&2
  exit 125
}
python3 "${repo_root}/scripts/aggregate_integrated_functional_acceptance.py" \
  record-build \
  --repo-root "${repo_root}" \
  --runtime-ws "${runtime_ws}" \
  --build-started-epoch-ns "${build_started_epoch_ns}" \
  --output "${integrated_build_manifest}"
[[ -f "${integrated_build_manifest}" && ! -L "${integrated_build_manifest}" ]] || {
  echo "formal final integrated build manifest was not materialized as a regular file" >&2
  exit 125
}
