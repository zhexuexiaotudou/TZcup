#!/usr/bin/env bash
# Build the audited gz-transport13 EINTR fix into a user-controlled prefix.

# ROS environment hooks are not nounset-safe.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
manifest="${repo_root}/patches/upstream/gz_transport13/manifest.json"
patch_file="${repo_root}/patches/upstream/gz_transport13/0001-publish-retry-current-frame-on-eintr.patch"
work_root="${FORMAL_GZ_TRANSPORT13_WORK_ROOT:-${HOME}/.cache/tzcup/gz_transport13_eintr_13_5_0}"
install_prefix="${FORMAL_GZ_TRANSPORT13_INSTALL_PREFIX:-${HOME}/.local/tzcup/gz_transport13_eintr_13_5_0}"
report_path=""
parallel_workers="${FORMAL_GZ_TRANSPORT13_PARALLEL_WORKERS:-2}"

usage() {
  echo "usage: $0 [--work-root PATH] [--install-prefix PATH] [--report PATH] [--parallel-workers 1|2]" >&2
}

while (($#)); do
  case "$1" in
    --work-root)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      work_root="$2"
      shift 2
      ;;
    --install-prefix)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      install_prefix="$2"
      shift 2
      ;;
    --report)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      report_path="$2"
      shift 2
      ;;
    --parallel-workers)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      parallel_workers="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

[[ "${parallel_workers}" =~ ^[12]$ ]] || {
  echo "parallel workers must be 1 or 2" >&2
  exit 2
}
[[ -n "${work_root}" && "${work_root}" = /* ]] || {
  echo "work root must be an absolute non-empty path" >&2
  exit 2
}
[[ -n "${install_prefix}" && "${install_prefix}" = /* ]] || {
  echo "install prefix must be an absolute non-empty path" >&2
  exit 2
}
[[ "${work_root}" != "/" && "${install_prefix}" != "/" ]] || {
  echo "refusing root as a build or install path" >&2
  exit 2
}

source_dir="${work_root}/source"
build_dir="${work_root}/build"
memory_report="${work_root}/windows_memory_preflight.json"
report_path="${report_path:-${work_root}/build_report.json}"
activation_path="${work_root}/activate_patched_runtime.sh"
protobuf_binding_report="${work_root}/protobuf_binding.json"

for path in \
    "${source_dir}" \
    "${build_dir}" \
    "${memory_report}" \
    "${report_path}" \
    "${activation_path}" \
    "${protobuf_binding_report}"; do
  [[ ! -e "${path}" && ! -L "${path}" ]] || {
    echo "refusing stale vendor build path: ${path}" >&2
    exit 2
  }
done
for library in \
    "${install_prefix}/lib/libgz-transport13.so" \
    "${install_prefix}/lib/libgz-transport13.so.13" \
    "${install_prefix}/lib/libgz-transport13.so.13.5.0"; do
  [[ ! -e "${library}" && ! -L "${library}" ]] || {
    echo "refusing to overwrite an existing gz-transport runtime: ${library}" >&2
    exit 2
  }
done

mkdir -p "${work_root}"
python3 "${repo_root}/scripts/formal_windows_memory_probe.py" \
  --check-start --output "${memory_report}"
linux_mem_available_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
[[ "${linux_mem_available_kib}" =~ ^[0-9]+$ ]] || {
  echo "unable to read Linux MemAvailable" >&2
  exit 125
}
min_linux_mem_available_kib="${FORMAL_GZ_TRANSPORT13_MIN_MEM_AVAILABLE_KIB:-4194304}"
[[ "${min_linux_mem_available_kib}" =~ ^[0-9]+$ ]] || {
  echo "FORMAL_GZ_TRANSPORT13_MIN_MEM_AVAILABLE_KIB must be an integer" >&2
  exit 2
}
((linux_mem_available_kib >= min_linux_mem_available_kib)) || {
  echo "refusing vendor build: Linux MemAvailable is below the configured floor" >&2
  exit 86
}

# ROS Jazzy's environment exposes an OR-Tools vendor prefix containing
# Protobuf 4.25.3.  gz-msgs10 on this host was generated with Ubuntu's
# Protobuf 3.21.12, so allowing FindGzProtobuf's config-mode lookup to select
# the OR-Tools copy causes an immediate generated-header ABI/version failure.
# Bind every Protobuf input explicitly and exclude that one foreign prefix;
# this is independent of include-directory ordering and fails closed if the
# pinned system toolchain is unavailable or has drifted.
system_protobuf_version="3.21.12"
system_protobuf_header_version="3021012"
system_protobuf_include="/usr/include"
system_protobuf_protoc="/usr/bin/protoc"
system_multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
system_protobuf_library="/usr/lib/${system_multiarch}/libprotobuf.so"
system_protobuf_lite_library="/usr/lib/${system_multiarch}/libprotobuf-lite.so"
system_protobuf_protoc_library="/usr/lib/${system_multiarch}/libprotoc.so"
ortools_vendor_prefix="/opt/ros/jazzy/opt/ortools_vendor"

[[ "${system_multiarch}" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "invalid system multiarch identity: ${system_multiarch}" >&2
  exit 3
}
for path in \
    "${system_protobuf_include}/google/protobuf/port_def.inc" \
    "${system_protobuf_library}" \
    "${system_protobuf_lite_library}" \
    "${system_protobuf_protoc_library}"; do
  [[ -f "${path}" ]] || {
    echo "required system Protobuf 3.21.12 input is missing: ${path}" >&2
    exit 3
  }
done
[[ -x "${system_protobuf_protoc}" ]] || {
  echo "required system protoc is missing or not executable: ${system_protobuf_protoc}" >&2
  exit 3
}
[[ "$("${system_protobuf_protoc}" --version)" == "libprotoc ${system_protobuf_version}" ]] || {
  echo "system protoc version drifted from ${system_protobuf_version}" >&2
  exit 3
}
[[ "$(PKG_CONFIG_LIBDIR="/usr/lib/${system_multiarch}/pkgconfig:/usr/share/pkgconfig" \
    pkg-config --modversion protobuf)" == "${system_protobuf_version}" ]] || {
  echo "system libprotobuf version drifted from ${system_protobuf_version}" >&2
  exit 3
}
[[ "$(awk '/^#define PROTOBUF_VERSION / {print $3; exit}' \
    "${system_protobuf_include}/google/protobuf/port_def.inc")" == \
    "${system_protobuf_header_version}" ]] || {
  echo "system Protobuf header version drifted from ${system_protobuf_header_version}" >&2
  exit 3
}

upstream_repository="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["upstream_repository"])' "${manifest}")"
upstream_commit="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["upstream_commit"])' "${manifest}")"
upstream_tree="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["upstream_tree"])' "${manifest}")"
upstream_node_sha="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["upstream_node_shared_sha256"])' "${manifest}")"
patched_node_sha="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["patched_node_shared_sha256"])' "${manifest}")"
expected_patch_sha="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["patch_sha256"])' "${manifest}")"

actual_patch_sha="$(sha256sum "${patch_file}" | awk '{print $1}')"
[[ "${actual_patch_sha}" == "${expected_patch_sha}" ]] || {
  echo "vendor patch SHA-256 mismatch" >&2
  exit 3
}

git clone --filter=blob:none --no-checkout "${upstream_repository}" "${source_dir}"
git -C "${source_dir}" fetch --depth 1 origin "${upstream_commit}"
git -C "${source_dir}" checkout --detach "${upstream_commit}"
[[ "$(git -C "${source_dir}" rev-parse HEAD)" == "${upstream_commit}" ]] || {
  echo "upstream commit mismatch" >&2
  exit 3
}
[[ "$(git -C "${source_dir}" rev-parse 'HEAD^{tree}')" == "${upstream_tree}" ]] || {
  echo "upstream tree mismatch" >&2
  exit 3
}
[[ "$(sha256sum "${source_dir}/src/NodeShared.cc" | awk '{print $1}')" == "${upstream_node_sha}" ]] || {
  echo "upstream NodeShared.cc SHA-256 mismatch" >&2
  exit 3
}
git -C "${source_dir}" apply --check "${patch_file}"
git -C "${source_dir}" apply "${patch_file}"
git -C "${source_dir}" diff --check
[[ "$(sha256sum "${source_dir}/src/NodeShared.cc" | awk '{print $1}')" == "${patched_node_sha}" ]] || {
  echo "patched NodeShared.cc SHA-256 mismatch" >&2
  exit 3
}

export CMAKE_BUILD_PARALLEL_LEVEL="${parallel_workers}"
export MAKEFLAGS="-j${parallel_workers}"
cmake -S "${source_dir}" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${install_prefix}" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_IGNORE_PREFIX_PATH="${ortools_vendor_prefix}" \
  -DProtobuf_DIR:PATH=Protobuf_DIR-NOTFOUND \
  -DProtobuf_INCLUDE_DIR:PATH="${system_protobuf_include}" \
  -DProtobuf_LIBRARY:FILEPATH="${system_protobuf_library}" \
  -DProtobuf_LIBRARY_RELEASE:FILEPATH="${system_protobuf_library}" \
  -DProtobuf_LITE_LIBRARY:FILEPATH="${system_protobuf_lite_library}" \
  -DProtobuf_LITE_LIBRARY_RELEASE:FILEPATH="${system_protobuf_lite_library}" \
  -DProtobuf_PROTOC_LIBRARY:FILEPATH="${system_protobuf_protoc_library}" \
  -DProtobuf_PROTOC_LIBRARY_RELEASE:FILEPATH="${system_protobuf_protoc_library}" \
  -DProtobuf_PROTOC_EXECUTABLE:FILEPATH="${system_protobuf_protoc}" \
  -DProtobuf_USE_STATIC_LIBS:BOOL=OFF \
  -DBUILD_TESTING=OFF

python3 - \
    "${build_dir}/CMakeCache.txt" \
    "${build_dir}/compile_commands.json" \
    "${protobuf_binding_report}" \
    "${system_protobuf_include}" \
    "${system_protobuf_library}" \
    "${system_protobuf_lite_library}" \
    "${system_protobuf_protoc_library}" \
    "${system_protobuf_protoc}" \
    "${system_protobuf_version}" \
    "${system_protobuf_header_version}" \
    "${ortools_vendor_prefix}" <<'PY'
import json
import os
import sys
from pathlib import Path

(
    cache_path_raw,
    commands_path_raw,
    report_path_raw,
    include_raw,
    library_raw,
    lite_library_raw,
    protoc_library_raw,
    protoc_raw,
    version,
    header_version,
    forbidden_prefix_raw,
) = sys.argv[1:]
cache_path = Path(cache_path_raw)
commands_path = Path(commands_path_raw)
report_path = Path(report_path_raw)
forbidden_prefix = os.path.realpath(forbidden_prefix_raw)


def fail(message: str) -> None:
    raise SystemExit(f"Protobuf binding validation failed: {message}")


def normalized(path: str) -> str:
    return os.path.realpath(path)


if not cache_path.is_file():
    fail(f"missing CMake cache: {cache_path}")
cache: dict[str, str] = {}
for line in cache_path.read_text(encoding="utf-8", errors="strict").splitlines():
    if not line or line.startswith(("//", "#")) or "=" not in line or ":" not in line:
        continue
    key_and_type, value = line.split("=", 1)
    key, _type = key_and_type.split(":", 1)
    cache[key] = value

expected_paths = {
    "Protobuf_INCLUDE_DIR": include_raw,
    "Protobuf_LIBRARY_RELEASE": library_raw,
    "Protobuf_LITE_LIBRARY_RELEASE": lite_library_raw,
    "Protobuf_PROTOC_LIBRARY_RELEASE": protoc_library_raw,
    "Protobuf_PROTOC_EXECUTABLE": protoc_raw,
}
for key, expected in expected_paths.items():
    actual = cache.get(key)
    if actual is None or normalized(actual) != normalized(expected):
        fail(f"{key} resolved to {actual!r}, expected {expected!r}")
if cache.get("Protobuf_DIR") not in {None, "", "Protobuf_DIR-NOTFOUND"}:
    fail(f"config-mode Protobuf unexpectedly resolved to {cache['Protobuf_DIR']!r}")
if normalized(cache.get("CMAKE_IGNORE_PREFIX_PATH", "")) != forbidden_prefix:
    fail("CMAKE_IGNORE_PREFIX_PATH does not exclude the OR-Tools vendor prefix")

if not commands_path.is_file():
    fail(f"missing compile database: {commands_path}")
commands = json.loads(commands_path.read_text(encoding="utf-8", errors="strict"))
if not isinstance(commands, list) or not commands:
    fail("compile database is empty")
for index, row in enumerate(commands):
    if not isinstance(row, dict):
        fail(f"compile command {index} is not an object")
    command = row.get("command")
    arguments = row.get("arguments")
    rendered = command if isinstance(command, str) else "\0".join(arguments or [])
    if not isinstance(rendered, str):
        fail(f"compile command {index} has no command or arguments")
    if forbidden_prefix in rendered or forbidden_prefix_raw in rendered:
        fail(f"compile command {index} still references OR-Tools vendor Protobuf")

payload = {
    "schema_version": 1,
    "status": "SYSTEM_PROTOBUF_3_21_12_BINDING_PASSED",
    "passed": True,
    "protobuf_version": version,
    "protobuf_header_version": int(header_version),
    "config_mode_protobuf_disabled": True,
    "forbidden_prefix": forbidden_prefix_raw,
    "compile_command_count": len(commands),
    "resolved": {key: normalized(value) for key, value in expected_paths.items()},
}
if report_path.exists() or report_path.is_symlink():
    fail(f"refusing stale binding report: {report_path}")
pending = report_path.with_name(f"{report_path.name}.pending.{os.getpid()}")
pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
pending.replace(report_path)
PY

cmake --build "${build_dir}" --parallel "${parallel_workers}"
cmake --install "${build_dir}"

# A final merged runtime is required to contain no symlink-installed payload.
# Preserve the usual loader and linker names as regular copies of the audited
# versioned ELF instead of leaving CMake's shared-library symlink chain.
for component in gz-transport13 gz-transport13-log gz-transport13-parameters; do
  versioned="${install_prefix}/lib/lib${component}.so.13.5.0"
  [[ -f "${versioned}" && ! -L "${versioned}" ]] || {
    echo "missing installed real library: ${versioned}" >&2
    exit 4
  }
  for name in "lib${component}.so.13" "lib${component}.so"; do
    target="${install_prefix}/lib/${name}"
    if [[ -L "${target}" ]]; then
      cp --remove-destination --preserve=mode,timestamps "${versioned}" "${target}"
    fi
    [[ -f "${target}" && ! -L "${target}" ]] || {
      echo "installed runtime alias is not a regular file: ${target}" >&2
      exit 4
    }
  done
done

python3 "${repo_root}/scripts/validate_gz_transport13_eintr_vendor.py" \
  --source-dir "${source_dir}" \
  --install-prefix "${install_prefix}" \
  --protobuf-binding "${protobuf_binding_report}" \
  --memory-preflight "${memory_report}" \
  --parallel-workers "${parallel_workers}" \
  --output "${report_path}"

python3 - "${activation_path}" "${install_prefix}" <<'PY'
import shlex
import sys
from pathlib import Path

path = Path(sys.argv[1])
prefix = Path(sys.argv[2])
library = shlex.quote(str(prefix / "lib"))
cmake = shlex.quote(str(prefix))
path.write_text(
    "# generated by build_gz_transport13_eintr_vendor.sh\n"
    f"export LD_LIBRARY_PATH={library}:\"${{LD_LIBRARY_PATH:-}}\"\n"
    f"export CMAKE_PREFIX_PATH={cmake}:\"${{CMAKE_PREFIX_PATH:-}}\"\n",
    encoding="utf-8",
)
PY
chmod 0644 "${activation_path}"

echo "patched gz-transport13 installed: ${install_prefix}"
echo "build report: ${report_path}"
echo "activation: source ${activation_path}"
