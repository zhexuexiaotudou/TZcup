#!/usr/bin/env bash
# Transport-only late-discovery proof for the frozen gz-transport13 runtime.
# This script never launches Gazebo, ROS 2, or Docker.  It uses one fresh
# GZ_PARTITION for two ordered endpoint pairs and fails closed on any runtime,
# discovery, topic-info, message-count, ldd, or /proc/maps mismatch.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_prefix=""
output_dir=""
while (($#)); do
  case "$1" in
    --runtime-prefix)
      runtime_prefix="${2:-}"
      shift 2
      ;;
    --output-dir)
      output_dir="${2:-}"
      shift 2
      ;;
    *)
      echo "usage: $0 --runtime-prefix /absolute/install --output-dir /absolute/new-output" >&2
      exit 2
      ;;
  esac
done

[[ "${runtime_prefix}" = /* && "${runtime_prefix}" != "/" ]] || {
  echo "--runtime-prefix must be an absolute non-root path" >&2
  exit 2
}
[[ "${output_dir}" = /* && "${output_dir}" != "/" ]] || {
  echo "--output-dir must be an absolute non-root path" >&2
  exit 2
}
[[ ! -e "${output_dir}" && ! -L "${output_dir}" ]] || {
  echo "refusing stale --output-dir: ${output_dir}" >&2
  exit 2
}
[[ -d "${runtime_prefix}" && ! -L "${runtime_prefix}" ]] || {
  echo "frozen runtime prefix must be a real directory: ${runtime_prefix}" >&2
  exit 2
}
runtime_prefix="$(realpath -- "${runtime_prefix}")"
runtime_lib="${runtime_prefix}/lib"
for library in \
  "${runtime_lib}/libgz-transport13.so" \
  "${runtime_lib}/libgz-transport13.so.13" \
  "${runtime_lib}/libgz-transport13.so.13.5.0"; do
  [[ -f "${library}" && ! -L "${library}" ]] || {
    echo "frozen runtime transport alias must be a regular file: ${library}" >&2
    exit 2
  }
done
transport_sha="$(sha256sum "${runtime_lib}/libgz-transport13.so.13" | awk '{print $1}')"
for library in \
  "${runtime_lib}/libgz-transport13.so" \
  "${runtime_lib}/libgz-transport13.so.13.5.0"; do
  [[ "$(sha256sum "${library}" | awk '{print $1}')" = "${transport_sha}" ]] || {
    echo "frozen transport aliases are not byte-identical" >&2
    exit 2
  }
done

for command in cmake sha256sum awk sed grep python3 readlink; do
  command -v "${command}" >/dev/null 2>&1 || {
    echo "required command is unavailable: ${command}" >&2
    exit 2
  }
done

mkdir -- "${output_dir}"
build_dir="${output_dir}/build"
partition="tzcup_gztransport13_late_${$}_$(date +%s%N)"
[[ "${GZ_PARTITION:-}" != "${partition}" ]] || {
  echo "generated GZ_PARTITION unexpectedly collides with caller environment" >&2
  exit 125
}
readonly runtime_prefix runtime_lib output_dir build_dir partition transport_sha

# Do not source setup.bash in this transport-only check.  Enumerate the exact
# Jazzy vendor config prefixes needed by the frozen transport export instead.
# In particular, do not add gz_transport_vendor: that would permit fallback to
# the unpatched system transport if the frozen target/config check regressed.
jazzy_vendor_prefixes=(
  /opt/ros/jazzy/opt/gz_cmake_vendor
  /opt/ros/jazzy/opt/gz_utils_vendor
  /opt/ros/jazzy/opt/gz_msgs_vendor
  /opt/ros/jazzy/opt/gz_math_vendor
)
cmake_prefix_path="${runtime_prefix}"
for prefix in "${jazzy_vendor_prefixes[@]}"; do
  [[ -d "${prefix}" && ! -L "${prefix}" ]] || {
    echo "required Jazzy gz dependency prefix is unavailable: ${prefix}" >&2
    exit 125
  }
  cmake_prefix_path+=";${prefix}"
done

# CMake discovery does not supply an env -i endpoint process with ELF loader
# paths.  Keep those paths closed to the frozen transport plus precisely the
# audited libraries it declares; gz_cmake_vendor contributes CMake config only.
runtime_vendor_library_prefixes=(
  /opt/ros/jazzy/opt/gz_utils_vendor
  /opt/ros/jazzy/opt/gz_msgs_vendor
  /opt/ros/jazzy/opt/gz_math_vendor
)
runtime_library_path="${runtime_lib}"
for prefix in "${runtime_vendor_library_prefixes[@]}"; do
  library_dir="${prefix}/lib"
  [[ -d "${library_dir}" && ! -L "${library_dir}" ]] || {
    echo "required audited runtime library directory is unavailable: ${library_dir}" >&2
    exit 125
  }
  runtime_library_path+=":${library_dir}"
done
printf '%s\n' "${runtime_library_path}" >"${output_dir}/runtime-library-path-contract.txt"
readonly runtime_library_path

printf '%s\n' "${cmake_prefix_path}" >"${output_dir}/cmake-prefix-contract.txt"
readonly cmake_prefix_path
dependency_audit_args=(
  --runtime-prefix "${runtime_prefix}"
  --output "${output_dir}/dependency-closure.json"
)
for prefix in "${jazzy_vendor_prefixes[@]}"; do
  dependency_audit_args+=(--vendor-prefix "${prefix}")
done
python3 "${repo_root}/scripts/audit_gz_transport13_late_discovery_dependencies.py" \
  "${dependency_audit_args[@]}"

publisher_pid=""
subscriber_pid=""
cleanup() {
  local status=$?
  for pid in "${publisher_pid}" "${subscriber_pid}"; do
    [[ -n "${pid}" ]] || continue
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${publisher_pid}" "${subscriber_pid}"; do
    [[ -n "${pid}" ]] || continue
    wait "${pid}" 2>/dev/null || true
  done
  exit "${status}"
}
trap cleanup EXIT INT TERM

# The first entry binds gz-transport13 to the frozen runtime.  The following
# vendor entries satisfy only its audited declared dependency config chain; CMakeLists
# rejects either a config directory or imported transport library outside it.
cmake -S "${repo_root}/scripts/gz_transport13_late_discovery_smoke" \
  -B "${build_dir}" \
  -DTZCUP_FROZEN_RUNTIME_PREFIX="${runtime_prefix}" \
  -DCMAKE_PREFIX_PATH="${cmake_prefix_path}" \
  >"${output_dir}/cmake-configure.log" 2>&1
cmake --build "${build_dir}" --parallel 1 >"${output_dir}/cmake-build.log" 2>&1
binary="${build_dir}/gz_transport13_late_discovery_smoke"
[[ -x "${binary}" && ! -L "${binary}" ]] || {
  echo "late-discovery smoke binary was not produced" >&2
  exit 125
}
binary_sha="$(sha256sum "${binary}" | awk '{print $1}')"
env LD_LIBRARY_PATH="${runtime_library_path}" \
  "${repo_root}/scripts/formal_dynamic_dependencies.sh" "${binary}" \
  >"${output_dir}/smoke-binary.ldd.txt"
if grep -Eq '=>[[:space:]]+not found' "${output_dir}/smoke-binary.ldd.txt"; then
  echo "ldd reports an unresolved runtime dependency" >&2
  exit 125
fi

assert_ldd_library() {
  local soname="$1"
  local expected_dir="$2"
  local -a resolved_paths=()

  mapfile -t resolved_paths < <(
    awk -v soname="${soname}" '$1 == soname && $2 == "=>" { print $3 }' \
      "${output_dir}/smoke-binary.ldd.txt"
  )
  [[ "${#resolved_paths[@]}" -eq 1 ]] || {
    echo "ldd must resolve exactly one ${soname}; found ${#resolved_paths[@]}" >&2
    exit 125
  }
  [[ "${resolved_paths[0]}" == "${expected_dir}/"* && -f "${resolved_paths[0]}" ]] || {
    echo "${soname} resolved outside its audited vendor prefix: ${resolved_paths[0]}" >&2
    exit 125
  }
  printf '%s\t%s\t%s\n' \
    "${soname}" "${resolved_paths[0]}" \
    "$(sha256sum "${resolved_paths[0]}" | awk '{print $1}')" \
    >>"${output_dir}/runtime-library-bindings.tsv"
}

: >"${output_dir}/runtime-library-bindings.tsv"
assert_ldd_library "libgz-utils2.so.2" "/opt/ros/jazzy/opt/gz_utils_vendor/lib"
assert_ldd_library "libgz-msgs10.so.10" "/opt/ros/jazzy/opt/gz_msgs_vendor/lib"
# gz-math7 is a CMake dependency of gz-msgs10, but this endpoint does not use
# a message whose retained ELF dependency needs libgz-math7 at runtime.  The
# dependency-closure audit still binds its config; do not require the loader to
# invent a DT_NEEDED entry that the linked binary does not have.

transport_ldd_lines="$(grep -F 'libgz-transport13.so.13 =>' "${output_dir}/smoke-binary.ldd.txt" || true)"
[[ "$(printf '%s\n' "${transport_ldd_lines}" | sed '/^$/d' | wc -l)" -eq 1 ]] &&
  [[ "${transport_ldd_lines}" == *"${runtime_lib}/libgz-transport13.so.13"* ]] || {
  echo "smoke binary does not resolve gz-transport13 from frozen runtime" >&2
  exit 125
}

capture_process_maps() {
  local role="$1" pid="$2" case_dir="$3"
  local maps="${case_dir}/${role}.maps" lines="${case_dir}/${role}.transport-maps.txt"
  [[ -r "/proc/${pid}/maps" ]] || {
    echo "cannot read live ${role} /proc/${pid}/maps" >&2
    return 1
  }
  cp -- "/proc/${pid}/maps" "${maps}"
  grep -F 'libgz-transport13.so.13' "${maps}" >"${lines}" || {
    echo "${role} did not map gz-transport13" >&2
    return 1
  }
  local -a paths=()
  mapfile -t paths < <(awk '{print $6}' "${lines}" | sed 's/ (deleted)$//' | sort -u)
  (( ${#paths[@]} == 1 )) || {
    echo "${role} mapped more than one gz-transport13 path" >&2
    return 1
  }
  local mapped="${paths[0]}"
  [[ "${mapped}" = "${runtime_lib}/libgz-transport13.so.13" ||
     "${mapped}" = "${runtime_lib}/libgz-transport13.so.13.5.0" ]] || {
    echo "${role} mapped transport outside frozen runtime: ${mapped}" >&2
    return 1
  }
  [[ -f "${mapped}" && ! -L "${mapped}" ]] || {
    echo "${role} mapped transport is not a frozen regular file" >&2
    return 1
  }
  local mapped_sha
  mapped_sha="$(sha256sum "${mapped}" | awk '{print $1}')"
  [[ "${mapped_sha}" = "${transport_sha}" ]] || {
    echo "${role} mapped transport hash differs from frozen alias" >&2
    return 1
  }
  printf '%s\t%s\t%s\t%s\n' "${role}" "${pid}" "${mapped}" "${mapped_sha}" \
    >>"${case_dir}/transport-process-binding.tsv"
}

run_case() {
  local name="$1" first="$2"
  local case_dir="${output_dir}/${name}"
  local topic="/${partition}/${name}"
  # A subscriber begun after the publisher must not be expected to replay the
  # early messages.  Keep publishing long enough for late discovery, then
  # require a short contiguous suffix received after that subscription starts.
  local publish_count=40
  local minimum_post_discovery_count=3
  mkdir -- "${case_dir}"
  : >"${case_dir}/transport-process-binding.tsv"

  start_publisher() {
    env -i PATH="${PATH}" HOME="${HOME:-/tmp}" GZ_IP=127.0.0.1 GZ_PARTITION="${partition}" \
      LD_LIBRARY_PATH="${runtime_library_path}" \
      "${binary}" --mode publisher --topic "${topic}" \
      --report "${case_dir}/publisher.json" --count "${publish_count}" \
      --period-ms 125 --hold-ms 4000 >"${case_dir}/publisher.log" 2>&1 &
    publisher_pid=$!
  }
  start_subscriber() {
    env -i PATH="${PATH}" HOME="${HOME:-/tmp}" GZ_IP=127.0.0.1 GZ_PARTITION="${partition}" \
      LD_LIBRARY_PATH="${runtime_library_path}" \
      "${binary}" --mode subscriber --topic "${topic}" \
      --report "${case_dir}/subscriber.json" --count "${minimum_post_discovery_count}" \
      --period-ms 100 --hold-ms 8000 >"${case_dir}/subscriber.log" 2>&1 &
    subscriber_pid=$!
  }
  if [[ "${first}" = publisher ]]; then
    start_publisher
    sleep 1
    start_subscriber
  else
    start_subscriber
    sleep 1
    start_publisher
  fi
  sleep 1
  capture_process_maps publisher "${publisher_pid}" "${case_dir}"
  capture_process_maps subscriber "${subscriber_pid}" "${case_dir}"
  wait "${publisher_pid}"
  publisher_pid=""
  wait "${subscriber_pid}"
  subscriber_pid=""
  python3 - "${case_dir}" "${name}" "${first}" "${partition}" "${topic}" "${publish_count}" "${minimum_post_discovery_count}" <<'PY'
import json
import sys
from pathlib import Path

case_dir = Path(sys.argv[1])
name, first, partition, topic, publish_count_raw, minimum_post_discovery_raw = sys.argv[2:]
publish_count = int(publish_count_raw)
minimum_post_discovery_count = int(minimum_post_discovery_raw)
publisher = json.loads((case_dir / "publisher.json").read_text(encoding="utf-8"))
subscriber = json.loads((case_dir / "subscriber.json").read_text(encoding="utf-8"))
rows = []
for line in (case_dir / "transport-process-binding.tsv").read_text(encoding="utf-8").splitlines():
    role, pid, path, sha256 = line.split("\t")
    rows.append({"role": role, "pid": int(pid), "library_path": path, "sha256": sha256})
passed = (
    len(rows) == 2
    and publisher["endpoint_ok"]
    and publisher["expected_count"] == publish_count
    and publisher["published_count"] == publish_count
    and publisher["topic_info_ok"]
    and publisher["topic_info_publisher_count"] >= 1
    and subscriber["endpoint_ok"]
    and subscriber["expected_count"] == minimum_post_discovery_count
    and subscriber["received_count"] >= minimum_post_discovery_count
    and subscriber["unique_received_count"] >= minimum_post_discovery_count
    and subscriber["numeric_sequence_count"] >= minimum_post_discovery_count
    and subscriber["max_consecutive_sequence_count"] >= minimum_post_discovery_count
    and subscriber["topic_info_ok"]
    and subscriber["topic_info_publisher_count"] >= 1
)
report = {
    "schema_version": 1,
    "report_id": "tzcup_gz_transport13_late_discovery_smoke_v1",
    "status": "GZ_TRANSPORT13_LATE_DISCOVERY_CASE_PASSED" if passed else "GZ_TRANSPORT13_LATE_DISCOVERY_CASE_REJECTED",
    "passed": passed,
    "case": name,
    "startup_order": first + "_first",
    "gz_partition": partition,
    "topic": topic,
    "publisher_message_count": publish_count,
    "minimum_post_discovery_count": minimum_post_discovery_count,
    "publisher": publisher,
    "subscriber": subscriber,
    "process_transport_bindings": rows,
}
(case_dir / "result.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
if not passed:
    raise SystemExit("late-discovery case evidence is incomplete")
PY
}

run_case publisher_first publisher
run_case subscriber_first subscriber
python3 - "${output_dir}" "${partition}" "${runtime_lib}/libgz-transport13.so.13" "${transport_sha}" "${binary}" "${binary_sha}" <<'PY'
import json
import sys
from pathlib import Path

output, partition, library, library_sha, binary, binary_sha = sys.argv[1:]
root = Path(output)
cases = [json.loads((root / name / "result.json").read_text(encoding="utf-8"))
         for name in ("publisher_first", "subscriber_first")]
passed = all(case.get("passed") is True for case in cases)
report = {
    "schema_version": 1,
    "report_id": "tzcup_gz_transport13_late_discovery_smoke_v1",
    "status": "GZ_TRANSPORT13_LATE_DISCOVERY_SMOKE_PASSED" if passed else "GZ_TRANSPORT13_LATE_DISCOVERY_SMOKE_REJECTED",
    "passed": passed,
    "gz_partition": partition,
    "runtime": {
        "library_path": library,
        "library_sha256": library_sha,
        "smoke_binary_path": binary,
        "smoke_binary_sha256": binary_sha,
        "smoke_binary_ldd": "smoke-binary.ldd.txt",
        "runtime_library_path_contract": "runtime-library-path-contract.txt",
        "runtime_library_bindings": "runtime-library-bindings.tsv",
    },
    "cases": cases,
}
(root / "result.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
if not passed:
    raise SystemExit("late-discovery smoke rejected")
PY
trap - EXIT INT TERM
echo "GZ_TRANSPORT13_LATE_DISCOVERY_SMOKE_PASSED ${output_dir}/result.json"
