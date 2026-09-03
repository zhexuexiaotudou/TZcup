#!/usr/bin/env bash
# Shared host/DDS/Gazebo isolation primitives for formal acceptance runners.
# Source this file; do not execute it as a standalone acceptance.

FORMAL_RUNTIME_EVIDENCE_PATHS=()
FORMAL_RUNTIME_MAX_AUTO_PARTICIPANT_INDEX=120
FORMAL_RUNTIME_MEMORY_WATCHDOG_PID=""
FORMAL_RUNTIME_MEMORY_WATCHDOG_JSON=""
FORMAL_RUNTIME_MEMORY_WATCHDOG_LOG=""
FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=0
FORMAL_RUNTIME_MEMORY_WATCHDOG_DELEGATED=0
FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE=86
FORMAL_RUNTIME_PGID_READY_ATTEMPTS=100
FORMAL_RUNTIME_PGID_READY_POLL_S=0.01
# A standalone runner owns a dedicated process group for targeted cleanup.  The
# final orchestrator already creates one session around the complete step, so a
# nested setsid would escape its exact-PGID RSS accounting and shutdown.
FORMAL_RUNTIME_SESSION_PREFIX=()
if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" != "1" ]]; then
  FORMAL_RUNTIME_SESSION_PREFIX=(setsid)
fi

formal_runtime_domain_is_linux_safe() {
  local domain="$1"
  [[ "${domain}" =~ ^[0-9]+$ ]] && \
    (( (domain >= 0 && domain <= 101) || (domain >= 215 && domain <= 231) ))
}

formal_runtime_max_dds_unicast_port() {
  local domain="$1"
  # DDSI-RTPS PB + DG*domain + d3 + PG*participant_index.
  echo $((7400 + 250 * domain + 11 + 2 * FORMAL_RUNTIME_MAX_AUTO_PARTICIPANT_INDEX))
}

formal_runtime_configure() {
  local base_domain="$1"
  local domain_count="${2:-1}"
  local base_domain_number domain_count_number index domain max_dds_port
  [[ "${base_domain}" =~ ^[0-9]+$ && ${#base_domain} -le 3 ]] || {
    echo "formal ROS domain must be a decimal integer: ${base_domain}" >&2
    return 2
  }
  [[ "${domain_count}" =~ ^[1-9][0-9]*$ && ${#domain_count} -le 3 ]] || {
    echo "formal runtime domain count must be a positive integer: ${domain_count}" >&2
    return 2
  }
  base_domain_number=$((10#${base_domain}))
  domain_count_number=$((10#${domain_count}))
  for ((index=0; index<domain_count_number; index++)); do
    domain=$((base_domain_number + index))
    formal_runtime_domain_is_linux_safe "${domain}" || {
      echo "formal ROS domain ${domain} intersects Linux ephemeral ports or exceeds the bounded DDS port range; use 0..101 or 215..231" >&2
      return 2
    }
    max_dds_port="$(formal_runtime_max_dds_unicast_port "${domain}")"
    (( max_dds_port < 65535 )) || {
      echo "formal ROS domain ${domain} reaches invalid DDS UDP port ${max_dds_port}" >&2
      return 2
    }
  done

  export ROS2CLI_DISABLE_DAEMON=1
  # This is a formal single-host acceptance boundary, not a user-selectable
  # networking profile.  Do not inherit a caller's wider discovery/interface
  # settings into a high-bandwidth Gazebo run.
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
  # Gazebo Transport has its own discovery/data plane and does not inherit the
  # CycloneDDS loopback policy below.  High-bandwidth camera/lidar topics must
  # never select the WSL virtual Ethernet adapter: on affected Windows hosts
  # that path can grow NDIS Nbuf/Nnbl/Nnbf nonpaged-pool allocations until the
  # machine exhausts commit.  Formal acceptance is single-host, so pin both
  # current and legacy Gazebo Transport interface variables to loopback.
  export GZ_IP=127.0.0.1
  export IGN_IP=127.0.0.1
  unset GZ_RELAY IGN_RELAY
  local helper_dir repo_root
  helper_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  repo_root="$(cd -- "${helper_dir}/.." && pwd)"
  export CYCLONEDDS_URI="file://${repo_root}/config/cyclonedds_localhost.xml"
  FORMAL_RUNTIME_LOCK_FILE="${FORMAL_GAZEBO_LOCK_FILE:-/tmp/tzcup_formal_gazebo.lock}"
  export FORMAL_RUNTIME_LOCK_FILE
  exec 9>"${FORMAL_RUNTIME_LOCK_FILE}"
  flock -n 9 || {
    echo "another formal Gazebo acceptance owns ${FORMAL_RUNTIME_LOCK_FILE}; run the matrix serially" >&2
    return 75
  }
}

formal_runtime_kill_group() {
  local pid="${1:-}"
  local signal attempt pgid=""
  [[ -n "${pid}" ]] || return 0
  # Interrupt the launch process first and let launch perform an ordered child
  # shutdown.  Broadcasting SIGINT to the entire process group races launch's
  # own shutdown propagation and has caused rclpy publish-after-shutdown errors
  # and ros_gz_bridge double-free aborts in otherwise healthy acceptances.
  if kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
  fi
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null || true)"
  pgid="${pgid//[[:space:]]/}"
  # Under the final orchestrator, the runner owns the outer process group and
  # background launch processes intentionally do not create nested sessions.
  # Never signal that shared outer group from a child cleanup.  Give ros2
  # launch a bounded graceful interval, then let the exact GZ_PARTITION cleanup
  # below terminate only this launch and its descendants.
  if [[ -n "${pgid}" && "${pgid}" != "${pid}" ]]; then
    for attempt in {1..40}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    return 0
  fi
  for attempt in {1..40}; do
    kill -0 -- "-${pid}" 2>/dev/null || break
    sleep 0.25
  done
  for signal in TERM KILL; do
    if kill -0 -- "-${pid}" 2>/dev/null; then
      kill -"${signal}" -- "-${pid}" 2>/dev/null || true
    fi
    for attempt in {1..20}; do
      kill -0 -- "-${pid}" 2>/dev/null || break
      sleep 0.25
    done
  done
  wait "${pid}" 2>/dev/null || true
  ! kill -0 -- "-${pid}" 2>/dev/null
}

formal_runtime_contain_rejected_leader() {
  local pid="${1:-}"
  local signal attempt
  [[ "${pid}" =~ ^[0-9]+$ ]] && (( pid > 1 )) || return 2
  # PGID readiness failed, so a fresh setsid transition may race the last ps
  # sample.  Never inspect or signal the observed parent PGID here.  Contain
  # only the exact child PID and the only private group it is allowed to form
  # (-PID), checking both throughout the bounded shutdown window.
  if kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
  fi
  for attempt in {1..40}; do
    if ! kill -0 "${pid}" 2>/dev/null && ! kill -0 -- "-${pid}" 2>/dev/null; then
      break
    fi
    sleep 0.25
  done
  for signal in TERM KILL; do
    kill -0 "${pid}" 2>/dev/null && kill -"${signal}" "${pid}" 2>/dev/null || true
    kill -0 -- "-${pid}" 2>/dev/null && kill -"${signal}" -- "-${pid}" 2>/dev/null || true
    for attempt in {1..20}; do
      if ! kill -0 "${pid}" 2>/dev/null && ! kill -0 -- "-${pid}" 2>/dev/null; then
        break
      fi
      sleep 0.25
    done
  done
  wait "${pid}" 2>/dev/null || true
  ! kill -0 "${pid}" 2>/dev/null && ! kill -0 -- "-${pid}" 2>/dev/null
}

formal_runtime_cleanup_partition() {
  local partition="${1:-}"
  [[ -n "${partition}" ]] || return 0
  /usr/bin/python3 - "${partition}" "$$" <<'PY'
import os, signal, sys, time

needle = ("GZ_PARTITION=" + sys.argv[1]).encode()
excluded = {os.getpid(), int(sys.argv[2])}

def matching():
    rows = []
    for raw in os.listdir("/proc"):
        if not raw.isdigit() or int(raw) in excluded:
            continue
        try:
            environment = open(f"/proc/{raw}/environ", "rb").read().split(b"\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if needle in environment:
            rows.append(int(raw))
    return rows

for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
    for pid in matching():
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 2.0
    while matching() and time.monotonic() < deadline:
        time.sleep(0.1)
remaining = matching()
if remaining:
    print("formal partition cleanup survivors: " + ",".join(map(str, remaining)), file=sys.stderr)
raise SystemExit(1 if remaining else 0)
PY
}

formal_runtime_cleanup_groups() {
  local partition="$1"
  shift
  local pid failed=0
  for pid in "$@"; do
    formal_runtime_kill_group "${pid}" || failed=1
  done
  formal_runtime_cleanup_partition "${partition}" || failed=1
  # Reap launch children only after partition cleanup.  Waiting before that
  # point can block forever when an orchestrated launch shares the runner PGID.
  for pid in "$@"; do
    wait "${pid}" 2>/dev/null || true
    kill -0 "${pid}" 2>/dev/null && failed=1
  done
  return "${failed}"
}

formal_runtime_wait_for_setsid_pgid() {
  local leader_pid="$1"
  local attempt pgid=""
  [[ "${leader_pid}" =~ ^[0-9]+$ ]] && (( leader_pid > 1 )) || {
    echo "formal memory watchdog requires a launch leader PID greater than one" >&2
    return 2
  }
  for ((attempt=1; attempt<=FORMAL_RUNTIME_PGID_READY_ATTEMPTS; attempt++)); do
    pgid="$(ps -o pgid= -p "${leader_pid}" 2>/dev/null || true)"
    pgid="${pgid//[[:space:]]/}"
    if [[ -z "${pgid}" ]]; then
      echo "formal launch leader ${leader_pid} disappeared before creating its own process group" >&2
      return 2
    fi
    [[ "${pgid}" =~ ^[0-9]+$ ]] || {
      echo "formal launch leader ${leader_pid} returned an invalid PGID: ${pgid}" >&2
      return 2
    }
    if [[ "${pgid}" == "${leader_pid}" ]]; then
      printf '%s\n' "${pgid}"
      return 0
    fi
    kill -0 "${leader_pid}" 2>/dev/null || {
      echo "formal launch leader ${leader_pid} disappeared while waiting for its setsid process group" >&2
      return 2
    }
    if (( attempt == FORMAL_RUNTIME_PGID_READY_ATTEMPTS )); then
      echo "formal launch leader ${leader_pid} did not create its own process group within the bounded readiness window; last_pgid=${pgid}" >&2
      return 2
    fi
    sleep "${FORMAL_RUNTIME_PGID_READY_POLL_S}"
  done
  return 2
}

formal_runtime_attest_orchestrated_step_session() {
  local runner_pid="$$"
  local pgid sid
  pgid="$(ps -o pgid= -p "${runner_pid}" 2>/dev/null || true)"
  sid="$(ps -o sid= -p "${runner_pid}" 2>/dev/null || true)"
  pgid="${pgid//[[:space:]]/}"
  sid="${sid//[[:space:]]/}"
  # An integrated step can launch a child runner inside its already-isolated
  # outer session.  The child is not the leader, but its PGID/SID must still
  # name that one dedicated session before it delegates the watchdog.
  if ! [[ "${pgid}" =~ ^[0-9]+$ && "${sid}" =~ ^[0-9]+$ && "${pgid}" == "${sid}" ]]; then
    echo "formal orchestrated runner must share one dedicated PGID/SID; pid=${runner_pid} pgid=${pgid:-missing} sid=${sid:-missing}" >&2
    return 2
  fi
}

formal_runtime_start_memory_watchdog() {
  local leader_pid="$1"
  local evidence_prefix="$2"
  local helper_dir pgid readiness_status
  case "${FORMAL_MEMORY_WATCHDOG_ENABLED:-1}" in
    0|false|FALSE|no|NO)
      echo "FORMAL_MEMORY_WATCHDOG_ENABLED cannot be disabled for formal runtime" >&2
      return 2
      ;;
    1|true|TRUE|yes|YES) ;;
    *)
      echo "FORMAL_MEMORY_WATCHDOG_ENABLED must be a boolean" >&2
      return 2
      ;;
  esac
  if (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT != 0 )); then
    echo "refusing to launch after an earlier memory-guard failure" >&2
    return "${FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT}"
  fi
  [[ -z "${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}" ]] || {
    echo "a formal memory watchdog is already active" >&2
    return 2
  }
  if [[ "${FORMAL_ORCHESTRATED_STEP_SESSION:-0}" == "1" ]]; then
    # The final orchestrator owns this exact PGID's watchdog.  Do not create a
    # nested watchdog for a launch child that correctly shares the outer PGID.
    if ! formal_runtime_attest_orchestrated_step_session; then
      FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=125
      return 125
    fi
    FORMAL_RUNTIME_MEMORY_WATCHDOG_DELEGATED=1
    return 0
  fi
  if pgid="$(formal_runtime_wait_for_setsid_pgid "${leader_pid}")"; then
    :
  else
    readiness_status=$?
    # A rejected setsid wrapper is still our exact child.  Stop that PID and
    # any exact -PID group it may create after the last readiness sample, so a
    # late setsid transition cannot leave an unguarded orphan runtime behind.
    if ! formal_runtime_contain_rejected_leader "${leader_pid}"; then
      echo "failed to contain rejected formal launch leader ${leader_pid}" >&2
      FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=125
      return 125
    fi
    return "${readiness_status}"
  fi
  FORMAL_RUNTIME_MEMORY_WATCHDOG_JSON="${evidence_prefix}.json"
  FORMAL_RUNTIME_MEMORY_WATCHDOG_LOG="${evidence_prefix}.log"
  [[ ! -e "${FORMAL_RUNTIME_MEMORY_WATCHDOG_JSON}" && ! -e "${FORMAL_RUNTIME_MEMORY_WATCHDOG_LOG}" ]] || {
    echo "refusing stale formal memory-watchdog evidence: ${evidence_prefix}" >&2
    return 2
  }
  helper_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  setsid "${helper_dir}/formal_memory_watchdog.sh" \
    --leader-pid "${leader_pid}" --pgid "${pgid}" \
    --json "${FORMAL_RUNTIME_MEMORY_WATCHDOG_JSON}" \
    --log "${FORMAL_RUNTIME_MEMORY_WATCHDOG_LOG}" 9>&- &
  FORMAL_RUNTIME_MEMORY_WATCHDOG_PID=$!
  FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=0
}

formal_runtime_memory_preflight() {
  local evidence_prefix="$1"
  local helper_dir result=0
  case "${FORMAL_MEMORY_WATCHDOG_ENABLED:-1}" in
    0|false|FALSE|no|NO)
      echo "FORMAL_MEMORY_WATCHDOG_ENABLED cannot be disabled for formal runtime" >&2
      return 2
      ;;
    1|true|TRUE|yes|YES) ;;
    *) echo "FORMAL_MEMORY_WATCHDOG_ENABLED must be a boolean" >&2; return 2 ;;
  esac
  case "${FORMAL_WINDOWS_MEMORY_GUARD_ENABLED:-1}" in
    0|false|FALSE|no|NO)
      echo "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED cannot be disabled for formal runtime" >&2
      return 2
      ;;
    1|true|TRUE|yes|YES) ;;
    *) echo "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED must be a boolean" >&2; return 2 ;;
  esac
  [[ ! -e "${evidence_prefix}.json" && ! -e "${evidence_prefix}.log" ]] || {
    echo "refusing stale formal Windows memory preflight evidence: ${evidence_prefix}" >&2
    return 2
  }
  helper_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  set +e
  python3 "${helper_dir}/formal_windows_memory_probe.py" --check-start \
    --output "${evidence_prefix}.json" >"${evidence_prefix}.log" 2>&1
  result=$?
  set -e
  if (( result == FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE )); then
    FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT="${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
    echo "formal runtime start refused by Windows commit/Docker memory gate" >&2
  elif (( result != 0 )); then
    FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=125
    echo "formal Windows memory start gate failed closed: rc=${result}" >&2
  fi
  return "${FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT}"
}

formal_runtime_stop_memory_watchdog() {
  local result=0
  [[ -n "${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}" ]] || return 0
  if kill -0 "${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}" 2>/dev/null; then
    kill -TERM "${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}" 2>/dev/null || true
  fi
  set +e
  wait "${FORMAL_RUNTIME_MEMORY_WATCHDOG_PID}" 2>/dev/null
  result=$?
  set -e
  FORMAL_RUNTIME_MEMORY_WATCHDOG_PID=""
  if (( result == FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE )); then
    FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT="${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
  elif (( result != 0 )); then
    echo "formal memory watchdog failed unexpectedly: rc=${result}" >&2
    FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT=125
  fi
  return 0
}

formal_runtime_memory_watchdog_tripped() {
  (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT == FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE ))
}

formal_runtime_install_traps() {
  FORMAL_RUNTIME_CLEANUP_FUNCTION="$1"
  trap 'formal_runtime_exit_trap "$?"' EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
}

formal_runtime_register_evidence_paths() {
  local path
  for path in "$@"; do
    [[ -n "${path}" ]] && FORMAL_RUNTIME_EVIDENCE_PATHS+=("${path}")
  done
}

formal_runtime_quarantine_evidence() {
  local path quarantine
  for path in "${FORMAL_RUNTIME_EVIDENCE_PATHS[@]:-}"; do
    [[ -e "${path}" ]] || continue
    quarantine="${path}.cleanup_failed.$$"
    if [[ -e "${quarantine}" ]]; then
      echo "refusing to overwrite retained cleanup-failure evidence: ${quarantine}" >&2
      continue
    fi
    mv -- "${path}" "${quarantine}" || true
  done
}

formal_runtime_exit_trap() {
  local status="$1"
  trap - EXIT INT TERM
  if ! "${FORMAL_RUNTIME_CLEANUP_FUNCTION}"; then
    echo "formal runtime cleanup failed closed" >&2
    formal_runtime_quarantine_evidence
    status=125
  fi
  formal_runtime_stop_memory_watchdog || true
  if formal_runtime_memory_watchdog_tripped; then
    echo "formal runtime stopped by memory watchdog" >&2
    status="${FORMAL_RUNTIME_MEMORY_BREACH_EXIT_CODE}"
  elif (( FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT != 0 )); then
    echo "formal runtime memory watchdog failed closed" >&2
    status=125
  fi
  exit "${status}"
}
