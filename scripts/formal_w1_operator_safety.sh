#!/usr/bin/env bash
# Physical simulation operator inputs for the formal W1 footprint gate.
# This deliberately does not start formal_same_map_baseline_support because
# that helper also owns evaluator-truth publication.
set -Eeuo pipefail

FORMAL_W1_OPERATOR_ESTOP='/formal_vehicle/simulation/command/emergency_stop'
FORMAL_W1_OPERATOR_ESTOP_RESET='/formal_vehicle/simulation/command/emergency_stop_reset'
FORMAL_W1_OPERATOR_MAIN_POWER='/formal_vehicle/simulation/command/main_power'
FORMAL_W1_OPERATOR_INHIBIT='/manipulation/base_motion_inhibited'
FORMAL_W1_OPERATOR_SAFETY='/safety/status_json'
FORMAL_W1_OPERATOR_RATE_HZ=10

formal_w1_operator_init() {
  : "${1:?run root}"
  FORMAL_W1_OPERATOR_ROOT="$1"
  FORMAL_W1_OPERATOR_STATUS_HELPER="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/formal_w1_operator_safety_status.py"
  [[ -f "$FORMAL_W1_OPERATOR_STATUS_HELPER" && ! -L "$FORMAL_W1_OPERATOR_STATUS_HELPER" ]] || return 98
  FORMAL_W1_OPERATOR_ACTIVE=false
  FORMAL_W1_OPERATOR_INHIBIT_PID=0
  FORMAL_W1_OPERATOR_ESTOP_FALSE_PID=0
  FORMAL_W1_OPERATOR_RESET_PID=0
  FORMAL_W1_OPERATOR_POWER_PID=0
  FORMAL_W1_OPERATOR_ESTOP_TRUE_PID=0
  FORMAL_W1_OPERATOR_PRE_COUNT=-1
  FORMAL_W1_OPERATOR_RELEASE_COUNT=-1
}

formal_w1_operator_record() {
  local name="$1" stage="$2" status_file="${3-}" rc="${4-0}"
  local path="$FORMAL_W1_OPERATOR_ROOT/$name"
  [[ ! -e "$path" && ! -L "$path" ]] || return 98
  python3 - "$path" "$stage" "$status_file" "$FORMAL_W1_OPERATOR_STATUS_HELPER" "$rc" \
    "$FORMAL_W1_OPERATOR_INHIBIT_PID" "$FORMAL_W1_OPERATOR_ESTOP_FALSE_PID" \
    "$FORMAL_W1_OPERATOR_RESET_PID" "$FORMAL_W1_OPERATOR_POWER_PID" \
    "$FORMAL_W1_OPERATOR_ESTOP_TRUE_PID" <<'PY'
import hashlib, json, os, subprocess, sys, time
from pathlib import Path
path = Path(sys.argv[1]); status_path = Path(sys.argv[3]) if sys.argv[3] else None
payload = {
    'report_id': 'formal_w1_physical_operator',
    'stage': sys.argv[2],
    'captured_wall_time_ns': time.time_ns(),
    'record_rc': int(sys.argv[5]),
    'publisher_pids': {
        'base_inhibit': int(sys.argv[6]), 'estop_false': int(sys.argv[7]),
        'estop_reset': int(sys.argv[8]), 'main_power': int(sys.argv[9]),
        'estop_true': int(sys.argv[10]),
    },
    'truth_used_for_control': False,
}
publisher_pids = payload['publisher_pids']
payload['active_operator_values'] = {
    '/formal_vehicle/simulation/command/emergency_stop': (
        True if publisher_pids['estop_true'] > 1 else
        False if publisher_pids['estop_false'] > 1 else None),
    '/formal_vehicle/simulation/command/emergency_stop_reset': (
        True if publisher_pids['estop_reset'] > 1 else None),
    '/formal_vehicle/simulation/command/main_power': (
        True if publisher_pids['main_power'] > 1 else None),
    '/manipulation/base_motion_inhibited': (
        True if publisher_pids['base_inhibit'] > 1 else None),
}
if status_path:
    raw = status_path.read_text(encoding='utf-8')
    value = json.loads(subprocess.check_output(
        [sys.executable, sys.argv[4], 'json', str(status_path)], text=True))
    payload['safety_status_capture'] = {
        'path': str(status_path), 'sha256': hashlib.sha256(raw.encode()).hexdigest(),
        'payload': value,
    }
pending = path.with_suffix(path.suffix + f'.pending.{os.getpid()}')
fd = os.open(pending, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
with os.fdopen(fd, 'w', encoding='utf-8') as stream:
    json.dump(payload, stream, indent=2, sort_keys=True); stream.write('\n')
os.replace(pending, path)
PY
}

formal_w1_operator_start_publisher() {
  local topic="$1" value="$2" variable="$3" log="$4"
  ros2 topic pub --rate "$FORMAL_W1_OPERATOR_RATE_HZ" "$topic" std_msgs/msg/Bool "{data: $value}" >"$log" 2>&1 &
  printf -v "$variable" '%s' "$!"
  sleep 1
  kill -0 "${!variable}" 2>/dev/null
}

formal_w1_operator_stop_publisher() {
  local variable="$1"
  local pid="${!variable}"
  local attempt
  [[ "$pid" =~ ^[0-9]+$ && "$pid" -gt 1 ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || return 1
    for attempt in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || return 1
    fi
  fi
  wait "$pid" 2>/dev/null || true
  printf -v "$variable" '%s' 0
}

formal_w1_operator_capture_safety() {
  local label="$1"
  local file="$FORMAL_W1_OPERATOR_ROOT/safety-$label.yaml"
  [[ ! -e "$file" && ! -L "$file" ]] || return 98
  timeout 5s ros2 topic echo --once --full-length "$FORMAL_W1_OPERATOR_SAFETY" std_msgs/msg/String >"$file" 2>&1
  [[ -f "$file" && ! -L "$file" ]] || return 98
  printf '%s\n' "$file"
}

formal_w1_operator_status_count() {
  python3 "$FORMAL_W1_OPERATOR_STATUS_HELPER" count "$1"
}

formal_w1_operator_require_status() {
  local file="$1" expected_state="$2" required_reason="$3" previous_count="$4"
  python3 "$FORMAL_W1_OPERATOR_STATUS_HELPER" require "$file" "$expected_state" "$required_reason" "$previous_count"
}

formal_w1_operator_wait_status() {
  local label="$1" expected_state="$2" required_reason="$3" previous_count="$4" file=''
  for attempt in $(seq 1 "${R065_W1_OPERATOR_SAFETY_POLLS:-30}"); do
    file="$(formal_w1_operator_capture_safety "$label-$attempt")" || return 3
    if formal_w1_operator_require_status "$file" "$expected_state" "$required_reason" "$previous_count"; then
      printf '%s\n' "$file"
      return 0
    fi
    sleep 1
  done
  return 3
}

formal_w1_operator_require_sole_physical_publishers() {
  local topic info count variable pid attempt=0 all_ready
  local timeout_seconds="${R065_W1_OPERATOR_PUBLISHER_DISCOVERY_TIMEOUT_SECONDS:-10}"
  local poll_seconds="${R065_W1_OPERATOR_PUBLISHER_DISCOVERY_POLL_SECONDS:-0.25}"
  [[ "$timeout_seconds" =~ ^[1-9][0-9]*$ ]] || return 98
  (( timeout_seconds <= 60 )) || return 98
  [[ "$poll_seconds" =~ ^[0-9]+([.][0-9]+)?$ ]] || return 98
  awk -v value="$poll_seconds" 'BEGIN { exit !(value > 0 && value <= 5) }' || return 98
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    attempt=$((attempt + 1))
    all_ready=true
    for topic in "$FORMAL_W1_OPERATOR_ESTOP" "$FORMAL_W1_OPERATOR_ESTOP_RESET" "$FORMAL_W1_OPERATOR_MAIN_POWER"; do
      case "$topic" in
        "$FORMAL_W1_OPERATOR_ESTOP") variable=FORMAL_W1_OPERATOR_ESTOP_FALSE_PID ;;
        "$FORMAL_W1_OPERATOR_ESTOP_RESET") variable=FORMAL_W1_OPERATOR_RESET_PID ;;
        "$FORMAL_W1_OPERATOR_MAIN_POWER") variable=FORMAL_W1_OPERATOR_POWER_PID ;;
        *) return 98 ;;
      esac
      pid="${!variable}"
      [[ "$pid" =~ ^[0-9]+$ && "$pid" -gt 1 ]] || return 3
      kill -0 "$pid" 2>/dev/null || return 3
      info="$FORMAL_W1_OPERATOR_ROOT/operator-$(basename "$topic").topic-info.$attempt.txt"
      [[ ! -e "$info" && ! -L "$info" ]] || return 98
      local remaining=$((deadline - SECONDS))
      (( remaining > 0 )) || return 3
      timeout --signal=TERM --kill-after=1s "${remaining}s" ros2 topic info --verbose "$topic" >"$info" 2>&1 || return 3
      kill -0 "$pid" 2>/dev/null || return 3
      count="$(awk '/^Publisher count: [0-9]+$/ { seen += 1; value = $3 } END { if (seen == 1) print value; else exit 1 }' "$info")" || return 3
      case "$count" in
        0) all_ready=false ;;
        1) ;;
        *) return 3 ;;
      esac
    done
    [[ "$all_ready" == true ]] && return 0
    sleep "$poll_seconds"
  done
  return 3
}

formal_w1_operator_start_and_release() {
  local baseline pre release
  formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_INHIBIT" true FORMAL_W1_OPERATOR_INHIBIT_PID "$FORMAL_W1_OPERATOR_ROOT/operator-base-inhibit.log"
  baseline="$(formal_w1_operator_capture_safety inhibit-baseline)"
  formal_w1_operator_require_status "$baseline" INHIBITED manipulator_base_inhibit -1
  pre="$(formal_w1_operator_wait_status pre-release INHIBITED manipulator_base_inhibit "$(formal_w1_operator_status_count "$baseline")")"
  FORMAL_W1_OPERATOR_PRE_COUNT="$(formal_w1_operator_status_count "$pre")"
  formal_w1_operator_record operator-pre-release.json pre_release "$pre"

  formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_ESTOP" false FORMAL_W1_OPERATOR_ESTOP_FALSE_PID "$FORMAL_W1_OPERATOR_ROOT/operator-estop-release.log"
  formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_ESTOP_RESET" true FORMAL_W1_OPERATOR_RESET_PID "$FORMAL_W1_OPERATOR_ROOT/operator-estop-reset.log"
  formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_MAIN_POWER" true FORMAL_W1_OPERATOR_POWER_PID "$FORMAL_W1_OPERATOR_ROOT/operator-main-power.log"
  formal_w1_operator_require_sole_physical_publishers
  release="$(formal_w1_operator_wait_status post-release BASE_COMMAND_STOPPED manipulator_base_inhibit "$FORMAL_W1_OPERATOR_PRE_COUNT")"
  FORMAL_W1_OPERATOR_RELEASE_COUNT="$(formal_w1_operator_status_count "$release")"
  formal_w1_operator_record operator-release-ready.json release_ready "$release"
  FORMAL_W1_OPERATOR_ACTIVE=true
}

formal_w1_operator_teardown() {
  local status='' rc=0
  # Stop conflicting release/reset publishers before introducing physical E-stop.
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_ESTOP_FALSE_PID || rc=1
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_RESET_PID || rc=1
  if [[ "$FORMAL_W1_OPERATOR_INHIBIT_PID" -gt 1 ]]; then
    formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_ESTOP" true FORMAL_W1_OPERATOR_ESTOP_TRUE_PID "$FORMAL_W1_OPERATOR_ROOT/operator-estop-rearm.log" || rc=1
    if [[ "$rc" -eq 0 ]]; then
      status="$(formal_w1_operator_wait_status teardown-estop INHIBITED manual_estop "$FORMAL_W1_OPERATOR_RELEASE_COUNT")" || rc=1
    fi
  fi
  formal_w1_operator_record operator-teardown.json teardown "$status" "$rc" || rc=1
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_POWER_PID || rc=1
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_ESTOP_TRUE_PID || rc=1
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_INHIBIT_PID || rc=1
  FORMAL_W1_OPERATOR_ACTIVE=false
  return "$rc"
}
