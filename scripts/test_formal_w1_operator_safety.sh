#!/usr/bin/env bash
# Exercise the production helper's real Bash lifecycle with a fake ROS CLI.
set -Eeuo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
tmp="$(mktemp -d)"
trap 'rm -rf -- "$tmp"' EXIT
mkdir "$tmp/bin" "$tmp/root"
cat >"$tmp/bin/ros2" <<'EOF'
#!/usr/bin/env bash
set -Eeuo pipefail
log="${FAKE_ROS_LOG:?}"; command="${1:?}"; shift
[[ "$command" == topic ]] || exit 97
case "${1:?}" in
  pub)
    shift; [[ "$1" = --rate && "$2" = 10 && "$4" = std_msgs/msg/Bool ]] || exit 96
    topic="$3"; value="$5"; printf '%s|%s|%s\n' "$$" "$topic" "$value" >>"$log"
    if [[ "$topic" = /formal_vehicle/simulation/command/emergency_stop ]]; then
      if [[ "$value" = '{data: false}' ]]; then marker="$FAKE_ROS_FALSE"; else marker="$FAKE_ROS_TRUE"; fi
      printf '%s\n' "$$" >"$marker"; trap 'rm -f -- "$marker"; exit 0' TERM INT EXIT
    fi
    while :; do sleep 60 & wait "$!"; done
    ;;
  info)
    [[ "$2" = --verbose ]] || exit 96
    case "${FAKE_INFO_MODE:-ready}" in
      ready) publishers=1 ;;
      zero) publishers=0 ;;
      two) publishers=2 ;;
      malformed) printf 'Publisher count: unknown\n'; exit 0 ;;
      hung) while :; do sleep 60 & wait "$!"; done ;;
      zero_to_one)
        seen=0; [[ -f "$FAKE_INFO_COUNT" ]] && seen="$(<"$FAKE_INFO_COUNT")"
        seen=$((seen + 1)); printf '%s\n' "$seen" >"$FAKE_INFO_COUNT"
        if (( seen <= 3 )); then publishers=0; else publishers=1; fi
        ;;
      *) exit 95 ;;
    esac
    printf 'Publisher count: %s\n' "$publishers"
    ;;
  echo)
    [[ "$2" = --once && "$3" = --full-length && "$4" = /safety/status_json && "$5" = std_msgs/msg/String ]] || exit 96
    count=0; [[ -f "$FAKE_ROS_COUNT" ]] && count="$(<"$FAKE_ROS_COUNT")"; count=$((count + 1)); printf '%s\n' "$count" >"$FAKE_ROS_COUNT"
    if [[ -f "$FAKE_ROS_TRUE" ]]; then state=INHIBITED; reason=manual_estop
    elif [[ -f "$FAKE_ROS_FALSE" ]]; then state=BASE_COMMAND_STOPPED; reason=manipulator_base_inhibit
    else state=INHIBITED; reason=manipulator_base_inhibit
    fi
    printf "data: >-\n  {\"state\": \"%s\", \"active_reasons\": \"%s\",\n  \"status_publish_count\": %s}\n---\n" "$state" "$reason" "$count"
    ;;
  *) exit 96 ;;
esac
EOF
chmod 700 "$tmp/bin/ros2"
export PATH="$tmp/bin:$PATH" FAKE_ROS_LOG="$tmp/publish.log" FAKE_ROS_FALSE="$tmp/estop-false.pid" FAKE_ROS_TRUE="$tmp/estop-true.pid" FAKE_ROS_COUNT="$tmp/count" FAKE_INFO_COUNT="$tmp/info-count"
source "$repo_root/scripts/formal_w1_operator_safety.sh"
formal_w1_operator_init "$tmp/root"
formal_w1_operator_start_and_release
python3 - "$tmp/root/operator-release-ready.json" <<'PY'
import json, sys
value=json.load(open(sys.argv[1],encoding='utf-8'))
assert value['active_operator_values']['/formal_vehicle/simulation/command/emergency_stop'] is False
assert value['safety_status_capture']['payload']['state'] == 'BASE_COMMAND_STOPPED'
assert value['truth_used_for_control'] is False
PY
formal_w1_operator_teardown
python3 - "$tmp/root/operator-teardown.json" <<'PY'
import json, sys
value=json.load(open(sys.argv[1],encoding='utf-8'))
assert value['active_operator_values']['/formal_vehicle/simulation/command/emergency_stop'] is True
assert value['safety_status_capture']['payload']['state'] == 'INHIBITED'
assert 'manual_estop' in value['safety_status_capture']['payload']['active_reasons'].split(',')
PY
[[ ! -e "$tmp/estop-false.pid" && ! -e "$tmp/estop-true.pid" ]]
grep -Fqx '/formal_vehicle/simulation/command/emergency_stop|{data: false}' <(cut -d'|' -f2- "$tmp/publish.log")
grep -Fqx '/formal_vehicle/simulation/command/emergency_stop|{data: true}' <(cut -d'|' -f2- "$tmp/publish.log")

start_release_publishers() {
  formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_ESTOP" false FORMAL_W1_OPERATOR_ESTOP_FALSE_PID "$FORMAL_W1_OPERATOR_ROOT/operator-estop-release.log"
  formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_ESTOP_RESET" true FORMAL_W1_OPERATOR_RESET_PID "$FORMAL_W1_OPERATOR_ROOT/operator-estop-reset.log"
  formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_MAIN_POWER" true FORMAL_W1_OPERATOR_POWER_PID "$FORMAL_W1_OPERATOR_ROOT/operator-main-power.log"
}

stop_release_publishers() {
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_ESTOP_FALSE_PID
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_RESET_PID
  formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_POWER_PID
}

export R065_W1_OPERATOR_PUBLISHER_DISCOVERY_TIMEOUT_SECONDS=1
export R065_W1_OPERATOR_PUBLISHER_DISCOVERY_POLL_SECONDS=0.01

rm -f -- "$FAKE_INFO_COUNT"
export FAKE_INFO_MODE=zero_to_one
mkdir "$tmp/zero-to-one"
formal_w1_operator_init "$tmp/zero-to-one"
start_release_publishers
formal_w1_operator_require_sole_physical_publishers
[[ -f "$tmp/zero-to-one/operator-emergency_stop.topic-info.1.txt" ]]
[[ -f "$tmp/zero-to-one/operator-emergency_stop.topic-info.2.txt" ]]
stop_release_publishers

export FAKE_INFO_MODE=zero
mkdir "$tmp/persistent-zero"
formal_w1_operator_init "$tmp/persistent-zero"
start_release_publishers
if formal_w1_operator_require_sole_physical_publishers; then exit 91; fi
[[ -f "$tmp/persistent-zero/operator-main_power.topic-info.1.txt" ]]
stop_release_publishers

export FAKE_INFO_MODE=two
mkdir "$tmp/two"
formal_w1_operator_init "$tmp/two"
start_release_publishers
if formal_w1_operator_require_sole_physical_publishers; then exit 92; fi
stop_release_publishers

export FAKE_INFO_MODE=malformed
mkdir "$tmp/malformed"
formal_w1_operator_init "$tmp/malformed"
start_release_publishers
if formal_w1_operator_require_sole_physical_publishers; then exit 94; fi
stop_release_publishers

export FAKE_INFO_MODE=hung
mkdir "$tmp/hung"
formal_w1_operator_init "$tmp/hung"
start_release_publishers
if formal_w1_operator_require_sole_physical_publishers; then exit 95; fi
stop_release_publishers

export FAKE_INFO_MODE=ready
mkdir "$tmp/dead"
formal_w1_operator_init "$tmp/dead"
start_release_publishers
formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_RESET_PID
if formal_w1_operator_require_sole_physical_publishers; then exit 93; fi
stop_release_publishers

R065_W1_OPERATOR_PUBLISHER_DISCOVERY_POLL_SECONDS=0 formal_w1_operator_require_sole_physical_publishers && exit 96 || true
R065_W1_OPERATOR_PUBLISHER_DISCOVERY_POLL_SECONDS=6 formal_w1_operator_require_sole_physical_publishers && exit 97 || true
printf '%s\n' 'formal W1 physical-operator Bash lifecycle fixture passed'
