#!/usr/bin/env bash
# Bound a formal runtime by host memory and one exact process group.
#
# The watchdog never searches by command name and never signals processes
# outside the caller-supplied PGID.  Thresholds are KiB because /proc/meminfo
# and /proc/<pid>/status expose KiB values.
set -uo pipefail

readonly FORMAL_MEMORY_WATCHDOG_BREACH_EXIT_CODE=86

leader_pid=""
target_pgid=""
json_path=""
log_path=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --leader-pid) leader_pid="${2:-}"; shift 2 ;;
    --pgid) target_pgid="${2:-}"; shift 2 ;;
    --json) json_path="${2:-}"; shift 2 ;;
    --log) log_path="${2:-}"; shift 2 ;;
    *) echo "unknown memory-watchdog argument: $1" >&2; exit 2 ;;
  esac
done

is_uint() { [[ "${1:-}" =~ ^[0-9]+$ ]]; }

for value in "${leader_pid}" "${target_pgid}"; do
  is_uint "${value}" && (( value > 1 )) || {
    echo "memory watchdog requires a PID/PGID greater than one" >&2
    exit 2
  }
done
[[ -n "${json_path}" && -n "${log_path}" ]] || {
  echo "memory watchdog requires fresh --json and --log paths" >&2
  exit 2
}
[[ ! -e "${json_path}" && ! -e "${log_path}" ]] || {
  echo "refusing stale memory-watchdog evidence" >&2
  exit 2
}
mkdir -p "$(dirname -- "${json_path}")" "$(dirname -- "${log_path}")"
: >"${log_path}"

min_available_kib="${FORMAL_MEMORY_MIN_AVAILABLE_KIB:-3145728}"
max_swap_used_kib="${FORMAL_MEMORY_MAX_SWAP_USED_KIB:-1048576}"
max_group_rss_kib="${FORMAL_MEMORY_MAX_GROUP_RSS_KIB:-9437184}"
poll_s="${FORMAL_MEMORY_POLL_S:-1}"
windows_probe_timeout_s="${FORMAL_WINDOWS_PROBE_TIMEOUT_S:-5}"
windows_probe_transient_retries="${FORMAL_WINDOWS_PROBE_TRANSIENT_RETRIES:-1}"
int_grace_s="${FORMAL_MEMORY_INT_GRACE_S:-8}"
term_grace_s="${FORMAL_MEMORY_TERM_GRACE_S:-5}"
windows_min_commit_available_bytes="${FORMAL_WINDOWS_RUNTIME_MIN_COMMIT_AVAILABLE_BYTES:-6442450944}"
windows_max_docker_private_bytes="${FORMAL_WINDOWS_RUNTIME_MAX_DOCKER_PRIVATE_BYTES:-8589934592}"
case "${FORMAL_WINDOWS_MEMORY_GUARD_ENABLED:-1}" in
  0|false|FALSE|no|NO) windows_guard_enabled=false ;;
  1|true|TRUE|yes|YES) windows_guard_enabled=true ;;
  *) echo "FORMAL_WINDOWS_MEMORY_GUARD_ENABLED must be a boolean" >&2; exit 2 ;;
esac
for value in "${min_available_kib}" "${max_swap_used_kib}" "${max_group_rss_kib}"; do
  is_uint "${value}" || { echo "memory watchdog thresholds must be unsigned KiB integers" >&2; exit 2; }
done
for value in "${windows_min_commit_available_bytes}" "${windows_max_docker_private_bytes}"; do
  is_uint "${value}" || { echo "Windows memory watchdog thresholds must be unsigned byte integers" >&2; exit 2; }
done
[[ "${poll_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "invalid memory watchdog poll interval" >&2; exit 2; }
[[ "${windows_probe_timeout_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "invalid Windows memory probe timeout" >&2; exit 2; }
is_uint "${windows_probe_transient_retries}" || { echo "invalid Windows memory probe transient retry count" >&2; exit 2; }
(( windows_probe_transient_retries <= 3 )) || { echo "Windows memory probe transient retry count must be at most 3" >&2; exit 2; }
awk -v value="${poll_s}" 'BEGIN {exit !(value >= 0.1)}' || { echo "memory watchdog poll interval must be at least 0.1 s" >&2; exit 2; }
awk -v value="${windows_probe_timeout_s}" 'BEGIN {exit !(value >= 0.1)}' || { echo "Windows memory probe timeout must be at least 0.1 s" >&2; exit 2; }
[[ "${int_grace_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "invalid memory watchdog INT grace" >&2; exit 2; }
[[ "${term_grace_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "invalid memory watchdog TERM grace" >&2; exit 2; }

leader_pgid="$(ps -o pgid= -p "${leader_pid}" 2>/dev/null | tr -d '[:space:]')"
[[ "${leader_pgid}" == "${target_pgid}" ]] || {
  echo "leader ${leader_pid} is not in exact PGID ${target_pgid}" >&2
  exit 2
}
watchdog_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d '[:space:]')"
[[ "${watchdog_pgid}" != "${target_pgid}" ]] || {
  echo "memory watchdog refuses to join the target process group" >&2
  exit 2
}

group_rss_kib() {
  local proc stat_tail proc_pgrp rss total=0
  for proc in /proc/[0-9]*; do
    [[ -r "${proc}/stat" && -r "${proc}/status" ]] || continue
    IFS= read -r stat_tail <"${proc}/stat" 2>/dev/null || continue
    # Drop pid and the parenthesized comm field; the remaining fields start
    # with state, ppid, pgrp.  comm may contain spaces or parentheses.
    stat_tail="${stat_tail##*) }"
    read -r _ _ proc_pgrp _ <<<"${stat_tail}" || continue
    [[ "${proc_pgrp}" == "${target_pgid}" ]] || continue
    rss="$(awk '$1 == "VmRSS:" {print $2; found=1; exit} END {if (!found) print 0}' "${proc}/status" 2>/dev/null)"
    is_uint "${rss}" || rss=0
    total=$((total + rss))
  done
  echo "${total}"
}

group_exists() {
  local proc stat_tail proc_pgrp
  for proc in /proc/[0-9]*; do
    [[ -r "${proc}/stat" ]] || continue
    IFS= read -r stat_tail <"${proc}/stat" 2>/dev/null || continue
    stat_tail="${stat_tail##*) }"
    read -r _ _ proc_pgrp _ <<<"${stat_tail}" || continue
    [[ "${proc_pgrp}" == "${target_pgid}" ]] && return 0
  done
  return 1
}

read_host_memory() {
  local key value _ mem_available=0 swap_total=0 swap_free=0
  while read -r key value _; do
    case "${key}" in
      MemAvailable:) mem_available="${value}" ;;
      SwapTotal:) swap_total="${value}" ;;
      SwapFree:) swap_free="${value}" ;;
    esac
  done </proc/meminfo
  is_uint "${mem_available}" || mem_available=0
  is_uint "${swap_total}" || swap_total=0
  is_uint "${swap_free}" || swap_free=0
  echo "${mem_available} $((swap_total - swap_free)) ${swap_free}"
}

write_json() {
  local status="$1" mem_available="$2" swap_used="$3" swap_free="$4" group_rss="$5"
  local low_available="$6" excessive_swap="$7" excessive_group="$8" int_sent="$9" term_sent="${10}" survivors="${11}"
  local win_limit="${12}" win_charge="${13}" win_available="${14}" docker_private="${15}"
  local vmmem_wsl_private="${16}" low_win_commit="${17}" excessive_docker="${18}" host_probe_failed="${19}"
  local nonpaged_pool="${20:-0}" pool_tags_available="${21:-false}"
  local ndis_tag_total="${22:-0}" suspected_ndis_pool_leak="${23:-false}"
  local report_breach_exit_code="${FORMAL_MEMORY_WATCHDOG_BREACH_EXIT_CODE}"
  local pending="${json_path}.pending.$$"
  [[ "${status}" != "FORMAL_WINDOWS_MEMORY_PROBE_FAILED_CLOSED" ]] || report_breach_exit_code=125
  printf '{\n  "report_id": "tzcup_formal_memory_watchdog_v1",\n  "status": "%s",\n  "breach_exit_code": %d,\n  "leader_pid": %d,\n  "target_pgid": %d,\n  "sample_epoch_ns": %s,\n  "thresholds_kib": {"min_mem_available": %d, "max_swap_used": %d, "max_group_rss": %d},\n  "windows_guard_enabled": %s,\n  "windows_thresholds_bytes": {"min_commit_available": %d, "max_docker_private": %d},\n  "observed_kib": {"mem_available": %d, "swap_used": %d, "swap_free": %d, "group_rss": %d},\n  "windows_observed_bytes": {"commit_limit": %d, "commit_charge": %d, "commit_available": %d, "docker_private": %d, "vmmem_wsl_private": %d, "nonpaged_pool": %d, "ndis_tracked_pool_tags": %d},\n  "windows_diagnostics": {"pool_tag_query_available": %s, "suspected_ndis_nonpaged_pool_leak": %s, "last_probe_failure": {"kind": "%s", "read_rc": %d, "field_count": %d, "failure_streak": %d, "probe_alive_at_check": %s, "had_prior_valid_sample": %s, "rejected_sequence": %d, "previous_accepted_sequence": %d}},\n  "reasons": {"low_mem_available": %s, "excessive_swap_used": %s, "excessive_group_rss": %s, "low_windows_commit_available": %s, "excessive_docker_private": %s, "windows_probe_failed": %s, "suspected_ndis_nonpaged_pool_leak": %s},\n  "signals": {"exact_pgid_only": true, "sigint_sent": %s, "sigterm_sent": %s, "docker_signalled_or_stopped": false},\n  "surviving_group_processes": %d\n}\n' \
    "${status}" "${report_breach_exit_code}" "${leader_pid}" "${target_pgid}" "$(date +%s%N)" \
    "${min_available_kib}" "${max_swap_used_kib}" "${max_group_rss_kib}" \
    "${windows_guard_enabled}" \
    "${windows_min_commit_available_bytes}" "${windows_max_docker_private_bytes}" \
    "${mem_available}" "${swap_used}" "${swap_free}" "${group_rss}" \
    "${win_limit}" "${win_charge}" "${win_available}" "${docker_private}" "${vmmem_wsl_private}" "${nonpaged_pool}" "${ndis_tag_total}" \
    "${pool_tags_available}" "${suspected_ndis_pool_leak}" \
    "${last_windows_probe_failure_kind}" "${last_windows_probe_read_rc}" "${last_windows_probe_field_count}" \
    "${last_windows_probe_failure_streak}" "${last_windows_probe_alive}" "${last_windows_probe_had_valid_sample}" \
    "${last_windows_probe_rejected_sequence}" "${last_windows_probe_previous_sequence}" \
    "${low_available}" "${excessive_swap}" "${excessive_group}" "${low_win_commit}" "${excessive_docker}" "${host_probe_failed}" "${suspected_ndis_pool_leak}" \
    "${int_sent}" "${term_sent}" "${survivors}" >"${pending}"
  mv -- "${pending}" "${json_path}"
}

wait_for_group_exit() {
  local grace="$1" deadline
  deadline="$(awk -v now="$(date +%s.%N)" -v grace="${grace}" 'BEGIN {printf "%.9f", now + grace}')"
  while group_exists; do
    awk -v now="$(date +%s.%N)" -v deadline="${deadline}" 'BEGIN {exit !(now >= deadline)}' && return 1
    sleep 0.1
  done
  return 0
}

stop_requested=0
windows_probe_pid=""
windows_probe_fd=""
cleanup_windows_probe() {
  if [[ -n "${windows_probe_pid}" ]]; then
    kill -TERM "${windows_probe_pid}" 2>/dev/null || true
    wait "${windows_probe_pid}" 2>/dev/null || true
    windows_probe_pid=""
  fi
}
trap 'stop_requested=1' INT TERM HUP
trap cleanup_windows_probe EXIT
printf '%s watchdog started leader=%s pgid=%s min_available_kib=%s max_swap_used_kib=%s max_group_rss_kib=%s\n' \
  "$(date -u +%FT%TZ)" "${leader_pid}" "${target_pgid}" "${min_available_kib}" "${max_swap_used_kib}" "${max_group_rss_kib}" >>"${log_path}"

if [[ "${windows_guard_enabled}" == true ]]; then
  helper_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  coproc FORMAL_WINDOWS_PROBE {
    exec python3 "${helper_dir}/formal_windows_memory_probe.py" --stream --interval-s "${poll_s}" 2>>"${log_path}"
  }
  windows_probe_pid="${FORMAL_WINDOWS_PROBE_PID}"
  windows_probe_fd="${FORMAL_WINDOWS_PROBE[0]}"
fi

last_win_epoch=0
last_win_limit=0
last_win_charge=0
last_win_available=0
last_docker_private=0
last_vmmem_wsl_private=0
last_nonpaged_pool=0
last_pool_tags_available=false
last_ndis_tag_total=0
last_suspected_ndis_pool_leak=false
have_valid_win_sample=false
windows_probe_failure_streak=0
previous_windows_probe_attempt_failure_kind=none
last_windows_probe_failure_kind=none
last_windows_probe_read_rc=0
last_windows_probe_field_count=0
last_windows_probe_failure_streak=0
last_windows_probe_alive=false
last_windows_probe_had_valid_sample=false
last_windows_probe_rejected_sequence=0
last_windows_probe_previous_sequence=0
while true; do
  read -r mem_available swap_used swap_free < <(read_host_memory)
  group_rss="$(group_rss_kib)"
  win_epoch=0
  win_limit=0
  win_charge=0
  win_available=0
  docker_private=0
  vmmem_wsl_private=0
  nonpaged_pool=0
  pool_tags_available=false
  ndis_tag_total=0
  suspected_ndis_pool_leak=false
  host_probe_failed=false
  host_probe_transient=false
  if [[ "${windows_guard_enabled}" == true ]]; then
    sample_win_epoch=""
    sample_win_limit=""
    sample_win_charge=""
    sample_win_available=""
    sample_docker_private=""
    sample_vmmem_wsl_private=""
    sample_nonpaged_pool=""
    sample_pool_tags_available=""
    sample_ndis_tag_total=""
    sample_suspected_ndis_pool_leak=""
    sample_line=""
    sample_fields=()
    sample_failure_kind=none
    if IFS= read -r -t "${windows_probe_timeout_s}" sample_line <&"${windows_probe_fd}"; then
      windows_probe_read_rc=0
      IFS=' ' read -r -a sample_fields <<<"${sample_line}"
      # Six fields are accepted only for older probe binaries; the current
      # probe emits four extra, diagnostic-only kernel-pool fields.
      if (( ${#sample_fields[@]} == 10 )); then
        sample_win_epoch="${sample_fields[0]}"
        sample_win_limit="${sample_fields[1]}"
        sample_win_charge="${sample_fields[2]}"
        sample_win_available="${sample_fields[3]}"
        sample_docker_private="${sample_fields[4]}"
        sample_vmmem_wsl_private="${sample_fields[5]}"
        sample_nonpaged_pool="${sample_fields[6]}"
        sample_pool_tags_available="${sample_fields[7]}"
        sample_ndis_tag_total="${sample_fields[8]}"
        sample_suspected_ndis_pool_leak="${sample_fields[9]}"
      elif (( ${#sample_fields[@]} == 6 )); then
        sample_win_epoch="${sample_fields[0]}"
        sample_win_limit="${sample_fields[1]}"
        sample_win_charge="${sample_fields[2]}"
        sample_win_available="${sample_fields[3]}"
        sample_docker_private="${sample_fields[4]}"
        sample_vmmem_wsl_private="${sample_fields[5]}"
        sample_nonpaged_pool=0
        sample_pool_tags_available=0
        sample_ndis_tag_total=0
        sample_suspected_ndis_pool_leak=0
      else
        windows_probe_read_rc=125
        sample_failure_kind=field_count
      fi
    else
      windows_probe_read_rc=$?
      if (( windows_probe_read_rc > 128 )); then
        sample_failure_kind=stdout_timeout
      elif (( windows_probe_read_rc == 1 )); then
        sample_failure_kind=probe_eof
      else
        sample_failure_kind=read_error
      fi
    fi
    sample_valid=true
    if (( windows_probe_read_rc != 0 )); then
      sample_valid=false
    else
      for value in "${sample_win_epoch}" "${sample_win_limit}" "${sample_win_charge}" "${sample_win_available}" "${sample_docker_private}" "${sample_vmmem_wsl_private}" "${sample_nonpaged_pool}" "${sample_pool_tags_available}" "${sample_ndis_tag_total}" "${sample_suspected_ndis_pool_leak}"; do
        if ! is_uint "${value}"; then
          sample_valid=false
          sample_failure_kind=non_uint
          break
        fi
      done
      if [[ "${sample_valid}" == true ]]; then
        if (( sample_win_epoch <= last_win_epoch )); then
          sample_valid=false
          sample_failure_kind=non_monotonic_epoch
        elif (( sample_win_charge > sample_win_limit )) || (( sample_win_available != sample_win_limit - sample_win_charge )); then
          sample_valid=false
          sample_failure_kind=commit_invariant
        elif (( sample_pool_tags_available > 1 )) || (( sample_suspected_ndis_pool_leak > 1 )) || (( sample_pool_tags_available == 0 && sample_ndis_tag_total != 0 )) || (( sample_pool_tags_available == 0 && sample_suspected_ndis_pool_leak != 0 )); then
          sample_valid=false
          sample_failure_kind=pool_diagnostic_invariant
        fi
      fi
    fi
    if [[ "${sample_valid}" == true ]]; then
      win_epoch="${sample_win_epoch}"
      win_limit="${sample_win_limit}"
      win_charge="${sample_win_charge}"
      win_available="${sample_win_available}"
      docker_private="${sample_docker_private}"
      vmmem_wsl_private="${sample_vmmem_wsl_private}"
      nonpaged_pool="${sample_nonpaged_pool}"
      ndis_tag_total="${sample_ndis_tag_total}"
      [[ "${sample_pool_tags_available}" == 1 ]] && pool_tags_available=true
      [[ "${sample_suspected_ndis_pool_leak}" == 1 ]] && suspected_ndis_pool_leak=true
      last_win_epoch="${win_epoch}"
      last_win_limit="${win_limit}"
      last_win_charge="${win_charge}"
      last_win_available="${win_available}"
      last_docker_private="${docker_private}"
      last_vmmem_wsl_private="${vmmem_wsl_private}"
      last_nonpaged_pool="${nonpaged_pool}"
      last_pool_tags_available="${pool_tags_available}"
      last_ndis_tag_total="${ndis_tag_total}"
      last_suspected_ndis_pool_leak="${suspected_ndis_pool_leak}"
      have_valid_win_sample=true
      windows_probe_failure_streak=0
      previous_windows_probe_attempt_failure_kind=none
    else
      post_timeout_duplicate_grace=false
      if [[ "${sample_failure_kind}" == non_monotonic_epoch && "${previous_windows_probe_attempt_failure_kind}" == stdout_timeout && "${windows_probe_failure_streak}" == 1 ]]; then
        # Bash read -t can discard a complete record at the timeout boundary
        # and then expose the same buffered record once more.  Discard exactly
        # one such duplicate after one timeout.  A following invalid record
        # still exceeds the unchanged one-transient-failure budget.
        sample_failure_kind=post_timeout_duplicate
        post_timeout_duplicate_grace=true
      else
        windows_probe_failure_streak=$((windows_probe_failure_streak + 1))
      fi
      win_limit="${last_win_limit}"
      win_charge="${last_win_charge}"
      win_available="${last_win_available}"
      docker_private="${last_docker_private}"
      vmmem_wsl_private="${last_vmmem_wsl_private}"
      nonpaged_pool="${last_nonpaged_pool}"
      pool_tags_available="${last_pool_tags_available}"
      ndis_tag_total="${last_ndis_tag_total}"
      suspected_ndis_pool_leak="${last_suspected_ndis_pool_leak}"
      probe_alive=false
      kill -0 "${windows_probe_pid}" 2>/dev/null && probe_alive=true
      last_windows_probe_failure_kind="${sample_failure_kind}"
      last_windows_probe_read_rc="${windows_probe_read_rc}"
      last_windows_probe_field_count="${#sample_fields[@]}"
      last_windows_probe_failure_streak="${windows_probe_failure_streak}"
      last_windows_probe_alive="${probe_alive}"
      last_windows_probe_had_valid_sample="${have_valid_win_sample}"
      last_windows_probe_previous_sequence="${last_win_epoch}"
      if is_uint "${sample_win_epoch}"; then
        last_windows_probe_rejected_sequence="${sample_win_epoch}"
      else
        last_windows_probe_rejected_sequence=0
      fi
      printf '%s Windows memory probe transient failure kind=%s read_rc=%s field_count=%s streak=%s/%s probe_alive=%s last_sample_available=%s rejected_sequence=%s previous_accepted_sequence=%s\n' \
        "$(date -u +%FT%TZ)" "${sample_failure_kind}" "${windows_probe_read_rc}" "${#sample_fields[@]}" "${windows_probe_failure_streak}" "$((windows_probe_transient_retries + 1))" "${probe_alive}" "${have_valid_win_sample}" "${last_windows_probe_rejected_sequence}" "${last_windows_probe_previous_sequence}" >>"${log_path}"
      previous_windows_probe_attempt_failure_kind="${sample_failure_kind}"
      if (( windows_probe_failure_streak > windows_probe_transient_retries )); then
        host_probe_failed=true
      else
        host_probe_transient=true
      fi
    fi
  fi
  if (( stop_requested )); then
    write_json "FORMAL_MEMORY_WATCHDOG_STOPPED" "${mem_available}" "${swap_used}" "${swap_free}" "${group_rss}" false false false false false 0 \
      "${win_limit}" "${win_charge}" "${win_available}" "${docker_private}" "${vmmem_wsl_private}" false false "${host_probe_failed}" \
      "${nonpaged_pool}" "${pool_tags_available}" "${ndis_tag_total}" "${suspected_ndis_pool_leak}"
    exit 0
  fi
  if ! group_exists; then
    write_json "FORMAL_MEMORY_WATCHDOG_COMPLETED" "${mem_available}" "${swap_used}" "${swap_free}" "${group_rss}" false false false false false 0 \
      "${win_limit}" "${win_charge}" "${win_available}" "${docker_private}" "${vmmem_wsl_private}" false false "${host_probe_failed}" \
      "${nonpaged_pool}" "${pool_tags_available}" "${ndis_tag_total}" "${suspected_ndis_pool_leak}"
    exit 0
  fi

  low_available=false
  excessive_swap=false
  excessive_group=false
  low_win_commit=false
  excessive_docker=false
  (( mem_available < min_available_kib )) && low_available=true
  (( swap_used > max_swap_used_kib )) && excessive_swap=true
  (( group_rss > max_group_rss_kib )) && excessive_group=true
  if [[ "${windows_guard_enabled}" == true && "${host_probe_failed}" == false ]]; then
    if [[ "${host_probe_transient}" == false ]]; then
      (( win_available < windows_min_commit_available_bytes )) && low_win_commit=true
      (( docker_private > windows_max_docker_private_bytes )) && excessive_docker=true
    fi
  fi
  printf '%s sample mem_available_kib=%s swap_used_kib=%s group_rss_kib=%s windows_commit_available_bytes=%s docker_private_bytes=%s vmmem_wsl_private_bytes=%s nonpaged_pool_bytes=%s pool_tags_available=%s ndis_tag_bytes=%s suspected_ndis_pool_leak=%s\n' \
    "$(date -u +%FT%TZ)" "${mem_available}" "${swap_used}" "${group_rss}" "${win_available}" "${docker_private}" "${vmmem_wsl_private}" "${nonpaged_pool}" "${pool_tags_available}" "${ndis_tag_total}" "${suspected_ndis_pool_leak}" >>"${log_path}"
  if [[ "${low_available}" == true || "${excessive_swap}" == true || "${excessive_group}" == true || "${low_win_commit}" == true || "${excessive_docker}" == true || "${host_probe_failed}" == true || "${suspected_ndis_pool_leak}" == true ]]; then
    printf '%s memory breach mem_available_kib=%s swap_used_kib=%s group_rss_kib=%s windows_commit_available_bytes=%s docker_private_bytes=%s vmmem_wsl_private_bytes=%s host_probe_failed=%s suspected_ndis_pool_leak=%s; signalling exact pgid=%s only\n' \
      "$(date -u +%FT%TZ)" "${mem_available}" "${swap_used}" "${group_rss}" "${win_available}" "${docker_private}" "${vmmem_wsl_private}" "${host_probe_failed}" "${suspected_ndis_pool_leak}" "${target_pgid}" >>"${log_path}"
    int_sent=false
    term_sent=false
    if group_exists; then
      kill -INT -- "-${target_pgid}" 2>/dev/null && int_sent=true
    fi
    if ! wait_for_group_exit "${int_grace_s}"; then
      if group_exists; then
        kill -TERM -- "-${target_pgid}" 2>/dev/null && term_sent=true
      fi
      wait_for_group_exit "${term_grace_s}" || true
    fi
    survivors=0
    group_exists && survivors=1
    breach_status="FORMAL_MEMORY_LIMIT_BREACHED"
    breach_code="${FORMAL_MEMORY_WATCHDOG_BREACH_EXIT_CODE}"
    if [[ "${host_probe_failed}" == true ]]; then
      breach_status="FORMAL_WINDOWS_MEMORY_PROBE_FAILED_CLOSED"
      breach_code=125
    fi
    write_json "${breach_status}" "${mem_available}" "${swap_used}" "${swap_free}" "${group_rss}" \
      "${low_available}" "${excessive_swap}" "${excessive_group}" "${int_sent}" "${term_sent}" "${survivors}" \
      "${win_limit}" "${win_charge}" "${win_available}" "${docker_private}" "${vmmem_wsl_private}" "${low_win_commit}" "${excessive_docker}" "${host_probe_failed}" \
      "${nonpaged_pool}" "${pool_tags_available}" "${ndis_tag_total}" "${suspected_ndis_pool_leak}"
    exit "${breach_code}"
  fi
  if [[ "${windows_guard_enabled}" != true || "${host_probe_transient}" == true ]]; then
    sleep "${poll_s}"
  fi
done
