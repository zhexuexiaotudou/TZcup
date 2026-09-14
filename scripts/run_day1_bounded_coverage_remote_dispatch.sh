#!/usr/bin/env bash
# Host-side one-shot dispatch for the bounded saved-map coverage mission.
set -eo pipefail

host_root=/root/autodl-tmp/tzcup-competition-sim-only-20260912
guest_root=/workspace/tzcup-competition-sim-only-20260912
run_id="${TZCUP_DAY1_COVERAGE_RUN_ID:-day1-bounded-coverage-20260914-01}"
run_host="$host_root/evidence/$run_id"
run_guest="$guest_root/evidence/$run_id"
runtime="$guest_root/runtime/runtime-ws-1a211400-lifecycle-health-v4-r1-27cac7f7773f"
source_root="$guest_root/evidence/localization-day1-final-20260914-02/source"
overlay="$guest_root/evidence/localization-day1-20260914-01/overlay"
map_source="$guest_root/evidence/competition-integrated-20260913-01"
episode="$guest_root/evidence/motion-cleaning-continuous-fixture-01/episode"
runtime_host="$host_root/runtime/runtime-ws-1a211400-lifecycle-health-v4-r1-27cac7f7773f"
source_root_host="$host_root/evidence/localization-day1-final-20260914-02/source"
overlay_host="$host_root/evidence/localization-day1-20260914-01/overlay"
map_source_host="$host_root/evidence/competition-integrated-20260913-01"
episode_host="$host_root/evidence/motion-cleaning-continuous-fixture-01/episode"
domain="${TZCUP_DAY1_COVERAGE_DOMAIN_ID:-94}"
partition="${TZCUP_DAY1_COVERAGE_PARTITION:-tzcup_day1_bounded_coverage_20260914_01}"
xdg_runtime="${TZCUP_DAY1_COVERAGE_XDG_RUNTIME:-/tmp/${partition}_xdg}"
wall_deadline_seconds=1200

if [[ -e "$run_host/run-01" || -e "$run_host/run-01.primary.rc" ]]; then
  echo "refusing existing run-01" >&2
  exit 2
fi
for required in \
  "$runtime_host/install/setup.bash" \
  "$source_root_host/scripts/run_formal_runtime_isolation.sh" \
  "$overlay_host/install/local_setup.bash" \
  "$map_source_host/occupancy.yaml" \
  "$episode_host/public/world.sdf" \
  "$run_host/ops/run_day1_bounded_coverage_runner.sh" \
  "$run_host/ops/day1_bounded_coverage_cleaning_bridge.py" \
  "$run_host/ops/day1_bounded_coverage_readiness.py" \
  "$run_host/ops/summarize_day1_bounded_coverage_run.py" \
  "$run_host/ops/day1_bounded_coverage_mission.yaml"; do
  [[ -e "$required" ]] || { echo "missing required path: $required" >&2; exit 3; }
done

set +e
env -u LD_PRELOAD /root/autodl-tmp/tzcup-runtime/bin/noble-exec-upstream \
  timeout --signal=TERM --kill-after=20s "$wall_deadline_seconds" \
  bash -lc "
set -eo pipefail
rm -rf '$xdg_runtime'
install -d -m 700 '$xdg_runtime'
export XDG_RUNTIME_DIR='$xdg_runtime'
export SOURCE='$source_root'
export RUNTIME='$runtime'
export OVERLAY='$overlay'
export OUTPUT='$run_guest/run-01'
export EPISODE='$episode'
export MAP_SOURCE='$map_source'
export OPS_DIR='$run_guest/ops'
export COVERAGE_CONFIG='$run_guest/ops/day1_bounded_coverage_mission.yaml'
export CLEANING_BRIDGE='$run_guest/ops/day1_bounded_coverage_cleaning_bridge.py'
export SUMMARIZER='$run_guest/ops/summarize_day1_bounded_coverage_run.py'
export PROBE_DOMAIN='$domain'
export PROBE_PARTITION='$partition'
bash '$run_guest/ops/run_day1_bounded_coverage_runner.sh'
"
rc=$?
set -e
printf '%s\n' "$rc" > "$run_host/run-01.primary.rc"

survivors="$(ps -eo pid,ppid,pgid,lstart,args | grep -F "$partition" | grep -v grep || true)"
printf '%s\n' "$survivors" > "$run_host/run-01.survivors.txt"
lock_available=false
if flock -n 9; then lock_available=true; fi 9>/tmp/tzcup_formal_gazebo.lock
python3 - "$run_host/run-01.resource_release.json" "$rc" "$lock_available" "$survivors" <<'PY'
import json
import pathlib
import sys

output = pathlib.Path(sys.argv[1])
rc = int(sys.argv[2])
lock_available = sys.argv[3] == "true"
survivors = [line for line in sys.argv[4].splitlines() if line.strip()]
payload = {
    "schema_version": 1,
    "primary_rc": rc,
    "wall_deadline_seconds": 1200,
    "formal_gazebo_lock_available": lock_available,
    "partition_survivor_count": len(survivors),
    "partition_survivors": survivors,
    "sim_resource_released": rc != 124 and lock_available and not survivors,
}
output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
exit "$rc"
