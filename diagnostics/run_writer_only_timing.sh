#!/usr/bin/env bash
set -euo pipefail
W=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-short09-lifecycle-health
E=/mnt/f/Project/TZcup/.workspace/evidence/short09-writer-only-timing-01
[ ! -e "$E" ]; mkdir "$E"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export PYTHONPATH="$W/starter_ws/src/sanitation_formal_campus_integration:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=172 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$W/config/cyclonedds_localhost.xml"
sha256sum "$W"/diagnostics/writer_only_timing_test.py "$W"/scripts/capture_formal_first_map_early_recording_audit.py "$W"/scripts/finalize_formal_first_map_localization_diagnostic.py "$W"/starter_ws/src/sanitation_formal_campus_integration/sanitation_formal_campus_integration/lifecycle_health_protocol.py "$W"/starter_ws/src/sanitation_formal_campus_integration/sanitation_formal_campus_integration/map_lifecycle_manager.py > "$E/executed-source.sha256"
set +e
timeout --signal=TERM --kill-after=10s 60s python3 -B "$W/diagnostics/writer_only_timing_test.py" --observer "$W/diagnostics/verify_live_frontier_short_09.py" --output "$E/run" > "$E/test.log" 2>&1
rc=$?
set -e
printf '%s\n' "$rc" > "$E/test.rc"
exit "$rc"
