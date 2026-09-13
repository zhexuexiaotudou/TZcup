#!/usr/bin/env bash
set -euo pipefail
W=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-short09-lifecycle-health
E=/mnt/f/Project/TZcup/.workspace/evidence/short09-ros-full-writer-02
[ ! -e "$E" ]; mkdir "$E"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export PYTHONPATH="$W/starter_ws/src/sanitation_formal_campus_integration:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=172 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$W/config/cyclonedds_localhost.xml"
set +e
timeout --signal=TERM --kill-after=10s 60s python3 -B "$W/diagnostics/ros_full_writer_health_test_02.py" --observer "$W/diagnostics/verify_live_frontier_short_09.py" --output "$E/run" > "$E/test.log" 2>&1
rc=$?
set -e
printf '%s\n' "$rc" > "$E/test.rc"
exit "$rc"
