#!/usr/bin/env bash
set -euo pipefail
W=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-short09-lifecycle-health
E=/mnt/f/Project/TZcup/.workspace/evidence/short09-writer-finalizer-reverify-01
R=/mnt/f/Project/TZcup/.workspace/evidence/short09-writer-only-timing-01/run/map
[ ! -e "$E" ]; mkdir "$E"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export PYTHONDONTWRITEBYTECODE=1
sha256sum "$W/scripts/finalize_formal_first_map_localization_diagnostic.py" "$W/scripts/test_finalize_formal_first_map_localization_diagnostic.py" "$W/scripts/test_capture_formal_first_map_early_recording_audit.py" > "$E/executed-source.sha256"
set +e
timeout --kill-after=5s 40s python3 -B -m pytest -q -p no:cacheprovider "$W/scripts/test_finalize_formal_first_map_localization_diagnostic.py" "$W/scripts/test_capture_formal_first_map_early_recording_audit.py" > "$E/tests.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$E/tests.rc"
[ "$rc" -eq 0 ] || exit "$rc"
timeout --kill-after=5s 30s python3 -B "$W/scripts/finalize_formal_first_map_localization_diagnostic.py" --run-root "$R" --bag-dir "$R/mapping_localization_diagnostic" --topic-manifest "$R/topics.txt" --output "$R/localization_diagnostic_receipt_strict_reverify_01.json" --recorder-stop-rc 0 --optional-topic /ground_truth/odom --runtime-binding "$R/runtime_binding.json" --started-epoch-ns 1789258358049205266 --require-writer-timing > "$E/finalizer.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$E/finalizer.rc"
exit "$rc"
