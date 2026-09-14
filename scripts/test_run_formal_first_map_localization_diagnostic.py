from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_formal_first_map_dynamic_prerequisite.sh"


def test_first_map_runner_captures_fail_closed_read_only_localization_mcap():
    source = RUNNER.read_text(encoding="utf-8")
    assert "capture_formal_first_map_early_recording_audit.py" in source
    assert '--role localization' in source
    assert '--role early' in source
    assert 'localization_bag_dir="${output}/mapping_localization_diagnostic"' in source
    assert "finalize_formal_first_map_localization_diagnostic.py" in source
    assert '"${localization_bag_pid}"' in source
    for topic in (
        "/odom", "/odom/unfiltered", "/odometry/gps", "/gnss/fix",
        "/formal_mapping/lifecycle_status",
    ):
        assert topic in source
    assert "/ground_truth/odom" in source
    assert '"world_truth_used_for_control": False' not in source
    assert "localization diagnostic rosbag failed" in source
    assert "mapping_localization_diagnostic_sha256" in source
    launch = source.index('ros2 launch sanitation_formal_campus_integration')
    assert source.index('--role localization') < launch
    assert source.index('--role early') < launch
    assert source.index('wait_for_formal_recording_arm') < source.index('for index in $(seq 1')


def test_blocked_or_unreaped_recorder_cannot_fall_through_to_handoff():
    source = RUNNER.read_text(encoding="utf-8")
    assert 'if python3 "${repo_root}/scripts/finalize_formal_first_map_localization_diagnostic.py"' in source
    assert 'return "${finalizer_rc}"' in source
    assert 'kill -0 "${bag_pid}"' in source
    assert 'wait "${bag_pid}"' in source
    handoff_marker = 'python3 - ' + '\\' + '\n      "${handoff_record}"'
    assert source.index('if ! finalize_localization_bag; then') < source.index(handoff_marker)


def test_cleanup_closes_recorder_before_stopping_mapping_publishers():
    source = RUNNER.read_text(encoding="utf-8")
    cleanup = source[source.index("cleanup() {"):source.index("formal_runtime_install_traps cleanup")]
    recorder_stop = 'finalize_early_recording_audit'
    publisher_stop = '"${estop_pid}" "${power_pid}" "${collector_pid}" "${launch_pid}"'
    assert cleanup.index(recorder_stop) < cleanup.index(publisher_stop)


def test_recording_arm_refuses_missing_or_invalid_actual_writer_evidence():
    source = RUNNER.read_text(encoding="utf-8")
    assert 'formal_recording_invalid_receipt="${output}/formal_recording_invalid.json"' in source
    assert '"${output}/formal_localization_recording_invalid.json"' in source
    assert 'FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS' in source
    assert 'wait_for_recording_writer_open' in source
    assert 'kill -0 "${early_recording_pid}"' in source
    assert 'kill -0 "${localization_bag_pid}"' in source
