from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_formal_first_map_dynamic_prerequisite.sh"


def test_first_map_runner_captures_fail_closed_read_only_localization_mcap():
    source = RUNNER.read_text(encoding="utf-8")
    assert "ros2 bag record --storage mcap" in source
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


def test_blocked_or_unreaped_recorder_cannot_fall_through_to_handoff():
    source = RUNNER.read_text(encoding="utf-8")
    assert 'if python3 "${repo_root}/scripts/finalize_formal_first_map_localization_diagnostic.py"' in source
    assert 'return "${finalizer_rc}"' in source
    assert 'kill -0 "${bag_pid}"' in source
    assert 'wait "${bag_pid}"' in source
    handoff_marker = 'python3 - ' + '\\' + '\n      "${handoff_record}"'
    assert source.index('if ! finalize_localization_bag; then') < source.index(handoff_marker)
