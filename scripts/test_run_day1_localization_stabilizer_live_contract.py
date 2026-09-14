from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_live_harness_is_serialized_and_fail_closed():
    source = (ROOT / "scripts/run_day1_localization_stabilizer_live.sh").read_text()
    assert 'PROBE_DOMAIN="${PROBE_DOMAIN:-83}"' in source
    assert (
        'PROBE_PARTITION="${PROBE_PARTITION:-tzcup_localization_stabilizer_20260915_01}"'
        in source
    )
    assert 'formal_runtime_configure "$ROS_DOMAIN_ID"' in source
    assert "sanitation_localization map_odom_stabilizer" in source
    assert "formal_localization_runtime_collector" in source
    assert "mkdir \"$OUTPUT\"" in source
    assert "map_odom_stabilizer:=true" in source
    assert 'PROBE_MAP_ODOM_BOOTSTRAP_SECONDS="${PROBE_MAP_ODOM_BOOTSTRAP_SECONDS:-120}"' in source
    assert "--map-odom-bootstrap-timeout" in source
    assert "--map-odom-owner /map_odom_stabilizer" in source
    assert "formal_runtime_cleanup_groups" in source
    assert "resource_release.json" in source
    assert "live_candidate_receipt.json" in source
    assert "ros2 topic echo --full-length --once" in source
    assert (
        'LOCALIZATION_SCORE_DEPS_DIR="${LOCALIZATION_SCORE_DEPS_DIR:-'
        '$SOURCE/.work/localization-score-deps}"' in source
    )
    assert "import mcap, mcap_ros2" in source
    assert 'echo "$driver_status" >"$OUTPUT/driver.rc"' in source
    assert 'echo "$focus_status" >"$OUTPUT/focus.rc"' in source
    assert 'echo "$validator_status" >"$OUTPUT/validator.rc"' in source
    assert source.index('echo "$driver_status" >"$OUTPUT/driver.rc"') < source.index(
        "validate_day1_localization_stabilizer_live.py"
    )


def test_runtime_sources_do_not_use_ground_truth_for_control():
    files = [
        ROOT
        / "starter_ws/src/sanitation_localization/sanitation_localization/map_odom_stabilizer.py",
        ROOT
        / "starter_ws/src/sanitation_localization/sanitation_localization/map_odom_stabilizer_core.py",
    ]
    for path in files:
        source = path.read_text()
        for forbidden in (
            "/ground_truth",
            "/world",
            "/gazebo",
            "/model",
            "simulation/reference",
        ):
            assert forbidden not in source
