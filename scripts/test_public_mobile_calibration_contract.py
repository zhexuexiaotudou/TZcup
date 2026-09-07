from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_mapping_high_bandwidth_default_is_opt_in_and_cleaning_stays_true():
    source = (ROOT / "starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus_map_lifecycle.launch.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n            "mapping_high_bandwidth_sensor_runtime", default_value="false"' in source
    assert 'LaunchConfiguration("mapping_high_bandwidth_sensor_runtime")\n                    if mode == "mapping" else "true"' in source


def test_public_mobile_runner_reuses_lifecycle_operator_and_remains_read_only():
    source = (ROOT / "scripts/run_public_mobile_gazebo_dosod_calibration.sh").read_text(encoding="utf-8")
    for required in ("formal_campus_map_lifecycle.launch.py", "mission_mode:=mapping", "mapping_high_bandwidth_sensor_runtime:=true", "READINESS_DEADLINE_SEC=60", "require_mapping_readiness", "emergency_stop", "collect_formal_map_lifecycle_runtime.py", "--write-scene-selector", "--deactivate-scene-selector", "formal_runtime_cleanup_groups"):
        assert required in source
    for forbidden in ("cmd_vel", "send_goal", "cancel_goal", "SetEntityPose", "ground_truth", "evaluator", "hidden"):
        assert forbidden not in source
    assert "stop_verified || cleanup_failed=1" in source
    assert "operator-estop-rearm.log" in source
    assert "--once >\"$RUN_ROOT/operator_stop.log\"" not in source
    assert 'if [[ -n "$collector_pid" ]]; then wait "$collector_pid"' in source
    assert 'write_receipt "$state" "$receipt_code" "$survivor"' in source
    assert "receipt_code=125" in source
    assert 'trap \'RUNNER_EXIT_CODE=$?; formal_runtime_exit_trap "$RUNNER_EXIT_CODE"\' EXIT' in source
    assert source.count("stop_verified") >= 3


def test_public_mobile_runner_has_explicit_non_formal_fixed_scene_pilot_and_full_review_gate():
    source = (ROOT / "scripts/run_public_mobile_gazebo_dosod_calibration.sh").read_text(encoding="utf-8")
    assert 'MODE="${PUBLIC_GAZEBO_CALIBRATION_MODE:-full}"' in source
    assert 'PILOT_SCENE="map-0-mission-0"' in source
    assert 'pilot quota must be 25' in source
    assert 'PUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT' in source
    assert 'PUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST' in source
    assert 'NON_FORMAL_PILOT_CAPTURED' in source
    assert 'preflight_args+=(--review-receipt' in source


def test_mapping_mobile_rgb_opt_in_retains_the_live_mid360_safety_source():
    source = (ROOT / "starter_ws/src/sanitation_formal_campus_integration/sanitation_formal_campus_integration/nav2_mode_config.py").read_text(encoding="utf-8")
    assert "high_bandwidth_sensor_runtime: bool" in source
    assert 'parameters["observation_sources"] = ["scan"]' in source
    assert 'parameters.pop("mid360", None)' in source
    assert 'if mission_mode != "mapping":' in source
    assert '"high-bandwidth mapping requires an enabled mid360 source"' in source
