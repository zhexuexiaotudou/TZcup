from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_dynamic_sensor_diagnostic_is_fail_closed_and_never_formal() -> None:
    source = (
        ROOT / "scripts/collect_formal_dynamic_sensor_diagnostic.py"
    ).read_text(encoding="utf-8")
    assert '"formal_eligible": False' in source
    assert '"spawn_mode": "dynamic_usercommands"' in source
    assert "ACTIVE_CONTROLLERS" in source
    assert "INACTIVE_CONTROLLERS" in source
    assert "controller_plane_ready_before_sensor_subscriptions" in source
    assert "validate_runtime_contract" in source
    assert "all_sensor_subscriptions_retired_after_bounded_evidence" in source
    assert "temporary.replace(output)" in source


def test_control_diagnostic_supports_dynamic_spawn_without_formal_report() -> None:
    source = (
        ROOT / "scripts/run_formal_preembedded_control_diagnostic.sh"
    ).read_text(encoding="utf-8")
    assert '"${mode}" == "dynamic"' in source
    assert "spawn_robot=true" in source
    assert 'world="${package_share}/worlds/formal_vehicle_validation.sdf"' in source


def test_dynamic_sensor_runner_is_isolated_guarded_and_high_bandwidth() -> None:
    source = (
        ROOT / "scripts/run_formal_dynamic_sensor_diagnostic.sh"
    ).read_text(encoding="utf-8")
    assert "collect_formal_dynamic_sensor_diagnostic.py" in source
    assert "formal_runtime_memory_preflight" in source
    assert "formal_runtime_start_memory_watchdog" in source
    assert "formal_runtime_cleanup_groups" in source
    assert "spawn_robot:=true" in source
    assert "headless_rendering:=true" in source
    assert "high_bandwidth_sensor_runtime:=true" in source
    assert "DYNAMIC_SENSOR_DIAGNOSTIC_CONTROL_CRASHED" in source
