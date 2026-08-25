from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_water_recovery_plugin_is_mass_conserving_and_condition_gated() -> None:
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc"
    ).read_text(encoding="utf-8")
    assert "pumpRatedFlowLMin /" in source
    assert "hydraulicDerating * dt" in source
    assert "cell.volumeL -= removedL" in source
    assert "tankMassKg += recoveredThisStepL * this->waterDensityKgL" in source
    assert all(
        token in source
        for token in (
            "brushReady",
            "squeegeeReady",
            "nozzleReady",
            "pumpReady",
            "!this->tankFull",
        )
    )
    assert "std::clamp" in source
    assert "mass_balance_error_fraction" in source
    assert "squeegeeClearanceM" in source
    assert "intakeClearanceM" in source
    assert "maximumNozzleHeightM" not in source
    assert 'Pose3d(0.040, 0, -0.005, 0, 0, 0)' in source
    cleaning = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
    ).read_text(encoding="utf-8")
    assert '<origin xyz="0.040 0 -0.005" rpy="0 0 0"/>' in cleaning


def test_formal_vehicle_registers_plugin_world_and_ros_bridges() -> None:
    xacro = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    world = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/worlds/formal_vehicle_validation.sdf"
    ).read_text(encoding="utf-8")
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert "libWaterRecoverySystem.so" in xacro
    assert "<pump_rated_flow_l_min>15.1</pump_rated_flow_l_min>" in xacro
    assert "<hydraulic_derating>0.70</hydraulic_derating>" in xacro
    assert "formal_recoverable_water_patch" in world
    assert world.count('visual name="water_strip_') == 24
    for topic in (
        "tank_mass_kg",
        "tank_level_fraction",
        "flow_l_min",
        "recovered_volume_l",
        "tank_full",
    ):
        assert f"water_recovery/{topic}" in launch
    assert 'water_evaluation_interfaces = LaunchConfiguration(' in launch
    assert 'DeclareLaunchArgument(\n                "water_evaluation_interfaces"' in launch
    assert 'default_value="false"' in launch
    assert "condition=IfCondition(water_evaluation_interfaces)" in launch
    for evaluator_topic in (
        "command/reset_ground_volume_l",
        "command/reset_tank_mass_kg",
        "ground_volume_l",
        "mass_balance_error_fraction",
        "status_json",
    ):
        assert launch.count(f"water_recovery/{evaluator_topic}") == 1
    source = (
        ROOT / "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc"
    ).read_text(encoding="utf-8")
    assert 'scoped.find("formal_recoverable_water_patch")' in source
    assert "this->visualLayoutReady)" in source


def test_runtime_acceptance_contains_required_positive_and_negative_gates() -> None:
    runner = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    for gate in (
        "pump_without_brush_recovery_is_zero",
        "recovery_rate_at_least_0_95",
        "ground_to_tank_mass_error_at_most_0_01",
        "pump_flow_within_rated_derated_limit",
        "tank_reaches_full",
        "full_tank_stops_ground_removal",
        "dynamic_payload_applied_matches_full_tank",
        "visual_water_fraction_matches_ground_state",
        "raised_mechanism_is_not_recovery_ready",
        "lowered_intake_gap_is_physical",
        "raised_disabled_reverse_does_not_recover",
        "nozzle_covered_all_24_water_columns",
    ):
        assert gate in runner
