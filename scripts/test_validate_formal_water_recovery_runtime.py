import ast
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_water_recovery_plugin_is_mass_conserving_and_condition_gated() -> None:
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc"
    ).read_text(encoding="utf-8")
    assert "pumpRatedFlowLMin /" in source
    assert "hydraulicDerating *" in source
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
            "!this->filterProtectionActive",
        )
    )
    assert "std::clamp" in source
    assert "mass_balance_error_fraction" in source
    assert "squeegeeClearanceM" in source
    assert "intakeClearanceM" in source
    for evidence_field in (
        "cleaning_lift_position_m",
        "squeegee_float_position_m",
        "squeegee_pitch_position_rad",
        "base_world_z_m",
        "base_world_roll_rad",
        "base_world_pitch_rad",
        "base_pose_available",
        "base_pose_source",
    ):
        assert evidence_field in source
    assert "SelectBasePoseSource" in source
    assert "BasePoseSource::kBaseLink" in source
    assert "BasePoseSource::kBaseFootprint" in source
    assert "BasePoseSource::kModelEntity" in source
    assert "this->basePoseAvailable &&" in source
    assert "maximumNozzleHeightM" not in source
    assert 'Pose3d(0.040, 0, -0.005, 0, 0, 0)' in source
    cleaning = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
    ).read_text(encoding="utf-8")
    assert '<origin xyz="0.040 0 -0.005" rpy="0 0 0"/>' in cleaning


def test_filter_flow_sensor_and_tank_probes_have_runtime_behavior() -> None:
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc"
    ).read_text(encoding="utf-8")
    xacro = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    for token in (
        "command/filter_blockage_fraction",
        "filter_differential_pressure_kpa",
        "filter_protection_active",
        "sensed_flow_l_min",
        "sensed_tank_level_fraction",
        "tank_low_probe_wet",
        "tank_high_probe_wet",
        "pump_current_a",
        "command/service_drain_open",
        "service_drain_requested_open",
        "service_drain_open",
        "service_drain_permitted",
        "service_drained_volume_l",
    ):
        assert token in source
    assert "requestedBlockage * requestedBlockage" in source
    assert "1.0 - requestedBlockage * requestedBlockage" in source
    assert "std::exp(-dt / this->sensorTimeConstantS)" in source
    assert "<filter_trip_pressure_kpa>35.0</filter_trip_pressure_kpa>" in xacro
    assert "<tank_high_probe_fraction>0.875</tank_high_probe_fraction>" in xacro
    assert "<service_drain_rate_l_min>12.0</service_drain_rate_l_min>" in xacro
    assert "cumulativeServiceDrainedL * this->waterDensityKgL" in source
    assert "serviceDrainPermitted" in source
    assert "serviceDrainOpen = this->serviceDrainPermitted" in source
    assert "!this->serviceDrainOpen" in source
    assert "maximumWheelVelocity < this->maximumDrainWheelRadS" in source


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
    assert xacro.count("<minimum_lift_position_m>0.095</minimum_lift_position_m>") == 2
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
    assert "this->liftPositionM >= this->minimumLiftPositionM" in source


def test_runtime_acceptance_contains_required_positive_and_negative_gates() -> None:
    runner = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    assert '"/safety/status_json"' in runner
    assert "cleaning_motors/command/reset_faults" in runner
    assert "recover_latched_cleaning_motor_fault" in runner
    assert "bounded two-attempt" in runner
    assert "side_brush_duty_metrics" in runner
    assert "side_brush_steady_current_within_0_75_a_continuous_rating" in runner
    assert "side_brush_steady_p05_speed_ratio_at_least_0_80" in runner
    assert "side_brush_direction_matches_command" in runner
    assert "side_brush_fault_free_throughout_normal_pass" in runner
    assert "side_brush_fault_free_throughout_full_case" in runner
    assert "central_roller_duty_metrics" in runner
    assert "central_roller_steady_current_within_0_75_a_continuous_rating" in runner
    assert "central_roller_steady_p05_speed_ratio_at_least_0_80" in runner
    assert "central_roller_peak_temperature_below_60_c" in runner
    assert "central_roller_fault_free_throughout_normal_pass" in runner
    assert "central_roller_fault_free_throughout_full_case" in runner
    assert "fail_on_cleaning_motor_fault" in runner
    assert "applied_before_service_drain" in runner
    assert "service_drain_tank_mass_reduction_kg" in runner
    assert "service_drain_payload_mass_reduction_kg" in runner
    for gate in (
        "pump_without_brush_recovery_is_zero",
        "blocked_filter_stops_recovery",
        "blocked_filter_trips_pressure_protection",
        "recovery_rate_at_least_0_95",
        "ground_to_tank_mass_error_at_most_0_01",
        "pump_flow_within_rated_derated_limit",
        "tank_reaches_full",
        "active_recovery_blocks_service_drain",
        "service_drain_reduces_tank_mass",
        "service_drain_stationary_interlock_permitted",
        "service_drain_reports_removed_volume",
        "service_drain_tank_mass_matches_reported_volume",
        "service_drain_payload_mass_matches_tank_reduction",
        "service_drain_closes_and_updates_payload",
        "full_tank_stops_ground_removal",
        "dynamic_payload_applied_matches_full_tank",
        "visual_water_fraction_matches_ground_state",
        "raised_mechanism_is_not_recovery_ready",
        "lowered_intake_gap_is_physical",
        "raised_disabled_reverse_does_not_recover",
        "nozzle_covered_all_24_water_columns",
        "squeegee_blade_has_ground_contact_during_recovery",
        "brush_disks_have_ground_contact_during_recovery",
    ):
        assert gate in runner
    assert 'Twist, "/cmd_vel_gate"' in runner
    assert 'Odometry, "/odom/unfiltered"' in runner
    assert 'Float64MultiArray, "/safety/command/brush"' in runner
    assert 'Float64MultiArray, "/safety/command/pump"' in runner
    assert 'DiagnosticArray, "/safety/status"' in runner
    assert 'Bool, "/safety/actuators_enabled"' in runner
    assert "confirmed_actuator_permit" in runner
    assert "node.actuator_permit is True" in runner
    assert "CleaningLiftRecoverySupervisor" in runner
    assert "trajectory_duration_s" in runner
    assert '"safety_recovery_reissue"' in runner
    assert '"failure_diagnostics"' in runner
    assert "/base_controller/odom" not in runner
    launch_runner = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "enable_safety_manager:=true" in launch_runner
    assert "start_simulation_safety_inputs:=true" in launch_runner
    assert "start_power_system_simulators:=true" in launch_runner
    assert "squeegee_evaluation_interfaces:=true" in launch_runner
    assert "FORMAL_WATER_LAUNCH_SETTLE_S" in launch_runner
    assert "high_bandwidth_sensor_runtime:=false" in launch_runner
    assert "prepare_formal_preembedded_sensor_world.py" in launch_runner
    assert 'world:="${preembedded_world}" spawn_robot:=false' in launch_runner
    assert "water_${selected}_preembedded_sensor_world.sdf" in launch_runner
    assert "water_${selected}_preembedded_sensor_world.json" in launch_runner
    assert "collect_formal_water_safety_preflight.py" in launch_runner
    assert "--stable-duration-s 5.0" in launch_runner

    preflight = (ROOT / "scripts/collect_formal_water_safety_preflight.py").read_text(
        encoding="utf-8"
    )
    for evidence_field in (
        "arrival_monotonic_s",
        "arrival_unix_ns",
        "fault_active",
        "physics_update_stale",
        "safety_state",
        "safety_permit",
        "active_reasons",
    ):
        assert evidence_field in preflight
    assert "fault_topic_rate_at_least_18_hz" in preflight
    assert "fault_zero_true_in_stable_window" in preflight
    assert "physics_update_zero_stale_in_stable_window" in preflight
    assert "safety_permit_enabled_continuous_window" in preflight
    assert "actuator_permission_enabled_continuous_window" in preflight


def test_water_recovery_requires_live_blade_and_brush_ground_contacts() -> None:
    validator = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    cleaning = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
    ).read_text(encoding="utf-8")

    assert "from ros_gz_interfaces.msg import Contacts" in validator
    assert "def begin_recovery_contact_window" in validator
    assert "def recovery_ground_contact_evidence" in validator
    assert "ground_contact_observed" in validator
    for topic in (
        "/cleaning/squeegee/contact",
        "/cleaning/left_side_brush/contact",
        "/cleaning/right_side_brush/contact",
        "/cleaning/central_roller/contact",
    ):
        assert topic in validator
        assert f"{topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts" in launch
    for sensor, collision in (
        ("${side}_side_brush_ground_contact", "${side}_side_brush_link_collision"),
        ("central_roller_ground_contact", "central_roller_link_collision"),
    ):
        assert sensor in cleaning
        assert f"<collision>{collision}</collision>" in cleaning


def test_normal_and_full_pass_paths_never_reset_a_motor_fault() -> None:
    path = ROOT / "scripts/validate_formal_water_recovery_runtime.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    functions = {
        node.name: node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    for name in ("run_normal", "run_full"):
        calls = [node for node in ast.walk(functions[name]) if isinstance(node, ast.Call)]
        called_names = {
            node.func.id
            for node in calls
            if isinstance(node.func, ast.Name)
        }
        called_attributes = {
            node.func.attr
            for node in calls
            if isinstance(node.func, ast.Attribute)
        }
        assert "recover_latched_cleaning_motor_fault" not in called_names
        assert "publish_motor_fault_reset" not in called_attributes


def test_runtime_reverse_uses_formal_safety_power_and_a300_drivetrain_chain() -> None:
    validator = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    safety_manager = (
        ROOT
        / "starter_ws/src/sanitation_safety/sanitation_safety/whole_vehicle_safety_manager.py"
    ).read_text(encoding="utf-8")
    simulation_inputs = (
        ROOT
        / "starter_ws/src/sanitation_safety/sanitation_safety/simulation_safety_inputs.py"
    ).read_text(encoding="utf-8")
    bms = (
        ROOT
        / "starter_ws/src/sanitation_power_system/sanitation_power_system/a300_bms_node.py"
    ).read_text(encoding="utf-8")
    adapter = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainCommandAdapter.cc"
    ).read_text(encoding="utf-8")
    plant = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainPlantCore.cc"
    ).read_text(encoding="utf-8")

    # The probe may command only the formal safety ingress and must observe
    # physical drivetrain odometry; it cannot reposition the model directly.
    assert 'Twist, "/cmd_vel_gate"' in validator
    assert 'Odometry, "/odom/unfiltered"' in validator
    assert "set_pose" not in validator.lower()

    assert '"command_input_topic", "/cmd_vel_gate"' in safety_manager
    assert '"base_command_output_topic", "/base_controller/cmd_vel"' in safety_manager
    assert '"actuator_enable_topic", "/safety/actuators_enabled"' in safety_manager
    for required_input in (
        '"safety_relay_topic", "/safety/relay_enabled"',
        '"bms_fault_topic", "/formal_vehicle/power/bms_fault"',
        '"traction_permitted_topic", "/formal_vehicle/power/traction_permitted"',
    ):
        assert required_input in safety_manager

    assert '"safe_command_input_topic", "/base_controller/cmd_vel"' in adapter
    assert '"safety_enable_input_topic", "/safety/actuators_enabled"' in adapter
    assert "const bool permitted = commandFresh && enableFresh" in adapter
    assert "this->commandValid && this->safetyEnable" in adapter
    for topic in (
        "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/cmd_vel",
        "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/actuator_enable",
    ):
        assert topic in adapter
        assert topic in launch
    assert 'executable="a300_drivetrain_command_adapter"' in launch
    odom_remap = (
        '(\n                '
        '"/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom",\n'
        '                "/odom/unfiltered",'
    )
    assert odom_remap in launch
    for fail_closed_input in (
        "input.emergency_stop",
        "!input.actuator_enable",
        "input.command_age_s > parameters_.command_timeout_s",
    ):
        assert fail_closed_input in plant

    # Simulation safety owns relay/contactor readiness and consumes the real
    # A300 simulator's battery state. The launch conditions keep the BMS and
    # charge manager coupled to the simulation-safety opt-in by default.
    assert "BatteryState" in simulation_inputs
    assert '"/formal_vehicle/power/battery_state"' in simulation_inputs
    assert 'Bool, "/safety/relay_enabled"' in simulation_inputs
    for bms_output in (
        'BatteryState, "/formal_vehicle/power/battery_state"',
        'Bool, "/formal_vehicle/power/bms_fault"',
        'Bool, "/formal_vehicle/power/traction_permitted"',
    ):
        assert bms_output in bms
    assert 'default_value=start_simulation_safety_inputs' in launch
    assert launch.count("condition=IfCondition(start_power_system_simulators)") == 2
    assert "condition=IfCondition(start_simulation_safety_inputs)" in launch


def test_runtime_physics_waits_on_sim_time_not_fixed_wall_duration() -> None:
    path = ROOT / "scripts/validate_formal_water_recovery_runtime.py"
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = {
        node.name: ast.get_source_segment(source, node) or ""
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    assert "def wait_for_sim_condition" in source
    assert "simulation clock stalled" in source
    assert "SIM_CLOCK_STALL_WALL_S = 45.0" in source
    assert "NORMAL_PASS_TIMEOUT_SIM_S = 90.0" in source
    assert "FULL_TANK_TIMEOUT_SIM_S = 30.0" in source
    for name in ("reset_episode", "reverse_to_water_start", "run_normal", "run_full"):
        assert "time.monotonic" not in functions[name]
        assert "wait_for_sim_condition" in functions[name] or "advance_sim_time" in functions[name]


def test_reset_and_full_tank_wait_for_real_plugin_state() -> None:
    source = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    functions = {
        node.name: ast.get_source_segment(source, node) or ""
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
    }
    reset = functions["reset_episode"]
    assert "for _ in range(10)" not in reset
    assert 'node.status["ground_volume_l"]' in reset
    assert 'node.status["tank_mass_kg"]' in reset
    assert "callback=lambda: node.publish_reset(ground_l, tank_kg)" in reset

    full = functions["run_full"]
    assert 'bool(node.status["tank_full"])' in full
    assert "FULL_TANK_TIMEOUT_SIM_S" in full
    assert '"tank_mass_clamped_to_capacity"' in full
    assert "TANK_CAPACITY_KG) <= 1e-5" in full


def test_runtime_fails_closed_when_physical_base_pose_is_unavailable() -> None:
    source = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    assert 'node.status.get("base_pose_available") is not True' in source
    assert '"physical base pose is unavailable: "' in source
    assert '"terminal_base_pose_source"' in source


def test_lift_diagnostic_retains_every_safety_transition_cause() -> None:
    validator = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    runner = (ROOT / "scripts/run_formal_water_recovery_runtime.sh").read_text(
        encoding="utf-8"
    )
    for evidence in (
        "safety_transition_history",
        "arrival_monotonic_s",
        "sim_time_s",
        "managed_controllers_active",
        "position_hold_ready",
        "front_bumper_contact",
        "rear_bumper_contact",
        "safety_relay_available",
        "bms_fault_available",
        "cleaning_motor_fault_available",
        "traction_permit_available",
        "physics_update_stale",
        "bumper_inputs",
    ):
        assert evidence in validator
    assert 'choices=("normal", "full", "diagnostic")' in validator
    assert "def wait_for_safety_permit_after_reset" in validator
    assert 'label=f"{scenario} safety permit recovery after preflight handoff"' in validator
    assert 'wait_for_safety_permit_after_reset(node, scenario="normal")' in validator
    assert 'wait_for_safety_permit_after_reset(node, scenario="full")' in validator
    assert 'wait_for_safety_permit_after_reset(node, scenario="diagnostic")' in validator
    assert "initial_status_count = node.safety_status_json_count" in validator
    assert "node.safety_status_json_count > initial_status_count" in validator
    assert "node.safety_json_permit is True" in validator
    assert "safety_handshake_events" in validator
    assert 'run_scenario diagnostic' in runner


def test_every_water_scenario_waits_for_a_fresh_safety_permit_before_lowering() -> None:
    path = ROOT / "scripts/validate_formal_water_recovery_runtime.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    functions = {
        node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)
    }

    for function_name in ("run_normal", "run_full", "run_lift_diagnostic"):
        calls: dict[str, list[int]] = {}
        for node in ast.walk(functions[function_name]):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                calls.setdefault(node.func.id, []).append(node.lineno)
        reset_line = min(calls["reset_episode"])
        permit_line = min(calls["wait_for_safety_permit_after_reset"])
        lower_line = min(calls["lower_until_geometry_ready"])
        assert reset_line < permit_line < lower_line


def test_full_case_uses_the_8_30kg_limit_and_has_physical_headroom() -> None:
    validator = (
        ROOT / "scripts/validate_formal_water_recovery_runtime.py"
    ).read_text(encoding="utf-8")
    vehicle = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    storage = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro"
    ).read_text(encoding="utf-8")
    water_plugin = (
        ROOT / "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc"
    ).read_text(encoding="utf-8")
    payload_plugin = (
        ROOT / "starter_ws/src/sanitation_gazebo_control/src/DynamicPayloadSystem.cc"
    ).read_text(encoding="utf-8")

    assert "TANK_CAPACITY_KG = 8.30" in validator
    assert "initial_tank = TANK_CAPACITY_KG - 0.0564" in validator
    assert "reset_episode(node, 0.40, initial_tank)" in validator
    assert "<wastewater_capacity_kg>8.30</wastewater_capacity_kg>" in vehicle
    assert "<tank_capacity_kg>8.30</tank_capacity_kg>" in vehicle
    assert "float(wastewater_load_mass_kg), 8.30" in storage
    assert "double tankCapacityKg{8.30}" in water_plugin
    assert "double waterCapacityKg{8.30}" in payload_plugin

    capacity_l = 8.30
    initial_tank_l = capacity_l - 0.0564
    initial_ground_l = 0.40
    recovered_to_full_l = capacity_l - initial_tank_l
    assert recovered_to_full_l == pytest.approx(0.0564)
    assert initial_ground_l - recovered_to_full_l == pytest.approx(0.3436)


def test_a300_load_path_does_not_claim_compliant_suspension() -> None:
    a300 = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/a300_platform.xacro"
    ).read_text(encoding="utf-8")
    assert 'name="${side}_suspension_beam_spacer_joint" type="fixed"' in a300
    assert 'name="${side}_suspension_beam_joint" type="fixed"' in a300
    assert "model therefore makes no unsupported suspension-compliance claim" in a300


def test_normal_thresholds_are_not_relaxed_for_low_real_time_factor() -> None:
    source = (ROOT / "scripts/validate_formal_water_recovery_runtime.py").read_text(
        encoding="utf-8"
    )
    assert "recovery_rate >= 0.95" in source
    assert "len(covered_columns) == 24" in source
    assert "mass_error <= 0.01" in source
    assert "recovered / initial_ground >= 0.955" in source
    assert 'node.publish_filter_blockage(1.0)' in source
    assert 'node.publish_filter_blockage(0.0)' in source
    assert "from rclpy.qos import qos_profile_clock" in source
    assert 'Clock, "/clock", self._on_clock, qos_profile_clock' in source
