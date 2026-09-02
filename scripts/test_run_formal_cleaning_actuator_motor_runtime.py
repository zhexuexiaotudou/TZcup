from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_runner_requires_frozen_overlay_and_runs_real_collector_then_validator():
    runner = (
        ROOT / "scripts/run_formal_cleaning_actuator_motor_runtime.sh"
    ).read_text(encoding="utf-8")
    assert "FORMAL_VEHICLE_RUNTIME_WS:?" in runner
    assert "formal_vehicle_sim.launch.py" in runner
    assert "--exercise-live" in runner
    assert "--snapshot-manifest" in runner
    assert "formal_runtime_gate_binding.py" in runner
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in runner
    assert "FORMAL_ACCEPTANCE_SESSION" in runner
    assert "FORMAL_CLEANING_MOTOR_RUNTIME_BINDING" in runner
    assert 'formal_runtime_register_evidence_paths "${output}" "${raw}" "${runtime_binding}"' in runner
    assert 'mv -- "${retained}" "${superseded}"' in runner
    assert '[[ -e "${retained}" || -L "${retained}" ]]' in runner
    assert "formal_source_bound_preflight.sh" in runner
    assert 'formal_source_bound_verify_overlay "${runtime_ws}/install"' in runner
    assert runner.index("formal_runtime_gate_binding.py") < runner.index(
        'ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py'
    )
    assert "collect_formal_cleaning_actuator_motor_runtime.py" in runner
    assert "validate_formal_cleaning_actuator_motor_runtime.py" in runner
    assert "ros2 topic pub" not in runner
    assert "enable_safety_manager:=true" in runner
    assert "start_simulation_safety_inputs:=true" in runner
    assert "start_power_system_simulators:=true" in runner
    assert "check_formal_water_preoperational_readiness.py" in runner
    assert "collect_formal_water_safety_preflight.py" in runner
    assert runner.index("collect_formal_water_safety_preflight.py") < runner.index(
        "collect_formal_cleaning_actuator_motor_runtime.py"
    )


def test_live_collector_uses_controller_commands_not_joint_state_mutation():
    collector = (
        ROOT / "scripts/collect_formal_cleaning_actuator_motor_runtime.py"
    ).read_text(encoding="utf-8")
    assert '"/safety/command/brush"' in collector
    assert '"/safety/command/pump"' in collector
    assert '"/cleaning_controller/joint_trajectory"' in collector
    assert "STALL_REFERENCE_M = 0.125" in collector
    assert "LIFT_TRAVEL_UPPER_M = 0.100" in collector
    assert "LIFT_TRAVEL_APPROACH_M = 0.0995" in collector
    assert "LIFT_TRAVEL_APPROACH_TIMEOUT_S = 155.0" in collector
    assert "STALL_LATCH_TIMEOUT_S = 6.5" in collector
    assert "RECOVERY_LIFT_TARGET_M = 0.060" in collector
    assert "RECOVERY_LIFT_TIMEOUT_S = 35.0" in collector
    assert "wall-clock observation budget, not simulated time" in collector
    assert "predicate=_near_lift_travel_stop" in collector
    assert "predicate=_stall_and_inhibit" in collector
    assert '"fault": bool(status["fault_active"])' in collector
    assert '"lift_reference_m": float(motors[3]["command"])' in collector
    assert '"motor_lift_command_m": float(motors[3]["command"])' in collector
    assert '"controller_lift_reference_m"' in collector
    assert '"lift_position_m": float(motors[3]["measured_position"])' in collector
    assert '"physical_travel_stop_stall", 0.25' in collector
    assert "recovery_retract" in collector
    assert "recovery_reference_arm" in collector
    assert "_recovery_reference_armed" in collector
    assert "recovery lift reference was not armed before explicit reset" in collector
    assert "predicate=_lift_retracted_and_idle" in collector
    assert "create_publisher(JointState" not in collector
    assert '"joint_state_mutation_used": False' in collector
    assert '"live_overtemperature_claimed": False' in collector
    assert "get_publishers_info_by_topic" in collector
    assert '"cleaning_vector_bridge_graph"' in collector


def test_runner_does_not_generate_snapshot_or_change_motor_parameters():
    runner = (
        ROOT / "scripts/run_formal_cleaning_actuator_motor_runtime.sh"
    ).read_text(encoding="utf-8")
    assert "generate_formal_vehicle_snapshot.py" in runner
    assert "--check --output \"${snapshot_manifest}\"" in runner
    assert "thermal_time_constant" not in runner
    assert "overtemperature_trip" not in runner
