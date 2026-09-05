from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_runner_launches_real_formal_vehicle_and_runtime_probe() -> None:
    source = (ROOT / "scripts" / "run_formal_vehicle_mobility_runtime.sh").read_text(encoding="utf-8")
    assert "formal_vehicle_sim.launch.py" in source
    assert "validate_formal_vehicle_mobility_runtime.py" in source
    assert "gui:=false" in source
    assert "--forward-speed 0.25" in source
    assert "--forward-duration 4.0" in source
    assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch' in source
    assert "enable_safety_manager:=true" in source
    assert "simulation_initial_estop_active:=false" in source
    assert "formal_a300_drivetrain_runtime.json" in source
    # Canonical mobility evidence is bound before Gazebo starts; a local
    # standalone runtime result cannot be spliced into the final session.
    assert ".work/final_frozen_runtime" in source
    assert "formal_runtime_gate_binding.py" in source
    assert "FORMAL_ACCEPTANCE_SESSION" in source
    assert '"${runtime_binding}"' in source
    assert source.index("formal_runtime_gate_binding.py") < source.index(
        'ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py'
    )


def test_probe_reads_named_ground_truth_from_gazebo_transport() -> None:
    source = (ROOT / "scripts" / "validate_formal_vehicle_mobility_runtime.py").read_text(encoding="utf-8")
    helper = (ROOT / "scripts" / "gazebo_ground_truth.py").read_text(encoding="utf-8")
    assert 'world_name="formal_vehicle_validation"' in source
    assert 'f"/world/{world_name}/pose/info"' in helper
    assert "read_gazebo_ground_truth()" in source
    assert 'MODEL_NAME = "tzcup_formal_sanitation_vehicle"' in source
    assert 'Twist, "/cmd_vel_gate"' in source
    assert 'Odometry, "/odom/unfiltered"' in source
    assert 'Bool, "/safety/actuators_enabled"' in source
    assert '"stopped_angular_velocity_rad_s"' in source
    assert "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_PASSED" in source
    assert "acceptance_session_binding" in source
    assert "runtime_gate_binding" in source
