from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_runner_launches_real_formal_vehicle_and_runtime_probe() -> None:
    source = (ROOT / "scripts" / "run_formal_vehicle_mobility_runtime.sh").read_text(encoding="utf-8")
    assert "formal_vehicle_sim.launch.py" in source
    assert "validate_formal_vehicle_mobility_runtime.py" in source
    assert "gui:=false" in source
    assert "--forward-speed 0.25" in source
    assert "--forward-duration 4.0" in source
    assert "setsid ros2 launch" in source


def test_probe_reads_named_ground_truth_from_gazebo_transport() -> None:
    source = (ROOT / "scripts" / "validate_formal_vehicle_mobility_runtime.py").read_text(encoding="utf-8")
    assert '"/world/formal_vehicle_validation/pose/info"' in source
    assert "read_gazebo_ground_truth()" in source
    assert 'MODEL_NAME = "tzcup_formal_sanitation_vehicle"' in source
    assert 'TwistStamped, "/base_controller/cmd_vel"' in source
