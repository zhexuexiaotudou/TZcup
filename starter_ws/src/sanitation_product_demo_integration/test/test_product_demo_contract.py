import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_product_demo_composes_one_planner_real_perception_and_fail_closed_power():
    source = (PACKAGE / "launch/product_demo.launch.py").read_text(encoding="utf-8")
    ast.parse(source)
    assert 'FindPackageShare("sanitation_formal_campus_integration")' in source
    assert 'FindPackageShare("sanitation_perception")' in source
    assert 'FindPackageShare("sanitation_active_cleaning")' in source
    assert 'FindPackageShare("sanitation_manipulation")' in source
    assert '"start_coverage": "false"' in source
    assert '"formal_pc_open_vocab.launch.py"' in source
    assert '"formal_active_cleaning.launch.py"' in source
    assert '"formal_physical_grasp.launch.py"' in source
    assert '"maximum_task_distance_m": LaunchConfiguration(' in source
    assert '"episode_seed": LaunchConfiguration("episode_seed")' in source
    assert 'period=50.0' in source
    assert 'period=55.0' in source
    assert 'period=58.0' in source
    assert 'executable="simulation_operator_gate"' in source
    assert '"formal_campus_map_lifecycle.launch.py"' in source
    assert '"mission_mode": "cleaning"' in source
    assert '"cleaning_planner": "rl_dirt_priority"' in source
    assert '"map_artifact_dir": runtime_root' in source
    assert '"saved_map_artifact_dir"' in source
    assert '"formal_campus.launch.py"' not in source
    assert "OpaqueFunction(function=_validate_product_inputs)" in source
    assert "if not artifact_root.is_dir():" in source
    assert "if not checkpoint.is_file():" in source
    assert "not math.isfinite(maximum_distance) or maximum_distance <= 0.0" in source


def test_operator_gate_repeats_power_estop_and_stops_after_returned_mission():
    source = (
        PACKAGE
        / "sanitation_product_demo_integration/simulation_operator_gate.py"
    ).read_text(encoding="utf-8")
    ast.parse(source)
    assert '"/product_demo/operator_start"' in source
    assert '"/active_cleaning/mission_complete"' in source
    assert '"/formal_vehicle/simulation/command/main_power"' in source
    assert '"/formal_vehicle/simulation/command/emergency_stop"' in source
    assert "mission_complete_safe_stop" in source


def test_top_level_package_breaks_the_formal_training_dependency_cycle():
    campus_package = (
        PACKAGE.parent / "sanitation_formal_campus_integration/package.xml"
    ).read_text(encoding="utf-8")
    assert "<exec_depend>sanitation_active_cleaning</exec_depend>" not in campus_package
    own_package = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>sanitation_active_cleaning</exec_depend>" in own_package
    assert "<exec_depend>sanitation_formal_campus_integration</exec_depend>" in own_package
