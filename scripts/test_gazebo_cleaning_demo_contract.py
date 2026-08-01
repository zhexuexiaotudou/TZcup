from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_gazebo_visualizer_package_contract() -> None:
    source = read(
        "starter_ws/src/sanitation_gazebo_visualization/"
        "sanitation_gazebo_visualization/cleaning_visualizer.py"
    )
    package = read("starter_ws/src/sanitation_gazebo_visualization/package.xml")
    setup = read("starter_ws/src/sanitation_gazebo_visualization/setup.py")

    for topic in (
        '"/ground_truth/odom"',
        '"/brush_enabled"',
        '"/coverage/state"',
        '"/coverage/component_state"',
        '"/coverage/current_path"',
    ):
        assert topic in source
    for marker_type in ("type:LINE_STRIP", "type:TEXT", "type:BOX", "type:SPHERE"):
        assert marker_type in source
    assert '"/marker"' in source
    assert '"gz.msgs.Marker"' in source
    assert '"gz.msgs.Empty"' in source
    assert "service_timeout_ms\", 3000" in source
    assert "pending_kinds" in source
    assert "trail_points" in source
    assert "tzcup_cleaning_zone" in source
    assert "tzcup_cleaning_home" in source
    assert "tzcup_cleaning_start" in source
    assert "ASSIGNED CLEANING AREA" in source
    assert '"z:0.55} orientation {w:1.0}} "' in source
    assert '"z:0.55}} orientation {w:1.0}} "' not in source
    assert "GOING HOME -> CLEANING START" in source
    assert 'ns:\\"tzcup_current_cleaning_path\\\" id:1' in source
    assert 'ns:\\"tzcup_cleaning_status\\\" id:1' in source
    assert 'ns:\\"tzcup_current_cleaning_path\\\" id:0' not in source
    assert 'ns:\\"tzcup_cleaning_status\\\" id:0' not in source
    assert "visibility:GUI" in source
    assert "world_to_map_x" in source
    assert "world_to_map_y" in source
    assert "world_to_map_yaw" in source
    assert '"/cmd_vel"' not in source
    assert "<exec_depend>rclpy</exec_depend>" in package
    assert "<exec_depend>nav_msgs</exec_depend>" in package
    assert "<exec_depend>std_msgs</exec_depend>" in package
    assert "<exec_depend>python3-yaml</exec_depend>" in package
    assert "<build_type>ament_python</build_type>" in package
    assert "cleaning_visualizer = sanitation_gazebo_visualization" in setup
    assert "gz-transport" not in package
    assert "gz-msgs" not in package
    assert "subprocess.run(" in source


def test_gazebo_only_launcher_contract() -> None:
    shell_launcher = read("scripts/run_visual_demo.sh")
    powershell_launcher = read("scripts/run_visual_demo.ps1")
    dedicated_launcher = read("scripts/run_gazebo_cleaning_demo.ps1")

    assert "--gazebo-only" in shell_launcher
    assert "--showcase" in shell_launcher
    assert "showcase_area.yaml" in shell_launcher
    assert "follow_offset: {x: -8.0, y: -8.0, z: 10.0}" in shell_launcher
    assert "OPEN_DASHBOARD=0" in shell_launcher
    assert "sanitation_gazebo_visualization cleaning_visualizer" in shell_launcher
    assert '[[ "${OPEN_DASHBOARD}" -eq 1 ]]' in shell_launcher
    assert "keep_open_stop=1" in shell_launcher
    assert "[switch]$GazeboOnly" in powershell_launcher
    assert '"--gazebo-only"' in powershell_launcher
    assert "[switch]$Showcase" in powershell_launcher
    assert '"--showcase"' in powershell_launcher
    assert "GazeboOnly = $true" in dedicated_launcher
    assert "[switch]$FullArea" in dedicated_launcher
    assert 'Showcase"] = $true' in dedicated_launcher
    assert "NoRviz" not in dedicated_launcher
    assert "Start-Process" not in dedicated_launcher


def test_operator_docs_name_the_gazebo_only_entry() -> None:
    command = "scripts\\run_gazebo_cleaning_demo.ps1"
    assert command in read("README.md")
    assert command in read("README_FIRST.md")
    assert command in read("docs/auto17-visual-demo.md")
    assert "青绿色" in read("README.md")
    assert "琥珀色" in read("README.md")


def test_showcase_area_is_small_bounded_and_installed() -> None:
    showcase = read("starter_ws/src/sanitation_tasks/config/showcase_area.yaml")
    task_setup = read("starter_ws/src/sanitation_tasks/setup.py")

    assert "mission_id: showcase_coverage_001" in showcase
    assert "expected_components: 9" in showcase
    assert "  - [-3.0, -4.0]" in showcase
    assert "  - [3.0, 1.0]" in showcase
    assert "keepout_polygons: []" in showcase
    assert '"config/showcase_area.yaml"' in task_setup
