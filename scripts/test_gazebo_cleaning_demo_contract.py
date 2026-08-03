from pathlib import Path

import yaml


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
    for marker_type in ("type:LINE_STRIP", "type:BOX"):
        assert marker_type in source
    assert "type:SPHERE" not in source
    assert "scale {x:0.14 y:0.14 z:0.018}" in source
    assert "type:TEXT" not in source, "Ogre2 MarkerManager does not support text markers reliably"
    assert '"/marker"' in source
    assert '"gz.msgs.Marker"' in source
    assert '"gz.msgs.Empty"' in source
    assert "service_timeout_ms\", 3000" in source
    assert "pending_kinds" in source
    assert "trail_points" in source
    assert "tzcup_cleaning_zone" in source
    assert "tzcup_cleaning_home" in source
    assert "tzcup_cleaning_start" in source
    assert 'ns:\\"tzcup_current_cleaning_path\\\" id:1' in source
    assert 'ns:\\"tzcup_current_cleaning_path\\\" id:0' not in source
    assert "visibility:GUI" in source
    assert "world_to_map_x" in source
    assert "world_to_map_y" in source
    assert "world_to_map_yaw" in source
    assert '"/coverage/gazebo_telemetry"' in source
    assert "gazebo_ground_truth_brush_footprint_evaluation_only" in source
    assert "cleaned_cells" in source
    assert "cleaning_targets" in source
    assert '"gz.msgs.Entity"' in source
    assert "_call_remove_service" in source
    assert "removed_from_scene" in source
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
    assert "--map-size" in shell_launcher
    assert "--simulation-speed" in shell_launcher
    assert 'ros2 lifecycle get /controller_server' in shell_launcher
    assert 'ros2 lifecycle get /planner_server' in shell_launcher
    assert "tf2_echo odom base_footprint" in shell_launcher
    assert "localization_readiness_tf.txt" in shell_launcher
    assert 'grep -Fxq \'active [3]\' <<< "${controller_state}"' in shell_launcher
    assert 'grep -Fxq \'active [3]\' <<< "${planner_state}"' in shell_launcher
    assert "--manual-control" in shell_launcher
    assert "showcase_area.yaml" in shell_launcher
    assert "follow_offset: {x: -8.0, y: -8.0, z: 10.0}" in shell_launcher
    assert "OPEN_DASHBOARD=0" in shell_launcher
    assert "sanitation_gazebo_visualization cleaning_visualizer" in shell_launcher
    assert '[[ "${OPEN_DASHBOARD}" -eq 1 ]]' in shell_launcher
    assert "keep_open_stop=1" in shell_launcher
    assert "--wait-matching-subscriptions 1" in shell_launcher
    assert "[switch]$GazeboOnly" in powershell_launcher
    assert '"--gazebo-only"' in powershell_launcher
    assert "[switch]$Showcase" in powershell_launcher
    assert '"--showcase"' in powershell_launcher
    assert "GazeboOnly = $true" in dedicated_launcher
    assert "[switch]$FullArea" in dedicated_launcher
    assert '[string]$MapSize = "small"' in dedicated_launcher
    assert '[string]$SimulationSpeed = "fast"' in dedicated_launcher
    assert "ManualControl = $true" in dedicated_launcher
    assert "NoRviz" not in dedicated_launcher
    assert "Start-Process" not in dedicated_launcher


def test_operator_docs_name_the_gazebo_only_entry() -> None:
    command = "scripts\\run_gazebo_cleaning_demo.ps1"
    assert command in read("README.md")
    assert command in read("README_FIRST.md")
    assert command in read("docs/auto17-visual-demo.md")
    assert "青绿色" in read("README.md")
    assert "橙色外框" in read("README.md")


def test_showcase_area_is_small_bounded_and_installed() -> None:
    showcase = read("starter_ws/src/sanitation_tasks/config/showcase_area.yaml")
    task_setup = read("starter_ws/src/sanitation_tasks/setup.py")

    assert "mission_id: showcase_coverage_001" in showcase
    assert "expected_components: 9" in showcase
    assert "  - [-3.0, -4.0]" in showcase
    assert "  - [3.0, 1.0]" in showcase
    assert "keepout_polygons: []" in showcase
    assert '"config/showcase_area.yaml"' in task_setup


def test_small_mode_is_a_physically_independent_competition_demo() -> None:
    world_path = (
        ROOT / "starter_ws" / "src" / "sanitation_worlds" / "worlds"
        / "sanitation_competition_demo.sdf"
    )
    root = __import__("xml.etree.ElementTree", fromlist=["ElementTree"]).parse(
        world_path
    ).getroot()
    world = root.find("world")
    assert world is not None
    assert world.get("name") == "sanitation_competition_demo"
    names = {model.get("name") for model in world.findall("model")}
    assert {
        "demo_arena_floor", "cleaning_lane_surface", "home_dock",
        "actual_cleaning_boundary_north", "actual_cleaning_boundary_south",
        "actual_cleaning_boundary_west", "actual_cleaning_boundary_east",
        "target_bottle_demo", "target_can_demo", "target_paper_demo",
        "target_cardboard_demo", "target_leaf_pile_demo", "puddle_demo",
        "target_bottle_demo_02", "target_can_demo_02", "target_paper_demo_02",
        "target_cardboard_demo_02", "target_leaf_pile_demo_02",
    } <= names
    floor = world.find("./model[@name='demo_arena_floor']")
    assert floor is not None
    assert "16 12 0.08" in __import__("xml.etree.ElementTree", fromlist=["ElementTree"]).tostring(
        floor, encoding="unicode"
    )
    assert world.findtext("./scene/grid") == "false"
    bottle = world.find("./model[@name='target_bottle_demo']")
    can = world.find("./model[@name='target_can_demo']")
    paper = world.find("./model[@name='target_paper_demo']")
    cardboard = world.find("./model[@name='target_cardboard_demo']")
    leaves = world.find("./model[@name='target_leaf_pile_demo']")
    puddle = world.find("./model[@name='puddle_demo']")
    assert bottle is not None and can is not None
    assert paper is not None and cardboard is not None
    assert leaves is not None and puddle is not None
    assert bottle.findtext(
        "./link/visual[@name='body']/geometry/cylinder/radius"
    ) == "0.033"
    assert bottle.findtext(
        "./link/visual[@name='body']/geometry/cylinder/length"
    ) == "0.145"
    assert {
        visual.get("name") for visual in bottle.findall("./link/visual")
    } == {
        "body", "base_ring", "label", "shoulder", "neck", "cap",
    }
    assert can.findtext(
        "./link/visual[@name='body']/geometry/cylinder/radius"
    ) == "0.033"
    assert can.findtext(
        "./link/visual[@name='body']/geometry/cylinder/length"
    ) == "0.103"
    assert can.find("./link/visual[@name='pull_tab']/geometry/box") is not None

    paper_outline = paper.find(
        "./link/visual[@name='sheet']/geometry/polyline"
    )
    assert paper_outline is not None
    assert paper_outline.findtext("height") == "0.004"
    assert len(paper_outline.findall("point")) == 9
    assert paper.find("./link/visual[@name='folded_corner']") is not None
    assert cardboard.findtext(
        "./link/visual[@name='carton_body']/geometry/box/size"
    ) == "0.205 0.145 0.052"
    assert cardboard.find("./link/visual[@name='left_flap']") is not None
    assert cardboard.find("./link/visual[@name='right_flap']") is not None
    assert cardboard.find("./link/visual[@name='packing_tape']") is not None

    leaf_blades = leaves.findall("./link/visual/geometry/polyline")
    assert len(leaf_blades) == 4
    assert all(len(blade.findall("point")) == 8 for blade in leaf_blades)
    assert len(leaves.findall("./link/visual[@name='vein_a']")) == 1
    assert leaves.find("./link/visual[@name='stem_a']") is not None
    assert not leaves.findall(".//sphere")

    water_patch = puddle.find(
        "./link/visual[@name='water_patch']/geometry/polyline"
    )
    assert water_patch is not None
    puddle_points = [
        tuple(float(value) for value in point.text.split())
        for point in water_patch.findall("point")
    ]
    assert len(puddle_points) == 12
    assert max(abs(x) for x, _ in puddle_points) <= 0.25
    assert max(abs(y) for _, y in puddle_points) <= 0.19
    assert not puddle.findall(".//cylinder")

    mission_path = (
        ROOT / "starter_ws" / "src" / "sanitation_tasks" / "config"
        / "competition_demo_area.yaml"
    )
    mission = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
    targets = mission["cleaning_targets"]
    assert len(targets) == 10
    assert {target["class"] for target in targets} == {
        "bottle", "can", "paper", "cardboard", "leaves",
    }
    assert all(
        sum(target["class"] == target_class for target in targets) == 2
        for target_class in {target["class"] for target in targets}
    )
    target_model_names = {target["model_name"] for target in targets}
    assert target_model_names <= names
    assert len(target_model_names) == len(targets)
    assert all(-3.0 <= target["position"][0] <= 3.0 for target in targets)
    assert all(-4.0 <= target["position"][1] <= 1.0 for target in targets)

    translation = mission["world_to_map_translation"]
    for model_name in target_model_names:
        model = world.find(f"./model[@name='{model_name}']")
        assert model is not None
        target = next(
            item for item in targets if item["model_name"] == model_name
        )
        world_x, world_y = (
            float(value) for value in model.findtext("pose").split()[:2]
        )
        assert abs(world_x + translation[0] - target["position"][0]) < 1e-9
        assert abs(world_y + translation[1] - target["position"][1]) < 1e-9
        assert not model.findall(".//collision")
    shell = read("scripts/run_visual_demo.sh")
    assert 'world_name="sanitation_competition_demo"' in shell
    assert 'mission_control_demo.config' in shell
    assert 'EXPECTED_COMPONENTS=13' in shell
    assert 'coverage_demo_overlap.yaml' in shell


def test_small_demo_uses_overlap_and_strict_actual_coverage_gate() -> None:
    mission = read(
        "starter_ws/src/sanitation_tasks/config/competition_demo_area.yaml"
    )
    coverage = read(
        "starter_ws/src/sanitation_coverage/config/coverage_demo_overlap.yaml"
    )
    probe = read(
        "starter_ws/src/sanitation_coverage/"
        "sanitation_coverage/coverage_probe.py"
    )
    assert "operation_width_m: 0.65" in mission
    assert "planning_swath_spacing_m: 0.45" in mission
    assert "swath_endpoint_extension_m: 0.20" in mission
    assert "empirical_coverage_threshold: 0.995" in mission
    assert "coverage_repair_max_passes: 2" in mission
    assert "expected_components: 13" in mission
    assert "operation_width: 0.45" in coverage
    assert 'empirical["coverage_rate"] >= empirical_threshold' in probe
    assert "execution_swaths" in probe
    assert "_execute_coverage_repairs" in probe


def test_gazebo_panel_renders_live_cleaning_metrics_and_map() -> None:
    header = read(
        "starter_ws/src/sanitation_gazebo_control/include/SanitationMissionControl.hh"
    )
    source = read(
        "starter_ws/src/sanitation_gazebo_control/src/SanitationMissionControl.cc"
    )
    qml = read("starter_ws/src/sanitation_gazebo_control/SanitationMissionControl.qml")
    assert "telemetryJson READ TelemetryJson NOTIFY TelemetryJsonChanged" in header
    assert '"/coverage/gazebo_telemetry"' in source
    for label in (
        "实时作业地图", "已清扫", "目标清除", "清扫效率", "累计里程",
        "当前速度", "仿真用时", "作业步骤", "实际轨迹",
        "外部任务区", "实际清扫区", "覆盖率只统计青色内框",
    ):
        assert label in qml
    for layer in ("planned_path", "trajectory", "cleaned_cells", "targets"):
        assert layer in qml


def test_native_gazebo_controls_use_safe_task_services() -> None:
    source = read(
        "starter_ws/src/sanitation_gazebo_control/src/"
        "SanitationMissionControl.cc"
    )
    qml = read(
        "starter_ws/src/sanitation_gazebo_control/"
        "SanitationMissionControl.qml"
    )
    coverage = read(
        "starter_ws/src/sanitation_coverage/"
        "sanitation_coverage/coverage_probe.py"
    )
    for command in ("start", "pause", "resume", "stop"):
        assert f'"/coverage/control/{command}"' in source
        assert f'"/coverage/control/{command}"' in coverage
    for label in ("开始", "暂停", "继续", "停止任务", "关闭 Gazebo"):
        assert label in qml
    assert '"/cmd_vel"' not in source
    assert "manual_start" in coverage
    assert 'self._set_state("PAUSED"' in coverage
    assert 'self._set_brush(False)' in coverage
