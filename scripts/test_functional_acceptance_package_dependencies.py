from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "starter_ws" / "src"

DEPENDENCY_TAGS = {
    "build_depend",
    "build_export_depend",
    "buildtool_depend",
    "depend",
    "exec_depend",
    "test_depend",
}


def dependency_tags(package: str) -> dict[str, str]:
    root = ET.parse(PACKAGE_ROOT / package / "package.xml").getroot()
    dependencies: dict[str, str] = {}
    for child in root:
        if child.tag not in DEPENDENCY_TAGS or not child.text:
            continue
        key = child.text.strip()
        assert key not in dependencies, f"{package}: duplicate dependency {key}"
        dependencies[key] = child.tag
    return dependencies


def assert_exec_dependencies(package: str, expected: set[str]) -> None:
    dependencies = dependency_tags(package)
    missing = expected - dependencies.keys()
    assert not missing, f"{package}: missing runtime dependencies: {sorted(missing)}"
    wrong_type = {
        key: dependencies[key]
        for key in expected
        if dependencies[key] not in {"depend", "exec_depend"}
    }
    assert not wrong_type, f"{package}: runtime dependencies use wrong tags: {wrong_type}"


def test_package_manifests_are_well_formed_and_dependency_keys_are_unique() -> None:
    for package in (
        "sanitation_vehicle_description",
        "sanitation_manipulation",
        "sanitation_gazebo_control",
        "sanitation_product_demo_integration",
    ):
        dependency_tags(package)


def test_formal_vehicle_launch_and_runners_declare_direct_runtime_dependencies() -> None:
    assert_exec_dependencies(
        "sanitation_vehicle_description",
        {
            "ament_index_python",
            "builtin_interfaces",
            "control_msgs",
            "controller_manager",
            "controller_manager_msgs",
            "geometry_msgs",
            "launch",
            "launch_ros",
            "nav_msgs",
            "rclpy",
            "robot_state_publisher",
            "ros_gz_bridge",
            "ros_gz_image",
            "ros_gz_interfaces",
            "ros_gz_sim",
            "rosgraph_msgs",
            "sanitation_gazebo_control",
            "sensor_msgs",
            "std_msgs",
            "trajectory_msgs",
            "python3-yaml",
            "xacro",
        },
    )


def test_formal_vehicle_launch_exports_installed_gazebo_plugin_path() -> None:
    source = (
        PACKAGE_ROOT
        / "sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'get_package_prefix("sanitation_gazebo_control")' in source
    assert 'SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH"' in source


def test_formal_manipulation_launch_and_runner_dependencies_are_explicit() -> None:
    assert_exec_dependencies(
        "sanitation_manipulation",
        {
            "action_msgs",
            "ament_index_python",
            "builtin_interfaces",
            "control_msgs",
            "controller_manager",
            "launch",
            "launch_ros",
            "rclpy",
            "robot_state_publisher",
            "ros_gz_bridge",
            "ros_gz_interfaces",
            "ros_gz_sim",
            "rosgraph_msgs",
            "sanitation_vehicle_description",
            "sensor_msgs",
            "std_msgs",
            "tf2_msgs",
            "trajectory_msgs",
            "xacro",
        },
    )


def test_gazebo_plugins_declare_build_and_runtime_dependencies() -> None:
    dependencies = dependency_tags("sanitation_gazebo_control")
    assert dependencies["ament_cmake"] == "buildtool_depend"
    assert dependencies["protobuf-dev"] == "build_depend"
    for key in {
        "gz_gui_vendor",
        "gz_msgs_vendor",
        "gz_sim_vendor",
        "gz_transport_vendor",
        "qtbase5-dev",
        "rclcpp",
        "std_msgs",
        "std_srvs",
    }:
        assert dependencies[key] == "depend"


def test_gazebo_plugin_package_exports_clean_install_discovery_hook() -> None:
    cmake = (
        PACKAGE_ROOT / "sanitation_gazebo_control/CMakeLists.txt"
    ).read_text(encoding="utf-8")
    hook = (
        PACKAGE_ROOT
        / "sanitation_gazebo_control/env-hooks/sanitation_gazebo_control.dsv.in"
    ).read_text(encoding="utf-8")
    assert "ament_environment_hooks(" in cmake
    assert "sanitation_gazebo_control.dsv.in" in cmake
    assert hook.splitlines() == [
        "prepend-non-duplicate;GZ_SIM_SYSTEM_PLUGIN_PATH;lib",
        "prepend-non-duplicate;LD_LIBRARY_PATH;lib",
    ]


def test_product_demo_declares_every_direct_composition_dependency() -> None:
    assert_exec_dependencies(
        "sanitation_product_demo_integration",
        {
            "launch",
            "launch_ros",
            "rclpy",
            "sanitation_active_cleaning",
            "sanitation_formal_campus_integration",
            "sanitation_manipulation",
            "sanitation_perception",
            "std_msgs",
        },
    )


def test_navigation_declares_direct_collision_monitor_runtime_dependency() -> None:
    assert_exec_dependencies("sanitation_navigation", {"nav2_collision_monitor"})
