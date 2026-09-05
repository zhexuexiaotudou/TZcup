"""Static contract for the native formal A300 drivetrain transport bridge."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_gazebo_control"
    / "src"
    / "A300DrivetrainNativeBridge.cc"
)
CMAKE = ROOT / "starter_ws/src/sanitation_gazebo_control/CMakeLists.txt"
PACKAGE_XML = ROOT / "starter_ws/src/sanitation_gazebo_control/package.xml"
LAUNCH = ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
RUNNER = ROOT / "scripts/run_formal_water_recovery_runtime.sh"

ROOT_TOPIC = "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain"


def _launch_node() -> ast.Call:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    matches = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "Node"
        and any(
            keyword.arg == "name"
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == "a300_drivetrain_bridge"
            for keyword in call.keywords
        )
    ]
    assert len(matches) == 1
    return matches[0]


def _keyword(call: ast.Call, name: str) -> ast.expr | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def _string_keyword(call: ast.Call, name: str) -> str | None:
    value = _keyword(call, name)
    return value.value if isinstance(value, ast.Constant) and isinstance(value.value, str) else None


def test_native_bridge_preserves_exact_five_endpoint_directions() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    for suffix in ("cmd_vel", "actuator_enable", "emergency_stop", "odom", "status"):
        assert f'{ROOT_TOPIC}/{suffix}"' in source
    assert source.count("Advertise<gz::msgs::Twist>") == 1
    assert source.count("Advertise<gz::msgs::Boolean>") == 2
    assert source.count("create_subscription<geometry_msgs::msg::Twist>") == 1
    assert source.count("create_subscription<std_msgs::msg::Bool>") == 2
    assert source.count("gz_node_.Subscribe(") == 2
    assert source.count("create_publisher<nav_msgs::msg::Odometry>") == 1
    assert source.count("create_publisher<std_msgs::msg::String>") == 1


def test_native_bridge_copies_complete_twist_odometry_and_status_payloads() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    for vector in ("linear", "angular"):
        for axis in ("x", "y", "z"):
            assert f"target.mutable_{vector}()->set_{axis}(source.{vector}.{axis});" in source
            assert (
                f"target.twist.twist.{vector}.{axis} = source.twist().{vector}().{axis}();"
                in source
            )
    for axis in ("x", "y", "z"):
        assert f"target.pose.pose.position.{axis} = source.pose().position().{axis}();" in source
    for axis in ("x", "y", "z", "w"):
        assert f"target.pose.pose.orientation.{axis} = source.pose().orientation().{axis}();" in source
    assert "target.header.stamp.sec = source.header().stamp().sec();" in source
    assert "target.header.stamp.nanosec = source.header().stamp().nsec();" in source
    assert 'HeaderValue(source.header(), "frame_id")' in source
    assert 'HeaderValue(source.header(), "child_frame_id")' in source
    assert "target.set_data(source.data);" in source
    assert "target.data = source.data();" in source


def test_native_bridge_stops_gazebo_callbacks_before_ros_shutdown() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    stop = source[source.index("  void Stop()") : source.index("private:")]
    assert "if (stopping_.exchange(true))" in stop
    assert stop.count("gz_node_.Unsubscribe(") == 2
    assert "const std::lock_guard<std::mutex> drain(callback_mutex_);" in stop
    assert source.count("const std::lock_guard<std::mutex> lock(callback_mutex_);") == 4
    assert source.count("if (stopping_.load()) {") == 4
    assert source.count("bridge->Stop();") == 2
    assert source.count("bridge.reset();") == 2
    assert source.index("bridge->Stop();") < source.rindex("rclcpp::shutdown();")


def test_formal_vehicle_launch_uses_native_bridge_without_changing_remaps() -> None:
    node = _launch_node()
    assert _string_keyword(node, "package") == "sanitation_gazebo_control"
    assert _string_keyword(node, "executable") == "a300_drivetrain_native_bridge"
    assert _string_keyword(node, "name") == "a300_drivetrain_bridge"
    assert _keyword(node, "arguments") is None
    remappings = ast.literal_eval(_keyword(node, "remappings"))
    assert remappings == [
        (f"{ROOT_TOPIC}/emergency_stop", "/emergency_stop"),
        (f"{ROOT_TOPIC}/odom", "/odom/unfiltered"),
    ]
    condition = _keyword(node, "condition")
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name) and condition.func.id == "IfCondition"
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "start_a300_transport_bridge"


def test_native_bridge_build_and_shutdown_contracts_are_bound() -> None:
    cmake = CMAKE.read_text(encoding="utf-8")
    package = PACKAGE_XML.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert "find_package(nav_msgs REQUIRED)" in cmake
    assert "add_executable(a300_drivetrain_native_bridge" in cmake
    assert "install(TARGETS a300_drivetrain_native_bridge" in cmake
    for dependency in ("geometry_msgs", "nav_msgs", "rclcpp", "std_msgs"):
        assert dependency in cmake.split("ament_target_dependencies(a300_drivetrain_native_bridge", 1)[1]
    assert "gz-transport13::gz-transport13" in cmake
    assert "gz-msgs10::gz-msgs10" in cmake
    assert "<depend>nav_msgs</depend>" in package
    ordered_targets = runner.split("ordered_targets = (", 1)[1].split(")\nnative_targets", 1)[0]
    assert '("native", "water_evaluation_bridge", "water_evaluation_bridge")' in ordered_targets
    assert '("native", "a300_drivetrain_native_bridge", "a300_drivetrain_bridge")' in ordered_targets
    assert '("parameter", "parameter_bridge", "a300_drivetrain_bridge")' not in ordered_targets
    assert "first_kind, first_executable, first_target = ordered_targets[0]" in runner
    assert "for kind, executable, target in ordered_targets[1:]:" in runner
    assert "remaining_native_nodes, native_malformed, native_unknown = native_bridge_census()" in runner
    assert "--required-clean-exit-process a300_drivetrain_native_bridge" in runner
