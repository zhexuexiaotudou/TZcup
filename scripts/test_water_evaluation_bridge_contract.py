"""Static contract for the isolated native formal-water evaluator bridge."""

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
    / "WaterEvaluationBridge.cc"
)
CMAKE = ROOT / "starter_ws" / "src" / "sanitation_gazebo_control" / "CMakeLists.txt"
LAUNCH = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "launch"
    / "formal_vehicle_sim.launch.py"
)
RUNNER = ROOT / "scripts" / "run_formal_water_recovery_runtime.sh"

ROOT_TOPIC = "/model/tzcup_formal_sanitation_vehicle/water_recovery"
ROS_TO_GZ = {
    f"{ROOT_TOPIC}/command/reset_ground_volume_l",
    f"{ROOT_TOPIC}/command/reset_tank_mass_kg",
    f"{ROOT_TOPIC}/command/filter_blockage_fraction",
}
GZ_TO_ROS_DOUBLE = {
    f"{ROOT_TOPIC}/ground_volume_l",
    f"{ROOT_TOPIC}/mass_balance_error_fraction",
    f"{ROOT_TOPIC}/filter_blockage_fraction",
}
GZ_TO_ROS_STRING = f"{ROOT_TOPIC}/status_json"


def _launch_water_node() -> ast.Call:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    return next(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "Node"
        and any(
            keyword.arg == "name"
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == "water_evaluation_bridge"
            for keyword in call.keywords
        )
    )


def _keyword_string(call: ast.Call, name: str) -> str | None:
    value = next((keyword.value for keyword in call.keywords if keyword.arg == name), None)
    return value.value if isinstance(value, ast.Constant) and isinstance(value.value, str) else None


def test_water_evaluation_bridge_has_exact_directional_endpoint_contract() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    endpoints = ROS_TO_GZ | GZ_TO_ROS_DOUBLE | {GZ_TO_ROS_STRING}
    assert all(source.count(endpoint) == 1 for endpoint in endpoints)
    assert source.count("Advertise<gz::msgs::Double>") == 3
    assert source.count("create_subscription<std_msgs::msg::Float64>") == 3
    assert source.count("create_publisher<std_msgs::msg::Float64>") == 3
    assert "create_publisher<std_msgs::msg::String>" in source
    assert source.count("Subscribe(") == 4
    assert "target.set_data(source.data);" in source
    assert "target.data = source.data();" in source


def test_water_evaluation_bridge_serializes_callbacks_and_copies_status_string() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    assert source.count("std::lock_guard<std::mutex> lock(callback_mutex_);") == 3
    assert source.count("if (stopping_.load()) {") == 3
    assert source.count("PublishGazeboDouble(*message") == 3
    assert source.count("PublishRosDouble(message") == 3
    assert "void OnStatus(const gz::msgs::StringMsg & source)" in source
    assert "target.data = source.data();" in source
    assert "status_ros_pub_->publish(target);" in source
    for callback in ("OnGroundVolume", "OnMassBalance", "OnFilterBlockage"):
        assert f"void {callback}(const gz::msgs::Double & message)" in source


def test_water_evaluation_bridge_stop_drains_four_gazebo_subscriptions() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    stop_start = source.index("  void Stop()")
    private_start = source.index("private:", stop_start)
    stop = source[stop_start:private_start]
    assert "if (stopping_.exchange(true))" in stop
    assert stop.count("gz_node_.Unsubscribe(") == 4
    assert "const std::lock_guard<std::mutex> drain(callback_mutex_);" in stop


def test_launch_uses_native_bridge_without_expanding_product_parameter_bridges() -> None:
    node = _launch_water_node()
    assert _keyword_string(node, "package") == "sanitation_gazebo_control"
    assert _keyword_string(node, "executable") == "water_evaluation_bridge"
    assert _keyword_string(node, "name") == "water_evaluation_bridge"
    assert _keyword_string(node, "package") != "ros_gz_bridge"
    assert _keyword_string(node, "executable") != "parameter_bridge"
    condition = next(keyword.value for keyword in node.keywords if keyword.arg == "condition")
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name) and condition.func.id == "IfCondition"
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "water_evaluation_interfaces"


def test_native_bridges_are_reaped_before_ordered_parameter_bridges() -> None:
    cmake = CMAKE.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert "add_executable(water_evaluation_bridge" in cmake
    assert "install(TARGETS water_evaluation_bridge" in cmake
    ordered_targets = runner.split("ordered_targets = (", 1)[1].split(")\nnative_targets", 1)[0]
    assert '("native", "water_evaluation_bridge", "water_evaluation_bridge")' in ordered_targets
    assert '("native", "a300_drivetrain_native_bridge", "a300_drivetrain_bridge")' in ordered_targets
    assert '("native", "cleaning_actuator_vector_bridge", "cleaning_actuator_motor_bridge")' in ordered_targets
    assert '("parameter", "parameter_bridge", "a300_drivetrain_bridge")' not in ordered_targets
    assert '"native_bridge_reaped"' in runner
    assert '"native_bridge_clean_exit"' not in runner
    assert '"native_bridge_exit_not_clean"' in runner
    assert 'native_state == "missing"' in runner
    assert 'saw_zombie=native_saw_zombie' in runner
    assert "first_kind, first_executable, first_target = ordered_targets[0]" in runner
    assert "stop_native_bridge(first_executable, first_target)" in runner
    assert "for kind, executable, target in ordered_targets[1:]:" in runner
    assert runner.index("stop_native_bridge(first_executable, first_target)") < runner.index(
        'record("ordered_shutdown_started"'
    )
    assert "remaining_native_nodes, native_malformed, native_unknown = native_bridge_census()" in runner
    assert "--required-clean-exit-process water_evaluation_bridge" in runner
    assert "--required-clean-exit-process a300_drivetrain_native_bridge" in runner
    assert "--required-clean-exit-process cleaning_actuator_vector_bridge" in runner
