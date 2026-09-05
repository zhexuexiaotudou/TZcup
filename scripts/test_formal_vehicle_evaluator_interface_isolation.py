"""Fail-closed separation between product telemetry and evaluator truth."""

from __future__ import annotations

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
PRODUCT_NATIVE_BRIDGE = ROOT / "starter_ws/src/sanitation_gazebo_control/src/FormalVehicleProductNativeBridge.cc"
CONTACT_NATIVE_BRIDGE = ROOT / "starter_ws/src/sanitation_gazebo_control/src/FormalContactEvaluationNativeBridge.cc"
WATER_NATIVE_BRIDGE = ROOT / "starter_ws/src/sanitation_gazebo_control/src/WaterEvaluationBridge.cc"


def _keyword(call: ast.Call, name: str) -> ast.expr | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def _string(node: ast.expr | None) -> str | None:
    return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else None


def _node(tree: ast.AST, name: str) -> ast.Call:
    return next(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "Node"
        and _string(_keyword(call, "name")) == name
    )


def _condition_name(call: ast.Call) -> str | None:
    condition = _keyword(call, "condition")
    if not isinstance(condition, ast.Call) or not condition.args:
        return None
    value = condition.args[0]
    return value.id if isinstance(value, ast.Name) else None


def _literal_list(call: ast.Call, name: str) -> list[object]:
    value = _keyword(call, name)
    assert isinstance(value, (ast.List, ast.Tuple)), f"{name} must be literal"
    return ast.literal_eval(value)


def _parameter_bridge_entries(tree: ast.AST, condition: str) -> list[str]:
    call = next(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "Node"
        and _string(_keyword(call, "package")) == "ros_gz_bridge"
        and _string(_keyword(call, "executable")) == "parameter_bridge"
        and _condition_name(call) == condition
    )
    return [item for item in _literal_list(call, "arguments") if isinstance(item, str)]


_ENDPOINT = re.compile(
    r"constexpr\s+(?P<direction>GazeboToRosEndpoint|RosToGazeboEndpoint)"
    r"<[\s\S]*?>\s+\w+\s*\{\s*\"(?P<topic>[^\"]+)\"\s*\};"
)
_GROUPED_GZ_TO_ROS = re.compile(
    r"constexpr\s+GroupedGazeboToRosEndpoint<\s*(?P<ros>[\w:]+)\s*,\s*(?P<gz>[\w:]+)\s*>"
    r"\s+\w+\s*\{\s*\"(?P<group>[^\"]+)\"\s*,\s*\"(?P<topic>[^\"]+)\"\s*\};",
    re.DOTALL,
)
_CPP_TYPES = {
    "std_msgs::msg::Float64": "std_msgs/msg/Float64",
    "ros_gz_interfaces::msg::Contacts": "ros_gz_interfaces/msg/Contacts",
    "gz::msgs::Double": "gz.msgs.Double",
    "gz::msgs::Contacts": "gz.msgs.Contacts",
}


def _product_endpoints() -> dict[str, str]:
    return {
        match["topic"]: "G2R" if match["direction"] == "GazeboToRosEndpoint" else "R2G"
        for match in _ENDPOINT.finditer(PRODUCT_NATIVE_BRIDGE.read_text(encoding="utf-8"))
    }


def _remaps(call: ast.Call) -> list[tuple[str, str]]:
    return [tuple(pair) for pair in _literal_list(call, "remappings")]


def _contact_endpoints(group: str) -> set[tuple[str, str, str]]:
    return {
        (match["topic"], _CPP_TYPES[match["ros"]], _CPP_TYPES[match["gz"]])
        for match in _GROUPED_GZ_TO_ROS.finditer(CONTACT_NATIVE_BRIDGE.read_text(encoding="utf-8"))
        if match["group"] == group
    }


def test_evaluator_bridges_are_explicit_opt_ins_and_default_off() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    for argument in ("water_evaluation_interfaces", "dry_bin_evaluation_interfaces"):
        declaration = next(
            call
            for call in ast.walk(tree)
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "DeclareLaunchArgument"
            and call.args
            and _string(call.args[0]) == argument
        )
        assert _string(_keyword(declaration, "default_value")) == "false"

    assert _condition_name(_node(tree, "water_evaluation_bridge")) == "water_evaluation_interfaces"
    assert _parameter_bridge_entries(tree, "dry_bin_evaluation_interfaces")


def test_default_product_native_bridge_excludes_evaluator_truth_and_entity_mutation() -> None:
    endpoints = _product_endpoints()
    forbidden = (
        "/command/reset_", "/ground_volume_l", "/mass_balance_error_fraction", "/status_json",
        "/set_pose", "/set_pose_vector", "/remove", "/create", "/payload/dry_mass_kg",
    )
    assert not {topic for topic in endpoints if any(token in topic for token in forbidden)}
    wastewater = "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg"
    assert wastewater not in endpoints
    assert endpoints[f"{wastewater}/applied"] == "G2R"


def test_product_native_bridge_commands_are_one_way_and_limited_to_operational_water() -> None:
    endpoints = _product_endpoints()
    assert {topic for topic, direction in endpoints.items() if direction == "R2G"} == {
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/enable",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/service_drain_open",
    }
    readonly = {
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_mass_kg",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_level_fraction",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/flow_l_min",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/recovered_volume_l",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_full",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_flow_l_min",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_tank_level_fraction",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_differential_pressure_kpa",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/pump_current_a",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_low_probe_wet",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_high_probe_wet",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_protection_active",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_open",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_permitted",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drained_volume_l",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/fill_level_fraction",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/full",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/sensor_ready",
    }
    assert all(endpoints[topic] == "G2R" for topic in readonly)


def test_water_and_dry_bin_evaluator_truth_remain_outside_product_native_bridge() -> None:
    product = _product_endpoints()
    source = WATER_NATIVE_BRIDGE.read_text(encoding="utf-8")
    water_root = "/model/tzcup_formal_sanitation_vehicle/water_recovery"
    truth = {
        f"{water_root}/command/reset_ground_volume_l", f"{water_root}/command/reset_tank_mass_kg",
        f"{water_root}/command/filter_blockage_fraction", f"{water_root}/ground_volume_l",
        f"{water_root}/mass_balance_error_fraction", f"{water_root}/filter_blockage_fraction",
        f"{water_root}/status_json",
    }
    assert all(topic in source for topic in truth)
    assert truth.isdisjoint(product)
    dry_entries = _parameter_bridge_entries(
        ast.parse(LAUNCH.read_text(encoding="utf-8")), "dry_bin_evaluation_interfaces"
    )
    assert {entry.split("@", 1)[0] for entry in dry_entries} == {
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/contained_object_count",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/contained_mass_kg",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json",
    }
    assert all("[" in entry and "]" not in entry.split("@", 1)[1] for entry in dry_entries)


def test_contact_native_instances_preserve_raw_safety_and_opt_in_evaluation_remaps() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    source = CONTACT_NATIVE_BRIDGE.read_text(encoding="utf-8")
    expected = {
        "formal_squeegee_evaluation_bridge": (
            "squeegee_evaluation_interfaces", "squeegee",
            [("/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/squeegee_link/sensor/squeegee_blade_ground_contact/contact", "/cleaning/squeegee/contact")],
        ),
        "formal_brush_contact_evaluation_bridge": (
            "squeegee_evaluation_interfaces", "brushes",
            [
                ("/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/left_side_brush_link/sensor/left_side_brush_ground_contact/contact", "/cleaning/left_side_brush/contact"),
                ("/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/right_side_brush_link/sensor/right_side_brush_ground_contact/contact", "/cleaning/right_side_brush/contact"),
                ("/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/central_roller_link/sensor/central_roller_ground_contact/contact", "/cleaning/central_roller/contact"),
            ],
        ),
        "front_bumper_contact_bridge": (
            "start_product_support_parameter_bridges", "front_bumper",
            [("/safety/front_bumper/contact", "/formal_vehicle/simulation/raw/front_bumper/contact")],
        ),
        "rear_bumper_contact_bridge": (
            "start_product_support_parameter_bridges", "rear_bumper",
            [("/safety/rear_bumper/contact", "/formal_vehicle/simulation/raw/rear_bumper/contact")],
        ),
    }
    for name, (condition, group, remaps) in expected.items():
        node = _node(tree, name)
        assert _string(_keyword(node, "package")) == "sanitation_gazebo_control"
        assert _string(_keyword(node, "executable")) == "formal_contact_evaluation_native_bridge"
        assert _condition_name(node) == condition
        assert _literal_list(node, "parameters") == [{"endpoint_group": group}]
        assert _remaps(node) == remaps
        assert f'"{group}"' in source


def test_water_evaluator_has_exact_lifecycle_safe_truth_interface() -> None:
    source = WATER_NATIVE_BRIDGE.read_text(encoding="utf-8")
    assert source.count("create_subscription<std_msgs::msg::Float64>") == 3
    assert source.count("Advertise<gz::msgs::Double>") == 3
    assert source.count("create_publisher<std_msgs::msg::Float64>") == 3
    assert source.count("create_publisher<std_msgs::msg::String>") == 1
    assert source.count("gz_node_.Unsubscribe(") == 4
    assert "std::atomic<bool> stopping_" in source
    assert "std::lock_guard<std::mutex> drain(callback_mutex_)" in source


def test_dry_bin_evaluator_is_exactly_typed_read_only_and_disjoint_from_product() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    entries = _parameter_bridge_entries(tree, "dry_bin_evaluation_interfaces")
    expected = {
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/contained_object_count": "std_msgs/msg/Int32[gz.msgs.Int32",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/contained_mass_kg": "std_msgs/msg/Float64[gz.msgs.Double",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json": "std_msgs/msg/String[gz.msgs.StringMsg",
    }
    assert {entry.split("@", 1)[0]: entry.split("@", 1)[1] for entry in entries} == expected
    assert all("[" in entry and "]" not in entry.split("@", 1)[1] for entry in entries)
    product = _product_endpoints()
    assert set(expected).isdisjoint(product)
    assert {
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/fill_level_fraction",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/full",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/sensor_ready",
    } <= set(product)


def test_squeegee_native_group_is_exactly_seven_typed_gazebo_to_ros_endpoints() -> None:
    root = "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance"
    contact = "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/squeegee_link/sensor/squeegee_blade_ground_contact/contact"
    expected = {
        (f"{root}/float_position_m", "std_msgs/msg/Float64", "gz.msgs.Double"),
        (f"{root}/float_velocity_m_s", "std_msgs/msg/Float64", "gz.msgs.Double"),
        (f"{root}/float_force_n", "std_msgs/msg/Float64", "gz.msgs.Double"),
        (f"{root}/pitch_position_rad", "std_msgs/msg/Float64", "gz.msgs.Double"),
        (f"{root}/pitch_velocity_rad_s", "std_msgs/msg/Float64", "gz.msgs.Double"),
        (f"{root}/pitch_torque_nm", "std_msgs/msg/Float64", "gz.msgs.Double"),
        (contact, "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts"),
    }
    assert _contact_endpoints("squeegee") == expected
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    declaration = next(
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument" and call.args
        and _string(call.args[0]) == "squeegee_evaluation_interfaces"
    )
    assert _string(_keyword(declaration, "default_value")) == "false"
    node = _node(tree, "formal_squeegee_evaluation_bridge")
    assert _condition_name(node) == "squeegee_evaluation_interfaces"
    assert _literal_list(node, "parameters") == [{"endpoint_group": "squeegee"}]
    assert _remaps(node) == [(contact, "/cleaning/squeegee/contact")]


def test_brush_native_group_is_exactly_three_typed_gazebo_to_ros_contacts() -> None:
    root = "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link"
    expected = {
        (f"{root}/left_side_brush_link/sensor/left_side_brush_ground_contact/contact", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts"),
        (f"{root}/right_side_brush_link/sensor/right_side_brush_ground_contact/contact", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts"),
        (f"{root}/central_roller_link/sensor/central_roller_ground_contact/contact", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts"),
    }
    assert _contact_endpoints("brushes") == expected
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    node = _node(tree, "formal_brush_contact_evaluation_bridge")
    assert _condition_name(node) == "squeegee_evaluation_interfaces"
    assert _literal_list(node, "parameters") == [{"endpoint_group": "brushes"}]
    assert _remaps(node) == [
        (f"{root}/left_side_brush_link/sensor/left_side_brush_ground_contact/contact", "/cleaning/left_side_brush/contact"),
        (f"{root}/right_side_brush_link/sensor/right_side_brush_ground_contact/contact", "/cleaning/right_side_brush/contact"),
        (f"{root}/central_roller_link/sensor/central_roller_ground_contact/contact", "/cleaning/central_roller/contact"),
    ]


def test_bumper_native_groups_are_exact_one_way_raw_safety_inputs() -> None:
    expected = {
        "front_bumper": ("/safety/front_bumper/contact", "/formal_vehicle/simulation/raw/front_bumper/contact"),
        "rear_bumper": ("/safety/rear_bumper/contact", "/formal_vehicle/simulation/raw/rear_bumper/contact"),
    }
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    product = _product_endpoints()
    for group, (source, target) in expected.items():
        assert _contact_endpoints(group) == {(source, "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts")}
        node = _node(tree, f"{group}_contact_bridge")
        assert _condition_name(node) == "start_product_support_parameter_bridges"
        assert _literal_list(node, "parameters") == [{"endpoint_group": group}]
        assert _remaps(node) == [(source, target)]
        assert source not in product and target not in product
