"""Fail-closed contract for product and evaluator Gazebo bridges."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "launch"
    / "formal_vehicle_sim.launch.py"
)


def _keyword(call: ast.Call, name: str) -> ast.expr | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def _constant_string(node: ast.expr | None) -> str | None:
    return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else None


def _bridge_calls(tree: ast.AST) -> list[ast.Call]:
    calls: list[ast.Call] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        if _constant_string(_keyword(node, "package")) != "ros_gz_bridge":
            continue
        if _constant_string(_keyword(node, "executable")) != "parameter_bridge":
            continue
        calls.append(node)
    return calls


def _arguments(call: ast.Call) -> list[str]:
    value = _keyword(call, "arguments")
    if value is None:
        # Config-file bridges are audited by their dedicated bridge-contract
        # validators; this module inspects only literal evaluator arguments.
        return []
    assert isinstance(value, (ast.List, ast.Tuple)), "literal parameter_bridge arguments must remain statically auditable"
    result = [_constant_string(item) for item in value.elts]
    assert all(item is not None for item in result), "parameter_bridge entries must be literal strings"
    return [item for item in result if item is not None]


def _condition_name(call: ast.Call) -> str | None:
    condition = _keyword(call, "condition")
    if not isinstance(condition, ast.Call) or not condition.args:
        return None
    launch_configuration = condition.args[0]
    return launch_configuration.id if isinstance(launch_configuration, ast.Name) else None


def _load_bridges() -> tuple[ast.Call, ast.Call, ast.Call, list[str], list[str], list[str]]:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    calls = _bridge_calls(tree)
    product = next(
        call
        for call in calls
        if any("/payload/wastewater_mass_kg/applied@" in item for item in _arguments(call))
    )
    water_evaluator = next(call for call in calls if _condition_name(call) == "water_evaluation_interfaces")
    dry_bin_evaluator = next(call for call in calls if _condition_name(call) == "dry_bin_evaluation_interfaces")
    return (
        product,
        water_evaluator,
        dry_bin_evaluator,
        _arguments(product),
        _arguments(water_evaluator),
        _arguments(dry_bin_evaluator),
    )


def _remapping_pairs(call: ast.Call) -> list[tuple[str, str]]:
    value = _keyword(call, "remappings")
    if value is None:
        return []
    assert isinstance(value, (ast.List, ast.Tuple))
    pairs: list[tuple[str, str]] = []
    for item in value.elts:
        assert isinstance(item, (ast.List, ast.Tuple)) and len(item.elts) == 2
        source = _constant_string(item.elts[0])
        destination = _constant_string(item.elts[1])
        assert source is not None and destination is not None
        pairs.append((source, destination))
    return pairs


def _topic(entry: str) -> str:
    return entry.split("@", 1)[0]


def _direction(entry: str) -> str:
    """Return ROS_TO_GZ, GZ_TO_ROS, or BIDIRECTIONAL."""
    assert entry.count("@") == 1, f"bridge direction must be explicit and one-way: {entry}"
    suffix = entry.split("@", 1)[1]
    if "]" in suffix and "[" not in suffix:
        return "ROS_TO_GZ"
    if "[" in suffix and "]" not in suffix:
        return "GZ_TO_ROS"
    return "BIDIRECTIONAL"


def test_evaluator_bridge_is_explicit_opt_in_and_default_off() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    declarations = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and _constant_string(call.args[0]) == "water_evaluation_interfaces"
    ]
    assert len(declarations) == 1
    assert _constant_string(_keyword(declarations[0], "default_value")) == "false"

    _, evaluator, _, _, _, _ = _load_bridges()
    condition = _keyword(evaluator, "condition")
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name) and condition.func.id == "IfCondition"
    assert len(condition.args) == 1
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "water_evaluation_interfaces"


def test_dry_bin_evaluator_bridge_is_explicit_opt_in_and_default_off() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    declarations = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and _constant_string(call.args[0]) == "dry_bin_evaluation_interfaces"
    ]
    assert len(declarations) == 1
    assert _constant_string(_keyword(declarations[0], "default_value")) == "false"

    _, _, evaluator, _, _, _ = _load_bridges()
    condition = _keyword(evaluator, "condition")
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name) and condition.func.id == "IfCondition"
    assert len(condition.args) == 1
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "dry_bin_evaluation_interfaces"


def test_default_product_bridge_has_no_truth_reset_or_entity_mutation() -> None:
    _, _, _, product_entries, _, _ = _load_bridges()
    topics = {_topic(entry) for entry in product_entries}

    forbidden_fragments = (
        "/command/reset_",
        "/ground_volume_l",
        "/mass_balance_error_fraction",
        "/status_json",
        "/set_pose",
        "/set_pose_vector",
        "/remove",
        "/create",
    )
    assert not {
        topic for topic in topics if any(fragment in topic for fragment in forbidden_fragments)
    }


def test_payload_mass_has_no_ros_write_or_dry_aggregate_interface() -> None:
    _, _, _, product_entries, _, _ = _load_bridges()
    topics = {_topic(entry) for entry in product_entries}

    # Dry waste is represented by the actual cube rigid bodies.  Publishing an
    # aggregate dry mass would count the same object twice.
    assert not {topic for topic in topics if "/payload/dry_mass_kg" in topic}

    wastewater_command = "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg"
    assert wastewater_command not in topics
    wastewater_observation = f"{wastewater_command}/applied"
    assert wastewater_observation in topics
    entry = next(item for item in product_entries if _topic(item) == wastewater_observation)
    assert _direction(entry) == "GZ_TO_ROS"


def test_product_commands_are_one_way_and_limited_to_operational_water_enable() -> None:
    _, _, _, product_entries, _, _ = _load_bridges()
    directions = {_topic(entry): _direction(entry) for entry in product_entries}
    ros_to_gz = {topic for topic, direction in directions.items() if direction == "ROS_TO_GZ"}
    assert ros_to_gz == {
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/enable",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/service_drain_open",
    }
    assert "BIDIRECTIONAL" not in directions.values()

    readonly_water = {
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
    assert readonly_water <= directions.keys()
    assert all(directions[topic] == "GZ_TO_ROS" for topic in readonly_water)


def test_only_opt_in_evaluator_bridge_contains_water_truth_and_resets() -> None:
    _, _, _, product_entries, evaluator_entries, dry_bin_entries = _load_bridges()
    product_topics = {_topic(entry) for entry in product_entries}
    evaluator_directions = {_topic(entry): _direction(entry) for entry in evaluator_entries}
    dry_bin_topics = {_topic(entry) for entry in dry_bin_entries}

    root = "/model/tzcup_formal_sanitation_vehicle/water_recovery"
    expected = {
        f"{root}/command/reset_ground_volume_l": "ROS_TO_GZ",
        f"{root}/command/reset_tank_mass_kg": "ROS_TO_GZ",
        f"{root}/command/filter_blockage_fraction": "ROS_TO_GZ",
        f"{root}/ground_volume_l": "GZ_TO_ROS",
        f"{root}/mass_balance_error_fraction": "GZ_TO_ROS",
        f"{root}/filter_blockage_fraction": "GZ_TO_ROS",
        f"{root}/status_json": "GZ_TO_ROS",
    }
    assert evaluator_directions == expected
    assert set(expected).isdisjoint(product_topics)
    assert f"{root}/command/service_drain_open" in product_topics
    assert set(expected).isdisjoint(dry_bin_topics)
    assert "BIDIRECTIONAL" not in evaluator_directions.values()


def test_dry_bin_evaluator_bridge_is_read_only_and_exactly_typed() -> None:
    _, _, _, product_entries, water_entries, evaluator_entries = _load_bridges()
    product_topics = {_topic(entry) for entry in product_entries}
    water_topics = {_topic(entry) for entry in water_entries}
    entries = {_topic(entry): entry for entry in evaluator_entries}
    directions = {topic: _direction(entry) for topic, entry in entries.items()}

    root = "/model/tzcup_formal_sanitation_vehicle/dry_bin"
    expected_types = {
        f"{root}/contained_object_count": "std_msgs/msg/Int32[gz.msgs.Int32",
        f"{root}/contained_mass_kg": "std_msgs/msg/Float64[gz.msgs.Double",
        f"{root}/status_json": "std_msgs/msg/String[gz.msgs.StringMsg",
    }
    assert {topic: entry.split("@", 1)[1] for topic, entry in entries.items()} == expected_types
    assert set(expected_types).isdisjoint(product_topics)
    assert set(expected_types).isdisjoint(water_topics)
    assert set(directions.values()) == {"GZ_TO_ROS"}
    assert {
        f"{root}/fill_level_fraction",
        f"{root}/full",
        f"{root}/sensor_ready",
    } <= product_topics


def test_bumper_bridges_are_raw_one_way_inputs_not_product_topic_publishers() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    calls = _bridge_calls(tree)
    expected = {
        "/safety/front_bumper/contact": (
            "/formal_vehicle/simulation/raw/front_bumper/contact"
        ),
        "/safety/rear_bumper/contact": (
            "/formal_vehicle/simulation/raw/rear_bumper/contact"
        ),
    }
    matched = {}
    for call in calls:
        entries = _arguments(call)
        if len(entries) != 1:
            continue
        topic = _topic(entries[0])
        if topic not in expected:
            continue
        assert _direction(entries[0]) == "GZ_TO_ROS"
        assert _remapping_pairs(call) == [(topic, expected[topic])]
        matched[topic] = call
    assert set(matched) == set(expected)


def test_squeegee_compliance_bridge_is_read_only_opt_in_and_default_off() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    declarations = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and _constant_string(call.args[0]) == "squeegee_evaluation_interfaces"
    ]
    assert len(declarations) == 1
    assert _constant_string(_keyword(declarations[0], "default_value")) == "false"
    bridge = next(
        call
        for call in _bridge_calls(tree)
        if _condition_name(call) == "squeegee_evaluation_interfaces"
    )
    entries = _arguments(bridge)
    assert len(entries) == 7
    assert all(_direction(entry) == "GZ_TO_ROS" for entry in entries)
    scoped_contact = (
        "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/"
        "link/squeegee_link/sensor/squeegee_blade_ground_contact/contact"
    )
    assert {_topic(entry) for entry in entries} == {
        scoped_contact,
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_position_m",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_velocity_m_s",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_force_n",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_position_rad",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_velocity_rad_s",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_torque_nm",
    }
    assert _remapping_pairs(bridge) == [(scoped_contact, "/cleaning/squeegee/contact")]


def test_brush_contact_bridge_uses_scoped_gz_sources_and_short_ros_contract() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    bridge = next(
        call
        for call in _bridge_calls(tree)
        if _constant_string(_keyword(call, "name")) == "formal_brush_contact_evaluation_bridge"
    )
    entries = _arguments(bridge)
    assert len(entries) == 3
    assert all(_direction(entry) == "GZ_TO_ROS" for entry in entries)
    root = "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link"
    expected = {
        f"{root}/left_side_brush_link/sensor/left_side_brush_ground_contact/contact": "/cleaning/left_side_brush/contact",
        f"{root}/right_side_brush_link/sensor/right_side_brush_ground_contact/contact": "/cleaning/right_side_brush/contact",
        f"{root}/central_roller_link/sensor/central_roller_ground_contact/contact": "/cleaning/central_roller/contact",
    }
    assert {_topic(entry) for entry in entries} == set(expected)
    assert _remapping_pairs(bridge) == list(expected.items())
