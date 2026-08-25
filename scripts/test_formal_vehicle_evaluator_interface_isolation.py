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
    assert isinstance(value, (ast.List, ast.Tuple)), "parameter_bridge arguments must remain statically auditable"
    result = [_constant_string(item) for item in value.elts]
    assert all(item is not None for item in result), "parameter_bridge entries must be literal strings"
    return [item for item in result if item is not None]


def _load_bridges() -> tuple[ast.Call, ast.Call, list[str], list[str]]:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    calls = _bridge_calls(tree)
    assert len(calls) == 2, "formal launch must have one product bridge and one opt-in evaluator bridge"
    product = next(call for call in calls if _keyword(call, "condition") is None)
    evaluator = next(call for call in calls if _keyword(call, "condition") is not None)
    return product, evaluator, _arguments(product), _arguments(evaluator)


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

    _, evaluator, _, _ = _load_bridges()
    condition = _keyword(evaluator, "condition")
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name) and condition.func.id == "IfCondition"
    assert len(condition.args) == 1
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "water_evaluation_interfaces"


def test_default_product_bridge_has_no_truth_reset_or_entity_mutation() -> None:
    _, _, product_entries, _ = _load_bridges()
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
    _, _, product_entries, _ = _load_bridges()
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
    _, _, product_entries, _ = _load_bridges()
    directions = {_topic(entry): _direction(entry) for entry in product_entries}
    ros_to_gz = {topic for topic, direction in directions.items() if direction == "ROS_TO_GZ"}
    assert ros_to_gz == {
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/enable"
    }
    assert "BIDIRECTIONAL" not in directions.values()

    readonly_water = {
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_mass_kg",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_level_fraction",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/flow_l_min",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/recovered_volume_l",
        "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_full",
    }
    assert readonly_water <= directions.keys()
    assert all(directions[topic] == "GZ_TO_ROS" for topic in readonly_water)


def test_only_opt_in_evaluator_bridge_contains_water_truth_and_resets() -> None:
    _, _, product_entries, evaluator_entries = _load_bridges()
    product_topics = {_topic(entry) for entry in product_entries}
    evaluator_directions = {_topic(entry): _direction(entry) for entry in evaluator_entries}

    root = "/model/tzcup_formal_sanitation_vehicle/water_recovery"
    expected = {
        f"{root}/command/reset_ground_volume_l": "ROS_TO_GZ",
        f"{root}/command/reset_tank_mass_kg": "ROS_TO_GZ",
        f"{root}/ground_volume_l": "GZ_TO_ROS",
        f"{root}/mass_balance_error_fraction": "GZ_TO_ROS",
        f"{root}/status_json": "GZ_TO_ROS",
    }
    assert evaluator_directions == expected
    assert set(expected).isdisjoint(product_topics)
    assert "BIDIRECTIONAL" not in evaluator_directions.values()
