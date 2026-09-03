"""Keep the formal vehicle launch strict while tolerating only closed contexts."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIMPLE_NODES = (
    "starter_ws/src/sanitation_power_system/sanitation_power_system/charge_interface_manager.py",
    "starter_ws/src/sanitation_power_system/sanitation_power_system/a300_bms_node.py",
    "starter_ws/src/sanitation_safety/sanitation_safety/service_drain_manager.py",
    "starter_ws/src/sanitation_vehicle_description/scripts/formal_robot_description_publisher.py",
    "starter_ws/src/sanitation_vehicle_description/scripts/formal_encoder_feedback_publisher.py",
    "starter_ws/src/sanitation_vehicle_description/scripts/formal_fisheye_camera_info_publisher.py",
)
THREADED_MANAGERS = (
    "starter_ws/src/sanitation_safety/sanitation_safety/simulation_safety_inputs.py",
    "starter_ws/src/sanitation_safety/sanitation_safety/whole_vehicle_safety_manager.py",
)


def _main_source(relative_path: str) -> str:
    source = (ROOT / relative_path).read_text(encoding="utf-8")
    assert "from rclpy._rclpy_pybind11 import RCLError" in source
    return source[source.rindex("def main") :]


def test_simple_formal_nodes_only_tolerate_closed_context_rcl_errors() -> None:
    for relative_path in SIMPLE_NODES:
        main = _main_source(relative_path)
        assert "except RCLError:" in main
        assert "if rclpy.ok(context=node.context):\n            raise" in main


def test_threaded_managers_preserve_live_context_fatal_paths() -> None:
    for relative_path in THREADED_MANAGERS:
        main = _main_source(relative_path)
        assert "except RCLError as error:" in main
        assert "if rclpy.ok(context=node.context):\n            fatal_error = error" in main
        assert "if fatal_error is not None:" in main
