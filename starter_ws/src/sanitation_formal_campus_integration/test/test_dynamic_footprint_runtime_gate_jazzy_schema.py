"""No-graph smoke for the real Jazzy GetParameters response schema."""

import ast
import importlib.util
import os
import sys
import time
from collections.abc import Sequence
from numbers import Real
from pathlib import Path

import pytest


GATE_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "sanitation_formal_campus_integration/dynamic_footprint_runtime_gate.py"
)


class _CompletedFuture:
    def __init__(self, response) -> None:
        self._response = response

    def result(self):
        return self._response


def _real_response_reader():
    rclpy_parameter = pytest.importorskip("rclpy.parameter")
    tree = ast.parse(GATE_SOURCE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_read_footprint_padding_response"
    )
    core_source = GATE_SOURCE.with_name("dynamic_footprint_core.py")
    spec = importlib.util.spec_from_file_location("_dynamic_footprint_core", core_source)
    assert spec is not None and spec.loader is not None
    core = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(core)
    namespace = {
        "Real": Real,
        "Sequence": Sequence,
        "math": __import__("math"),
        "parameter_value_to_python": rclpy_parameter.parameter_value_to_python,
        "point32_coordinate_quantization_bound": core.point32_coordinate_quantization_bound,
    }
    exec(
        compile(ast.Module(body=[function], type_ignores=[]), str(GATE_SOURCE), "exec"),
        namespace,
    )
    return namespace[function.name]


def test_real_jazzy_parameter_value_schema_decodes_without_a_ros_graph() -> None:
    messages = pytest.importorskip("rcl_interfaces.msg")
    services = pytest.importorskip("rcl_interfaces.srv")
    padding = messages.ParameterValue()
    padding.type = messages.ParameterType.PARAMETER_DOUBLE
    padding.double_value = 0.01
    frame = messages.ParameterValue()
    frame.type = messages.ParameterType.PARAMETER_STRING
    frame.string_value = "base_link"
    response = services.GetParameters.Response()
    response.values = [padding, frame]

    assert not hasattr(padding, "value")
    assert not hasattr(frame, "value")
    assert _real_response_reader()(_CompletedFuture(response), 0.01)[:2] == (
        0.01,
        "base_link",
    )


def test_real_jazzy_gate_node_uses_sim_time_without_a_graph() -> None:
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("rcl_interfaces.msg")
    clock_messages = pytest.importorskip("rosgraph_msgs.msg")
    source_root = str(GATE_SOURCE.parents[1])
    sys.path.insert(0, source_root)
    try:
        from sanitation_formal_campus_integration.dynamic_footprint_runtime_gate import (
            DynamicFootprintRuntimeGate,
        )

        domain_id = os.environ.get("TZCUP_R068_TEST_DOMAIN", "231")
        rclpy.init(domain_id=int(domain_id))
        gate = DynamicFootprintRuntimeGate(
            GATE_SOURCE.parents[4]
            / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml",
            0.1,
        )
        clock_node = rclpy.create_node("r068_jazzy_clock_smoke_publisher")
        try:
            assert gate.get_parameter("use_sim_time").value is True
            assert gate.get_clock().now().nanoseconds == 0
            publisher = clock_node.create_publisher(clock_messages.Clock, "/clock", 10)
            expected_ns = 12_345_678_901
            message = clock_messages.Clock()
            message.clock.sec = expected_ns // 1_000_000_000
            message.clock.nanosec = expected_ns % 1_000_000_000
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                publisher.publish(message)
                rclpy.spin_once(gate, timeout_sec=0.05)
                if gate.get_clock().now().nanoseconds == expected_ns:
                    break
            assert gate.get_clock().now().nanoseconds == expected_ns
            message.clock.nanosec += 1
            publisher.publish(message)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(gate, timeout_sec=0.05)
                if gate.get_clock().now().nanoseconds > expected_ns:
                    break
            assert gate.get_clock().now().nanoseconds > expected_ns
        finally:
            clock_node.destroy_node()
            gate.destroy_node()
            rclpy.shutdown()
    finally:
        sys.path.remove(source_root)
