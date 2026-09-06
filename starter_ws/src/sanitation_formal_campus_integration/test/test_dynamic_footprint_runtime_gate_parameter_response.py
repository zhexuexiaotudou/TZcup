import ast
import math
import time
from collections.abc import Sequence
from pathlib import Path
from types import SimpleNamespace

import pytest


GATE_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "sanitation_formal_campus_integration/dynamic_footprint_runtime_gate.py"
)


class _CompletedFuture:
    def __init__(self, value=None, error=None) -> None:
        self._value = value
        self._error = error

    def result(self):
        if self._error is not None:
            raise self._error
        return self._value

    @staticmethod
    def done() -> bool:
        return True


class _Response:
    def __init__(self, values) -> None:
        self.values = values


class _Value:
    def __init__(self, value) -> None:
        self.value = value


def _response_reader():
    tree = ast.parse(GATE_SOURCE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_read_footprint_padding_response"
    )
    namespace = {
        "Sequence": Sequence,
        "math": math,
        "point32_coordinate_quantization_bound": lambda left, right: 1e-6,
    }
    exec(
        compile(ast.Module(body=[function], type_ignores=[]), str(GATE_SOURCE), "exec"),
        namespace,
    )
    return namespace[function.name]


def _runtime_padding_reader():
    tree = ast.parse(GATE_SOURCE.read_text(encoding="utf-8"))
    gate_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "DynamicFootprintRuntimeGate"
    )
    method = next(
        node
        for node in gate_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_read_footprint_padding"
    )
    wrapper = ast.ClassDef(
        name="_RuntimePaddingReader",
        bases=[],
        keywords=[],
        body=[method],
        decorator_list=[],
    )
    namespace = {
        "INPUT_TOPICS": ("/local_costmap/footprint",),
        "PADDING_PARAMETER": "footprint_padding",
        "ROBOT_BASE_FRAME_PARAMETER": "robot_base_frame",
        "_read_footprint_padding_response": _response_reader(),
        "point32_coordinate_quantization_bound": lambda left, right: 1e-6,
        "rclpy": SimpleNamespace(spin_once=lambda *_args, **_kwargs: None),
        "time": time,
    }
    module = ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[]))
    exec(compile(module, str(GATE_SOURCE), "exec"), namespace)
    return namespace[wrapper.name]


def test_jazzy_get_parameters_response_uses_values_sequence() -> None:
    reader = _response_reader()
    value, frame, bound = reader(
        _CompletedFuture(_Response([_Value(0.01), _Value("base_link")])), 0.01
    )
    assert value == pytest.approx(0.01)
    assert frame == "base_link"
    assert bound == pytest.approx(1e-6)


@pytest.mark.parametrize(
    ("future", "message"),
    [
        (_CompletedFuture(error=RuntimeError("service failed")), "service failed"),
        (_CompletedFuture(None), "parameter response missing"),
        (_CompletedFuture(object()), "parameter response values must be a sequence"),
        (_CompletedFuture(_Response(None)), "parameter response values must be a sequence"),
        (_CompletedFuture(_Response(object())), "parameter response values must be a sequence"),
        (_CompletedFuture(_Response("not-values")), "parameter response values must be a sequence"),
        (_CompletedFuture(_Response(b"not-values")), "parameter response values must be a sequence"),
        (_CompletedFuture(_Response([])), "parameter result count"),
        (_CompletedFuture(_Response([_Value(0.01)])), "parameter result count"),
        (
            _CompletedFuture(_Response([_Value(0.01), _Value("base_link"), _Value(0)])),
            "parameter result count",
        ),
        (_CompletedFuture(_Response([_Value(float("nan")), _Value("base_link")])), "not finite"),
        (_CompletedFuture(_Response([_Value(float("inf")), _Value("base_link")])), "not finite"),
        (_CompletedFuture(_Response([_Value(float("-inf")), _Value("base_link")])), "not finite"),
        (_CompletedFuture(_Response([_Value(-0.01), _Value("base_link")])), "not finite"),
        (
            _CompletedFuture(_Response([_Value(0.02), _Value("base_link")])),
            "does not match declared profile padding",
        ),
        (_CompletedFuture(_Response([_Value(object()), _Value("base_link")])), "float"),
        (_CompletedFuture(_Response([_Value(0.01), _Value("")])), "relative frame"),
        (_CompletedFuture(_Response([_Value(0.01), _Value("/base_link")])), "relative frame"),
        (_CompletedFuture(_Response([_Value(0.01), _Value(17)])), "relative frame"),
    ],
)
def test_jazzy_get_parameters_response_rejects_malformed_values(future, message) -> None:
    with pytest.raises(Exception, match=message):
        _response_reader()(future, 0.01)


def test_runtime_padding_reader_records_topic_scoped_primary_failure() -> None:
    future = _CompletedFuture(_Response([_Value(0.02), _Value("base_link")]))

    class _ReadyClient:
        @staticmethod
        def services_are_ready() -> bool:
            return True

        @staticmethod
        def get_parameters(_names):
            return future

    gate = _runtime_padding_reader()()
    gate._timeout_sec = 0.1
    gate._padding_clients = {"/local_costmap/footprint": _ReadyClient()}
    gate._padding_m = {}
    gate._declared_padding_m = 0.01
    gate._padding_quantization_bound_m = 0.0
    gate._robot_base_frame = {}
    gate._last_failure_reason = "not_run"

    with pytest.raises(RuntimeError, match="invalid_footprint_padding"):
        gate._read_footprint_padding()
    assert gate._last_failure_reason.startswith(
        "/local_costmap/footprint:invalid_footprint_padding:"
    )
    assert "does not match declared profile padding" in gate._last_failure_reason
