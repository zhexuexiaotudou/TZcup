import ast
import importlib.util
import time
from collections.abc import Sequence
from numbers import Real
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


class _ParameterValue:
    PARAMETER_BOOL = 1
    PARAMETER_INTEGER = 2
    PARAMETER_DOUBLE = 3
    PARAMETER_STRING = 4

    def __init__(
        self,
        type_: int,
        *,
        bool_value: bool = False,
        integer_value: int = 0,
        double_value: float = 0.0,
        string_value: str = "",
    ) -> None:
        self.type = type_
        self.bool_value = bool_value
        self.integer_value = integer_value
        self.double_value = double_value
        self.string_value = string_value


def _parameter_value_to_python(value: _ParameterValue):
    if not isinstance(value, _ParameterValue):
        raise TypeError("not a ParameterValue")
    if value.type == value.PARAMETER_BOOL:
        return value.bool_value
    if value.type == value.PARAMETER_INTEGER:
        return value.integer_value
    if value.type == value.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == value.PARAMETER_STRING:
        return value.string_value
    raise ValueError("unsupported ParameterValue type")


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
        "Real": Real,
        "math": __import__("math"),
        "parameter_value_to_python": _parameter_value_to_python,
        "point32_coordinate_quantization_bound": _point32_quantization_bound(),
    }
    exec(
        compile(ast.Module(body=[function], type_ignores=[]), str(GATE_SOURCE), "exec"),
        namespace,
    )
    return namespace[function.name]


def _point32_quantization_bound():
    core_source = GATE_SOURCE.with_name("dynamic_footprint_core.py")
    spec = importlib.util.spec_from_file_location("_dynamic_footprint_core", core_source)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.point32_coordinate_quantization_bound


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
        "point32_coordinate_quantization_bound": _point32_quantization_bound(),
        "rclpy": SimpleNamespace(spin_once=lambda *_args, **_kwargs: None),
        "time": time,
    }
    module = ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[]))
    exec(compile(module, str(GATE_SOURCE), "exec"), namespace)
    return namespace[wrapper.name]


def test_gate_declares_its_own_sim_time_override() -> None:
    tree = ast.parse(GATE_SOURCE.read_text(encoding="utf-8"))
    gate_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "DynamicFootprintRuntimeGate"
    )
    init = next(
        node
        for node in gate_class.body
        if isinstance(node, ast.FunctionDef) and node.name == "__init__"
    )
    call = next(
        node
        for node in ast.walk(init)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "__init__"
        and isinstance(node.func.value, ast.Call)
        and isinstance(node.func.value.func, ast.Name)
        and node.func.value.func.id == "super"
    )
    overrides = next(keyword.value for keyword in call.keywords if keyword.arg == "parameter_overrides")
    assert isinstance(overrides, ast.List) and len(overrides.elts) == 1
    override = overrides.elts[0]
    assert isinstance(override, ast.Call)
    assert isinstance(override.func, ast.Name) and override.func.id == "Parameter"
    assert isinstance(override.args[0], ast.Constant) and override.args[0].value == "use_sim_time"
    assert any(
        keyword.arg == "value"
        and isinstance(keyword.value, ast.Constant)
        and keyword.value.value is True
        for keyword in override.keywords
    )


def test_jazzy_get_parameters_response_uses_values_sequence() -> None:
    reader = _response_reader()
    value, frame, bound = reader(
        _CompletedFuture(
            _Response(
                [
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.01),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ]
            )
        ),
        0.01,
    )
    assert value == pytest.approx(0.01)
    assert frame == "base_link"
    assert bound == _point32_quantization_bound()(0.01, 0.01)


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
        (
            _CompletedFuture(
                _Response([_ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.01)])
            ),
            "parameter result count",
        ),
        (
            _CompletedFuture(
                _Response(
                    [
                        _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.01),
                        _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                        _ParameterValue(_ParameterValue.PARAMETER_INTEGER),
                    ]
                )
            ),
            "parameter result count",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=float("nan")),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ])
            ),
            "not finite",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=float("inf")),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ])
            ),
            "not finite",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=float("-inf")),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ])
            ),
            "not finite",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=-0.01),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ])
            ),
            "not finite",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.02),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ])
            ),
            "does not match declared profile padding",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_BOOL, bool_value=True),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ])
            ),
            "non-bool numeric",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="0.01"),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
                ])
            ),
            "non-bool numeric",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.01),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value=""),
                ])
            ),
            "relative frame",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.01),
                    _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="/base_link"),
                ])
            ),
            "relative frame",
        ),
        (
            _CompletedFuture(
                _Response([
                    _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.01),
                    _ParameterValue(_ParameterValue.PARAMETER_INTEGER, integer_value=17),
                ])
            ),
            "relative frame",
        ),
        (
            _CompletedFuture(_Response([object(), _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link")])),
            "parameter value conversion",
        ),
    ],
)
def test_jazzy_get_parameters_response_rejects_malformed_values(future, message) -> None:
    with pytest.raises(Exception, match=message):
        _response_reader()(future, 0.01)


def test_runtime_padding_reader_records_topic_scoped_primary_failure() -> None:
    future = _CompletedFuture(
        _Response(
            [
                _ParameterValue(_ParameterValue.PARAMETER_DOUBLE, double_value=0.02),
                _ParameterValue(_ParameterValue.PARAMETER_STRING, string_value="base_link"),
            ]
        )
    )

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
