"""Regression tests for the ROS shutdown race in the safety manager."""

from __future__ import annotations

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("controller_manager_msgs")
pytest.importorskip("ros_gz_interfaces")

from rclpy._rclpy_pybind11 import RCLError

import sanitation_safety.whole_vehicle_safety_manager as manager_module


class _FakeNode:
    context = object()

    def __init__(self) -> None:
        self.stopped = False
        self.destroyed = False

    def _raise_if_publish_failed(self) -> None:
        return None

    def _stop_publish_loop(self) -> None:
        self.stopped = True

    def destroy_node(self) -> None:
        self.destroyed = True


class _FailingExecutor:
    instances: list["_FailingExecutor"] = []

    def __init__(self, num_threads: int) -> None:
        self.num_threads = num_threads
        self.node = None
        self.stopped = False
        self.instances.append(self)

    def add_node(self, node: _FakeNode) -> None:
        self.node = node

    def spin_once(self, timeout_sec: float) -> None:
        raise RCLError("the given context is not valid")

    def shutdown(self) -> None:
        self.stopped = True


def _install_fakes(monkeypatch: pytest.MonkeyPatch) -> _FakeNode:
    _FailingExecutor.instances.clear()
    node = _FakeNode()
    monkeypatch.setattr(manager_module, "WholeVehicleSafetyManager", lambda: node)
    monkeypatch.setattr(manager_module, "MultiThreadedExecutor", _FailingExecutor)
    monkeypatch.setattr(manager_module.rclpy, "init", lambda args=None: None)
    return node


def test_shutdown_context_rcl_error_is_clean(monkeypatch: pytest.MonkeyPatch) -> None:
    node = _install_fakes(monkeypatch)
    ok_values = iter((True, False, False))
    monkeypatch.setattr(manager_module.rclpy, "ok", lambda *args, **kwargs: next(ok_values))

    manager_module.main()

    executor = _FailingExecutor.instances[0]
    assert executor.node is node
    assert executor.stopped
    assert node.stopped
    assert node.destroyed


def test_live_context_rcl_error_remains_fatal(monkeypatch: pytest.MonkeyPatch) -> None:
    node = _install_fakes(monkeypatch)
    shutdown_calls: list[bool] = []
    monkeypatch.setattr(manager_module.rclpy, "ok", lambda *args, **kwargs: True)
    monkeypatch.setattr(manager_module.rclpy, "shutdown", lambda: shutdown_calls.append(True))

    with pytest.raises(RuntimeError, match="whole_vehicle_safety_manager_fatal") as caught:
        manager_module.main()

    assert isinstance(caught.value.__cause__, RCLError)
    executor = _FailingExecutor.instances[0]
    assert executor.stopped
    assert node.stopped
    assert node.destroyed
    assert shutdown_calls == [True]
