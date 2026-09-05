"""Regression tests for the ROS shutdown race in the charge manager."""

from __future__ import annotations

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("ros_gz_interfaces")

from rclpy._rclpy_pybind11 import RCLError

import sanitation_power_system.charge_interface_manager as manager_module


class _FakeNode:
    context = object()

    def __init__(self) -> None:
        self.destroyed = False

    def destroy_node(self) -> None:
        self.destroyed = True


def _raise_context_error(_node: _FakeNode) -> None:
    raise RCLError("the given context is not valid")


def test_shutdown_context_rcl_error_is_clean(monkeypatch: pytest.MonkeyPatch) -> None:
    node = _FakeNode()
    monkeypatch.setattr(manager_module, "ChargeInterfaceManager", lambda: node)
    monkeypatch.setattr(manager_module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(manager_module.rclpy, "spin", _raise_context_error)
    monkeypatch.setattr(manager_module.rclpy, "ok", lambda *args, **kwargs: False)

    manager_module.main()

    assert node.destroyed


def test_live_context_rcl_error_remains_fatal(monkeypatch: pytest.MonkeyPatch) -> None:
    node = _FakeNode()
    shutdown_calls: list[bool] = []
    monkeypatch.setattr(manager_module, "ChargeInterfaceManager", lambda: node)
    monkeypatch.setattr(manager_module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(manager_module.rclpy, "spin", _raise_context_error)
    monkeypatch.setattr(manager_module.rclpy, "ok", lambda *args, **kwargs: True)
    monkeypatch.setattr(manager_module.rclpy, "shutdown", lambda: shutdown_calls.append(True))

    with pytest.raises(RCLError, match="context is not valid"):
        manager_module.main()

    assert node.destroyed
    assert shutdown_calls == [True]
