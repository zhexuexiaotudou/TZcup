"""No-board tests for the same-shell Python ABI collection gate."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location("runtime_probe", Path(__file__).with_name("preflight_s100p_board_runtime.py"))
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_collect_python_imports_prefers_distribution_metadata_version():
    modules = {name: SimpleNamespace(__version__="1.0", __file__=f"/opt/tros/{name}.py") for name in MODULE.PYTHON_IMPORTS}
    rows = MODULE.collect_python_imports(modules.__getitem__, lambda name: f"dist-{name}")
    assert rows["rclpy"]["version"] == "dist-rclpy"


def test_collect_python_imports_allows_missing_version_after_successful_import():
    modules = {name: SimpleNamespace(__file__=f"/opt/tros/{name}.py") for name in MODULE.PYTHON_IMPORTS}

    def missing_distribution(_: str) -> str:
        raise MODULE.importlib.metadata.PackageNotFoundError

    rows = MODULE.collect_python_imports(modules.__getitem__, missing_distribution, lambda: {})
    assert rows["rclpy"]["version"] is None


def test_collect_python_imports_uses_distribution_mapped_from_tf2_ros_module():
    modules = {name: SimpleNamespace(__file__=f"/opt/tros/{name}.py") for name in MODULE.PYTHON_IMPORTS}

    def distribution_version(name: str) -> str:
        if name == "tf2-ros-py":
            return "0.25.16"
        raise MODULE.importlib.metadata.PackageNotFoundError

    rows = MODULE.collect_python_imports(
        modules.__getitem__,
        distribution_version,
        lambda: {"tf2_ros": ["tf2-ros-py"]},
    )
    assert rows["tf2_ros"]["version"] == "0.25.16"


def test_collect_python_imports_rejects_missing_absolute_module_path():
    modules = {name: SimpleNamespace(__version__="1.0", __file__=f"/opt/tros/{name}.py") for name in MODULE.PYTHON_IMPORTS}
    modules["cv2"] = SimpleNamespace(__version__="1.0", __file__="relative/cv2.py")
    with pytest.raises(RuntimeError, match="cv2"):
        MODULE.collect_python_imports(modules.__getitem__, lambda _: "1.0")


def test_collect_python_imports_rejects_an_empty_fallback_version():
    modules = {name: SimpleNamespace(__version__="1.0", __file__=f"/opt/tros/{name}.py") for name in MODULE.PYTHON_IMPORTS}
    modules["cv2"] = SimpleNamespace(__version__="", __file__="/opt/tros/cv2.py")

    def missing_distribution(_: str) -> str:
        raise MODULE.importlib.metadata.PackageNotFoundError

    with pytest.raises(RuntimeError, match="cv2"):
        MODULE.collect_python_imports(modules.__getitem__, missing_distribution, lambda: {})
