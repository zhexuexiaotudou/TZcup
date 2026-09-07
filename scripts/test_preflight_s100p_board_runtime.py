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


def test_collect_python_imports_requires_every_module_version_and_absolute_path():
    modules = {name: SimpleNamespace(__version__="1.0", __file__=f"/opt/tros/{name}.py") for name in MODULE.PYTHON_IMPORTS}
    assert set(MODULE.collect_python_imports(modules.__getitem__)) == set(MODULE.PYTHON_IMPORTS)
    modules["cv2"] = SimpleNamespace(__version__="", __file__="/opt/tros/cv2.py")
    with pytest.raises(RuntimeError, match="cv2"):
        MODULE.collect_python_imports(modules.__getitem__)
