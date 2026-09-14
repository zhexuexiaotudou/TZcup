import importlib.util
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/replay_controlled_perception_policy.py"
SPEC = importlib.util.spec_from_file_location("replay_controlled_perception_policy", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_static_transform_lookup_uses_parent_child_convention():
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 2.0, 3.0]
    edges = [("map", "camera", matrix)]
    direct = MODULE.lookup_static_transform(edges, "map", "camera")
    reverse = MODULE.lookup_static_transform(edges, "camera", "map")
    assert np.allclose(direct, matrix)
    assert np.allclose(reverse, MODULE.rigid_inverse(matrix))
    assert np.allclose(reverse @ direct, np.eye(4))


def test_static_transform_lookup_composes_multiple_hops():
    map_to_base = np.eye(4)
    map_to_base[:3, 3] = [1.0, 0.0, 0.0]
    base_to_camera = np.eye(4)
    base_to_camera[:3, 3] = [0.0, 2.0, 0.0]
    result = MODULE.lookup_static_transform(
        [("map", "base_footprint", map_to_base), ("base_footprint", "camera", base_to_camera)],
        "map",
        "camera",
    )
    assert np.allclose(result[:3, 3], [1.0, 2.0, 0.0])
