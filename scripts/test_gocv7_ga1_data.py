import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_module():
    path = ROOT / "scripts/prepare_gocv7_ga1.py"
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bbox_is_native_xyxy():
    module = load_module()
    mask = np.zeros((8, 10), dtype=bool)
    mask[2:6, 3:9] = True
    assert module.bbox(mask) == [3, 2, 9, 6]
    assert module.bbox(np.zeros_like(mask)) is None


def test_world_isolated_split_contract():
    module = load_module()
    assert module.SPLIT_BY_WORLD_INDEX == {
        0: "GA1_TRAIN",
        1: "GA1_TRAIN",
        2: "GA1_TRAIN",
        3: "GA1_HOLDOUT",
    }
    manifest = {"worlds": [{"world_id": f"world_{index}"} for index in range(4)]}
    assert module.world_indices(manifest)["world_3"] == 3
