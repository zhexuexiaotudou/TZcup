#!/usr/bin/env python3

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "starter_ws/src/sanitation_learning"))

try:
    import torch  # noqa: F401
except ImportError:
    torch = None

from sanitation_learning.g4_tiled_fcos import class_aware_nms


@pytest.mark.skipif(torch is None, reason="Torch unavailable on Windows host")
def test_global_nms_suppresses_same_class_but_not_different_class():
    items = [
        {"class_name": "metal_can", "score": 0.9, "bbox_xyxy": [0, 0, 10, 10]},
        {"class_name": "metal_can", "score": 0.8, "bbox_xyxy": [1, 1, 11, 11]},
        {"class_name": "paper_litter", "score": 0.7, "bbox_xyxy": [1, 1, 11, 11]},
    ]
    kept = class_aware_nms(items)
    assert [(item["class_name"], item["score"]) for item in kept] == [
        ("metal_can", 0.9), ("paper_litter", 0.7)
    ]
