#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("route_a", ROOT / "scripts/prepare_rgdrv8_route_a.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader
SPEC.loader.exec_module(MODULE)


def test_sampling_ratios_follow_protocol() -> None:
    assert MODULE.RATIOS == {"small_positive": 0.25, "metal_targeted": 0.20, "general_positive": 0.25, "hard_negative": 0.30}
    assert sum(MODULE.RATIOS.values()) == 1.0


def test_category_prioritizes_small_then_metal() -> None:
    image = {"id": 1}
    assert MODULE.category(image, []) == "hard_negative"
    assert MODULE.category(image, [{"category_id": 2, "bbox": [0, 0, 40, 40], "bbox_short_side_px": 12}]) == "small_positive"
    assert MODULE.category(image, [{"category_id": 2, "bbox": [0, 0, 40, 40], "bbox_short_side_px": 40}]) == "metal_targeted"
    assert MODULE.category(image, [{"category_id": 3, "bbox": [0, 0, 40, 40], "bbox_short_side_px": 40}]) == "general_positive"
