#!/usr/bin/env python3

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "starter_ws/src/sanitation_learning"))

from sanitation_learning.mrv2_sampling import build_mrv2_epoch_rows


def _row(index, *, negative=False):
    return {
        "world_id": "w", "scene_seed": index // 10, "frame_index": index,
        "negative_only": negative,
    }


def test_mrv2_epoch_sampling_preserves_fixed_ratios_and_uses_replacement():
    rows = [_row(index, negative=20 <= index < 40) for index in range(100)]
    small = {("w", 0, index) for index in range(5)}
    metal = {("w", index // 10, index) for index in range(40, 60)}
    selected, report = build_mrv2_epoch_rows(
        rows, small_keys=small, metal_keys=metal, frame_count=100, seed=17
    )
    assert len(selected) == 100
    assert report["ratios"] == {
        "small_object": 0.30,
        "negative_only": 0.20,
        "metal_can": 0.15,
        "general": 0.35,
    }
    assert report["replacement_used"]["small_object"] is True
    assert report["unique_pool_frames"]["negative_only"] == 20
