#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import importlib.util
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g6_small_specialist import (  # noqa: E402
    SmallSpecialistDataset,
    build_small_specialist_samples,
    ground_roi_tiles,
    map_tile_box_to_native,
)


def test_ground_roi_tiles_overlap_and_cover_actionable_ground() -> None:
    tiles = ground_roi_tiles()
    assert len(tiles) == 6
    assert all(x1 - x0 == 320 and y1 - y0 == 240 for x0, y0, x1, y1 in tiles)
    assert tiles[0] == (0, 120, 320, 360)
    assert tiles[-1] == (320, 240, 640, 480)


def test_specialist_samples_only_train_native_lt18_targets(tmp_path: Path) -> None:
    rgb_path = tmp_path / "frame.png"
    cv2.imwrite(str(rgb_path), np.zeros((480, 640, 3), dtype=np.uint8))
    row = {
        "scene_seed": 1,
        "frame_index": 0,
        "split": "train",
        "rgb_path": rgb_path,
        "negative_area_taxonomies": ["crack"],
    }
    records = {
        (1, 0): [
            {"class_id": "metal_can", "bbox_short_side_px": 10, "bbox_xyxy": [200, 200, 210, 222]},
            {"class_id": "paper_litter", "bbox_short_side_px": 20, "bbox_xyxy": [300, 300, 320, 340]},
            {"class_id": "leaf_pile", "bbox_short_side_px": 9, "bbox_xyxy": [400, 300, 409, 320]},
        ]
    }
    samples = build_small_specialist_samples([row], records, negative_stride=1)
    positives = [sample for sample in samples if not sample["hard_negative"]]
    assert len(positives) == 1
    assert [item["class_id"] for item in positives[0]["targets"]] == ["metal_can"]
    if importlib.util.find_spec("torch") is not None:
        image, target, _ = SmallSpecialistDataset(positives)[0]
        assert tuple(image.shape) == (3, 480, 640)
        assert target["labels"].tolist() == [0]
        assert min(target["boxes"][0][2:] - target["boxes"][0][:2]) == 20


def test_tile_box_round_trip_uses_native_coordinates() -> None:
    assert map_tile_box_to_native([80, 40, 120, 100], (160, 240, 480, 480)) == [
        200.0,
        260.0,
        220.0,
        290.0,
    ]
