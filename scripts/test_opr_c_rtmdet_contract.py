#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.opr_c_rtmdet import bounded_frames, index_instances, to_coco  # noqa: E402


def test_opr_c_coco_conversion_preserves_negative_frames_and_closed_classes() -> None:
    frames = [
        {"scene_seed": 1, "frame_index": 0, "rgb_path": "rgb/a.png", "world_id": "w1", "negative_only": False},
        {"scene_seed": 1, "frame_index": 1, "rgb_path": "rgb/b.png", "world_id": "w1", "negative_only": True},
    ]
    instances = index_instances(
        [
            {
                "scene_seed": 1,
                "frame_index": 0,
                "class_id": "metal_can",
                "bbox_xyxy": [1, 2, 6, 9],
                "bbox_short_side_px": 5,
                "visible": True,
            }
        ]
    )
    payload = to_coco(frames, instances)
    assert len(payload["images"]) == 2
    assert len(payload["annotations"]) == 1
    assert payload["annotations"][0]["category_id"] == 2
    assert payload["annotations"][0]["bbox"] == [1.0, 2.0, 5.0, 7.0]


def test_opr_c_bounded_frames_keeps_hard_negatives() -> None:
    frames = [
        {"scene_seed": 1, "frame_index": index, "negative_only": index >= 8}
        for index in range(10)
    ]
    instances = {(1, index): [{"class_id": "paper_litter"}] for index in range(8)}
    selected = bounded_frames(frames, instances, maximum=5, seed=7)
    assert len(selected) == 5
    assert sum(not instances.get((1, row["frame_index"])) for row in selected) == 1
