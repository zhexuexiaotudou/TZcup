from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.mrv2_teacher import (  # noqa: E402
    replace_small_truth_with_teacher,
    select_small_teacher_pseudo_labels,
)


def test_teacher_pseudo_labels_are_high_confidence_small_and_train_keyed():
    key = ("train_world", 1, 2)
    truth = {
        key: [
            {"semantic_class": "metal_can", "bbox_xyxy": [10, 10, 20, 26]},
            {"semantic_class": "paper_litter", "bbox_xyxy": [40, 40, 80, 90]},
        ]
    }
    frames = [{
        "world_id": key[0], "scene_seed": key[1], "frame_index": key[2],
        "detections": [
            {"score": 0.91, "bbox_xyxy": [10, 10, 20, 26]},
            {"score": 0.69, "bbox_xyxy": [40, 40, 80, 90]},
        ],
    }]
    selected, report = select_small_teacher_pseudo_labels(frames, truth)
    assert list(selected) == [key]
    assert selected[key][0]["semantic_class"] == "metal_can"
    assert selected[key][0]["pseudo_label_role"].startswith("train_only")
    assert report["small_train_truth"] == 1
    assert report["pseudo_label_count"] == 1


def test_teacher_geometry_replaces_matching_truth_without_duplication():
    truth = [
        {"semantic_class": "metal_can", "bbox_xyxy": [10, 10, 20, 20]},
        {"semantic_class": "paper_litter", "bbox_xyxy": [30, 30, 50, 50]},
    ]
    pseudo = [{
        "semantic_class": "metal_can", "bbox_xyxy": [9, 9, 21, 21],
        "teacher_score": 0.93,
    }]
    replaced = replace_small_truth_with_teacher(truth, pseudo)
    assert len(replaced) == 2
    assert replaced[0]["bbox_xyxy"] == [9, 9, 21, 21]
    assert replaced[1] == truth[1]
