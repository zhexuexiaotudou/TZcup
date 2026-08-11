#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g6_small_specialist import fuse_opr_a  # noqa: E402


def test_fusion_classifies_specialist_then_suppresses_same_class_duplicate() -> None:
    general = [
        {"bbox_xyxy": [100, 100, 130, 140], "class_name": "metal_can", "class_index": 2, "score": 0.90}
    ]
    specialist = [
        {"bbox_xyxy": [101, 101, 131, 141], "objectness": 0.95},
        {"bbox_xyxy": [300, 300, 310, 322], "objectness": 0.80},
    ]

    def classify(candidate):
        name = "metal_can" if candidate["bbox_xyxy"][0] < 200 else "paper_litter"
        return {"class_name": name, "class_score": 0.99}

    fused = fuse_opr_a(general, specialist, classify, classifier_threshold=0.8)
    assert len(fused) == 2
    assert {item["class_name"] for item in fused} == {"metal_can", "paper_litter"}
    assert sum(
        item.get("proposal_source") == "OPR-A_small_specialist" for item in fused
    ) == 2


def test_fusion_rejects_background_or_low_confidence_specialist() -> None:
    specialist = [{"bbox_xyxy": [10, 20, 20, 35], "objectness": 0.99}]
    assert fuse_opr_a(
        [], specialist, lambda _: {"class_name": "background", "class_score": 0.99}, classifier_threshold=0.8
    ) == []
    assert fuse_opr_a(
        [], specialist, lambda _: {"class_name": "metal_can", "class_score": 0.4}, classifier_threshold=0.8
    ) == []
