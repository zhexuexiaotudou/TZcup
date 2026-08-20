import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[4] / "scripts" / "audit_rgdrv8_ga1_failures.py"
SPEC = importlib.util.spec_from_file_location("audit_rgdrv8_ga1_failures", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def prediction(class_name, score, bbox):
    return {"class_name": class_name, "score": score, "bbox_xyxy": bbox}


def test_miss_taxonomy_separates_low_score_wrong_class_and_iou():
    truth = {"class_name": "paper_litter", "bbox_xyxy": [10, 10, 20, 20], "bbox_short_side_px": 10}
    cause, facts = MODULE.classify_miss(truth=truth, all_predictions=[prediction("paper_litter", 0.4, [10, 10, 20, 20])], threshold=0.67)
    assert cause == "LOW_SCORE_CORRECT_CLASS"
    assert facts["secondary_taxonomy"] == ["SMALL_OBJECT"]
    cause, _ = MODULE.classify_miss(truth=truth, all_predictions=[prediction("metal_can", 0.8, [10, 10, 20, 20])], threshold=0.67)
    assert cause == "WRONG_CLASS_HIGH_SCORE"
    cause, _ = MODULE.classify_miss(truth=truth, all_predictions=[prediction("paper_litter", 0.8, [14, 14, 24, 24])], threshold=0.67)
    assert cause == "BOX_IOU_FAIL"


def test_false_taxonomy_is_explicitly_visual_heuristic():
    image = np.full((40, 40, 3), 250, dtype=np.uint8)
    image[10:20, 10:15] = 10
    label, evidence = MODULE.visual_false_taxonomy(image, prediction("metal_can", 0.9, [8, 8, 22, 22]), {})
    assert label in MODULE.FALSE_TAXONOMY
    assert "not_background_ground_truth" in evidence["method"]


def test_taxonomy_contracts_are_complete():
    assert set(MODULE.MISS_TAXONOMY) == {"NO_PROPOSAL", "LOW_SCORE_CORRECT_CLASS", "WRONG_CLASS_HIGH_SCORE", "BOX_IOU_FAIL", "SMALL_OBJECT", "OCCLUSION", "REFLECTION", "DARK_OBJECT", "BACKGROUND_BLEND", "MOTION_BLUR", "OTHER"}
    assert set(MODULE.FALSE_TAXONOMY) == {"ROAD_PAINT", "SPECULAR_HIGHLIGHT", "WET_ROAD", "SHADOW", "LEAF_ORGANIC_CLUTTER", "STONE", "METAL_LIKE_BACKGROUND", "PLASTIC_LIKE_BACKGROUND", "PAPER_LIKE_BACKGROUND", "EDGE_OR_SEAM", "UNKNOWN_HARD_NEGATIVE"}
