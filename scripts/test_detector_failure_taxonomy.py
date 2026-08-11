from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from perception_ddrv4_failure_taxonomy import CATEGORIES, aggregate_frame, classify_truth, instance_index, summarize


def truth(**overrides):
    value = {
        "bbox_xyxy": [10, 10, 30, 30], "label": 2, "class_id": "metal_can",
        "size": "medium_18_48", "distance": "near_0_3m", "distance_m": 2.0,
        "material": "dark_can", "lighting": "shadow", "world": "g7v4_test",
        "occluded": False, "annotation_valid": True,
    }
    value.update(overrides)
    return value


def prediction(score=0.8, label=2, bbox=None):
    return {"bbox_xyxy": bbox or [10, 10, 30, 30], "score": score, "label": label}


def test_failure_categories_are_complete_and_deterministic():
    assert set(CATEGORIES) == {"NO_PROPOSAL", "SCORE_BELOW_THRESHOLD", "BOX_IOU_FAIL", "WRONG_CLASS", "DUPLICATE_NMS", "BACKGROUND_CONFUSION", "OCCLUDED", "OUT_OF_EFFECTIVE_RANGE", "ANNOTATION_QA_FAILURE"}
    assert classify_truth(truth(), [prediction()]) == "MATCH"
    assert classify_truth(truth(), [prediction(score=0.2)]) == "SCORE_BELOW_THRESHOLD"
    assert classify_truth(truth(), [prediction(label=1)]) == "WRONG_CLASS"
    assert classify_truth(truth(), [prediction(bbox=[22, 10, 42, 30])]) == "BOX_IOU_FAIL"
    assert classify_truth(truth(occluded=True), []) == "OCCLUDED"
    assert classify_truth(truth(distance_m=6.5), []) == "OUT_OF_EFFECTIVE_RANGE"
    assert classify_truth(truth(annotation_valid=False), []) == "ANNOTATION_QA_FAILURE"
    assert classify_truth(truth(), []) == "NO_PROPOSAL"


def test_false_predictions_are_background_or_duplicate():
    frame = aggregate_frame([truth()], [prediction(), prediction(score=0.7), prediction(label=1, bbox=[100, 100, 120, 120])], ["wet_road"])
    categories = [item["category"] for item in frame["events"]]
    assert categories.count("DUPLICATE_NMS") == 1
    assert categories.count("BACKGROUND_CONFUSION") == 1


def test_summary_reports_dimensions_and_raw_proposal_recall():
    report = summarize("G7_DETECTOR_DEVELOPMENT", "IN_DOMAIN_HOLDOUT", [{"truth": [truth()], "predictions": [prediction(score=0.2)], "background_taxonomy": []}])
    assert report["recall"] == 0.0
    assert report["proposal_recall_at_0_001"] == 1.0
    assert report["failure_taxonomy"]["SCORE_BELOW_THRESHOLD"] == 1
    assert report["dimensions"]["material"]["dark_can"]["SCORE_BELOW_THRESHOLD"] == 1


def test_area_instances_are_excluded_from_detector_taxonomy():
    assert instance_index([{"class_id": "leaf_pile", "scene_seed": 1, "frame_index": 0}]) == {}
