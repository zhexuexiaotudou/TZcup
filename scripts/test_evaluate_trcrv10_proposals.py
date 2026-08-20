import numpy as np

import evaluate_trcrv10_proposals as proposal


def test_truth_bbox_uses_all_target_labels() -> None:
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[3:11, 5:17] = 2
    assert proposal.truth_bbox(mask) == {"bbox_xyxy": [5, 3, 17, 11], "short_side_px": 8}


def test_persistence_and_class_agnostic_metrics() -> None:
    truth = {}
    predictions = {}
    for index in range(5):
        truth[("positive", index)] = {"bbox_xyxy": [0, 0, 10, 10], "short_side_px": 10 + index}
        predictions[("positive", index)] = [{"bbox_xyxy": [0, 0, 10, 10], "score": 0.8, "label": 2}]
        truth[("negative", index)] = None
        predictions[("negative", index)] = []
    result = proposal.evaluate_records(truth, predictions, threshold=0.5, persistence=3)
    assert result["metrics"]["eventual_proposal_recall"] == 1.0
    assert result["metrics"]["small_eventual_proposal_recall"] == 1.0
    assert result["metrics"]["proposal_fp_per_frame"] == 0.0
    assert result["pass"]


def test_flood_gate_is_fail_closed() -> None:
    truth = {("positive", index): {"bbox_xyxy": [0, 0, 10, 10], "short_side_px": 10} for index in range(2)}
    predictions = {
        key: [
            {"bbox_xyxy": [0, 0, 10, 10], "score": 0.9},
            {"bbox_xyxy": [20, 20, 30, 30], "score": 0.9},
            {"bbox_xyxy": [35, 35, 40, 40], "score": 0.9},
        ]
        for key in truth
    }
    result = proposal.evaluate_records(truth, predictions, threshold=0.5, persistence=2)
    assert result["metrics"]["proposal_fp_per_frame"] == 2.0
    assert not result["gates"]["proposal_flood_hard_limit"]
    assert not result["pass"]


def test_raw_inference_contract_accepts_historical_mission_id() -> None:
    source = open(proposal.__file__, encoding="utf-8").read()
    assert 'row.get("scene", row.get("mission_id"))' in source
