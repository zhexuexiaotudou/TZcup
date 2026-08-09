from sanitation_learning.g4_calibration import (
    filter_discovery_frames,
    select_area_threshold,
    select_discovery_threshold,
)

import numpy as np


def _frame(detections, truth=(), negative=False):
    return {
        "detections": list(detections),
        "truth": list(truth),
        "negative_only": negative,
    }


def test_discovery_calibration_prefers_feasible_high_recall_threshold():
    box = [10.0, 10.0, 30.0, 30.0]
    frames = [
        _frame(
            [
                {"score": 0.9, "bbox_xyxy": box},
                {"score": 0.4, "bbox_xyxy": [100.0, 100.0, 130.0, 130.0]},
            ],
            [{"bbox_xyxy": box}],
        ),
        _frame([], negative=True),
    ]
    result = select_discovery_threshold(
        frames,
        thresholds=(0.3, 0.8, 0.95),
        false_candidates_per_min_max=2.0,
        negative_fp_per_frame_max=0.05,
    )
    assert result["threshold"] == 0.8
    assert result["product_eligible"] is True
    assert result["metrics"]["all_gt_candidate_recall"] == 1.0


def test_discovery_filter_does_not_mutate_source_frames():
    frames = [_frame([{"score": 0.2, "bbox_xyxy": [0, 0, 1, 1]}])]
    filtered = filter_discovery_frames(frames, 0.5)
    assert filtered[0]["detections"] == []
    assert len(frames[0]["detections"]) == 1


def test_area_calibration_uses_task_boundary_not_two_channel_mean():
    truth = np.zeros((2, 8, 8), dtype=np.float32)
    truth[0, 2:6, 2:6] = 1.0
    probabilities = np.zeros_like(truth)
    probabilities[0] = truth[0] * 0.9
    result = select_area_threshold(
        [
            {
                "probabilities": probabilities,
                "truth": truth,
                "negative_only": False,
                "thresholds": (0.5, 0.5),
            }
        ],
        "leaf",
        thresholds=(0.5,),
        boundary_f1_min=0.7,
    )
    assert result["iou"] == 1.0
    assert result["boundary_f1"] == 1.0
    assert result["product_eligible"] is True
