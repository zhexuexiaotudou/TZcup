from __future__ import annotations

from pathlib import Path
import sys

import pytest
import numpy as np


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_evaluation import (  # noqa: E402
    average_precision,
    background_specificity,
    decode_discovery_outputs,
)


def _frame(*, truth: bool, detection: bool, score: float = 0.9) -> dict:
    box = [10.0, 10.0, 20.0, 20.0]
    return {
        "truth": [{"bbox_xyxy": box}] if truth else [],
        "detections": (
            [{"bbox_xyxy": box, "score": score}] if detection else []
        ),
    }


def test_ap50_does_not_credit_unreached_recall_levels() -> None:
    frames = [
        _frame(truth=True, detection=True),
        _frame(truth=True, detection=False),
    ]
    # One of two truths is recalled with perfect precision. The 101-point AP
    # therefore covers only recall levels 0.00..0.50 (51 points).
    assert average_precision(frames) == pytest.approx(51.0 / 101.0)


def test_ap50_penalizes_a_higher_ranked_false_positive() -> None:
    frames = [
        _frame(truth=False, detection=True, score=0.99),
        _frame(truth=True, detection=True, score=0.90),
        _frame(truth=True, detection=False),
    ]
    assert average_precision(frames) == pytest.approx(25.5 / 101.0)


def test_background_specificity_is_true_background_rejection_rate() -> None:
    confusion = {
        "background": {"tp": 80, "fp": 5, "fn": 20},
    }
    assert background_specificity(confusion) == pytest.approx(0.80)


def test_decoded_discovery_boxes_are_clipped_to_model_canvas() -> None:
    objectness = np.zeros((1, 120, 160), dtype=np.float32)
    offset = np.zeros((2, 120, 160), dtype=np.float32)
    size = np.zeros((2, 120, 160), dtype=np.float32)
    objectness[0, 119, 159] = 0.9
    offset[:, 119, 159] = 0.9
    size[:, 119, 159] = 10.0
    detections = decode_discovery_outputs(
        objectness,
        offset,
        size,
        score_threshold=0.5,
    )
    assert len(detections) == 1
    x1, y1, x2, y2 = detections[0].bbox_xyxy
    assert 0.0 <= x1 < x2 <= 640.0
    assert 0.0 <= y1 < y2 <= 480.0
