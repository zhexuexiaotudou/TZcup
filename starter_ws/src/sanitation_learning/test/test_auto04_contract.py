from __future__ import annotations

import numpy as np

from sanitation_learning.auto04_contract import (
    Detection,
    box_iou,
    classwise_nms,
    decode_centernet_outputs,
    encode_centernet_targets,
)


def test_center_target_round_trip() -> None:
    boxes = [{"class_index": 1, "bbox_xyxy": [20.0, 28.0, 60.0, 76.0]}]
    targets = encode_centernet_targets(
        boxes,
        input_width=96,
        input_height=96,
        stride=4,
        class_count=3,
    )
    decoded = decode_centernet_outputs(
        targets["heatmap"],
        targets["offset"],
        targets["size"],
        stride=4,
        score_threshold=0.99,
    )
    assert len(decoded) == 1
    assert decoded[0].class_index == 1
    assert box_iou(decoded[0].bbox_xyxy, tuple(boxes[0]["bbox_xyxy"])) == 1.0


def test_negative_target_has_no_detection() -> None:
    targets = encode_centernet_targets(
        [],
        input_width=96,
        input_height=96,
        stride=4,
        class_count=3,
    )
    assert (
        decode_centernet_outputs(
            targets["heatmap"],
            targets["offset"],
            targets["size"],
            stride=4,
            score_threshold=0.5,
        )
        == []
    )


def test_nms_is_classwise_and_score_ranked() -> None:
    candidates = [
        Detection(0, 0.7, (10.0, 10.0, 30.0, 30.0)),
        Detection(0, 0.9, (11.0, 11.0, 31.0, 31.0)),
        Detection(1, 0.8, (11.0, 11.0, 31.0, 31.0)),
    ]
    kept = classwise_nms(candidates, iou_threshold=0.5)
    assert [item.score for item in kept] == [0.9, 0.8]
    assert [item.class_index for item in kept] == [0, 1]


def test_decode_applies_score_ranked_max_detections() -> None:
    heatmap = np.zeros((1, 4, 4), np.float32)
    heatmap[0, 0, 0] = 0.7
    heatmap[0, 3, 3] = 0.9
    size = np.ones((2, 4, 4), np.float32)
    decoded = decode_centernet_outputs(
        heatmap,
        np.zeros((2, 4, 4), np.float32),
        size,
        stride=4,
        score_threshold=0.5,
        max_detections=1,
    )
    assert len(decoded) == 1
    assert np.isclose(decoded[0].score, 0.9)


def test_invalid_shapes_fail_closed() -> None:
    heatmap = np.zeros((3, 4, 4), np.float32)
    try:
        decode_centernet_outputs(
            heatmap,
            np.zeros((2, 3, 3), np.float32),
            np.zeros((2, 4, 4), np.float32),
            stride=4,
            score_threshold=0.5,
        )
    except ValueError as exc:
        assert "offset shape mismatch" in str(exc)
    else:
        raise AssertionError("invalid output shape must fail")
