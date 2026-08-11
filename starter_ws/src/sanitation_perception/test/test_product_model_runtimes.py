from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from sanitation_perception.area_runtime import decode_area, preprocess_area
from sanitation_perception.classifier_runtime import (
    classifier_batch_input,
    classifier_input,
    classify_candidate,
    classify_candidates,
)
from sanitation_perception.detector_runtime import decode_discovery, preprocess_discovery
from sanitation_perception.product_postprocess import (
    project_area_predictions,
    project_discrete_predictions,
)


def camera(width=640, height=480):
    return {
        "width": width,
        "height": height,
        "fx": 343.0,
        "fy": 343.0,
        "cx": width / 2.0,
        "cy": height / 2.0,
        "pixel_sigma": 0.5,
        "depth_sigma_m": 0.02,
    }


def test_discovery_decode_and_classifier_acceptance_are_multi_instance() -> None:
    rgb = np.zeros((480, 640, 3), np.uint8)
    assert preprocess_discovery(rgb).shape == (1, 3, 480, 640)
    flat = np.full((1, 15, 120, 160), -20.0, np.float32)
    flat[0, 0, 20, 30] = 10.0
    flat[0, 3:5, 20, 30] = 0.5
    flat[0, 9:11, 20, 30] = (10.0, 8.0)
    flat[0, 0, 70, 90] = 9.0
    flat[0, 3:5, 70, 90] = 0.5
    flat[0, 9:11, 70, 90] = (12.0, 9.0)
    candidates = decode_discovery(
        flat, score_threshold=0.8, nms_iou_threshold=0.5
    )
    assert len(candidates) == 2
    assert classifier_input(rgb, candidates[0]["bbox_xyxy"]).shape == (
        1, 3, 192, 192
    )
    accepted = classify_candidate(
        np.array([[0.0, 0.1, 4.0, 0.2]], np.float32),
        candidates[0],
        score_threshold=0.75,
    )
    assert accepted["accepted"] is True
    assert accepted["class_id"] == "metal_can"
    batch = classifier_batch_input(rgb, candidates, fixed_batch_size=16)
    assert batch.shape == (16, 3, 192, 192)
    logits = np.zeros((16, 4), np.float32)
    logits[:2, 2] = 4.0
    assert len(
        classify_candidates(logits, candidates, score_threshold=0.75)
    ) == 2


def test_candidate_flood_is_bounded_before_single_classifier_batch() -> None:
    flat = np.full((1, 15, 120, 160), -20.0, np.float32)
    for index in range(40):
        y, x = 2 + (index // 8) * 20, 2 + (index % 8) * 20
        flat[0, 0, y, x] = 10.0
        flat[0, 3:5, y, x] = 0.5
        flat[0, 9:11, y, x] = 6.0
    candidates = decode_discovery(
        flat,
        score_threshold=0.8,
        nms_iou_threshold=0.5,
        maximum_candidates=16,
    )
    assert len(candidates) == 16


def test_area_preprocess_matches_training_contract_and_decodes_native_mask() -> None:
    learning = Path(__file__).resolve().parents[2] / "sanitation_learning"
    if str(learning) not in sys.path:
        sys.path.insert(0, str(learning))
    from sanitation_learning.g4_data import build_area_input

    yy, xx = np.mgrid[0:480, 0:640]
    rgb = np.stack((xx % 255, yy % 255, (xx + yy) % 255), axis=-1).astype(np.uint8)
    depth = (2.0 + yy * 0.001 + xx * 0.0002).astype(np.float32)
    for task in ("leaf", "puddle"):
        live, geometry = preprocess_area(rgb, depth, camera(), task=task)
        training = build_area_input(rgb, depth, task=task, camera_info=camera())
        assert live.shape == (1, 10, 384, 512)
        assert np.allclose(live[0], training.transpose(2, 0, 1), atol=1e-6)
        assert geometry["valid_depth_ratio"] == pytest.approx(1.0)
    output = np.full((1, 2, 384, 512), -10.0, np.float32)
    output[0, 0, 100:200, 150:300] = 10.0
    decoded = decode_area(output, mask_threshold=0.8, native_size=(640, 480))
    assert decoded["mask"].shape == (480, 640)
    assert int(decoded["mask"].sum()) > 0


def test_projection_uses_predictions_and_rejects_invalid_depth() -> None:
    depth = np.full((480, 640), 2.0, np.float32)
    transform = np.eye(4)
    prediction = {
        "bbox_xyxy": (280.0, 200.0, 360.0, 280.0),
        "class_id": "plastic_bottle",
        "confidence": 0.9,
        "class_probabilities": {"plastic_bottle": 0.9, "background": 0.1},
    }
    discrete = project_discrete_predictions(
        [prediction], depth, camera(), transform
    )
    assert len(discrete) == 1
    assert discrete[0]["source_backend"] == "onnxruntime"
    probability = np.zeros((480, 640), np.float32)
    probability[260:340, 220:400] = 0.95
    areas = {
        "leaf": {"mask": probability > 0.8, "probability": probability},
        "puddle": {
            "mask": np.zeros_like(probability, dtype=bool),
            "probability": np.zeros_like(probability),
        },
    }
    regions = project_area_predictions(
        areas,
        depth,
        camera(),
        transform,
        minimum_pixels=20,
        minimum_physical_area_m2=0.05,
        minimum_physical_area_m2_by_class={
            "leaf_pile": 0.02,
            "puddle": 0.05,
        },
    )
    assert len(regions) == 1
    assert len(regions[0]["polygon_xy_m"]) >= 3
    assert regions[0]["physical_area_m2"] > 0.0
    invalid = np.zeros_like(depth)
    assert project_discrete_predictions(
        [prediction], invalid, camera(), transform
    ) == []
