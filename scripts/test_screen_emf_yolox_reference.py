import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from screen_emf_yolox_reference import (
    ScreeningError,
    build_dataset,
    evaluate_threshold,
    reject_forbidden_path,
    sha256,
    yolox_preprocess,
)


def test_forbidden_data_guards_cover_generic_and_named_sets():
    for path in ("x/G5/train", "x/G5_V2/train", "x/VAL_NEW/train", "x/SEALED_FINAL"):
        with pytest.raises(ScreeningError):
            reject_forbidden_path(path)
    reject_forbidden_path("x/g10/train_development")


def test_yolox_preprocess_is_top_left_bgr_114_without_rescale():
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    image[:, :, 0] = 7
    tensor, ratio = yolox_preprocess(image)
    assert tensor.shape == (1, 3, 416, 416)
    assert tensor.dtype == np.float32
    assert ratio == pytest.approx(2.08)
    assert tensor[0, 0, 0, 0] == 7.0
    assert tensor[0, 0, 300, 0] == 114.0


def test_dataset_rebases_locks_and_preserves_source_metadata(tmp_path: Path):
    data_root = tmp_path / "data"
    image_path = data_root / "world" / "rgb" / "frame.png"
    image_path.parent.mkdir(parents=True)
    cv2.imwrite(str(image_path), np.zeros((10, 20, 3), dtype=np.uint8))
    coco = {
        "info": {},
        "categories": [
            {"id": 1, "name": "plastic_bottle"},
            {"id": 2, "name": "metal_can"},
            {"id": 3, "name": "paper_litter"},
        ],
        "images": [{
            "id": 1,
            "file_name": "OLD\\world\\rgb\\frame.png",
            "width": 20,
            "height": 10,
            "mission_id": "scene_1",
            "world_id": "world_1",
            "scene_seed": 1,
            "frame_index": 0,
            "negative_only": False,
            "source_split": "train",
        }],
        "annotations": [{"id": 1, "image_id": 1, "category_id": 1, "bbox": [1, 2, 3, 4]}],
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    coco_path = tmp_path / "train.json"
    coco_path.write_text(json.dumps(coco), encoding="utf-8")
    manifest = build_dataset(
        coco_path,
        expected_coco_sha256=sha256(coco_path),
        source_prefix="OLD",
        data_root=data_root,
    )
    assert manifest["image_count"] == 1
    assert manifest["annotation_count"] == 1
    assert manifest["target_counts"]["plastic_bottle"] == 1
    assert manifest["images"][0]["world_id"] == "world_1"
    assert len(manifest["images"][0]["sha256"]) == 64


def test_reference_metrics_never_claim_product_semantics():
    dataset = {
        "annotation_count": 1,
        "image_count": 2,
        "unannotated_frame_count": 1,
        "target_counts": {"plastic_bottle": 1, "metal_can": 0, "paper_litter": 0},
        "images": [
            {"image_id": 1, "annotations": [{"category_name": "plastic_bottle", "bbox_xyxy": [0, 0, 10, 10]}]},
            {"image_id": 2, "annotations": []},
        ],
    }
    inference = {
        1: [{"bbox_xyxy": [0, 0, 10, 10], "confidence": 0.8}],
        2: [{"bbox_xyxy": [0, 0, 5, 5], "confidence": 0.8}],
    }
    metrics = evaluate_threshold(dataset, inference, 0.5)
    assert metrics["proposal_recall_class_agnostic"] == 1.0
    assert metrics["proposal_false_positives_per_frame"] == 0.5
    assert metrics["semantic_precision_recall_f1"] == "not_applicable_semantic_mapping_absent"
    assert metrics["product_candidate_pass"] is False
