import json
from pathlib import Path

import numpy as np
import pytest

from sanitation_perception.formal_random_scene_evaluator_core import (
    BoxObservation,
    EvaluationContractError,
    TruthBox,
    finalize_acceptance,
    load_evaluator_truth,
    match_boxes,
    projection_error_metrics,
    rasterize_dirt_truth,
    segmentation_metrics,
)


def test_truth_loader_rejects_non_evaluator_or_control_enabled_truth(tmp_path: Path):
    path = tmp_path / "truth.json"
    truth = {
        "schema_version": 1,
        "namespace": "/evaluation/scenario_ground_truth",
        "control_use_prohibited": True,
        "discrete_cubes": [],
        "dirt_patches": [],
    }
    path.write_text(json.dumps(truth), encoding="utf-8")
    assert load_evaluator_truth(path)["control_use_prohibited"] is True
    truth["namespace"] = "/perception/ground_truth"
    path.write_text(json.dumps(truth), encoding="utf-8")
    with pytest.raises(EvaluationContractError):
        load_evaluator_truth(path)
    truth["namespace"] = "/evaluation/truth"
    truth["control_use_prohibited"] = False
    path.write_text(json.dumps(truth), encoding="utf-8")
    with pytest.raises(EvaluationContractError):
        load_evaluator_truth(path)


def test_box_matching_is_class_aware_and_one_to_one():
    truth = [TruthBox("cube-a", "litter_cube", (10, 10, 30, 30))]
    predictions = [
        BoxObservation("litter_cube", 0.9, (10, 10, 30, 30)),
        BoxObservation("litter_cube", 0.8, (11, 11, 29, 29)),
        BoxObservation("puddle", 0.7, (10, 10, 30, 30)),
    ]
    result = match_boxes(predictions, truth, iou_threshold=0.5)
    assert result["true_positive_count"] == 1
    assert result["false_positive_count"] == 2
    assert result["matches"][0]["truth_object_id"] == "cube-a"


def test_rotated_dirt_raster_and_segmentation_metrics():
    patches = [
        {
            "pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.78539816339},
            "size_m": [2.0, 0.5],
        }
    ]
    truth = rasterize_dirt_truth(
        patches, width=80, height=80, resolution=0.05, origin_x=-2.0, origin_y=-2.0
    )
    assert 360 <= np.count_nonzero(truth) <= 440
    perfect = segmentation_metrics(truth, truth)
    assert perfect["iou"] == 1.0
    assert perfect["recall"] == 1.0
    empty = segmentation_metrics(np.zeros_like(truth), truth)
    assert empty["iou"] == 0.0
    assert empty["recall"] == 0.0


def test_finalize_is_fail_closed_without_real_runtime_evidence():
    report = finalize_acceptance(
        episode_id="formal-map-000-mission-000",
        detection={
            "true_positive_count": 8,
            "false_positive_count": 0,
            "visible_unique_truth_count": 10,
            "matched_unique_truth_count": 8,
            "evaluated_frame_count": 10,
        },
        segmentation={"iou": 0.7, "recall": 0.9},
        projection=projection_error_metrics([0.05, 0.10]),
        freshness={
            "rgb_topic_count": 4,
            "depth_topic_count": 2,
            "camera_info_topic_count": 4,
            "depth_rgb_skew_max_s": 0.1,
            "tf_success_ratio": 1.0,
            "tf_age_max_s": 0.1,
            "diagnostic_ground_truth_input_used": False,
            "real_camera_message_count": 0,
        },
    )
    assert report["status"] == "BLOCKED_ACCURACY_OR_RUNTIME"
    assert report["truth_isolation"]["synthetic_offline_image_used"] is False
    assert not report["metric_checks"]["real_camera_messages_seen"]
    assert not report["metric_checks"]["visible_cube_truth_present"]
    assert "visible_cube_truth_present" in report["blocked_checks"]["runtime"]


def test_finalize_requires_each_product_output_stream_during_sampling():
    freshness = {
        "rgb_topic_count": 4,
        "depth_topic_count": 2,
        "camera_info_topic_count": 4,
        "depth_rgb_skew_max_s": 0.1,
        "tf_success_ratio": 1.0,
        "tf_age_max_s": 0.1,
        "diagnostic_ground_truth_input_used": False,
        "real_camera_message_count": 10,
        "product_detection_message_count": 0,
        "product_mask_message_count": 0,
        "product_target_message_count": 0,
    }
    report = finalize_acceptance(
        episode_id="formal-map-000-mission-000",
        detection={
            "true_positive_count": 8,
            "false_positive_count": 0,
            "visible_unique_truth_count": 10,
            "matched_unique_truth_count": 8,
            "evaluated_frame_count": 10,
        },
        segmentation={"iou": 0.7, "recall": 0.9},
        projection=projection_error_metrics([0.05, 0.10]),
        freshness=freshness,
    )
    assert report["blocked_checks"]["runtime"] == [
        "product_detection_messages_seen",
        "product_mask_messages_seen",
        "product_target_messages_seen",
    ]


def test_finalize_blocks_when_no_truth_isolated_3d_track_projection_exists():
    report = finalize_acceptance(
        episode_id="formal-map-000-mission-000",
        detection={
            "true_positive_count": 8,
            "false_positive_count": 0,
            "visible_unique_truth_count": 10,
            "matched_unique_truth_count": 8,
            "evaluated_frame_count": 10,
        },
        segmentation={"iou": 0.7, "recall": 0.9},
        projection=projection_error_metrics([]),
        freshness={
            "rgb_topic_count": 4,
            "depth_topic_count": 2,
            "camera_info_topic_count": 4,
            "depth_rgb_skew_max_s": 0.1,
            "tf_success_ratio": 1.0,
            "tf_age_max_s": 0.1,
            "diagnostic_ground_truth_input_used": False,
            "real_camera_message_count": 10,
            "product_detection_message_count": 1,
            "product_mask_message_count": 1,
            "product_target_message_count": 1,
        },
    )
    assert report["status"] == "BLOCKED_ACCURACY_OR_RUNTIME"
    assert report["metric_checks"]["map_projection_samples_present"] is False
    assert report["metric_checks"]["map_projection_rmse"] is False
    assert report["metric_checks"]["map_projection_p95"] is False
    assert "map_projection_samples_present" in report["blocked_checks"]["accuracy"]
