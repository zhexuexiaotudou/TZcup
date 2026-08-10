import numpy as np
import pytest
import inspect

from sanitation_learning.g4_direct_fcos import build_direct_fcos
from sanitation_learning.oprv3_moving import (
    actionable_window_eligible,
    bbox_from_mask,
    bbox_iou,
    empirical_special_coverage,
    summarize_encounter,
    summarize_route,
)


def test_actionable_window_eligibility_has_no_model_score_input():
    frozen = {
        "minimum_actionable_range_m": 0.8,
        "maximum_actionable_range_m": 2.0,
        "minimum_visibility_ratio": 0.6,
        "minimum_depth_valid_ratio": 0.8,
    }
    assert actionable_window_eligible(
        visible_bbox=[1, 1, 4, 4],
        distance_m=1.5,
        scene_visibility_ratio=0.7,
        depth_valid_ratio=0.9,
        frozen_window=frozen,
    )
    assert not actionable_window_eligible(
        visible_bbox=[1, 1, 4, 4],
        distance_m=2.1,
        scene_visibility_ratio=0.7,
        depth_valid_ratio=0.9,
        frozen_window=frozen,
    )


def test_checkpoint_detector_load_can_disable_pretrained_download():
    parameter = inspect.signature(build_direct_fcos).parameters[
        "checkpoint_load_control"
    ]
    assert parameter.default is False
    source = inspect.getsource(build_direct_fcos)
    assert "if checkpoint_load_control" in source
    assert "weights_backbone=None" in source
    assert '"strict_checkpoint_load"' in source


def _frame(index, *, actionable=True, action=False, correct=False):
    return {
        "frame_index": index,
        "frame_stamp_ns": index * 100_000_000,
        "distance_m": 2.0 - index * 0.1,
        "visible": True,
        "actionable_window": actionable,
        "action_detection": action,
        "correct_action_detection": correct,
    }


def test_mask_bbox_and_iou_use_half_open_pixel_coordinates():
    mask = np.zeros((10, 10), dtype=bool)
    mask[2:5, 3:7] = True
    assert bbox_from_mask(mask) == [3.0, 2.0, 7.0, 5.0]
    assert bbox_iou([0, 0, 4, 4], [2, 2, 6, 6]) == pytest.approx(4 / 28)


def test_confirmation_requires_three_consecutive_correct_action_frames():
    target = {
        "model_name": "target",
        "class_id": "metal_can",
        "asset_id": "can",
        "xyz_m": [0, 0, 0],
    }
    frames = [
        _frame(0, action=True, correct=True),
        _frame(1, action=False, correct=False),
        _frame(2, action=True, correct=True),
        _frame(3, action=True, correct=True),
        _frame(4, action=True, correct=True),
    ]
    report = summarize_encounter(target, frames, confirmation_count=3)
    assert report["eventual_detection"]
    assert report["eventual_correct_class"]
    assert report["eventual_track_confirmation"]
    assert report["confirmation_frame"] == 4


def test_encounter_requires_frozen_minimum_sampled_actionable_frames():
    target = {
        "model_name": "target",
        "class_id": "metal_can",
        "asset_id": "can",
        "xyz_m": [0, 0, 0],
    }
    report = summarize_encounter(
        target,
        [_frame(0, action=True, correct=True), _frame(1, action=True, correct=True)],
        confirmation_count=3,
        minimum_visible_frames=3,
    )
    assert not report["entered_actionable_window"]
    assert report["sampled_actionable_frame_count"] == 2
    assert report["insufficient_sampled_actionable_frames"]
    assert not report["eventual_detection"]


def test_route_summary_keeps_every_gt_target_and_reports_exclusions():
    entered = {
        "class_name": "plastic_bottle",
        "entered_actionable_window": True,
        "ever_in_camera_frustum": True,
        "eventual_detection": False,
        "eventual_correct_class": False,
        "eventual_track_confirmation": False,
        "missed_in_window": True,
        "distance_to_first_detection_m": None,
        "time_to_first_detection_s": None,
        "occlusion_bucket": "none",
    }
    excluded = {
        **entered,
        "entered_actionable_window": False,
        "ever_in_camera_frustum": False,
        "missed_in_window": False,
        "occlusion_bucket": "partial",
    }
    metrics = summarize_route(
        [entered, excluded],
        {
            "actionable_predictions": 0,
            "wrong_actionable_predictions": 0,
            "wrong_actionable_target_rate": 0.0,
            "negative_frame_actionable_predictions": 0,
        },
    )
    assert metrics["all_gt_targets"] == 2
    assert metrics["entered_actionable_window"] == 1
    assert metrics["never_in_camera_frustum"] == 1
    assert metrics["occluded_entirely"] == 1
    assert metrics["missed_in_window"] == 1
    assert metrics["eventual_detection_recall"] == 0.0


def test_special_coverage_requires_declared_scene_and_empirical_gt_facts():
    context = {
        "scenes": {
            1: {
                "world_id": "world_g4_01_asphalt_campus",
                "oprv3_coverage_requirements": {
                    "turning": True,
                    "behind_vehicle_fov_entry": True,
                },
            },
            2: {
                "world_id": "world_g4_01_asphalt_campus",
                "oprv3_coverage_requirements": {"occlusion": True},
            },
            3: {
                "world_id": "world_g4_03_wet_courtyard",
                "oprv3_coverage_requirements": {"reflection": True},
            },
        },
        "capture_reports": {
            1: {
                "capture_pass": True,
                "observed_absolute_yaw_change_rad": 1.4,
                "records": [
                    {"motion_phase": "turn_into_target_fov"},
                    {"motion_phase": "straight_approach"},
                ],
            },
            2: {"capture_pass": True, "records": []},
            3: {"capture_pass": True, "records": []},
        },
    }
    routes = {
        "MRV2-A": {
            "encounters": [
                {
                    "scene_seed": 1,
                    "occlusion_bucket": "none",
                    "frames": [
                        {"visible": False, "vehicle_yaw_rad": 1.57, "visible_mask_area_px": 0},
                        {"visible": True, "vehicle_yaw_rad": 0.9, "visible_mask_area_px": 12},
                    ],
                },
                {
                    "scene_seed": 2,
                    "occlusion_bucket": "heavy",
                    "frames": [
                        {"visible": False, "vehicle_yaw_rad": 0.0, "visible_mask_area_px": 0, "declared_occluder_bbox_iou": 0.0},
                        {"visible": True, "vehicle_yaw_rad": 0.0, "visible_mask_area_px": 9, "declared_occluder_bbox_iou": 0.08},
                    ],
                },
            ]
        }
    }
    assert empirical_special_coverage(context, routes) == {
        "behind_vehicle_fov_entry": True,
        "turning": True,
        "occlusion": True,
        "reflection": True,
    }
    context["capture_reports"][1]["observed_absolute_yaw_change_rad"] = 0.4
    assert not empirical_special_coverage(context, routes)["turning"]
