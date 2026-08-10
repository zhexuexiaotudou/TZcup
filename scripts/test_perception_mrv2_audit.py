#!/usr/bin/env python3
"""Pure contracts for the MRV2-00 audit."""

from __future__ import annotations

import numpy as np

from perception_mrv2_audit import (
    apply_morphology,
    classify_metal_truth,
    finalize_area,
    size_bin,
)


def test_size_bins_are_fixed_at_required_boundaries():
    assert [size_bin(value) for value in (0, 7.99, 8, 11.99, 12, 17.99, 18, 31.99, 32, 47.99, 48)] == [
        "lt_8", "lt_8", "8_to_12", "8_to_12", "12_to_18", "12_to_18",
        "18_to_32", "18_to_32", "32_to_48", "32_to_48", "ge_48",
    ]


def test_morphology_is_deterministic_and_boolean():
    mask = np.zeros((9, 9), dtype=bool)
    mask[2:7, 2:7] = True
    mask[0, 0] = True
    opened = apply_morphology(mask, "open3")
    assert opened.dtype == bool
    assert not opened[0, 0]
    assert opened[4, 4]


def test_metal_outcomes_distinguish_wrong_class_threshold_iou_and_miss():
    truth = {"bbox_xyxy": [0, 0, 10, 10]}
    correct_low = {"class_name": "metal_can", "score": 0.4, "bbox_xyxy": [0, 0, 10, 10]}
    wrong = {"class_name": "plastic_bottle", "score": 0.9, "bbox_xyxy": [0, 0, 10, 10]}
    poor_box = {"class_name": "metal_can", "score": 0.9, "bbox_xyxy": [6, 0, 16, 10]}
    assert classify_metal_truth(truth, [correct_low], [], 0.6) == "score_below_threshold"
    assert classify_metal_truth(truth, [wrong], [wrong], 0.6) == "detector_found_but_wrong_class"
    assert classify_metal_truth(truth, [poor_box], [], 0.6) == "box_iou_below_0_5"
    assert classify_metal_truth(truth, [], [], 0.6) == "detector_missed"


def test_area_report_keeps_raw_and_postprocessed_boundary_separate():
    acc = {
        "intersection": [8, 9], "union": [10, 10],
        "boundary_intersection": [3, 4], "boundary_union": [5, 6],
        "raw_boundary_intersection": [1, 2], "raw_boundary_union": [7, 8],
        "truth_pixels": [100, 200],
        "negative_frames": 10, "negative_fp_frames": 1,
    }
    report = finalize_area(acc)
    assert report["postprocessed_mask_boundary_f1"] != report["raw_network_boundary_head_f1"]
    assert report["negative_area_fp_per_frame"] == 0.1
