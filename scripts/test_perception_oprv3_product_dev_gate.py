import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("perception_oprv3_product_dev_gate.py")
SPEC = importlib.util.spec_from_file_location("oprv3_product_dev_gate", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _moving():
    return {
        "OPRV3_02_pass": True,
        "source_commits": ["a" * 40],
        "G5_SEALED_FINAL_read": False,
        "aggregate_mrv2_a": {
            "eventual_detection_recall": 1.0,
            "eventual_correct_class_recall": 1.0,
            "actionable_predictions": 100,
            "wrong_actionable_predictions": 0,
            "wrong_actionable_target_rate": 0.0,
        },
        "development_breakdown": {
            "small_object_eventual_recall": 1.0,
            "per_class_eventual_detection_recall": {
                "metal_can": 1.0,
                "paper_litter": 1.0,
            },
        },
    }


def _area():
    return {
        "G5_SEALED_FINAL_read": False,
        "cross_world_aggregate": {
            "area": {
                "iou_by_class": {"leaf_pile": 0.91, "puddle": 0.72},
                "macro_miou": 0.82,
                "boundary_f1": 0.64,
                "negative_area_fp_per_frame": 0.13,
            }
        },
    }


def test_gate_passes_moving_object_metrics_but_fails_unknown_and_area_sections():
    report = MODULE.build_report(_moving(), _area())
    assert report["sections"]["object_level_online_discovery"]["pass"]
    assert not report["sections"]["map_and_track"]["pass"]
    assert not report["sections"]["area"]["pass"]
    assert not report["sections"]["performance"]["pass"]
    assert not report["OPRV3_X86_DEV_PASS"]
    assert report["freeze_allowed"] is False
    assert report["next_action"].startswith("OPRV3-06")


def test_gate_rejects_unpassed_moving_or_sealed_input():
    moving = _moving()
    moving["OPRV3_02_pass"] = False
    with pytest.raises(ValueError, match="has not passed"):
        MODULE.build_report(moving, _area())
    moving["OPRV3_02_pass"] = True
    moving["G5_SEALED_FINAL_read"] = True
    with pytest.raises(ValueError, match="sealed-final"):
        MODULE.build_report(moving, _area())


def test_gate_accepts_only_complete_product_map_and_performance_evidence():
    area = _area()
    area["cross_world_aggregate"]["area"].update(
        {
            "iou_by_class": {"leaf_pile": 0.91, "puddle": 0.92},
            "macro_miou": 0.915,
            "boundary_f1": 0.76,
            "negative_area_fp_per_frame": 0.04,
        }
    )
    product_map = {
        "G5_SEALED_FINAL_read": False,
        "GT_used_by_product_pipeline": False,
        "metrics": {
            "product_target_precision": 1.0,
            "false_confirmed_target_rate": 0.0,
            "map_localization_coverage": 1.0,
            "map_rmse_m": 0.05,
            "id_consistency": 0.99,
            "duplicate_target_rate": 0.0,
            "track_fragmentation": 0.0,
            "removed_target_stale_action": 0,
            "wrong_class_leading_to_wrong_clean_action": 0,
            "pre_fov_target_creation": 0,
        }
    }
    performance = {
        "G5_SEALED_FINAL_read": False,
        "metrics": {
            "effective_hz": 12.0,
            "end_to_end_p95_ms": 120.0,
            "drop_rate": 0.0,
            "formal_product_pipeline_executed": True,
        }
    }
    report = MODULE.build_report(
        _moving(), area, product_map, performance
    )
    assert report["sections"]["precision_and_wrong_behavior"]["pass"]
    assert report["sections"]["official_object_recognition_mapping"]["pass"]
    assert report["sections"]["official_object_recognition_mapping"]["metrics"][
        "object_level_f1"
    ] == 1.0
    assert report["sections"]["map_and_track"]["pass"]
    assert report["sections"]["performance"]["pass"]
    assert report["OPRV3_X86_DEV_PASS"]
    assert report["freeze_allowed"]
    assert report["next_action"] == "create OPRV3-08 x86 freeze"


def test_false_confirmed_product_target_fails_precision_section():
    area = _area()
    area["cross_world_aggregate"]["area"].update(
        {
            "iou_by_class": {"leaf_pile": 0.91, "puddle": 0.92},
            "macro_miou": 0.915,
            "boundary_f1": 0.76,
            "negative_area_fp_per_frame": 0.04,
        }
    )
    product_map = {
        "G5_SEALED_FINAL_read": False,
        "GT_used_by_product_pipeline": False,
        "metrics": {
            "product_target_precision": 0.80,
            "false_confirmed_target_rate": 0.20,
            "map_localization_coverage": 1.0,
            "map_rmse_m": 0.05,
            "id_consistency": 1.0,
            "duplicate_target_rate": 0.0,
            "track_fragmentation": 0.0,
            "removed_target_stale_action": 0,
            "wrong_class_leading_to_wrong_clean_action": 0,
            "pre_fov_target_creation": 0,
        },
    }
    report = MODULE.build_report(_moving(), area, product_map)
    assert report["sections"]["precision_and_wrong_behavior"]["pass"] is False
    assert report["sections"]["official_object_recognition_mapping"]["pass"] is False
    assert report["sections"]["precision_and_wrong_behavior"]["gates"][
        "actionable_target_precision"
    ] is False
    assert report["OPRV3_X86_DEV_PASS"] is False
