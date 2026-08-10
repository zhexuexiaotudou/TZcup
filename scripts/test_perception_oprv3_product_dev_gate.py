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
