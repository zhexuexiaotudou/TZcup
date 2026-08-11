from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _module():
    path = ROOT / "scripts" / "perception_oprv3_moving_dev_gate.py"
    spec = importlib.util.spec_from_file_location("moving_dev_gate", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _encounter(class_name: str, *, size: float = 12.0, detected: bool = True):
    return {
        "class_name": class_name,
        "entered_actionable_window": True,
        "insufficient_sampled_actionable_frames": False,
        "eventual_detection": detected,
        "eventual_correct_class": detected,
        "eventual_track_confirmation": detected,
        "clean_opportunity_missed": not detected,
        "frames": [
            {
                "actionable_window": True,
                "visible_bbox_short_side_px": size,
            }
        ],
    }


def _benchmark(module, *, detected: bool = True):
    coverage = {key: True for key in module.REQUIRED_COVERAGE}
    encounters = [
        _encounter("plastic_bottle", detected=detected),
        _encounter("metal_can", detected=detected),
        _encounter("paper_litter", detected=detected),
        _encounter("leaf_pile", size=30.0, detected=detected),
        _encounter("puddle", size=40.0, detected=detected),
    ]
    return {
        "source_commit": "a" * 40,
        "G5_SEALED_FINAL_read": False,
        "capture_audit": {"mission_count": 20},
        "required_coverage": coverage,
        "routes": {
            "MRV2-A": {
                "encounters": encounters,
                "metrics": {
                    "actionable_predictions": 50,
                    "wrong_actionable_predictions": 0,
                    "negative_frame_actionable_predictions": 0,
                },
            }
        },
    }


def test_complete_unfiltered_moving_matrix_passes():
    module = _module()
    report = module.build_report([_benchmark(module)])
    assert report["OPRV3_02_pass"] is True
    assert report["development_breakdown"]["small_object_eligible_targets"] == 3


def test_missing_special_coverage_fails_closed():
    module = _module()
    benchmark = _benchmark(module)
    benchmark["required_coverage"]["reflection"] = False
    report = module.build_report([benchmark])
    assert report["OPRV3_02_pass"] is False
    assert report["gates"]["required_coverage"] is False


def test_missed_small_target_is_not_excluded():
    module = _module()
    benchmark = _benchmark(module)
    benchmark["routes"]["MRV2-A"]["encounters"][1]["eventual_detection"] = False
    report = module.build_report([benchmark])
    assert report["development_breakdown"]["small_object_eligible_targets"] == 3
    assert report["development_breakdown"]["small_object_eventual_recall"] == 2 / 3
    assert report["OPRV3_02_pass"] is False


def test_sealed_evidence_is_rejected():
    module = _module()
    benchmark = _benchmark(module)
    benchmark["G5_SEALED_FINAL_read"] = True
    try:
        module.build_report([benchmark])
    except ValueError as exc:
        assert "sealed-final" in str(exc)
    else:
        raise AssertionError("sealed evidence was accepted")


def test_mixed_source_revisions_fail_closed():
    module = _module()
    first = _benchmark(module)
    second = _benchmark(module)
    second["source_commit"] = "b" * 40
    report = module.build_report([first, second])
    assert report["gates"]["single_full_source_commit"] is False
    assert report["OPRV3_02_pass"] is False
