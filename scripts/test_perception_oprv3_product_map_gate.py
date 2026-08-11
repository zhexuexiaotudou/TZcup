import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("perception_oprv3_product_map_gate.py")
SPEC = importlib.util.spec_from_file_location("oprv3_product_map_gate", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _benchmark(error=0.05):
    counts = {
        "localization_squared_error_sum": error * error * 5,
        "localization_error_count": 5,
        "identity_consistency_sum": 4.95,
        "identity_target_count": 5,
        "duplicate_target_count": 0,
        "fragmented_target_count": 0,
        "eligible_target_count": 5,
        "matched_target_count": 5,
        "confirmed_product_target_count": 5,
        "wrong_class_confirmed_action_count": 0,
        "pre_fov_creation_count": 0,
        "removed_target_stale_action_count": 0,
        "removal_capture_count": 0,
        "projection_frame_failure_count": 0,
    }
    return {
        "source_commit": "a" * 40,
        "G5_SEALED_FINAL_read": False,
        "routes": {
            "MRV2-A": {
                "product_map": {
                    "GT_used_by_product_pipeline": False,
                    "aggregation_counts": counts,
                    "missions": [{"scene_seed": 1}],
                }
            }
        },
    }


def test_product_map_gate_aggregates_without_hiding_unmatched_targets():
    report = MODULE.build_report(
        [_benchmark(0.03), _benchmark(0.04)], route="MRV2-A"
    )
    assert report["mission_count"] == 2
    assert report["metrics"]["map_localization_coverage"] == 1.0
    assert report["metrics"]["product_target_precision"] == 1.0
    assert report["metrics"]["map_rmse_m"] == pytest.approx(
        (0.5 * (0.03**2 + 0.04**2)) ** 0.5
    )
    assert report["metrics"]["removed_target_stale_action"] is None


def test_product_map_gate_requires_executed_independent_removal_evidence():
    removal = {
        "post_removal_capture_executed": True,
        "GT_used_by_product_pipeline": False,
        "GT_used_only_for_post_run_scoring": True,
        "stale_action_count": 0,
    }
    report = MODULE.build_report(
        [_benchmark()], route="MRV2-A", removal=removal
    )
    assert report["metrics"]["removed_target_stale_action"] == 0
    with pytest.raises(ValueError, match="not an executed capture"):
        MODULE.build_report(
            [_benchmark()], route="MRV2-A", removal={**removal, "post_removal_capture_executed": False}
        )


def test_product_map_gate_accepts_prediction_derived_removal_capture():
    benchmark = _benchmark()
    counts = benchmark["routes"]["MRV2-A"]["product_map"][
        "aggregation_counts"
    ]
    counts["removal_capture_count"] = 1
    counts["removed_target_stale_action_count"] = 0
    report = MODULE.build_report([benchmark], route="MRV2-A")
    assert report["metrics"]["removed_target_stale_action"] == 0
    assert report["removal_status"] == "prediction_derived_dynamic_removal_capture"


def test_product_map_gate_rejects_mixed_source_revisions():
    first = _benchmark()
    second = _benchmark()
    second["source_commit"] = "b" * 40
    with pytest.raises(ValueError, match="different source commits"):
        MODULE.build_report([first, second], route="MRV2-A")
