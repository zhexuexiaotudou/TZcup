from pathlib import Path


def test_formal_moving_evaluator_accepts_hash_bound_crv6_selection():
    source=(Path(__file__).parent/"perception_oprv3_moving_benchmark.py").read_text(encoding="utf-8")
    assert 'selection.get("selection_data") == "G7_MOVING_HOLDOUT_ONLY"' in source
    assert 'expected = selection.get("checkpoint_sha256")' in source
    assert 'selection.get("MOVING_VAL_read_before_selection_freeze")' in source
    assert 'read_text(encoding="utf-8-sig")' in source


def test_formal_evaluator_reports_crv6_projection_and_separate_map_metrics():
    source=(Path(__file__).parent/"perception_oprv3_moving_benchmark.py").read_text(encoding="utf-8")
    for token in (
        '"valid_depth_correct_detection_projection_success"',
        '"direct_projection_median_error_m"',
        '"direct_projection_p95_error_m"',
        '"map_localization_median_error_m"',
        '"map_localization_p95_error_m"',
        '"discrete_product_target_precision"',
        '"area_product_target_precision"',
        '"combined_product_target_precision"',
    ):
        assert token in source
