from sanitation_learning.oprv3_online import (
    ClassWindow, ObservableTargetEncounter, evaluate_eventual_metrics,
    production_schema_fields,
)


def test_all_gt_targets_remain_in_exhaustive_partitions_and_missed_denominator():
    window = ClassWindow("paper_litter", 0.8, 2.5, 3, 0.6)
    rows = []
    for frame in range(4):
        stamp = frame / 15
        rows.extend([
            ObservableTargetEncounter("never", "paper_litter", stamp, False, 0.0, 0.0, None),
            ObservableTargetEncounter("occluded", "paper_litter", stamp, True, 0.0, 1.0, 1.5, occluded=True),
            ObservableTargetEncounter("actionable", "paper_litter", stamp, True, 0.9, 1.0, 1.5),
            ObservableTargetEncounter("too-far", "paper_litter", stamp, True, 0.9, 1.0, 3.0),
        ])
    report = evaluate_eventual_metrics(rows, [], [], {"paper_litter": window}, frame_rate_hz=15)
    assert report["counts"] == {
        "all_gt_targets": 4,
        "never_in_camera_frustum": 1,
        "occluded_entirely": 1,
        "visible_but_never_actionable": 1,
        "entered_actionable_window": 1,
        "detected_in_window": 0,
        "missed_in_window": 1,
        "clean_opportunity_missed": 1,
    }
    assert report["subset_audit"]["partition_is_exhaustive"] is True
    assert report["metrics"]["eventual_detection_recall"] == 0.0


def test_production_schema_has_no_gt_identity_or_coordinates():
    fields = production_schema_fields()
    assert "target_id" not in fields
    assert "gt_target_id" not in fields
    assert "gt_map_xy_m" not in fields
