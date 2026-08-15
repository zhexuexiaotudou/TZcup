from sanitation_learning.oprv3_online import (
    ClassWindow, EvaluatorMatch, ObservableTargetEncounter, ProductionObservation,
    evaluate_eventual_metrics,
)


def test_eventual_metrics_accept_later_correct_detection_inside_frozen_window():
    window = ClassWindow("metal_can", 0.8, 2.0, 3, 0.6)
    encounters = [
        ObservableTargetEncounter("can-1", "metal_can", index / 15, True, 0.9, 1.0, 1.8 - index * 0.1)
        for index in range(6)
    ]
    observation = ProductionObservation("obs-1", 4 / 15, "metal_can", 0.9, "track-1", True, (1.0, 2.0))
    report = evaluate_eventual_metrics(
        encounters, [observation], [EvaluatorMatch("obs-1", "can-1", 0.04)],
        {"metal_can": window}, frame_rate_hz=15,
    )
    assert report["counts"]["entered_actionable_window"] == 1
    assert report["metrics"]["eventual_detection_recall"] == 1.0
    assert report["metrics"]["eventual_correct_class_recall"] == 1.0
    assert report["metrics"]["eventual_map_localization_recall"] == 1.0
    assert report["metrics"]["clean_opportunity_miss_rate"] == 0.0
