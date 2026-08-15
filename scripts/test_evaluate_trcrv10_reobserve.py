import evaluate_trcrv10_reobserve as evaluator


def test_resolved_or_truthfully_unreachable_passes() -> None:
    rows = [
        {"truth_kind": "target", "reobserve_count": 1, "outcome": "CLASSIFICATION_CONDITION_REACHED", "extra_distance_m": 1, "extra_time_s": 2, "baseline_distance_m": 10},
        {"truth_kind": "target", "reobserve_count": 2, "outcome": "UNREACHABLE_FOR_VISUAL_CONFIRMATION", "extra_distance_m": 2, "extra_time_s": 4, "baseline_distance_m": 10},
        {"truth_kind": "negative", "reobserve_count": 2, "outcome": "DEFER", "extra_distance_m": 1, "extra_time_s": 2, "baseline_distance_m": 10},
    ]
    assert evaluator.evaluate(rows)["pass"]


def test_unbounded_false_candidate_fails() -> None:
    rows = [
        {"truth_kind": "target", "reobserve_count": 1, "outcome": "CLASSIFICATION_CONDITION_REACHED"},
        {"truth_kind": "negative", "reobserve_count": 3, "outcome": "DEFER"},
    ]
    assert not evaluator.evaluate(rows)["pass"]
