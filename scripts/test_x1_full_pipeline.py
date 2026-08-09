from perception_prod_x1_full_pipeline import apply_classifier_threshold, static_gate


def _report(value: float):
    discrete = {
        "macro_precision": value,
        "macro_recall": value,
        "macro_f1": value,
        "paper_precision": value,
        "small_object_recall": value,
        "per_class": {
            name: {"recall": value}
            for name in ("plastic_bottle", "metal_can", "paper_litter")
        },
    }
    area = {
        "iou_by_class": {"leaf_pile": value, "puddle": value},
        "macro_miou": value,
        "boundary_f1": value,
        "negative_area_fp_per_frame": 0.0,
    }
    candidate = {
        "all_gt_candidate_recall": value,
        "small_object_candidate_recall": value,
        "false_candidates_per_min": 0.0,
        "negative_only_fp_per_frame": 0.0,
    }
    return {
        "splits": {"VAL": {"candidate": candidate, "discrete": discrete, "area": area}},
        "cross_world_aggregate": {
            "candidate": candidate,
            "discrete": discrete,
            "area": area,
        },
    }


def test_x1_static_gate_passes_only_when_all_fixed_metrics_pass():
    decision = static_gate(_report(0.95))
    assert decision["static_gate_pass"] is True
    assert all(decision["gates"].values())


def test_x1_static_gate_fails_without_small_object_recall():
    report = _report(0.95)
    report["splits"]["VAL"]["candidate"]["small_object_candidate_recall"] = 0.69
    decision = static_gate(report)
    assert decision["static_gate_pass"] is False
    assert decision["gates"]["small_candidate_recall_at_least_0_70"] is False


def test_classifier_threshold_is_applied_without_reinference():
    scored = [{"predictions": [{
        "candidate_class_index": 2,
        "candidate_class_name": "metal_can",
        "candidate_class_score": 0.8,
        "background_score": 0.1,
        "class_index": 2,
        "class_name": "metal_can",
        "score": 0.8,
    }]}]
    assert apply_classifier_threshold(scored, 0.75)[0]["predictions"][0]["class_index"] == 2
    assert apply_classifier_threshold(scored, 0.85)[0]["predictions"][0]["class_index"] == 0
