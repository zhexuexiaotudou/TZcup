from __future__ import annotations

from pathlib import Path
import sys

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_gates import (  # noqa: E402
    P4_FIXED_THRESHOLDS,
    P5_FIXED_THRESHOLDS,
    decision_from_policy_result,
    evaluate_policy,
    load_policy,
)


REPO = Path(__file__).resolve().parents[4]
P4_POLICY = (
    REPO
    / "starter_ws"
    / "src"
    / "sanitation_learning"
    / "config"
    / "perception_p4_screening_policy.yaml"
)
P5_POLICY = (
    REPO
    / "starter_ws"
    / "src"
    / "sanitation_learning"
    / "config"
    / "perception_p5_final_policy.yaml"
)


def test_p4_policy_thresholds_match_regression_pins_exactly() -> None:
    policy = load_policy(P4_POLICY)
    by_id = {gate["id"]: gate for gate in policy["gates"]}
    assert set(by_id) == set(P4_FIXED_THRESHOLDS)
    for gate_id, expected in P4_FIXED_THRESHOLDS.items():
        assert by_id[gate_id]["threshold"] == expected, gate_id


def test_p5_policy_thresholds_match_regression_pins_exactly() -> None:
    policy = load_policy(P5_POLICY)
    by_id = {gate["id"]: gate for gate in policy["gates"]}
    assert set(by_id) == set(P5_FIXED_THRESHOLDS)
    for gate_id, expected in P5_FIXED_THRESHOLDS.items():
        assert by_id[gate_id]["threshold"] == expected, gate_id


def test_p4_thresholds_are_not_lowered_below_spec() -> None:
    # Fixed values from the authoritative P4/P5 spec; these assertions guard
    # against accidental weakening.
    assert P4_FIXED_THRESHOLDS["in_domain_candidate_recall_ge"] == 0.80
    assert P4_FIXED_THRESHOLDS["in_domain_false_candidates_per_min_le"] == 2.0
    assert P4_FIXED_THRESHOLDS["in_domain_macro_f1_ge"] == 0.90
    assert P4_FIXED_THRESHOLDS["cross_world_macro_f1_ge"] == 0.70
    assert P4_FIXED_THRESHOLDS["paper_precision_ge"] == 0.80
    assert P4_FIXED_THRESHOLDS["small_object_recall_ge"] == 0.70
    assert P4_FIXED_THRESHOLDS["leaf_iou_ge"] == 0.75
    assert P4_FIXED_THRESHOLDS["puddle_iou_ge"] == 0.75
    assert P4_FIXED_THRESHOLDS["boundary_f1_ge"] == 0.70
    assert P4_FIXED_THRESHOLDS["negative_area_fp_per_frame_le"] == 0.05
    assert P4_FIXED_THRESHOLDS["same_color_negative_specificity_ge"] == 0.95
    assert P4_FIXED_THRESHOLDS["color_material_stress_macro_f1_ge"] == 0.60
    assert P4_FIXED_THRESHOLDS["selected_models_product_eligible"] is True


def test_p5_thresholds_are_not_lowered_below_spec() -> None:
    assert P5_FIXED_THRESHOLDS["discrete_macro_precision_ge"] == 0.95
    assert P5_FIXED_THRESHOLDS["discrete_macro_recall_ge"] == 0.95
    assert P5_FIXED_THRESHOLDS["discrete_macro_f1_ge"] == 0.95
    assert P5_FIXED_THRESHOLDS["per_class_recall_ge"] == 0.90
    assert P5_FIXED_THRESHOLDS["paper_precision_ge"] == 0.90
    assert P5_FIXED_THRESHOLDS["AP50_ge"] == 0.95
    assert P5_FIXED_THRESHOLDS["AP50_95_ge"] == 0.70
    assert P5_FIXED_THRESHOLDS["small_object_recall_ge"] == 0.85
    assert P5_FIXED_THRESHOLDS["leaf_iou_ge"] == 0.80
    assert P5_FIXED_THRESHOLDS["puddle_iou_ge"] == 0.80
    assert P5_FIXED_THRESHOLDS["macro_miou_ge"] == 0.80
    assert P5_FIXED_THRESHOLDS["boundary_f1_ge"] == 0.80
    assert P5_FIXED_THRESHOLDS["same_color_specificity_ge"] == 0.98
    assert P5_FIXED_THRESHOLDS["missing_class_hallucination_le"] == 0.01


def test_evaluate_policy_passes_and_fails() -> None:
    policy = load_policy(P5_POLICY)
    metrics = {
        "discrete": {
            "macro_precision": 0.96,
            "macro_recall": 0.96,
            "macro_f1": 0.96,
            "per_class": {
                "paper_litter": {"precision": 0.92},
            },
            "small_object_recall": 0.90,
        },
        "per_class_recall": 0.92,
        "AP50": 0.96,
        "AP50_95": 0.75,
        "area": {
            "iou_by_class": {"leaf_pile": 0.82, "puddle": 0.81},
            "macro_miou": 0.82,
            "boundary_f1": 0.83,
        },
        "same_color_specificity": 0.99,
        "missing_class_hallucination": 0.0,
    }
    result = evaluate_policy(policy, metrics)
    assert result["pass"] is True
    failing = dict(metrics)
    failing["AP50"] = 0.5
    failed = evaluate_policy(policy, failing)
    assert failed["pass"] is False
    assert failed["gates"]["AP50_ge"]["passed"] is False


def test_missing_metric_is_not_evaluated_and_fails_closed() -> None:
    policy = load_policy(P4_POLICY)
    result = evaluate_policy(policy, {})
    assert result["pass"] is False
    assert len(result["not_evaluated"]) == len(policy["gates"])
    decision = decision_from_policy_result(result, "P4_SCREENING_PASS")
    assert decision["P4_SCREENING_PASS"] is False
    assert decision["P4_SCREENING_PASS_BLOCKED"] is True


def test_policy_validation_rejects_bad_files(tmp_path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("schema_version: 2\npolicy_id: x\ngates: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_policy(bad)
    bad.write_text(
        "schema_version: 1\npolicy_id: x\ngates:\n"
        "  - id: g\n    metric: m\n    operator: ge\n    threshold: true\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="threshold must be a number"):
        load_policy(bad)


def test_boolean_metrics_only_work_for_boolean_equality_gates() -> None:
    numeric_policy = {
        "policy_id": "numeric",
        "stage": "test",
        "gates": [
            {
                "id": "recall",
                "metric": "recall",
                "operator": "ge",
                "threshold": 0.8,
            }
        ],
    }
    numeric_result = evaluate_policy(numeric_policy, {"recall": True})
    assert numeric_result["pass"] is False
    assert numeric_result["gates"]["recall"]["not_evaluated"] is True

    false_equals_false = {
        "policy_id": "boolean",
        "stage": "test",
        "gates": [
            {
                "id": "flag",
                "metric": "flag",
                "operator": "eq",
                "threshold": False,
            }
        ],
    }
    boolean_result = evaluate_policy(false_equals_false, {"flag": False})
    assert boolean_result["pass"] is True
