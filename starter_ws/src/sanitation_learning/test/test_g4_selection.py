from __future__ import annotations

import sys
from pathlib import Path

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_selection import (  # noqa: E402
    ConstraintAwareSelector,
    ConstraintSpec,
    ObjectiveSpec,
    area_selector,
    classifier_selector,
    discovery_selector,
    select_best_candidate,
)


def test_constraint_violation_cannot_win_on_lower_loss() -> None:
    selector = ConstraintAwareSelector(
        [ConstraintSpec("validation_fp_per_frame", "le", 0.05)],
        ObjectiveSpec("validation_loss", maximize=False),
    )
    first = selector.consider(
        1,
        {
            "validation_loss": 0.1,
            "validation_fp_per_frame": 0.0,
        },
    )
    assert first["selected"] is True
    violating = selector.consider(
        2,
        {
            "validation_loss": 0.01,
            "validation_fp_per_frame": 0.9,
        },
    )
    assert violating["selected"] is False
    assert violating["violated_constraints"]
    assert selector.best()["selected_epoch"] == 1
    assert selector.best()["selection_score"] == 0.1


def test_missing_metric_fails_closed() -> None:
    selector = discovery_selector()
    verdict = selector.consider(
        1,
        {
            "validation_all_gt_candidate_recall": 0.9,
            "validation_negative_only_fp_per_frame": 0.0,
        },
    )
    assert verdict["selected"] is False
    assert any(
        item["reason"] == "missing_metric"
        for item in verdict["violated_constraints"]
    )


def test_selection_is_deterministic_and_tie_breaks_by_epoch() -> None:
    constraints = [ConstraintSpec("validation_specificity", "ge", 0.95)]
    objective = ObjectiveSpec("validation_f1", maximize=True)
    candidates = [
        {
            "epoch": 4,
            "metrics": {
                "validation_specificity": 0.98,
                "validation_f1": 0.90,
            },
        },
        {
            "epoch": 7,
            "metrics": {
                "validation_specificity": 0.99,
                "validation_f1": 0.91,
            },
        },
        {
            "epoch": 9,
            "metrics": {
                "validation_specificity": 0.96,
                "validation_f1": 0.80,
            },
        },
    ]
    first = select_best_candidate(
        candidates, constraints=constraints, objective=objective
    )
    second = select_best_candidate(
        candidates, constraints=constraints, objective=objective
    )
    assert first == second
    assert first["selected_epoch"] == 7
    assert first["selection_score"] == 0.91


def test_earliest_epoch_wins_on_tie() -> None:
    constraints = [ConstraintSpec("validation_specificity", "ge", 0.95)]
    objective = ObjectiveSpec("validation_f1", maximize=True)
    candidates = [
        {
            "epoch": epoch,
            "metrics": {
                "validation_specificity": 0.96,
                "validation_f1": 0.90,
            },
        }
        for epoch in (2, 5, 8)
    ]
    result = select_best_candidate(
        candidates, constraints=constraints, objective=objective
    )
    assert result["selected_epoch"] == 2


def test_no_valid_candidate_reports_failure() -> None:
    result = select_best_candidate(
        [
            {
                "epoch": 1,
                "metrics": {
                    "validation_specificity": 0.5,
                    "validation_f1": 0.99,
                },
            }
        ],
        constraints=[ConstraintSpec("validation_specificity", "ge", 0.95)],
        objective=ObjectiveSpec("validation_f1", maximize=True),
    )
    assert result["selected"] is False
    assert result["selected_epoch"] is None


def test_infeasible_epochs_keep_diagnostic_checkpoint_by_constraint_distance() -> None:
    selector = ConstraintAwareSelector(
        [ConstraintSpec("validation_fp", "le", 0.05)],
        ObjectiveSpec("validation_f1", maximize=True),
    )
    first = selector.consider(
        1, {"validation_fp": 0.50, "validation_f1": 0.95}
    )
    assert first["selected"] is False
    assert first["checkpoint_selected"] is True
    closer = selector.consider(
        2, {"validation_fp": 0.10, "validation_f1": 0.70}
    )
    assert closer["checkpoint_selected"] is True
    lower_loss_but_worse_constraint = selector.consider(
        3, {"validation_fp": 0.40, "validation_f1": 0.99}
    )
    assert lower_loss_but_worse_constraint["checkpoint_selected"] is False
    best = selector.best()
    assert best["selected"] is False
    assert best["diagnostic_checkpoint"]["selected_epoch"] == 2
    assert best["diagnostic_checkpoint"]["product_eligible"] is False


def test_task_specific_selectors_use_p4_constraints() -> None:
    discovery = discovery_selector()
    assert discovery.constraints[0].metric == (
        "validation_negative_only_fp_per_frame"
    )
    assert discovery.constraints[0].threshold == 0.05
    classifier = classifier_selector()
    assert classifier.constraints[0].metric == "validation_paper_precision"
    area = area_selector()
    assert area.constraints[0].metric == "validation_negative_area_fp_per_frame"
    assert area.constraints[0].threshold == 0.05


def test_invalid_operator_rejected() -> None:
    with pytest.raises(ValueError, match="unsupported constraint operator"):
        ConstraintSpec("metric", "gt", 1.0)
