"""Constraint-aware, deterministic checkpoint/model selection.

A candidate may not win solely by a lower validation loss: hard
false-positive/specificity constraints must be satisfied first, then the
task objective is maximized (or minimized).  Selection is deterministic and
kept pure-Python so it can be unit-tested without PyTorch.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping


_OPERATORS = {
    "ge": lambda value, threshold: value >= threshold,
    "le": lambda value, threshold: value <= threshold,
}


@dataclass(frozen=True)
class ConstraintSpec:
    metric: str
    operator: str
    threshold: float

    def __post_init__(self) -> None:
        if self.operator not in _OPERATORS:
            raise ValueError(f"unsupported constraint operator {self.operator!r}")
        if not isinstance(self.threshold, (int, float)) or isinstance(
            self.threshold, bool
        ):
            raise ValueError("constraint threshold must be a number")


@dataclass(frozen=True)
class ObjectiveSpec:
    metric: str
    maximize: bool


class ConstraintAwareSelector:
    """Select the best epoch/candidate under hard constraints.

    ``consider(epoch, metrics)`` returns a verdict with the selection score,
    violated constraints and a snapshot of the validation metrics.  Ties are
    broken by the earliest epoch, making selection deterministic.
    """

    def __init__(
        self,
        constraints: Iterable[ConstraintSpec | Mapping],
        objective: ObjectiveSpec | Mapping,
        tie_breaker: ObjectiveSpec | Mapping | None = None,
    ):
        self.constraints: tuple[ConstraintSpec, ...] = tuple(
            constraint
            if isinstance(constraint, ConstraintSpec)
            else ConstraintSpec(
                metric=constraint["metric"],
                operator=constraint["operator"],
                threshold=constraint["threshold"],
            )
            for constraint in constraints
        )
        self.objective = (
            objective
            if isinstance(objective, ObjectiveSpec)
            else ObjectiveSpec(
                metric=objective["metric"],
                maximize=bool(objective["maximize"]),
            )
        )
        self.tie_breaker = (
            tie_breaker
            if isinstance(tie_breaker, ObjectiveSpec)
            else ObjectiveSpec(
                metric=tie_breaker["metric"],
                maximize=bool(tie_breaker["maximize"]),
            )
            if tie_breaker is not None
            else None
        )
        if not self.constraints:
            raise ValueError("constraint-aware selection requires constraints")
        self._best_epoch: int | None = None
        self._best_score: float | None = None
        self._best_metrics: dict | None = None
        self._best_tie_score: float | None = None
        self._fallback_rank: tuple[float, float, float, int] | None = None
        self._fallback_epoch: int | None = None
        self._fallback_score: float | None = None
        self._fallback_metrics: dict | None = None
        self._fallback_violations: list[dict] = []

    @staticmethod
    def _score(metrics: Mapping, objective: ObjectiveSpec) -> float:
        value = metrics.get(objective.metric)
        if value is None:
            raise ValueError(
                f"objective metric {objective.metric!r} missing from metrics"
            )
        return float(value)

    def violated_constraints(self, metrics: Mapping) -> list[dict]:
        violated: list[dict] = []
        for constraint in self.constraints:
            value = metrics.get(constraint.metric)
            if value is None:
                violated.append(
                    {
                        "metric": constraint.metric,
                        "reason": "missing_metric",
                        "threshold": constraint.threshold,
                    }
                )
                continue
            numeric_value = float(value)
            numeric_threshold = float(constraint.threshold)
            passed = _OPERATORS[constraint.operator](
                numeric_value, numeric_threshold
            )
            if not passed:
                distance = (
                    numeric_threshold - numeric_value
                    if constraint.operator == "ge"
                    else numeric_value - numeric_threshold
                )
                violated.append(
                    {
                        "metric": constraint.metric,
                        "value": numeric_value,
                        "operator": constraint.operator,
                        "threshold": constraint.threshold,
                        "normalized_violation": float(
                            distance / max(abs(numeric_threshold), 1e-12)
                        ),
                    }
                )
        return violated

    def _improves(self, score: float) -> bool:
        if self._best_score is None:
            return True
        if self.objective.maximize:
            return score > self._best_score
        return score < self._best_score

    def _tie_score(self, metrics: Mapping) -> float:
        if self.tie_breaker is None:
            return 0.0
        if metrics.get(self.tie_breaker.metric) is None:
            return 0.0
        return self._score(metrics, self.tie_breaker)

    def _tie_improves(self, score: float) -> bool:
        if self.tie_breaker is None or self._best_tie_score is None:
            return False
        if self.tie_breaker.maximize:
            return score > self._best_tie_score
        return score < self._best_tie_score

    def consider(
        self,
        epoch: int,
        metrics: Mapping,
    ) -> dict:
        """Evaluate one epoch; returns the full selection verdict."""
        epoch = int(epoch)
        if epoch < 1:
            raise ValueError("epoch must be a positive integer")
        violated = self.violated_constraints(metrics)
        score = self._score(metrics, self.objective)
        tie_score = self._tie_score(metrics)
        accepted = False
        checkpoint_selected = False
        if not violated:
            if (
                self._best_score is None
                or self._improves(score)
                or (score == self._best_score and self._tie_improves(tie_score))
            ):
                self._best_epoch = epoch
                self._best_score = score
                self._best_tie_score = tie_score
                self._best_metrics = {
                    str(key): value for key, value in metrics.items()
                }
                accepted = True
                checkpoint_selected = True
        elif self._best_epoch is None and all(
            "normalized_violation" in item for item in violated
        ):
            violation_total = float(
                sum(item["normalized_violation"] for item in violated)
            )
            objective_rank = -score if self.objective.maximize else score
            tie_rank = (
                -tie_score
                if self.tie_breaker is not None and self.tie_breaker.maximize
                else tie_score
            )
            fallback_rank = (violation_total, objective_rank, tie_rank, epoch)
            if (
                self._fallback_rank is None
                or fallback_rank < self._fallback_rank
            ):
                self._fallback_rank = fallback_rank
                self._fallback_epoch = epoch
                self._fallback_score = score
                self._fallback_metrics = dict(metrics)
                self._fallback_violations = [dict(item) for item in violated]
                checkpoint_selected = True
        return {
            "epoch": epoch,
            "selected": bool(accepted),
            "checkpoint_selected": bool(checkpoint_selected),
            "product_eligible": not violated,
            "selection_score": score,
            "tie_breaker_score": tie_score if self.tie_breaker else None,
            "selected_epoch": self._best_epoch,
            "best_selection_score": self._best_score,
            "violated_constraints": violated,
            "validation_metrics": {
                str(key): value for key, value in metrics.items()
            },
        }

    def best(self) -> dict:
        if self._best_epoch is None:
            return {
                "selected": False,
                "selected_epoch": None,
                "selection_score": None,
                "violated_constraints": [],
                "reason": "no_candidate_satisfied_constraints",
                "diagnostic_checkpoint": self.checkpoint_best(),
            }
        return {
            "selected": True,
            "selected_epoch": self._best_epoch,
            "selection_score": self._best_score,
            "tie_breaker_score": self._best_tie_score,
            "validation_metrics": dict(self._best_metrics or {}),
            "violated_constraints": [],
        }

    def checkpoint_best(self) -> dict:
        """Return the best loadable state, even when no epoch is eligible.

        An infeasible fallback is ranked by normalized hard-constraint
        violation before the objective.  It is explicitly diagnostic-only and
        can never be mistaken for a product-eligible selection.
        """
        if self._best_epoch is not None:
            return {
                "selected_epoch": self._best_epoch,
                "selection_score": self._best_score,
                "tie_breaker_score": self._best_tie_score,
                "validation_metrics": dict(self._best_metrics or {}),
                "violated_constraints": [],
                "product_eligible": True,
                "status": "constraint_feasible",
            }
        if self._fallback_epoch is not None:
            return {
                "selected_epoch": self._fallback_epoch,
                "selection_score": self._fallback_score,
                "tie_breaker_score": (
                    self._fallback_metrics.get(self.tie_breaker.metric)
                    if self.tie_breaker is not None and self._fallback_metrics
                    else None
                ),
                "validation_metrics": dict(self._fallback_metrics or {}),
                "violated_constraints": [
                    dict(item) for item in self._fallback_violations
                ],
                "product_eligible": False,
                "status": "diagnostic_fallback_only",
            }
        return {
            "selected_epoch": None,
            "selection_score": None,
            "validation_metrics": None,
            "violated_constraints": [],
            "product_eligible": False,
            "status": "no_loadable_candidate",
        }


def select_best_candidate(
    candidates: Iterable[dict],
    *,
    constraints: Iterable[ConstraintSpec | Mapping],
    objective: ObjectiveSpec | Mapping,
) -> dict:
    """Pure-function deterministic selection over epoch metric snapshots.

    Each candidate is a dict with ``epoch`` and ``metrics``.  Candidates that
    violate constraints are discarded; the remaining candidates are ordered by
    objective (descending when maximizing, ascending otherwise) and then by
    epoch ascending.
    """
    constraint_list = tuple(
        constraint
        if isinstance(constraint, ConstraintSpec)
        else ConstraintSpec(
            metric=constraint["metric"],
            operator=constraint["operator"],
            threshold=constraint["threshold"],
        )
        for constraint in constraints
    )
    objective_spec = (
        objective
        if isinstance(objective, ObjectiveSpec)
        else ObjectiveSpec(
            metric=objective["metric"],
            maximize=bool(objective["maximize"]),
        )
    )
    normalized: list[dict] = []
    for candidate in candidates:
        epoch = int(candidate["epoch"])
        metrics = candidate["metrics"]
        violated: list[dict] = []
        for constraint in constraint_list:
            value = metrics.get(constraint.metric)
            if value is None or not _OPERATORS[constraint.operator](
                float(value), float(constraint.threshold)
            ):
                violated.append(constraint.metric)
        if violated:
            continue
        normalized.append(
            {
                "epoch": epoch,
                "selection_score": float(metrics[objective_spec.metric]),
                "validation_metrics": dict(metrics),
            }
        )
    if not normalized:
        return {
            "selected": False,
            "selected_epoch": None,
            "selection_score": None,
            "validation_metrics": None,
            "violated_constraints": [],
            "reason": "no_candidate_satisfied_constraints",
        }
    key = (
        lambda item: (
            -float(item["selection_score"])
            if objective_spec.maximize
            else float(item["selection_score"]),
            int(item["epoch"]),
        )
    )
    best = min(normalized, key=key)
    return {
        "selected": True,
        "selected_epoch": int(best["epoch"]),
        "selection_score": float(best["selection_score"]),
        "validation_metrics": best["validation_metrics"],
        "violated_constraints": [],
    }


def discovery_selector(
    *,
    candidate_recall_min: float = 0.80,
    negative_fp_per_frame_max: float = 0.05,
    false_candidates_per_min_max: float = 2.0,
    objective_metric: str = "validation_all_gt_candidate_recall",
) -> ConstraintAwareSelector:
    """P4 discovery selection: constraints first, then recall."""
    return ConstraintAwareSelector(
        [
            ConstraintSpec(
                "validation_all_gt_candidate_recall",
                "ge",
                candidate_recall_min,
            ),
            ConstraintSpec(
                "validation_negative_only_fp_per_frame",
                "le",
                negative_fp_per_frame_max,
            ),
            ConstraintSpec(
                "validation_false_candidates_per_min",
                "le",
                false_candidates_per_min_max,
            ),
        ],
        ObjectiveSpec(objective_metric, maximize=True),
        ObjectiveSpec("validation_loss", maximize=False),
    )


def classifier_selector(
    *,
    macro_f1_min: float = 0.90,
    min_discrete_recall_min: float = 0.70,
    paper_precision_min: float = 0.80,
    background_specificity_min: float = 0.95,
    objective_metric: str = "validation_macro_f1",
) -> ConstraintAwareSelector:
    """P4 classifier selection: specificity constraints first, then F1."""
    return ConstraintAwareSelector(
        [
            ConstraintSpec(
                "validation_macro_f1", "ge", macro_f1_min
            ),
            ConstraintSpec(
                "validation_min_discrete_recall",
                "ge",
                min_discrete_recall_min,
            ),
            ConstraintSpec(
                "validation_paper_precision", "ge", paper_precision_min
            ),
            ConstraintSpec(
                "validation_background_specificity",
                "ge",
                background_specificity_min,
            ),
        ],
        ObjectiveSpec(objective_metric, maximize=True),
        ObjectiveSpec("validation_loss", maximize=False),
    )


def area_selector(
    *,
    iou_min: float = 0.75,
    negative_area_fp_per_frame_max: float = 0.05,
    boundary_f1_min: float = 0.70,
    objective_metric: str = "validation_area_balanced_score",
) -> ConstraintAwareSelector:
    """P4 area selection: enforce gates, then balance IoU and boundary F1."""
    return ConstraintAwareSelector(
        [
            ConstraintSpec("validation_iou", "ge", iou_min),
            ConstraintSpec(
                "validation_negative_area_fp_per_frame",
                "le",
                negative_area_fp_per_frame_max,
            ),
            ConstraintSpec(
                "validation_boundary_f1", "ge", boundary_f1_min
            ),
        ],
        ObjectiveSpec(objective_metric, maximize=True),
        ObjectiveSpec("validation_loss", maximize=False),
    )


__all__ = [
    "ConstraintAwareSelector",
    "ConstraintSpec",
    "ObjectiveSpec",
    "area_selector",
    "classifier_selector",
    "discovery_selector",
    "select_best_candidate",
]
