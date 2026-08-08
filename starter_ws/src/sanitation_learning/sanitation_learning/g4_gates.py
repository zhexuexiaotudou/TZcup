"""Canonical P4/P5 gate policies and machine-evaluable gate evaluation.

All fixed thresholds are centralized in:

- ``config/perception_p4_screening_policy.yaml``
- ``config/perception_p5_final_policy.yaml``

The evaluator resolves dotted metric paths against a flat or nested report and
is fail-closed: a missing metric becomes ``not_evaluated`` and fails the gate.
Placeholder booleans such as ``False`` are never accepted as evidence; a gate
must either carry a real computed value or an explicit ``not_evaluated``
record.
"""

from __future__ import annotations

from pathlib import Path

import yaml


_OPERATORS = {
    "ge": lambda value, threshold: value >= threshold,
    "le": lambda value, threshold: value <= threshold,
    "eq": lambda value, threshold: value == threshold,
}


def _resolve_metric(metrics: dict, path: str):
    current = metrics
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def load_policy(path) -> dict:
    """Load and validate a canonical gate policy file."""
    policy = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(policy, dict):
        raise ValueError(f"gate policy must be a mapping: {path}")
    if policy.get("schema_version") != 1:
        raise ValueError("gate policy schema_version must be 1")
    if not isinstance(policy.get("policy_id"), str) or not policy.get(
        "policy_id"
    ):
        raise ValueError("gate policy policy_id is required")
    gates = policy.get("gates")
    if not isinstance(gates, list) or not gates:
        raise ValueError("gate policy gates must be a non-empty list")
    for gate in gates:
        if not isinstance(gate, dict):
            raise ValueError("each gate must be a mapping")
        for field in ("id", "metric", "operator", "threshold"):
            if field not in gate:
                raise ValueError(f"gate missing required field {field!r}: {gate}")
        if gate["operator"] not in _OPERATORS:
            raise ValueError(
                f"unsupported gate operator {gate['operator']!r}"
            )
        threshold = gate["threshold"]
        if gate["operator"] == "eq":
            if not isinstance(threshold, (bool, int, float)):
                raise ValueError(
                    f"eq gate threshold must be boolean or numeric, "
                    f"got {threshold!r}"
                )
        elif isinstance(threshold, bool) or not isinstance(
            threshold, (int, float)
        ):
            raise ValueError(
                f"gate threshold must be a number, got {threshold!r}"
            )
    return policy


def evaluate_policy(policy: dict, metrics: dict) -> dict:
    """Evaluate every gate in a policy against a metrics report.

    Returns per-gate results with ``passed``, the resolved value (or
    ``None``), the threshold and ``not_evaluated`` when the metric is absent.
    """
    gate_results: dict[str, dict] = {}
    not_evaluated: list[str] = []
    for gate in policy["gates"]:
        gate_id = gate["id"]
        value = _resolve_metric(metrics, gate["metric"])
        if value is None:
            gate_results[gate_id] = {
                "passed": False,
                "value": None,
                "threshold": gate["threshold"],
                "operator": gate["operator"],
                "not_evaluated": True,
                "reason": "missing_metric",
            }
            not_evaluated.append(gate_id)
            continue
        if isinstance(value, bool):
            if gate["operator"] != "eq" or not isinstance(
                gate["threshold"], bool
            ):
                gate_results[gate_id] = {
                    "passed": False,
                    "value": value,
                    "threshold": gate["threshold"],
                    "operator": gate["operator"],
                    "not_evaluated": True,
                    "reason": "boolean_metric_for_non_boolean_gate",
                }
                not_evaluated.append(gate_id)
                continue
            passed = value == gate["threshold"]
            gate_results[gate_id] = {
                "passed": passed,
                "value": value,
                "threshold": gate["threshold"],
                "operator": gate["operator"],
                "not_evaluated": False,
            }
            continue
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            gate_results[gate_id] = {
                "passed": False,
                "value": value,
                "threshold": gate["threshold"],
                "operator": gate["operator"],
                "not_evaluated": True,
                "reason": "non_numeric_metric",
            }
            not_evaluated.append(gate_id)
            continue
        passed = _OPERATORS[gate["operator"]](
            numeric, float(gate["threshold"])
        )
        gate_results[gate_id] = {
            "passed": bool(passed),
            "value": numeric,
            "threshold": gate["threshold"],
            "operator": gate["operator"],
            "not_evaluated": False,
        }
    passed = (
        bool(gate_results)
        and not not_evaluated
        and all(item["passed"] for item in gate_results.values())
    )
    return {
        "policy_id": policy["policy_id"],
        "stage": policy.get("stage"),
        "pass": passed,
        "gates": gate_results,
        "not_evaluated": sorted(not_evaluated),
    }


def decision_from_policy_result(result: dict, status_key: str) -> dict:
    """Build a top-level pass/blocked decision record."""
    passed = bool(result["pass"])
    return {
        status_key: passed,
        f"{status_key}_BLOCKED": not passed,
        "not_evaluated_gates": result["not_evaluated"],
    }


# Regression pins: these mirror the canonical YAML policies.  A test asserts
# the YAML values are exactly equal so thresholds can never be lowered
# silently.
P4_FIXED_THRESHOLDS = {
    "in_domain_candidate_recall_ge": 0.80,
    "in_domain_false_candidates_per_min_le": 2.0,
    "in_domain_negative_only_fp_per_frame_le": 0.05,
    "in_domain_macro_precision_ge": 0.90,
    "in_domain_macro_recall_ge": 0.90,
    "in_domain_macro_f1_ge": 0.90,
    "cross_world_macro_f1_ge": 0.70,
    "val_per_class_recall_ge": 0.70,
    "paper_precision_ge": 0.80,
    "small_object_recall_ge": 0.70,
    "leaf_iou_ge": 0.75,
    "puddle_iou_ge": 0.75,
    "macro_miou_ge": 0.75,
    "boundary_f1_ge": 0.70,
    "negative_area_fp_per_frame_le": 0.05,
    "color_material_stress_macro_f1_ge": 0.60,
    "same_color_negative_specificity_ge": 0.95,
    "D1_D5_reports_complete": True,
    "onnx_task_specific_parity_pass": True,
    "onnx_custom_ops_zero": True,
}

P5_FIXED_THRESHOLDS = {
    "discrete_macro_precision_ge": 0.95,
    "discrete_macro_recall_ge": 0.95,
    "discrete_macro_f1_ge": 0.95,
    "per_class_recall_ge": 0.90,
    "paper_precision_ge": 0.90,
    "AP50_ge": 0.95,
    "AP50_95_ge": 0.70,
    "small_object_recall_ge": 0.85,
    "leaf_iou_ge": 0.80,
    "puddle_iou_ge": 0.80,
    "macro_miou_ge": 0.80,
    "boundary_f1_ge": 0.80,
    "same_color_specificity_ge": 0.98,
    "missing_class_hallucination_le": 0.01,
}


__all__ = [
    "P4_FIXED_THRESHOLDS",
    "P5_FIXED_THRESHOLDS",
    "decision_from_policy_result",
    "evaluate_policy",
    "load_policy",
]
