#!/usr/bin/env python3
"""Fail-closed functional check for one bounded avoidance recovery run."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
from typing import Any


FUNCTIONAL_PASS = "FUNCTIONAL_PASS_1_OF_1_NOT_OFFICIAL_95"
FUNCTIONAL_FAIL = "FUNCTIONAL_FAIL"
OUT_OF_BOUNDS = (
    "Robot is out of bounds of the costmap",
    "is out of map bounds",
    "outside bounds",
)


class EvidenceError(ValueError):
    """Raised when a required evidence file cannot be interpreted."""


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f"cannot read JSON evidence: {path}") from exc
    if not isinstance(value, dict):
        raise EvidenceError(f"JSON evidence must be an object: {path}")
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise EvidenceError(f"{label} must be finite")
    return result


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceError(f"{label} must be an integer")
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise EvidenceError(f"{label} must be a non-empty string")
    return value


def _bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise EvidenceError(f"{label} must be boolean")
    return value


def _has_map_drift(log_path: Path) -> list[str]:
    if not log_path.is_file() or log_path.is_symlink():
        raise EvidenceError(f"runtime log is missing or not regular: {log_path}")
    hits: list[str] = []
    for line_number, line in enumerate(
        log_path.read_text(encoding="utf-8", errors="replace").splitlines(),
        start=1,
    ):
        if any(marker in line for marker in OUT_OF_BOUNDS):
            hits.append(f"{line_number}: {line.strip()}")
    return hits


def _planner_failures(log_path: Path) -> list[str]:
    pattern = re.compile(
        r"GridBased plugin failed to plan from .*?outside bounds"
    )
    failures: list[str] = []
    for line_number, line in enumerate(
        log_path.read_text(encoding="utf-8", errors="replace").splitlines(),
        start=1,
    ):
        if pattern.search(line):
            failures.append(f"{line_number}: {line.strip()}")
    return failures


def evaluate(run_root: Path) -> dict[str, Any]:
    runtime_root = run_root / "runtime"
    product = _load_json(runtime_root / "runtime_telemetry.json")
    environment = _load_json(runtime_root / "environment_truth_telemetry.json")
    single_run = _load_json(run_root / "single_run_evaluation.json")
    log_path = runtime_root / "dynamic.launch.log"
    drift = _has_map_drift(log_path)
    planner_bounds_failures = _planner_failures(log_path)

    goal_accepted = _bool(product.get("goal_accepted"), "goal_accepted")
    goal_succeeded = _bool(
        product.get("nav2_goal_succeeded"), "nav2_goal_succeeded"
    )
    completion_reason = _string(
        product.get("completion_reason"), "completion_reason"
    )
    goal_source = _string(product.get("mission_goal_source"), "mission_goal_source")
    physical_travel_m = _finite_number(
        product.get("physical_travel_distance_m"), "physical_travel_distance_m"
    )
    candidates = _integer(
        product.get("dynamic_interaction_candidate_count"),
        "dynamic_interaction_candidate_count",
    )
    interventions = _integer(
        product.get("collision_monitor_intervention_count"),
        "collision_monitor_intervention_count",
    )
    physical_collisions = _integer(
        environment.get("collision_count"), "environment.collision_count"
    )

    single = single_run.get("single_run")
    if not isinstance(single, dict):
        raise EvidenceError("single_run_evaluation.single_run must be an object")
    numerator = _integer(single.get("numerator"), "single_run.numerator")
    denominator = _integer(single.get("denominator"), "single_run.denominator")
    functional_success = _bool(
        single.get("functional_success"), "single_run.functional_success"
    )
    official = single_run.get("official_metric")
    if not isinstance(official, dict):
        raise EvidenceError("single_run_evaluation.official_metric must be an object")
    official_status = _string(
        official.get("status"), "official_metric.status"
    )

    checks = {
        "goal_accepted": goal_accepted,
        "goal_succeeded": goal_succeeded,
        "action_result_completion": completion_reason == "action_result",
        "public_goal_source": (
            goal_source
            == "public_manifest_fixed_start_transformed_to_saved_map_local_plus_nominal_leg"
        ),
        "physical_travel_at_least_5m": physical_travel_m >= 5.0,
        "dynamic_interaction_observed": candidates >= 1,
        "collision_monitor_intervened": interventions >= 1,
        "zero_physical_collisions": physical_collisions == 0,
        "no_global_costmap_drift": not drift,
        "no_planner_outside_bounds": not planner_bounds_failures,
        "single_run_evaluator_passed": (
            numerator == 1
            and denominator == 1
            and functional_success
        ),
        "official_metric_not_measured": official_status == "NOT_MEASURED",
    }
    blockers = [name for name, passed in checks.items() if not passed]
    passed = not blockers
    return {
        "schema_version": 1,
        "status": FUNCTIONAL_PASS if passed else FUNCTIONAL_FAIL,
        "passed": passed,
        "claim_boundary": (
            "A pass is one functional run only. It is not an official >=95% "
            "dynamic-avoidance success-rate measurement."
        ),
        "run_root": str(run_root),
        "metrics": {
            "completion_reason": completion_reason,
            "physical_travel_distance_m": physical_travel_m,
            "dynamic_interaction_candidate_count": candidates,
            "collision_monitor_intervention_count": interventions,
            "physical_collision_count": physical_collisions,
            "single_run_numerator": numerator,
            "single_run_denominator": denominator,
            "official_metric_status": official_status,
        },
        "checks": checks,
        "blockers": blockers,
        "global_costmap_drift_lines": drift,
        "planner_outside_bounds_lines": planner_bounds_failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    try:
        result = evaluate(args.run_root)
    except EvidenceError as exc:
        result = {
            "schema_version": 1,
            "status": FUNCTIONAL_FAIL,
            "passed": False,
            "claim_boundary": (
                "A pass is one functional run only. It is not an official "
                ">=95% dynamic-avoidance success-rate measurement."
            ),
            "run_root": str(args.run_root),
            "metrics": {},
            "checks": {},
            "blockers": [str(exc)],
            "global_costmap_drift_lines": [],
            "planner_outside_bounds_lines": [],
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
