#!/usr/bin/env python3
"""Validate and summarize one bounded saved-map coverage run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


class SummaryError(RuntimeError):
    pass


def _mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SummaryError(f"{label} must be an object")
    return value


def _number(value: object, label: str) -> float:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
    ):
        raise SummaryError(f"{label} must be a finite number")
    return float(value)


def _integer(value: object, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise SummaryError(f"{label} must be an integer")
    return value


def summarize(
    coverage: dict[str, Any],
    cleaning: dict[str, Any],
    trajectory_row_count: int,
) -> dict[str, Any]:
    if _integer(coverage.get("schema_version"), "coverage.schema_version") != 2:
        raise SummaryError("coverage report schema version is not 2")
    empirical = _mapping(
        coverage.get("empirical_metrics"), "coverage.empirical_metrics"
    )
    planned = _mapping(
        coverage.get("planned_metrics"), "coverage.planned_metrics"
    )
    coverage_rate = _number(
        empirical.get("coverage_rate"), "coverage_rate"
    )
    covered_area = _number(
        empirical.get("covered_area_m2"), "covered_area_m2"
    )
    cleanable_area = _number(
        empirical.get("cleanable_area_m2"), "cleanable_area_m2"
    )
    path_length = _number(
        empirical.get("actual_path_length_m"), "actual_path_length_m"
    )
    duration = _number(
        empirical.get("actual_duration_sec"), "actual_duration_sec"
    )
    net_efficiency = _number(
        empirical.get("net_efficiency_m2_h"), "net_efficiency_m2_h"
    )
    brush_distance = _number(
        empirical.get("brush_enabled_distance_m"), "brush_enabled_distance_m"
    )
    cleaned_cells = _integer(
        cleaning.get("cleaned_cell_delta"), "cleaning.cleaned_cell_delta"
    )
    cleaned_area = _number(
        cleaning.get("cleaned_area_delta_m2"),
        "cleaning.cleaned_area_delta_m2",
    )
    ready_count = _integer(
        cleaning.get("all_three_ready_sample_count"),
        "cleaning.all_three_ready_sample_count",
    )
    coverage_success = coverage.get("success") is True
    coverage_complete = bool(
        coverage.get("full_execution_success") is True
        and coverage.get("coverage_quality_success") is True
        and coverage.get("safety_success") is True
        and coverage.get("localization_success") is True
    )
    valid = bool(
        coverage_success
        and coverage_complete
        and trajectory_row_count > 0
        and cleaning.get("permit_observed") is True
        and cleaning.get("lift_requested") is True
        and cleaning.get("brush_disabled_on_exit") is True
        and ready_count > 0
        and cleaned_cells > 0
        and cleaned_area > 0.0
    )
    terminal_state = "COMPLETED" if valid else "FAILED"
    blockers: list[str] = []
    if not coverage_success:
        blockers.append("coverage_report_success_false")
    if not coverage_complete:
        blockers.append("coverage_quality_or_safety_gate_false")
    if trajectory_row_count <= 0:
        blockers.append("coverage_trajectory_empty")
    if cleaning.get("permit_observed") is not True:
        blockers.append("safety_permit_not_observed")
    if cleaning.get("lift_requested") is not True:
        blockers.append("cleaning_lift_not_requested")
    if ready_count <= 0:
        blockers.append("cleaning_tools_never_ready")
    if cleaned_cells <= 0 or cleaned_area <= 0.0:
        blockers.append("no_ground_dirt_cells_cleared")
    return {
        "schema_version": 1,
        "artifact_kind": "day1_bounded_saved_map_coverage_result",
        "terminal_state": terminal_state,
        "valid_bounded_mission": valid,
        "blockers": blockers,
        "mission_id": coverage.get("mission_id"),
        "coverage": {
            "coverage_rate": coverage_rate,
            "coverage_percent": coverage_rate * 100.0,
            "covered_area_m2": covered_area,
            "cleanable_area_m2": cleanable_area,
            "planned_coverage_fraction": _number(
                planned.get("coverage_rate"),
                "coverage.planned_metrics.coverage_rate",
            ),
            "actual_path_length_m": path_length,
            "actual_duration_sec": duration,
            "average_path_speed_mps": path_length / duration if duration > 0 else 0.0,
            "brush_enabled_distance_m": brush_distance,
            "brush_state_transitions": _integer(
                empirical.get("brush_state_transitions"),
                "empirical.brush_state_transitions",
            ),
            "net_efficiency_m2_h": net_efficiency,
            "gross_efficiency_m2_h": _number(
                empirical.get("gross_efficiency_m2_h"),
                "empirical.gross_efficiency_m2_h",
            ),
            "trajectory_row_count": trajectory_row_count,
            "coverage_metric_basis": empirical.get("metric_basis"),
            "ground_truth_used_for_control": coverage.get(
                "ground_truth_used_for_control"
            ),
        },
        "cleaning": {
            "initial_cleaned_cell_count": _mapping(
                cleaning.get("initial"), "cleaning.initial"
            ).get("cleaned_cell_count"),
            "terminal_cleaned_cell_count": _mapping(
                cleaning.get("terminal"), "cleaning.terminal"
            ).get("cleaned_cell_count"),
            "cleaned_cell_count": cleaned_cells,
            "cleaned_area_m2": cleaned_area,
            "initial_dirt_cell_count": _mapping(
                cleaning.get("initial"), "cleaning.initial"
            ).get("cell_count"),
            "global_cleaned_fraction": _mapping(
                cleaning.get("terminal"), "cleaning.terminal"
            ).get("cleaned_fraction"),
            "all_three_ready_sample_count": ready_count,
            "maximum_roller_velocity_rad_s": _number(
                cleaning.get("maximum_roller_velocity_rad_s"),
                "cleaning.maximum_roller_velocity_rad_s",
            ),
            "brush_disabled_on_exit": cleaning.get("brush_disabled_on_exit"),
            "dirt_system_disabled_on_exit": cleaning.get(
                "dirt_system_disabled_on_exit"
            ),
        },
        "localization": coverage.get(
            "localization_regression_during_coverage"
        ),
        "completion_evidence": {
            "full_execution_success": coverage.get("full_execution_success"),
            "coverage_quality_success": coverage.get(
                "coverage_quality_success"
            ),
            "safety_success": coverage.get("safety_success"),
            "localization_success": coverage.get("localization_success"),
            "coverage_control_released": coverage.get(
                "brush_disabled_on_exit"
            ),
        },
        "efficiency_boundary": {
            "measured_for_bounded_mission_only": True,
            "official_competition_efficiency_pass": False,
            "official_competition_efficiency_gate_evaluated": False,
            "net_efficiency_formula": "covered_area_m2 / actual_duration_sec * 3600",
            "duration_source": "coverage probe wall duration after planning",
            "path_source": "evaluation-only ground-truth trajectory",
            "full_20000_m2_claim": False,
        },
        "video_evidence": {
            "produced": False,
            "reason": "headless rental host; no video capture path configured",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--coverage-report", type=Path, required=True)
    parser.add_argument("--coverage-trajectory", type=Path, required=True)
    parser.add_argument("--cleaning-status", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        coverage = json.loads(
            args.coverage_report.read_text(encoding="utf-8")
        )
        cleaning = json.loads(
            args.cleaning_status.read_text(encoding="utf-8")
        )
        if not isinstance(coverage, dict) or not isinstance(cleaning, dict):
            raise SummaryError("input reports must be JSON objects")
        with args.coverage_trajectory.open(
            "r", newline="", encoding="utf-8"
        ) as stream:
            trajectory_row_count = sum(1 for _ in csv.DictReader(stream))
        result = summarize(coverage, cleaning, trajectory_row_count)
    except (OSError, json.JSONDecodeError, SummaryError) as exc:
        result = {
            "schema_version": 1,
            "artifact_kind": "day1_bounded_saved_map_coverage_result",
            "terminal_state": "BLOCKED",
            "valid_bounded_mission": False,
            "blockers": ["summary_input_or_validation_failure"],
            "error": str(exc),
        }
    if args.output.exists():
        raise SystemExit("refusing existing summary output")
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return 0 if result.get("valid_bounded_mission") is True else 2


if __name__ == "__main__":
    raise SystemExit(main())
