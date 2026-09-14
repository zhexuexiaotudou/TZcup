#!/usr/bin/env python3
"""Auditable person-hour and energy model for social-benefit scenarios."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REQUIRED_FIELDS = (
    "manual_cleaning_hours_per_day",
    "manual_supervision_hours_per_day",
    "manual_rework_hours_per_day",
    "autonomous_supervision_hours_per_day",
    "autonomous_rework_hours_per_day",
    "vehicle_kwh_per_day",
    "manual_equipment_kwh_per_day",
)


def evaluate_scenario(scenario: dict[str, object]) -> dict[str, object]:
    missing = [key for key in REQUIRED_FIELDS if key not in scenario]
    if missing:
        raise ValueError(f"scenario is missing fields: {missing}")
    values: dict[str, float] = {}
    for key in REQUIRED_FIELDS:
        value = scenario[key]
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
            raise ValueError(f"{key} must be a non-negative number")
        values[key] = float(value)

    baseline = (
        values["manual_cleaning_hours_per_day"]
        + values["manual_supervision_hours_per_day"]
        + values["manual_rework_hours_per_day"]
    )
    autonomous = (
        values["autonomous_supervision_hours_per_day"]
        + values["autonomous_rework_hours_per_day"]
    )
    reduction = baseline - autonomous
    rate = reduction / baseline if baseline else 0.0
    energy_delta = (
        values["vehicle_kwh_per_day"] - values["manual_equipment_kwh_per_day"]
    )
    return {
        "id": scenario.get("id"),
        "label": scenario.get("label"),
        "baseline_person_hours": round(baseline, 6),
        "autonomous_person_hours": round(autonomous, 6),
        "person_hour_reduction": round(reduction, 6),
        "person_hour_reduction_rate": round(rate, 6),
        "energy_delta_kwh": round(energy_delta, 6),
        "quality_claimable": False,
        "project_result_claimable": False,
        "evidence_status": "ILLUSTRATIVE_ASSUMPTION_NOT_PROJECT_RESULT",
    }


def evaluate_document(document: dict[str, object]) -> dict[str, object]:
    scenarios = document.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("scenarios must be a non-empty list")
    if document.get("quality_claimable") is not False:
        raise ValueError("quality_claimable must remain false without field measurements")
    return {
        "status": "MODEL_ONLY_NOT_FIELD_MEASURED",
        "quality_claimable": False,
        "official_points_claimed": 0.0,
        "scenarios": [evaluate_scenario(item) for item in scenarios],
        "claim_policy": "Scenario arithmetic is not an observed project outcome.",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        type=Path,
        default=Path("data/social-benefit-scenarios.json"),
    )
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    path = root / args.scenario if not args.scenario.is_absolute() else args.scenario
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        report = evaluate_document(document)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        report = {"status": "ERROR", "errors": [str(exc)]}

    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.write:
        evidence = root / "evidence"
        evidence.mkdir(exist_ok=True)
        (evidence / "social-benefit-model.json").write_text(
            payload, encoding="utf-8"
        )
    print(payload, end="")
    return 0 if report.get("status") != "ERROR" else 1


if __name__ == "__main__":
    raise SystemExit(main())
