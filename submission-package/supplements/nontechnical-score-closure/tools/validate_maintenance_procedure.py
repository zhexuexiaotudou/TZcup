#!/usr/bin/env python3
"""Validate the frozen three-step maintenance procedure."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def validate(procedure: dict[str, object]) -> list[str]:
    errors: list[str] = []
    if procedure.get("status") != "DESIGN_ONLY_NOT_PHYSICALLY_MEASURED":
        errors.append("procedure status must remain design-only")
    if procedure.get("required_tools") != []:
        errors.append("required_tools must be empty")

    preconditions = procedure.get("preconditions")
    if not isinstance(preconditions, list) or len(preconditions) < 3:
        errors.append("at least three safety and isolation preconditions are required")

    procedures = procedure.get("procedures")
    if not isinstance(procedures, list):
        errors.append("procedures must be a list")
        return errors
    ids = {item.get("id") for item in procedures if isinstance(item, dict)}
    if ids != {"BRUSH", "FILTER"}:
        errors.append("procedures must contain exactly BRUSH and FILTER")

    for item in procedures:
        if not isinstance(item, dict):
            errors.append("procedure entry must be an object")
            continue
        steps = item.get("steps")
        if not isinstance(steps, list) or len(steps) != 3:
            errors.append(f"{item.get('id', 'unknown')} must have exactly three steps")
            continue
        numbers = [step.get("number") for step in steps if isinstance(step, dict)]
        if numbers != [1, 2, 3]:
            errors.append(f"{item.get('id', 'unknown')} step numbers must be 1,2,3")
        for step in steps:
            if not isinstance(step, dict) or not str(step.get("action", "")).strip():
                errors.append(f"{item.get('id', 'unknown')} has an empty action")

    acceptance = procedure.get("physical_acceptance")
    if not isinstance(acceptance, list) or len(acceptance) < 5:
        errors.append("physical acceptance must cover steps, tools, poka-yoke, retention, and interlock")
    elif any(
        isinstance(item, dict) and item.get("status") == "PASS"
        for item in acceptance
    ):
        errors.append("physical acceptance cannot be PASS without measurement")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("data/maintenance-procedure.json"),
    )
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    path = root / args.input if not args.input.is_absolute() else args.input
    try:
        procedure = json.loads(path.read_text(encoding="utf-8"))
        errors = validate(procedure)
        status = "PASS" if not errors else "FAIL"
    except (OSError, json.JSONDecodeError) as exc:
        errors = [str(exc)]
        status = "ERROR"

    report = {
        "status": status,
        "input": path.as_posix(),
        "step_limit": 3,
        "required_tools": [],
        "errors": errors,
        "claim_status": "DESIGN_ONLY_NOT_PHYSICALLY_MEASURED",
    }
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.write:
        evidence = root / "evidence"
        evidence.mkdir(exist_ok=True)
        (evidence / "maintenance-procedure-validation.json").write_text(
            payload, encoding="utf-8"
        )
    print(payload, end="")
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
