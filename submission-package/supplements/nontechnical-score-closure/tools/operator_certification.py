#!/usr/bin/env python3
"""Evaluate the frozen three-day operator certification ledger."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable


REQUIRED_COLUMNS = (
    "operator_id",
    "day",
    "item_id",
    "category",
    "rating",
    "prompt_count",
    "critical_error",
    "evidence_path",
)
ALLOWED_RATINGS = {"A", "B", "C"}
ALLOWED_CATEGORIES = {"safety", "operation"}


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "y"}:
        return True
    if normalized in {"false", "0", "no", "n"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def evaluate(rows: Iterable[dict[str, str]]) -> dict[str, object]:
    materialized = list(rows)
    errors: list[str] = []
    safety_rows = 0
    operation_rows = 0
    safety_failures: list[str] = []
    operation_ab = 0
    operators: set[str] = set()

    for index, row in enumerate(materialized, start=2):
        operator_id = row.get("operator_id", "").strip()
        if not operator_id:
            errors.append(f"row {index}: empty operator_id")
            continue
        operators.add(operator_id)

        try:
            day = int(row.get("day", ""))
        except ValueError:
            errors.append(f"row {index}: day must be an integer")
            continue
        if day not in {1, 2, 3}:
            errors.append(f"row {index}: day must be 1, 2, or 3")

        rating = row.get("rating", "").strip().upper()
        if rating not in ALLOWED_RATINGS:
            errors.append(f"row {index}: rating must be A, B, or C")

        category = row.get("category", "").strip().lower()
        if category not in ALLOWED_CATEGORIES:
            errors.append(f"row {index}: category must be safety or operation")
            continue

        try:
            prompt_count = int(row.get("prompt_count", ""))
        except ValueError:
            errors.append(f"row {index}: prompt_count must be a non-negative integer")
            prompt_count = -1
        if prompt_count < 0:
            errors.append(f"row {index}: prompt_count must be a non-negative integer")

        try:
            critical_error = _parse_bool(row.get("critical_error", ""))
        except ValueError as exc:
            errors.append(f"row {index}: {exc}")
            continue

        if not row.get("item_id", "").strip():
            errors.append(f"row {index}: empty item_id")
        if not row.get("evidence_path", "").strip():
            errors.append(f"row {index}: empty evidence_path")

        if category == "safety":
            safety_rows += 1
            if rating != "A" or critical_error:
                safety_failures.append(f"{operator_id}:{row.get('item_id', '')}")
        else:
            operation_rows += 1
            if rating in {"A", "B"}:
                operation_ab += 1

    safety_pass = safety_rows > 0 and not safety_failures
    operation_rate = operation_ab / operation_rows if operation_rows else 0.0
    operation_pass = operation_rows > 0 and operation_rate >= 0.90

    status = (
        "NOT_MEASURED"
        if not materialized
        else "PASS"
        if not errors and safety_pass and operation_pass
        else "FAIL"
    )
    return {
        "status": status,
        "rows": len(materialized),
        "operators": len(operators),
        "safety_rows": safety_rows,
        "operation_rows": operation_rows,
        "safety_failures": safety_failures,
        "operation_ab_rate": round(operation_rate, 6),
        "criteria": {
            "safety_all_a_and_no_critical_error": safety_pass,
            "operation_ab_at_least_0_90": operation_pass,
        },
        "errors": errors,
        "claim_policy": "NOT_MEASURED or FAIL cannot be reported as three-day mastery.",
    }


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        columns = tuple(reader.fieldnames or ())
        if columns != REQUIRED_COLUMNS:
            raise ValueError(
                f"unexpected columns: {columns!r}; expected {REQUIRED_COLUMNS!r}"
            )
        return list(reader)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("data/operator-certification-template.csv"),
    )
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    try:
        rows = load_rows(root / args.input if not args.input.is_absolute() else args.input)
        report = evaluate(rows)
    except (OSError, ValueError) as exc:
        report = {
            "status": "ERROR",
            "errors": [str(exc)],
            "claim_policy": "An invalid ledger cannot support a certification claim.",
        }

    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.write:
        evidence = root / "evidence"
        evidence.mkdir(exist_ok=True)
        output = evidence / "operator-certification-report.json"
        output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["status"] in {"PASS", "NOT_MEASURED"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
