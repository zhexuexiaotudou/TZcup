#!/usr/bin/env python3
"""Reliability event ledger and binomial qualification calculations."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable


def binomial_cdf(k: int, n: int, p: float) -> float:
    if n <= 0:
        raise ValueError("n must be positive")
    if not 0 <= k <= n:
        raise ValueError("k must satisfy 0 <= k <= n")
    if not 0.0 <= p <= 1.0:
        raise ValueError("p must be in [0, 1]")
    if p == 0.0:
        return 1.0
    if p == 1.0:
        return 1.0 if k == n else 0.0

    terms = [
        math.exp(
            math.lgamma(n + 1)
            - math.lgamma(i + 1)
            - math.lgamma(n - i + 1)
            + i * math.log(p)
            + (n - i) * math.log1p(-p)
        )
        for i in range(k + 1)
    ]
    return min(1.0, math.fsum(terms))


def clopper_pearson_upper_bound(
    failures: int,
    trials: int,
    confidence: float = 0.95,
) -> float:
    if trials <= 0:
        raise ValueError("trials must be positive")
    if not 0 <= failures <= trials:
        raise ValueError("failures must satisfy 0 <= failures <= trials")
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must be in (0, 1)")

    target_cdf = 1.0 - confidence
    low = failures / trials
    high = 1.0
    for _ in range(120):
        midpoint = (low + high) / 2.0
        if binomial_cdf(failures, trials, midpoint) > target_cdf:
            low = midpoint
        else:
            high = midpoint
    return (low + high) / 2.0


def required_zero_failure_days(
    confidence: float = 0.95,
    target_rate: float = 0.01,
) -> int:
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must be in (0, 1)")
    if not 0.0 < target_rate < 1.0:
        raise ValueError("target_rate must be in (0, 1)")
    return math.ceil(math.log(1.0 - confidence) / math.log(1.0 - target_rate))


def minimum_days_for_failures(
    failures: int,
    confidence: float = 0.95,
    target_rate: float = 0.01,
    search_limit: int = 100_000,
) -> int:
    if failures < 0:
        raise ValueError("failures must be non-negative")
    trials = max(1, failures + 1)
    while trials <= search_limit:
        if clopper_pearson_upper_bound(failures, trials, confidence) < target_rate:
            return trials
        trials += 1
    raise RuntimeError("qualification sample exceeds search_limit")


def evaluate_vehicle_days(rows: Iterable[dict[str, str]]) -> dict[str, object]:
    materialized = list(rows)
    if not materialized:
        return {
            "status": "NOT_MEASURED",
            "vehicle_days": 0,
            "primary_failures": 0,
            "upper_bound_95": None,
            "target_met": False,
            "errors": [],
        }

    errors: list[str] = []
    unique_days: set[tuple[str, str]] = set()
    failures = 0
    for index, row in enumerate(materialized, start=2):
        vehicle_id = row.get("vehicle_id", "").strip()
        service_day = row.get("service_day", "").strip()
        fault = row.get("primary_fault", "").strip().lower()
        if not vehicle_id or not service_day:
            errors.append(f"row {index}: vehicle_id and service_day are required")
        if fault not in {"true", "false", "1", "0", "yes", "no"}:
            errors.append(f"row {index}: primary_fault must be boolean")
            continue
        unique_days.add((vehicle_id, service_day))
        if fault in {"true", "1", "yes"}:
            failures += 1

    trials = len(unique_days)
    upper_bound = (
        clopper_pearson_upper_bound(failures, trials)
        if trials and not errors
        else None
    )
    return {
        "status": "MEASURED" if trials and not errors else "FAIL",
        "vehicle_days": trials,
        "primary_failures": failures,
        "upper_bound_95": upper_bound,
        "target_met": bool(
            upper_bound is not None and upper_bound < 0.01 and not errors
        ),
        "errors": errors,
    }


def qualification_plan(confidence: float = 0.95, target_rate: float = 0.01) -> dict[str, object]:
    zero = required_zero_failure_days(confidence, target_rate)
    one = minimum_days_for_failures(1, confidence, target_rate)
    two = minimum_days_for_failures(2, confidence, target_rate)
    return {
        "status": "PLAN_ONLY_NOT_MEASURED",
        "confidence": confidence,
        "target_upper_bound": target_rate,
        "minimum_vehicle_days": {
            "zero_primary_failures": zero,
            "one_primary_failure": one,
            "two_primary_failures": two,
        },
        "claim_policy": "The sample-size table is a qualification gate, not an observed failure rate.",
    }


def load_observations(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = ("vehicle_id", "service_day", "primary_fault")
        if tuple(reader.fieldnames or ()) != required:
            raise ValueError(
                f"unexpected columns: {reader.fieldnames!r}; expected {required!r}"
            )
        return list(reader)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", action="store_true")
    parser.add_argument("--observations", type=Path)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    try:
        if args.observations:
            path = (
                root / args.observations
                if not args.observations.is_absolute()
                else args.observations
            )
            report = {
                "plan": qualification_plan(),
                "observations": evaluate_vehicle_days(load_observations(path)),
            }
        else:
            report = qualification_plan()
    except (OSError, ValueError, RuntimeError) as exc:
        report = {"status": "ERROR", "errors": [str(exc)]}

    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.write:
        evidence = root / "evidence"
        evidence.mkdir(exist_ok=True)
        (evidence / "reliability-qualification.json").write_text(
            payload, encoding="utf-8"
        )
    print(payload, end="")
    return 0 if report.get("status") != "ERROR" else 1


if __name__ == "__main__":
    raise SystemExit(main())
