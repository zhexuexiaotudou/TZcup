#!/usr/bin/env python3
"""Evaluate bounded re-observation traces on G10 HOLDOUT."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def evaluate(rows: list[dict]) -> dict:
    true_candidates = [row for row in rows if row["truth_kind"] == "target"]
    false_candidates = [row for row in rows if row["truth_kind"] == "negative"]
    resolved = [row for row in true_candidates if int(row["reobserve_count"]) <= 2 and
                row["outcome"] in {"CLASSIFICATION_CONDITION_REACHED", "UNREACHABLE_FOR_VISUAL_CONFIRMATION"}]
    extra_distance = [float(row.get("extra_distance_m", 0.0)) for row in rows]
    extra_time = [float(row.get("extra_time_s", 0.0)) for row in rows]
    metrics = {
        "true_candidates": len(true_candidates),
        "resolved_or_unreachable_within_two_rate": len(resolved) / max(len(true_candidates), 1),
        "false_candidate_unbounded_approach_count": sum(int(row["reobserve_count"]) > 2 for row in false_candidates),
        "maximum_reobserve_count": max((int(row["reobserve_count"]) for row in rows), default=0),
        "median_extra_distance_m": float(np.median(extra_distance)) if extra_distance else 0.0,
        "p95_extra_distance_m": float(np.percentile(extra_distance, 95)) if extra_distance else 0.0,
        "median_extra_time_s": float(np.median(extra_time)) if extra_time else 0.0,
        "p95_extra_time_s": float(np.percentile(extra_time, 95)) if extra_time else 0.0,
        "coverage_efficiency_impact_fraction": sum(extra_distance) / max(sum(float(row.get("baseline_distance_m", 0.0)) for row in rows), 1e-12),
    }
    gates = {
        "resolved_or_unreachable_within_two": metrics["resolved_or_unreachable_within_two_rate"] >= .95,
        "false_candidate_unbounded_approach_zero": metrics["false_candidate_unbounded_approach_count"] == 0,
        "maximum_reobserve_bounded": metrics["maximum_reobserve_count"] <= 2,
    }
    return {"metrics": metrics, "gates": gates, "pass": bool(true_candidates) and all(gates.values())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = json.loads(args.trace.read_text(encoding="utf-8"))
    if payload.get("split") != "G10_HOLDOUT":
        raise ValueError("re-observation gate accepts G10_HOLDOUT only")
    result = evaluate(payload["rows"])
    report = {
        "schema_version": 1, "protocol": "TRCRV10", "stage": "TRCRV10-06-REOBSERVE-HOLDOUT",
        **result, "production_runtime_gt_used": False, "gt_role": "offline_evaluator_only",
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
        "TRCRV10_REOBSERVE_PASS": result["pass"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"metrics": report["metrics"], "gates": report["gates"], "pass": report["pass"]}, indent=2))
    return 0 if report["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
