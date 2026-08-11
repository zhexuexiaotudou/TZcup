#!/usr/bin/env python3
"""Aggregate prediction-derived OPRV3 product map/behavior evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_report(
    benchmarks: list[dict], *, route: str, removal: dict | None = None
) -> dict:
    if not benchmarks:
        raise ValueError("at least one moving benchmark is required")
    totals = {
        "localization_squared_error_sum": 0.0,
        "localization_error_count": 0,
        "identity_consistency_sum": 0.0,
        "identity_target_count": 0,
        "duplicate_target_count": 0,
        "fragmented_target_count": 0,
        "eligible_target_count": 0,
        "matched_target_count": 0,
        "confirmed_product_target_count": 0,
        "wrong_class_confirmed_action_count": 0,
        "pre_fov_creation_count": 0,
        "removed_target_stale_action_count": 0,
        "removal_capture_count": 0,
        "projection_frame_failure_count": 0,
    }
    mission_count = 0
    source_commits = set()
    for benchmark in benchmarks:
        if benchmark.get("G5_SEALED_FINAL_read") is not False:
            raise ValueError("moving benchmark violates sealed-final boundary")
        source_commit = str(benchmark.get("source_commit", ""))
        if not re.fullmatch(r"[0-9a-f]{40}", source_commit):
            raise ValueError("moving benchmark lacks a full source commit")
        source_commits.add(source_commit)
        product_map = benchmark.get("routes", {}).get(route, {}).get(
            "product_map"
        )
        if not product_map:
            raise ValueError(f"moving benchmark has no {route} product map")
        if product_map.get("GT_used_by_product_pipeline") is not False:
            raise ValueError("product map pipeline consumed GT")
        counts = product_map.get("aggregation_counts", {})
        missing = set(totals) - set(counts)
        if missing:
            raise ValueError(
                f"product map aggregation counts are incomplete: {sorted(missing)}"
            )
        for key in totals:
            totals[key] += counts[key]
        mission_count += len(product_map.get("missions", []))
    if len(source_commits) != 1:
        raise ValueError("moving benchmarks were produced from different source commits")

    eligible = int(totals["eligible_target_count"])
    matched = int(totals["matched_target_count"])
    localization_count = int(totals["localization_error_count"])
    identity_count = int(totals["identity_target_count"])
    removed_stale = (
        int(totals["removed_target_stale_action_count"])
        if int(totals["removal_capture_count"]) > 0
        else None
    )
    removal_status = (
        "prediction_derived_dynamic_removal_capture"
        if int(totals["removal_capture_count"]) > 0
        else "not_executed"
    )
    if removal is not None:
        if removal.get("post_removal_capture_executed") is not True:
            raise ValueError("removal evidence is not an executed capture")
        if removal.get("GT_used_by_product_pipeline") is not False:
            raise ValueError("removal product pipeline consumed GT")
        if removal.get("GT_used_only_for_post_run_scoring") is not True:
            raise ValueError("removal GT boundary is incomplete")
        external_stale = int(removal["stale_action_count"])
        removed_stale = (removed_stale or 0) + external_stale
        removal_status = "independent_post_removal_capture"
    metrics = {
        "eligible_targets": eligible,
        "matched_eligible_targets": matched,
        "confirmed_product_targets": int(
            totals["confirmed_product_target_count"]
        ),
        "product_target_precision": matched / max(
            int(totals["confirmed_product_target_count"]), 1
        ),
        "false_confirmed_target_rate": max(
            0, int(totals["confirmed_product_target_count"]) - matched
        ) / max(int(totals["confirmed_product_target_count"]), 1),
        "map_localization_coverage": matched / max(eligible, 1),
        "map_rmse_m": (
            math.sqrt(
                float(totals["localization_squared_error_sum"])
                / localization_count
            )
            if localization_count
            else None
        ),
        "id_consistency": (
            float(totals["identity_consistency_sum"]) / identity_count
            if identity_count
            else None
        ),
        "duplicate_target_rate": (
            int(totals["duplicate_target_count"]) / max(eligible, 1)
        ),
        "track_fragmentation": (
            int(totals["fragmented_target_count"]) / max(eligible, 1)
        ),
        "wrong_class_leading_to_wrong_clean_action": int(
            totals["wrong_class_confirmed_action_count"]
        ),
        "pre_fov_target_creation": int(totals["pre_fov_creation_count"]),
        "removed_target_stale_action": removed_stale,
        "projection_frame_failures": int(
            totals["projection_frame_failure_count"]
        ),
    }
    return {
        "schema_version": 1,
        "protocol": "OPRV3-07-product-map",
        "source_commit": next(iter(source_commits)),
        "route": route,
        "evaluator": "ProductTrackerV2_plus_DynamicTrashMap",
        "mission_count": mission_count,
        "GT_used_by_product_pipeline": False,
        "GT_used_only_for_post_run_scoring": True,
        "metrics": metrics,
        "aggregation_counts": totals,
        "removal_status": removal_status,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", type=Path, action="append", required=True)
    parser.add_argument("--route", default="MRV2-A")
    parser.add_argument("--removal-evidence", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    benchmarks = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in args.benchmark
    ]
    removal = (
        json.loads(args.removal_evidence.read_text(encoding="utf-8"))
        if args.removal_evidence
        else None
    )
    report = build_report(benchmarks, route=args.route, removal=removal)
    report["inputs"] = {
        "benchmarks": [
            {"path": path.as_posix(), "sha256": sha256(path)}
            for path in args.benchmark
        ],
        "removal": (
            {
                "path": args.removal_evidence.as_posix(),
                "sha256": sha256(args.removal_evidence),
            }
            if args.removal_evidence
            else None
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
