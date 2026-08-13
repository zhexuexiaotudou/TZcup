#!/usr/bin/env python3
"""Recompute the complete TRCRV10 integrated HOLDOUT gate from encounter traces."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


CLASSES = ("plastic_bottle", "metal_can", "paper_litter")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compute(rows: list[dict]) -> dict:
    targets = [row for row in rows if row["truth_kind"] == "target"]
    small = [row for row in targets if row["first_visible_short_side_px"] < 18]
    correct = [row for row in targets if row.get("confirmed_class") == row["truth_class"]]
    correct_ids = {row["encounter_id"] for row in correct}
    confirmed = [row for row in rows if row.get("confirmed_actionable")]
    correct_confirmed = [row for row in confirmed if row["truth_kind"] == "target" and row.get("confirmed_class") == row["truth_class"]]
    wrong_confirmed = [row for row in confirmed if row["truth_kind"] == "target" and row.get("confirmed_class") != row["truth_class"]]
    negative_confirmed = [row for row in confirmed if row["truth_kind"] == "negative"]
    per_class = {}
    for class_id in CLASSES:
        selected = [row for row in targets if row["truth_class"] == class_id]
        per_class[class_id] = sum(row["encounter_id"] in correct_ids for row in selected) / max(len(selected), 1)
    clean_opportunities = [row for row in targets if row.get("clean_opportunity")]
    missed = [row for row in clean_opportunities if not row.get("clean_now")]
    distances = [float(row.get("extra_distance_m", 0.0)) for row in rows]
    times = [float(row.get("extra_time_s", 0.0)) for row in rows]
    metrics = {
        "encounters": len(rows), "target_encounters": len(targets),
        "eventual_correct_class_recall": len(correct) / max(len(targets), 1),
        "per_class_eventual_correct_recall": per_class,
        "small_target_eventual_correct_class_recall": sum(row["encounter_id"] in correct_ids for row in small) / max(len(small), 1),
        "confirmed_actionable_precision": len(correct_confirmed) / max(len(confirmed), 1),
        "wrong_confirmed_actionable_rate": len(wrong_confirmed) / max(len(confirmed), 1),
        "negative_only_confirmed_actionable_rate": len(negative_confirmed) / max(sum(row["truth_kind"] == "negative" for row in rows), 1),
        "false_CLEAN_NOW": sum(row.get("clean_now") and row["truth_kind"] == "negative" for row in rows),
        "wrong_class_CLEAN_NOW": sum(row.get("clean_now") and row["truth_kind"] == "target" and row.get("confirmed_class") != row["truth_class"] for row in rows),
        "clean_opportunity_miss": len(missed) / max(len(clean_opportunities), 1),
        "maximum_reobserve_count": max((int(row.get("reobserve_count", 0)) for row in rows), default=0),
        "mean_extra_distance_m": float(np.mean(distances)) if distances else 0.0,
        "median_extra_distance_m": float(np.median(distances)) if distances else 0.0,
        "p95_extra_distance_m": float(np.percentile(distances, 95)) if distances else 0.0,
        "mean_extra_time_s": float(np.mean(times)) if times else 0.0,
        "median_extra_time_s": float(np.median(times)) if times else 0.0,
        "p95_extra_time_s": float(np.percentile(times, 95)) if times else 0.0,
    }
    gates = {
        "eventual_correct_class_recall": metrics["eventual_correct_class_recall"] >= .95,
        "per_class_correct_recall": all(value >= .95 for value in per_class.values()),
        "small_correct_class_recall": metrics["small_target_eventual_correct_class_recall"] >= .90,
        "confirmed_actionable_precision": metrics["confirmed_actionable_precision"] >= .98,
        "wrong_confirmed_actionable": metrics["wrong_confirmed_actionable_rate"] <= .01,
        "negative_confirmed_actionable": metrics["negative_only_confirmed_actionable_rate"] <= .01,
        "false_clean_zero": metrics["false_CLEAN_NOW"] == 0,
        "wrong_class_clean_zero": metrics["wrong_class_CLEAN_NOW"] == 0,
        "clean_opportunity_miss": metrics["clean_opportunity_miss"] <= .02,
        "maximum_reobserve": metrics["maximum_reobserve_count"] <= 2,
    }
    return {"metrics": metrics, "gates": gates, "pass": bool(targets) and all(gates.values())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--g10-qa", type=Path, required=True)
    parser.add_argument("--proposal", type=Path, required=True)
    parser.add_argument("--classifier", type=Path, required=True)
    parser.add_argument("--verifier", type=Path, required=True)
    parser.add_argument("--reobserve", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    trace = json.loads(args.trace.read_text(encoding="utf-8"))
    if trace.get("split") != "G10_HOLDOUT":
        raise ValueError("integrated gate accepts G10_HOLDOUT only")
    reports = {
        "g10_qa": json.loads(args.g10_qa.read_text(encoding="utf-8")),
        "proposal": json.loads(args.proposal.read_text(encoding="utf-8")),
        "classifier": json.loads(args.classifier.read_text(encoding="utf-8")),
        "verifier": json.loads(args.verifier.read_text(encoding="utf-8")),
        "reobserve": json.loads(args.reobserve.read_text(encoding="utf-8")),
    }
    dependencies = {
        "g10_qa": reports["g10_qa"].get("G10_CAPTURE_QA_PASS") is True,
        "proposal": reports["proposal"].get("TRCRV10_PROPOSAL_PASS") is True,
        "classifier": reports["classifier"].get("TRCRV10_CLOSE_RANGE_CLASSIFIER_PASS") is True,
        "verifier": reports["verifier"].get("TRCRV10_ACTION_VERIFIER_PASS") is True,
        "reobserve": reports["reobserve"].get("TRCRV10_REOBSERVE_PASS") is True,
    }
    result = compute(trace["rows"])
    passed = all(dependencies.values()) and result["pass"]
    report = {
        "schema_version": 1, "protocol": "TRCRV10", "stage": "TRCRV10-07-INTEGRATED-HOLDOUT",
        "input_sha256": {name: sha256(path) for name, path in {
            "trace": args.trace, "g10_qa": args.g10_qa, "proposal": args.proposal,
            "classifier": args.classifier, "verifier": args.verifier, "reobserve": args.reobserve,
        }.items()},
        "dependency_gates": dependencies, **result,
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
        "sealed_access_authorized_next": passed,
        "TRCRV10_INTEGRATED_HOLDOUT_PASS": passed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"dependencies": dependencies, "metrics": report["metrics"], "gates": report["gates"], "pass": passed}, indent=2))
    return 0 if passed else 4


if __name__ == "__main__":
    raise SystemExit(main())
