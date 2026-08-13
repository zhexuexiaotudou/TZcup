#!/usr/bin/env python3
"""Score a frozen V1 verifier trace on G10 HOLDOUT without GT runtime leakage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def metrics(rows: list[dict]) -> dict:
    accepted = [row for row in rows if row["decision"] == "ACCEPT"]
    correct_accepted = [row for row in accepted if row["truth_kind"] == "target" and row["predicted_class"] == row["truth_class"]]
    wrong_accepted = [row for row in accepted if row["truth_kind"] == "target" and row["predicted_class"] != row["truth_class"]]
    negative_accepted = [row for row in accepted if row["truth_kind"] == "negative"]
    correct_targets = [row for row in rows if row["truth_kind"] == "target"]
    small_targets = [row for row in correct_targets if row.get("small_at_first_proposal")]
    accepted_correct_ids = {row["encounter_id"] for row in correct_accepted}
    result = {
        "accepted": len(accepted),
        "confirmed_actionable_precision": len(correct_accepted) / max(len(accepted), 1),
        "wrong_confirmed_actionable_rate": len(wrong_accepted) / max(len(accepted), 1),
        "negative_only_accepted_target_rate": len(negative_accepted) / max(sum(row["truth_kind"] == "negative" for row in rows), 1),
        "wrong_class_accepted_target_rate": len(wrong_accepted) / max(len(correct_targets), 1),
        "correct_target_acceptance_recall": len({row["encounter_id"] for row in correct_targets} & accepted_correct_ids) / max(len({row["encounter_id"] for row in correct_targets}), 1),
        "small_target_acceptance_recall": len({row["encounter_id"] for row in small_targets} & accepted_correct_ids) / max(len({row["encounter_id"] for row in small_targets}), 1),
        "false_CLEAN_NOW": len(negative_accepted),
        "wrong_class_CLEAN_NOW": len(wrong_accepted),
    }
    gates = {
        "precision_hard_minimum": result["confirmed_actionable_precision"] >= .98,
        "wrong_confirmed_hard_maximum": result["wrong_confirmed_actionable_rate"] <= .01,
        "negative_accepted_rate": result["negative_only_accepted_target_rate"] <= .005,
        "wrong_class_accepted_rate": result["wrong_class_accepted_target_rate"] <= .01,
        "correct_target_acceptance_recall": result["correct_target_acceptance_recall"] >= .95,
        "small_target_acceptance_recall": result["small_target_acceptance_recall"] >= .90,
        "false_clean_zero": result["false_CLEAN_NOW"] == 0,
        "wrong_class_clean_zero": result["wrong_class_CLEAN_NOW"] == 0,
    }
    return {"metrics": result, "gates": gates, "pass": bool(rows) and all(gates.values())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = json.loads(args.trace.read_text(encoding="utf-8"))
    if payload.get("split") != "G10_HOLDOUT":
        raise ValueError("verifier gate accepts G10_HOLDOUT only")
    result = metrics(payload["rows"])
    report = {
        "schema_version": 1, "protocol": "TRCRV10", "stage": "TRCRV10-05-ACTION-VERIFIER-HOLDOUT",
        "verifier": "V1_rules_only", **result,
        "gt_role": "offline_evaluator_only", "production_runtime_gt_used": False,
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
        "TRCRV10_ACTION_VERIFIER_PASS": result["pass"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"metrics": report["metrics"], "gates": report["gates"], "pass": report["pass"]}, indent=2))
    return 0 if report["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
