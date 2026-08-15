#!/usr/bin/env python3
"""Select one authorized proposal candidate and operating point on G10 HOLDOUT."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from evaluate_trcrv10_proposals import evaluate_records, load_predictions, load_truth


THRESHOLDS = tuple(round(value / 100, 2) for value in range(5, 96, 2))
PERSISTENCE = (2, 3, 4, 5)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rank(row: dict) -> tuple:
    metrics = row["metrics"]
    return (
        row["pass"],
        min(metrics["eventual_proposal_recall"], metrics["small_eventual_proposal_recall"]),
        -metrics["proposal_fp_per_frame"],
        -metrics["persistence_frames"],
        metrics["threshold"],
    )


def select(truth: dict, candidate_predictions: dict[str, dict]) -> tuple[dict, dict]:
    candidates = {}
    ranked = []
    for candidate_id, predictions in sorted(candidate_predictions.items()):
        sweep = []
        for threshold in THRESHOLDS:
            for persistence in PERSISTENCE:
                result = evaluate_records(truth, predictions, threshold, persistence)
                row = {"candidate_id": candidate_id, **result}
                row.pop("missions")
                sweep.append(row)
                ranked.append(row)
        candidates[candidate_id] = sweep
    selected = max(ranked, key=rank)
    return selected, candidates


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-scenes", type=Path, required=True)
    parser.add_argument("--candidate", action="append", required=True, help="CANDIDATE_ID=raw_inference.json")
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    registry = json.loads(args.registry.read_text(encoding="utf-8"))
    authorized = {
        row["candidate_id"] for row in registry["candidates"]
        if row.get("checkpoint_exists") and row.get("checkpoint_hash_matches")
    }
    candidate_paths = {}
    for value in args.candidate:
        candidate_id, raw_path = value.split("=", 1)
        if candidate_id not in authorized:
            raise ValueError(f"candidate is not an available, hash-matched registry entry: {candidate_id}")
        candidate_paths[candidate_id] = Path(raw_path)
    truth = load_truth(args.capture_scenes)
    selected, candidates = select(
        truth, {candidate_id: load_predictions(path) for candidate_id, path in candidate_paths.items()}
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    model_selection = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "stage": "TRCRV10-03-PROPOSAL-MODEL-SELECTION",
        "selection_data": "G10_HOLDOUT_ONLY",
        "candidate_registry_sha256": sha256(args.registry),
        "raw_inference_sha256": {key: sha256(path) for key, path in candidate_paths.items()},
        "selected_candidate_id": selected["candidate_id"],
        "selected_pass": selected["pass"],
        "ranking_policy": "hard_gates_then_maximize_weakest_recall_then_minimize_fp_per_frame",
        "candidates": candidates,
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    threshold = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "candidate_id": selected["candidate_id"],
        "score_algorithm": "max_class_score_as_litter_objectness",
        "threshold": selected["metrics"]["threshold"],
        "candidate_persistence_frames": selected["metrics"]["persistence_frames"],
        "frozen": selected["pass"],
    }
    report = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "stage": "TRCRV10-03-PROPOSAL-HOLDOUT",
        "selection_data": "G10_HOLDOUT_ONLY",
        "selected_candidate_id": selected["candidate_id"],
        **selected,
        "semantic_gt_role": "offline_evaluator_only",
        "production_runtime_gt_used": False,
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "TRCRV10_PROPOSAL_PASS": selected["pass"],
    }
    (args.output_dir / "PROPOSAL_MODEL_SELECTION.json").write_text(json.dumps(model_selection, indent=2) + "\n")
    (args.output_dir / "PROPOSAL_THRESHOLD.json").write_text(json.dumps(threshold, indent=2) + "\n")
    (args.output_dir / "PROPOSAL_HOLDOUT_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"candidate": selected["candidate_id"], "metrics": selected["metrics"], "pass": selected["pass"]}, indent=2))
    return 0 if selected["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
