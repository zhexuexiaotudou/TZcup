#!/usr/bin/env python3
"""Register the only proposal candidates authorized by TRCRV10.

This is deliberately an evidence/provenance step.  It does not select a model,
read a V10 holdout split, or tune a threshold.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def candidate(candidate_id: str, checkpoint: Path, expected_sha: str, evidence: list[Path], metrics: dict) -> dict:
    exists = checkpoint.is_file()
    actual_sha = sha256(checkpoint) if exists else None
    return {
        "candidate_id": candidate_id,
        "checkpoint": str(checkpoint.resolve()),
        "checkpoint_exists": exists,
        "expected_sha256": expected_sha,
        "actual_sha256": actual_sha,
        "checkpoint_hash_matches": exists and actual_sha == expected_sha,
        "evidence": [str(path.resolve()) for path in evidence],
        "historical_metrics": metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--v8-root", type=Path, required=True)
    parser.add_argument("--v9-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    route_selection_path = args.v8_root / "route_a/selection/ROUTE_A_HOLDOUT_SELECTION.json"
    ga1_path = args.v8_root / "ga1_forensics/RGDRV8_GA1_FAILURE_TAXONOMY.json"
    t3_path = args.v9_root / "04_t3/selection/T3_G9_HOLDOUT_REPORT.json"
    route = load(route_selection_path)
    ga1 = load(ga1_path)
    t3 = load(t3_path)

    rows = [
        candidate(
            "rgdrv8_route_a_best",
            args.v8_root / "route_a/training/run" / route["selected_checkpoint"],
            route["selected_checkpoint_sha256"],
            [route_selection_path],
            route["selected_metrics"],
        ),
        {
            "candidate_id": "rgdrv8_ga1_best",
            "checkpoint": None,
            "checkpoint_exists": False,
            "expected_sha256": ga1["checkpoint_sha256"],
            "actual_sha256": None,
            "checkpoint_hash_matches": False,
            "evidence": [str(ga1_path.resolve())],
            "historical_metrics": ga1["summary"],
            "unavailable_reason": "historical evidence does not contain a locally resolvable checkpoint path",
        },
        candidate(
            "tgarv9_grounding_dino_best",
            args.v9_root / "04_t3/training/run" / t3["selected_checkpoint"],
            t3["selected_checkpoint_sha256"],
            [
                args.v9_root / "04_t3/decision/T3_ARCHITECTURE_DECISION.json",
                t3_path,
            ],
            t3["selected_metrics"],
        ),
    ]
    authorized_ids = {
        "rgdrv8_route_a_best",
        "rgdrv8_ga1_best",
        "tgarv9_grounding_dino_best",
    }
    gates = {
        "candidate_allowlist_exact": {row["candidate_id"] for row in rows} == authorized_ids,
        "available_checkpoint_hashes_match": all(
            row["checkpoint_hash_matches"] for row in rows if row["checkpoint_exists"]
        ),
        "new_detector_training_forbidden": True,
        "v10_holdout_read": False,
        "g10_dev_val_sealed_read": False,
        "val_new_read": False,
        "g5_v2_read": False,
    }
    payload = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "stage": "TRCRV10-03-PROPOSAL-CANDIDATE-REGISTRY",
        "purpose": "provenance registry only; selection and thresholds remain unfrozen",
        "selection_status": "NOT_RUN_G10_HOLDOUT_PENDING",
        "score_algorithms_allowed": ["max_class_score", "class_agnostic_proposal_output"],
        "candidate_persistence_range_frames": [2, 5],
        "gates": gates,
        "candidates": rows,
        "pass": all(value is True or value is False and key.endswith("_read") for key, value in gates.items()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output.resolve()), "pass": payload["pass"]}))
    return 0 if payload["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
