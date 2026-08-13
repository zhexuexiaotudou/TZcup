#!/usr/bin/env python3
"""Freeze the TRCRV10 evidence boundary and product-task semantics."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


PROTOCOL = "TASK-REFORMULATION-CLOSE-RANGE-VERIFICATION-V10"
EXPECTED_V9_COMMIT = "ad497f06e40cc7eed690c35cd562c58ecfef4e14"
REQUIRED_DIRECTORIES = (
    "baseline", "identifiability", "g10", "proposal",
    "close_range_classifier", "action_verifier", "reobserve",
    "holdout_gate", "dev_val", "tracker_map", "online_dev",
    "performance", "freeze", "g5v2", "moving_30seed",
    "spot_clean_30seed", "post_clean", "soak", "faults", "replay",
    "x86_release", "final",
)
STATES = (
    "OBSERVATION", "CANDIDATE", "OBSERVE_AGAIN", "CLOSE_RANGE_READY",
    "CLASSIFIED", "ACTION_VERIFIED", "CONFIRMED", "SCHEDULED",
    "CLEANING", "VERIFYING", "CLEANED", "REJECTED", "EXPIRED",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def evidence(path: Path) -> dict:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], text=True).strip()


def metric(name: str, threshold: object, unit: str, source: str) -> dict:
    return {
        "metric": name,
        "threshold": threshold,
        "evaluation_unit": unit,
        "source": source,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--v9-root", type=Path, required=True)
    parser.add_argument("--baseline-commit", default=EXPECTED_V9_COMMIT)
    args = parser.parse_args()

    if args.baseline_commit != EXPECTED_V9_COMMIT:
        raise RuntimeError("TRCRV10 must derive from the exact accepted V9 commit")
    protocol_commit = git("rev-parse", "HEAD")
    if git("merge-base", "--is-ancestor", args.baseline_commit, protocol_commit) != "":
        raise RuntimeError("unexpected git merge-base output")
    if git("status", "--porcelain"):
        raise RuntimeError("baseline freeze requires a clean protocol worktree")

    final_dir = args.v9_root / "final"
    required_v9 = {
        "status": final_dir / "PERCEPTION_TGARV9_FINAL_STATUS.json",
        "blockers": final_dir / "PERCEPTION_TGARV9_FINAL_BLOCKERS.json",
        "registry": final_dir / "PERCEPTION_TGARV9_MODEL_REGISTRY.json",
        "report": final_dir / "TEMPORAL_GEOMETRY_ARCHITECTURE_RECOVERY_V9_REPORT.md",
    }
    missing = [name for name, path in required_v9.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"V9 evidence missing: {missing}")
    v9_status = read_json(required_v9["status"])
    if not (
        v9_status.get("TGARV9_ALL_ROUTES_EXHAUSTED") is True
        and v9_status.get("MODEL_BLOCKED_INTERNAL") is True
        and v9_status.get("SIMULATION_PRODUCT_COMPLETE") is False
        and v9_status.get("VAL_NEW_read") is False
        and v9_status.get("G5_V2_read") is False
    ):
        raise RuntimeError("V9 final status does not preserve the required blocker boundary")

    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(f"artifact root is not empty: {args.output}")
    args.output.mkdir(parents=True, exist_ok=True)
    for directory in REQUIRED_DIRECTORIES:
        (args.output / directory).mkdir()
    baseline = args.output / "baseline"

    write_json(baseline / "REPO_BASELINE.json", {
        "schema_version": 1,
        "protocol": PROTOCOL,
        "baseline_commit": args.baseline_commit,
        "protocol_commit": protocol_commit,
        "baseline_is_ancestor": True,
        "worktree_clean_at_freeze": True,
        "head_tree": git("rev-parse", "HEAD^{tree}"),
        "branch": git("branch", "--show-current"),
        "required_directories": list(REQUIRED_DIRECTORIES),
    })
    write_json(baseline / "HISTORICAL_MODEL_STATUS.json", {
        "schema_version": 1,
        "immutable": True,
        "historical_facts": {
            "A1_A2_A3": "FAILED",
            "X1_X3": "FAILED",
            "historical_X2": "HISTORICAL_ASSET_BLOCK",
            "MRV2_A_B_C": "FAILED",
            "OPR_A_B_C": "FAILED",
            "RGDRV8_routes": {"A": "FAILED", "B": "FAILED", "C": "FAILED"},
            "TGARV9_routes": {"T1": "FAILED", "T2": "FAILED", "T3": "FAILED"},
            "historical_D1_B_static": "PASS_PRESERVED_EXACT_BYTES_LOST",
            "old_G5_SEALED_FINAL": "PERMANENTLY_CONSUMED",
            "G5_V2_SEALED_FINAL": "UNREAD",
        },
        "v9_final_status": v9_status,
        "v9_evidence": {name: evidence(path) for name, path in required_v9.items()},
        "interpretation": "V10 changes the product decision architecture; it does not rewrite any failed detector result.",
    })
    write_json(baseline / "UNREAD_DATA_BOUNDARY.json", {
        "schema_version": 1,
        "protocol": PROTOCOL,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "G10_DEV_VAL_SEALED_read": False,
        "formal_30_seed_read": False,
        "rules": [
            "Identifiability, G10 TRAIN, and G10 HOLDOUT development may not access VAL_NEW, G5_V2, or G10 DEV_VAL SEALED.",
            "G10 DEV_VAL is opened atomically only after the integrated HOLDOUT gate passes and the whole product chain is frozen.",
            "VAL_NEW is opened after the same freeze only for dataset-supported proposal and domain cross-check metrics.",
            "G5_V2 remains sealed until a valid x86 freeze exists.",
        ],
    })
    write_json(baseline / "TASK_SEMANTICS_V10.json", {
        "schema_version": 1,
        "protocol": PROTOCOL,
        "states": list(STATES),
        "pipeline": [
            "class_agnostic_proposal", "persistent_candidate", "rgbd_map_candidate",
            "safe_approach_or_observe_again", "close_range_four_class_classifier",
            "independent_action_verifier", "multi_frame_consensus", "confirmed",
            "scheduler", "clean_now",
        ],
        "actionability": {
            "OBSERVATION": False,
            "CANDIDATE": False,
            "OBSERVE_AGAIN": False,
            "CLASSIFIED": False,
            "ACTION_VERIFIED": "necessary_but_not_sufficient_until_consensus",
            "CONFIRMED": True,
        },
        "invariants": [
            "CLASSIFIED never directly enters the scheduler clean path.",
            "ACTION_VERIFIED is required before CONFIRMED and any scheduler clean path.",
            "GT class, GT coordinates, semantic masks, and instance IDs are evaluator/training-only.",
            "No T4, T5, T6, or unbounded detector architecture search is authorized.",
            "Maximum OBSERVE_AGAIN count is two per candidate.",
            "Any false-candidate or wrong-class CLEAN_NOW is a hard failure.",
        ],
    })
    metrics = [
        metric("eventual_class_agnostic_proposal_recall", ">=0.98", "target encounter", "V10_DEVELOPMENT_GATE"),
        metric("first_visible_small_proposal_recall", ">=0.95", "small target encounter", "V10_DEVELOPMENT_GATE"),
        metric("close_range_macro_f1", ">=0.98", "four-class crop", "V10_DEVELOPMENT_GATE"),
        metric("close_range_each_class_precision_recall", ">=0.97", "class", "V10_DEVELOPMENT_GATE"),
        metric("close_range_background_specificity", ">=0.995", "background crop", "V10_DEVELOPMENT_GATE"),
        metric("action_verifier_actionable_precision", ">=0.99 preferred; >=0.98 hard", "confirmed actionable target", "PRODUCT_SAFETY_GATE"),
        metric("action_verifier_negative_accept_rate", "<=0.005", "negative-only proposal", "PRODUCT_SAFETY_GATE"),
        metric("confirmed_actionable_precision", ">=0.98", "confirmed actionable target", "PRODUCT_SAFETY_GATE"),
        metric("wrong_confirmed_actionable_rate", "<=0.01", "confirmed actionable target", "PRODUCT_SAFETY_GATE"),
        metric("actual_false_candidate_cleaning", "=0", "cleaning action", "PRODUCT_SAFETY_GATE"),
        metric("actual_wrong_target_cleaning", "=0", "cleaning action", "PRODUCT_SAFETY_GATE"),
        metric("historical_v9_t3_correct_class_recall", "observed=0.8301886792", "track", "HISTORICAL_INTERNAL_GATE"),
        metric("historical_v9_t3_confirmed_precision", "observed=0.9295774648", "track", "HISTORICAL_INTERNAL_GATE"),
    ]
    write_json(baseline / "GATE_PROVENANCE_V10.json", {
        "schema_version": 1,
        "protocol": PROTOCOL,
        "metrics": metrics,
        "separation_rule": "Proposal recall, close-range classification, verifier specificity, confirmed-target precision, wrong actionability, and actual cleaning are distinct evaluation units and must never be collapsed into one accuracy value.",
        "sources": ["COMPETITION_OFFICIAL", "PRODUCT_SAFETY_GATE", "HISTORICAL_INTERNAL_GATE", "V10_DEVELOPMENT_GATE"],
    })
    print(json.dumps({
        "artifact_root": str(args.output.resolve()),
        "baseline_commit": args.baseline_commit,
        "protocol_commit": protocol_commit,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "G10_DEV_VAL_SEALED_read": False,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
