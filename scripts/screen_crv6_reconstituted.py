#!/usr/bin/env python3
"""Freeze an R1 threshold on holdout, then run one non-gating static VAL diagnostic."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from screen_ddrv4_d1 import (  # noqa: E402
    apply_gates, atomic_json, best_checkpoint, infer, load_truth, metrics,
    select_threshold, sha256,
)
from sanitation_learning.ddrv4_boundary import G7_DATASET_ID, require_ddrv4_selection_inputs  # noqa: E402
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402

HISTORICAL_D1B_SHA256 = "481374d4839e72f05fff0d6d2f6135bc7d715d5c2faf84e75d7d97ca3fc6a361"


def init_model(candidate_dir: Path):
    from mmdet.apis import init_detector
    return init_detector(
        str(candidate_dir / "ddrv4_d1_rtmdet_s_config.py"),
        str(best_checkpoint(candidate_dir)), device="cuda:0",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g7-root", required=True, type=Path)
    parser.add_argument("--prepared", required=True, type=Path)
    parser.add_argument("--candidate-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    require_ddrv4_selection_inputs([G7_DATASET_ID])
    if args.output.exists():
        raise FileExistsError(f"CRV6 static output exists: {args.output}")
    prep = json.loads((args.prepared / "D1_PREP_REPORT.json").read_text(encoding="utf-8"))
    if prep.get("untouched_val_used_for_selection") is not False:
        raise RuntimeError("static VAL boundary is invalid")
    checkpoint = best_checkpoint(args.candidate_dir)
    candidate_sha = sha256(checkpoint)
    if candidate_sha == HISTORICAL_D1B_SHA256:
        raise RuntimeError("R1 candidate unexpectedly matches historical hash; use recovered route")
    args.output.mkdir(parents=True, exist_ok=False)
    patch_mmdet_cuda_nms()
    started = time.perf_counter()

    model = init_model(args.candidate_dir)
    holdout = load_truth(args.prepared / "holdout.json", args.g7_root)
    selected, sweep = select_threshold(infer(model, holdout, args.batch_size))
    selection = {
        "schema_version": 1, "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "stage": "CRV6-01-SELECTION", "candidate_id": "D1B_RECON_R1",
        "checkpoint_sha256": candidate_sha,
        "selection_data": "G7_STATIC_IN_DOMAIN_HOLDOUT_ONLY",
        "selection_annotation_sha256": sha256(args.prepared / "holdout.json"),
        "selected_threshold": selected["threshold"],
        "selected_operating_point": selected, "threshold_sweep": sweep,
        "G7_static_VAL_read_before_selection_freeze": False,
        "G7_static_VAL_used_for_selection": False,
        "historical_D1B_checkpoint_impersonated": False,
        "selection_frozen_unix_ns": time.time_ns(),
    }
    selection_path = args.output / "CRV6_R1_SELECTION.json"
    atomic_json(selection_path, selection)
    if json.loads(selection_path.read_text(encoding="utf-8")) != selection:
        raise RuntimeError("selection freeze failed round-trip verification")

    # Only after the immutable selection file exists is historical static VAL read once.
    val = load_truth(args.prepared / "val.json", args.g7_root)
    diagnostic = apply_gates(metrics(
        infer(model, val, args.batch_size), float(selected["threshold"])
    ))
    report = {
        "schema_version": 1, "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "stage": "CRV6-02", "candidate_id": "D1B_RECON_R1",
        "checkpoint_sha256": candidate_sha,
        "historical_D1B_sha256": HISTORICAL_D1B_SHA256,
        "candidate_hash_differs_from_historical": candidate_sha != HISTORICAL_D1B_SHA256,
        "threshold": selected["threshold"],
        "threshold_selected_on": "G7_STATIC_IN_DOMAIN_HOLDOUT_ONLY",
        "selection_freeze": {"path": selection_path.name, "sha256": sha256(selection_path)},
        "G7_static_VAL_role": "NON_GATING_HISTORICAL_REGRESSION",
        "G7_static_VAL_evaluation_count": 1,
        "G7_static_VAL_used_for_selection": False,
        "diagnostic_targets": {"precision": 0.93, "recall": 0.93, "macro_f1": 0.93},
        "proceed_threshold": 0.95, "severe_regression_threshold": 0.90,
        "VAL_diagnostic": diagnostic,
        "CRV6_STATIC_DIAGNOSTIC_AT_LEAST_0_95": all(
            diagnostic[key] >= 0.95 for key in ("precision", "recall", "macro_f1")
        ),
        "CRV6_RECONSTITUTION_SEVERE_REGRESSION": any(
            diagnostic[key] < 0.90 for key in ("precision", "recall", "macro_f1")
        ),
        "duration_s": time.perf_counter() - started,
    }
    atomic_json(args.output / "CRV6_STATIC_REGRESSION_REPORT.json", report)
    return 0 if report["CRV6_STATIC_DIAGNOSTIC_AT_LEAST_0_95"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
