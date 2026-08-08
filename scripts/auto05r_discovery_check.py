#!/usr/bin/env python3
"""Quick G4 discovery candidate check for architecture/training iterations."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_data import (  # noqa: E402
    index_instance_records,
    load_frame_rows,
    load_instance_records,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    discovery_metrics,
    discovery_predictions,
)
from sanitation_learning.g4_train import train_discovery  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=5e-4)
    parser.add_argument("--max-train-frames", type=int, default=600)
    parser.add_argument("--max-eval-frames", type=int, default=100)
    parser.add_argument("--threshold", type=float, default=0.35)
    args = parser.parse_args()

    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    rows = load_frame_rows(args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root)
    records = load_instance_records(args.evidence_dir / "g4_instance_records.jsonl")
    instances_by_key = index_instance_records(records)
    train_rows = [row for row in rows if row["split"] == "train"][: args.max_train_frames]
    val_rows = [row for row in rows if row["split"] == "val"][: args.max_eval_frames]
    test_rows = [row for row in rows if row["split"] == "test"][: args.max_eval_frames]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model, training = train_discovery(
        train_rows,
        instances_by_key,
        device=device,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=20260807,
        val_rows=val_rows[:50],
        checkpoint_path=output / "discovery_check.pt",
        early_stopping_patience=8,
        load_best=True,
    )
    results = {}
    for name, eval_rows in (("val", val_rows), ("test", test_rows)):
        frames = discovery_predictions(
            model,
            eval_rows,
            instances_by_key,
            device=device,
            threshold=args.threshold,
        )
        results[name] = discovery_metrics(frames)
    report = {
        "architecture": "resnet18_fpn" if _has_torchvision() else "small_fpn",
        "threshold": args.threshold,
        "train_frames": len(train_rows),
        "results": results,
        "training": training,
    }
    report_path = output / "discovery_check_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "architecture": report["architecture"],
                "val_recall": results["val"]["all_gt_candidate_recall"],
                "val_fp_per_min": results["val"]["false_candidates_per_min"],
                "val_neg_fp": results["val"]["negative_only_fp_per_frame"],
                "test_recall": results["test"]["all_gt_candidate_recall"],
                "test_fp_per_min": results["test"]["false_candidates_per_min"],
                "test_neg_fp": results["test"]["negative_only_fp_per_frame"],
            },
            indent=2,
        )
    )
    return 0


def _has_torchvision() -> bool:
    try:
        import torchvision  # noqa: F401

        return True
    except Exception:
        return False


if __name__ == "__main__":
    raise SystemExit(main())
