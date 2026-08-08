#!/usr/bin/env python3
"""Evaluate sliding-window crop discovery proposals."""

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
    sliding_discovery_predictions,
)
from sanitation_learning.g4_models import build_g4_models  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--discovery-checkpoint", required=True, type=Path)
    parser.add_argument("--max-eval-frames", type=int, default=20)
    args = parser.parse_args()

    rows = load_frame_rows(args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root)
    records = load_instance_records(args.evidence_dir / "g4_instance_records.jsonl")
    instances_by_key = index_instance_records(records)
    val_rows = [row for row in rows if row["split"] == "val"][: args.max_eval_frames]
    test_rows = [row for row in rows if row["split"] == "test"][: args.max_eval_frames]
    model = build_g4_models()["discovery"]
    state = torch.load(args.discovery_checkpoint, map_location="cpu")
    model.load_state_dict(state["state_dict"])
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device).eval()
    for threshold in (0.35, 0.5, 0.7):
        results = {}
        for name, eval_rows in (("val", val_rows), ("test", test_rows)):
            frames = sliding_discovery_predictions(
                model,
                eval_rows,
                instances_by_key,
                device=device,
                threshold=threshold,
            )
            results[name] = discovery_metrics(frames)
        print(
            json.dumps(
                {
                    "threshold": threshold,
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


if __name__ == "__main__":
    raise SystemExit(main())
