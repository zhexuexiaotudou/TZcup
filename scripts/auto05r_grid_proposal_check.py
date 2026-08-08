#!/usr/bin/env python3
"""Evaluate grid+crop classifier proposals against G4 screening candidate gates."""

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
    classify_detections,
    discrete_metrics,
    discovery_metrics,
    grid_proposal_predictions,
    match_discrete_predictions,
)
from sanitation_learning.g4_models import build_g4_models  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--classifier-checkpoint", required=True, type=Path)
    parser.add_argument("--max-eval-frames", type=int, default=100)
    args = parser.parse_args()

    rows = load_frame_rows(args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root)
    records = load_instance_records(args.evidence_dir / "g4_instance_records.jsonl")
    instances_by_key = index_instance_records(records)
    val_rows = [row for row in rows if row["split"] == "val"][: args.max_eval_frames]
    test_rows = [row for row in rows if row["split"] == "test"][: args.max_eval_frames]
    classifier = build_g4_models()["classifier"]
    state = torch.load(args.classifier_checkpoint, map_location="cpu")
    classifier.load_state_dict(state["state_dict"])
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    classifier.to(device).eval()
    for threshold in (0.5, 0.7, 0.9):
        outputs = {}
        for name, eval_rows in (("val", val_rows), ("test", test_rows)):
            frames = grid_proposal_predictions(
                classifier,
                eval_rows,
                instances_by_key,
                device=device,
                class_threshold=threshold,
                scales=(16, 24, 32, 48, 64, 96, 128, 192, 256),
                stride_ratio=0.5,
                max_detections=100,
            )
            candidate = discovery_metrics(frames)
            classified = classify_detections(
                classifier,
                frames,
                device=device,
                class_threshold=threshold,
            )
            discrete = discrete_metrics(match_discrete_predictions(classified))
            outputs[name] = {
                "candidate": candidate,
                "discrete": discrete,
            }
        print(
            json.dumps(
                {
                    "threshold": threshold,
                    "val_recall": outputs["val"]["candidate"][
                        "all_gt_candidate_recall"
                    ],
                    "val_fp_per_min": outputs["val"]["candidate"][
                        "false_candidates_per_min"
                    ],
                    "val_neg_fp": outputs["val"]["candidate"][
                        "negative_only_fp_per_frame"
                    ],
                    "val_discrete_f1": outputs["val"]["discrete"]["macro_f1"],
                    "test_recall": outputs["test"]["candidate"][
                        "all_gt_candidate_recall"
                    ],
                    "test_fp_per_min": outputs["test"]["candidate"][
                        "false_candidates_per_min"
                    ],
                    "test_neg_fp": outputs["test"]["candidate"][
                        "negative_only_fp_per_frame"
                    ],
                    "test_discrete_f1": outputs["test"]["discrete"]["macro_f1"],
                },
                indent=2,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
