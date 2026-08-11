#!/usr/bin/env python3
"""Recover export/report after a completed G6 Area run failed post-training."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import time

import cv2
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g6_area_recovery import (  # noqa: E402
    AREA_CLASSES,
    AREA_SIZE,
    G6AreaDataset,
    G6BoundaryAwareAreaNet,
)
from train_g6_area_recovery import (  # noqa: E402
    evaluate,
    export_onnx_and_check,
    read_manifest,
    sha256,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--prior-export-failure", type=Path, action="append", default=[]
    )
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    started = time.perf_counter()
    if not torch.cuda.is_available():
        raise RuntimeError("G6 Area export recovery requires CUDA parity reference")
    args.output.mkdir(parents=True, exist_ok=False)
    source_checkpoint = args.source / "g6_area_shared.pt"
    source_onnx = args.source / "leaf.onnx"
    checkpoint = torch.load(source_checkpoint, map_location="cuda", weights_only=False)
    if checkpoint.get("stage") != "OPRV3-06-G6-AREA":
        raise RuntimeError("source is not an OPRV3-06 G6 Area checkpoint")
    if checkpoint.get("G5_SEALED_FINAL_read") is not False:
        raise RuntimeError("source checkpoint violates the sealed G5 boundary")
    destination_checkpoint = args.output / "g6_area_shared.pt"
    shutil.copy2(source_checkpoint, destination_checkpoint)
    model = G6BoundaryAwareAreaNet(checkpoint["base_channels"]).cuda().eval()
    model.load_state_dict(checkpoint["state_dict"])
    rows = read_manifest(args.dataset)
    holdout_rows = [
        row
        for row in rows
        if row["split"] == "train" and row["world_id"] == "g6_train_world_10"
    ]
    holdout_loader = DataLoader(
        G6AreaDataset(args.dataset, holdout_rows, augment=False),
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=True,
    )
    selected_metrics = evaluate(model, holdout_loader, torch.device("cuda"))
    onnx_records = export_onnx_and_check(model, args.output, torch.device("cuda"))
    report = {
        "schema_version": 1,
        "stage": "OPRV3-06-G6-AREA-TRAINING",
        "development_only": True,
        "G5_SEALED_FINAL_read": False,
        "G5_V2_SEALED_FINAL_read": False,
        "recovered_after_post_training_export_failure": True,
        "source_failure": {
            "directory": args.source.as_posix(),
            "completed_training_checkpoint": source_checkpoint.name,
            "checkpoint_sha256": sha256(source_checkpoint),
            "failure": "leaf ONNX raw-logit absolute-error-only gate exceeded 1e-4",
            "failed_leaf_onnx_sha256": sha256(source_onnx)
            if source_onnx.exists()
            else None,
            "training_history_persisted": False,
            "training_history_note": "epoch lines were emitted to stdout before the post-training export exception",
        },
        "prior_export_failures": [
            {
                "directory": path.as_posix(),
                "preserved": path.exists(),
                "leaf_onnx_sha256": sha256(path / "leaf.onnx")
                if (path / "leaf.onnx").exists()
                else None,
                "puddle_onnx_sha256": sha256(path / "puddle.onnx")
                if (path / "puddle.onnx").exists()
                else None,
            }
            for path in args.prior_export_failure
        ],
        "data": {
            "dataset": args.dataset.as_posix(),
            "fit_frames": sum(
                row["split"] == "train" and row["world_id"] != "g6_train_world_10"
                for row in rows
            ),
            "fit_worlds": sorted(
                {
                    row["world_id"]
                    for row in rows
                    if row["split"] == "train"
                    and row["world_id"] != "g6_train_world_10"
                }
            ),
            "selection_frames": len(holdout_rows),
            "selection_world": "g6_train_world_10",
            "taxonomy_balanced_hard_negatives": True,
        },
        "model": {
            "architecture": "shared_high_resolution_boundary_aware_encoder_decoder",
            "base_channels": checkpoint["base_channels"],
            "input_shape": [1, 10, AREA_SIZE[1], AREA_SIZE[0]],
            "semantic_heads": list(AREA_CLASSES),
            "independent_boundary_heads": list(AREA_CLASSES),
            "checkpoint": destination_checkpoint.name,
            "checkpoint_sha256": sha256(destination_checkpoint),
            "checkpoint_status": "training_complete_candidate_not_frozen",
        },
        "training": {
            "epochs_completed_in_source": 6,
            "selected_epoch": checkpoint["selected_epoch"],
            "selection_score": checkpoint["selection_score"],
            "additional_training_during_recovery": False,
        },
        "selected_holdout_metrics_at_0_5": selected_metrics,
        "onnx": onnx_records,
        "environment": {
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(0),
            "cv2": cv2.__version__,
        },
        "duration_s": time.perf_counter() - started,
    }
    report_path = args.output / "OPRV3_G6_AREA_TRAINING.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(report_path), "sha256": sha256(report_path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
