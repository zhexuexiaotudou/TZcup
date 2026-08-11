#!/usr/bin/env python3
"""Train the bounded OPRV3-06 G6 Area candidate on development data only."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import sys
import time

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g6_area_recovery import (  # noqa: E402
    AREA_CLASSES,
    AREA_SIZE,
    G6AreaDataset,
    G6AreaTaskExport,
    G6BoundaryAwareAreaNet,
    g6_area_loss,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_manifest(root: Path) -> list[dict]:
    path = root / "G6_FRAME_MANIFEST.jsonl"
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def semantic_boundary_tensor(mask: torch.Tensor) -> torch.Tensor:
    value = mask.float()
    eroded = 1.0 - torch.nn.functional.max_pool2d(1.0 - value, 3, 1, 1)
    return (value > eroded).float()


@torch.no_grad()
def evaluate(model, loader, device) -> dict:
    model.eval()
    intersection = torch.zeros(2, dtype=torch.float64, device=device)
    union = torch.zeros(2, dtype=torch.float64, device=device)
    edge_intersection = torch.zeros(2, dtype=torch.float64, device=device)
    edge_union = torch.zeros(2, dtype=torch.float64, device=device)
    negative_frames = 0
    negative_fp_frames = 0
    for batch in loader:
        features = batch["features"].to(device, non_blocking=True)
        targets = batch["targets"].to(device, non_blocking=True).bool()
        negative = batch["negative"].to(device, non_blocking=True).bool()
        output = model(features)
        predicted = torch.sigmoid(output["semantic_logits"]) >= 0.5
        intersection += (predicted & targets).sum(dim=(0, 2, 3))
        union += (predicted | targets).sum(dim=(0, 2, 3))
        predicted_edge = semantic_boundary_tensor(predicted)
        truth_edge = semantic_boundary_tensor(targets)
        edge_intersection += (predicted_edge.bool() & truth_edge.bool()).sum(
            dim=(0, 2, 3)
        )
        edge_union += (predicted_edge.bool() | truth_edge.bool()).sum(dim=(0, 2, 3))
        overlap = (predicted & negative).sum(dim=(1, 2, 3))
        negative_frames += int(features.shape[0])
        negative_fp_frames += int((overlap >= 32).sum().item())
    iou = (intersection / torch.clamp(union, min=1)).cpu().tolist()
    boundary = (
        2.0
        * edge_intersection
        / torch.clamp(edge_intersection + edge_union, min=1)
    ).cpu().tolist()
    return {
        "iou_by_class": dict(zip(AREA_CLASSES, iou)),
        "macro_miou": float(np.mean(iou)),
        "boundary_f1_by_class": dict(zip(AREA_CLASSES, boundary)),
        "boundary_f1": float(np.mean(boundary)),
        "negative_area_frames": negative_frames,
        "negative_area_fp_frames": negative_fp_frames,
        "negative_area_fp_per_frame": negative_fp_frames / max(negative_frames, 1),
    }


def atomic_torch_save(payload: dict, path: Path) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(payload, temporary)
    os.replace(temporary, path)


def export_onnx_and_check(model, output_dir: Path, device) -> dict:
    import onnx
    import onnxruntime as ort

    model.eval()
    generator = torch.Generator(device=device)
    generator.manual_seed(20260811)
    sample = torch.rand(
        (1, 10, AREA_SIZE[1], AREA_SIZE[0]), generator=generator, device=device
    )
    records = {}
    for task_index, task in enumerate(("leaf", "puddle")):
        wrapper = G6AreaTaskExport(model, task_index).eval()
        path = output_dir / f"{task}.onnx"
        torch.onnx.export(
            wrapper,
            sample,
            path,
            input_names=["area_input"],
            output_names=["area_output"],
            opset_version=17,
            dynamic_axes=None,
            do_constant_folding=True,
        )
        graph = onnx.load(str(path))
        custom_ops = sum(1 for node in graph.graph.node if node.domain not in ("", "ai.onnx"))
        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        torch_output = wrapper(sample).detach().cpu().numpy()
        onnx_output = session.run(None, {"area_input": sample.cpu().numpy()})[0]
        max_error = float(np.max(np.abs(torch_output - onnx_output)))
        torch_probability = 1.0 / (
            1.0 + np.exp(-np.clip(torch_output, -80.0, 80.0))
        )
        onnx_probability = 1.0 / (
            1.0 + np.exp(-np.clip(onnx_output, -80.0, 80.0))
        )
        probability_error = float(
            np.max(np.abs(torch_probability - onnx_probability))
        )
        semantic_agreement = float(
            np.mean(
                (torch_probability[:, 0] >= 0.5)
                == (onnx_probability[:, 0] >= 0.5)
            )
        )
        boundary_agreement = float(
            np.mean(
                (torch_probability[:, 1] >= 0.5)
                == (onnx_probability[:, 1] >= 0.5)
            )
        )
        parity_pass = (
            custom_ops == 0
            and probability_error <= 0.005
            and semantic_agreement >= 0.99999
            and boundary_agreement >= 0.99999
        )
        records[task] = {
            "path": path.name,
            "sha256": sha256(path),
            "bytes": path.stat().st_size,
            "opset": 17,
            "fixed_input_shape": [1, 10, AREA_SIZE[1], AREA_SIZE[0]],
            "custom_ops": custom_ops,
            "parity_max_logit_absolute_error": max_error,
            "parity_max_probability_absolute_error": probability_error,
            "parity_semantic_mask_agreement": semantic_agreement,
            "parity_boundary_mask_agreement": boundary_agreement,
            "parity_thresholds": {
                "probability_absolute_error_at_most": 0.005,
                "semantic_mask_agreement_at_least": 0.99999,
                "boundary_mask_agreement_at_least": 0.99999,
                "custom_ops": 0,
            },
            "parity_pass": parity_pass,
            "parity_provider": "CPUExecutionProvider",
        }
        if not records[task]["parity_pass"]:
            raise RuntimeError(f"{task} ONNX parity failed: {records[task]}")
    return records


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--base-channels", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=8e-4)
    parser.add_argument("--seed", type=int, default=20260811)
    args = parser.parse_args()
    started = time.perf_counter()
    if not torch.cuda.is_available():
        raise RuntimeError("OPRV3-06 formal training requires CUDA")
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    device = torch.device("cuda")
    args.output.mkdir(parents=True, exist_ok=False)

    rows = read_manifest(args.dataset)
    train_rows = [row for row in rows if row["split"] == "train"]
    holdout_world = "g6_train_world_10"
    fit_rows = [row for row in train_rows if row["world_id"] != holdout_world]
    holdout_rows = [row for row in train_rows if row["world_id"] == holdout_world]
    if not fit_rows or not holdout_rows:
        raise RuntimeError("G6 Area world-isolated fit/holdout split is empty")
    train_dataset = G6AreaDataset(args.dataset, fit_rows, augment=True, seed=args.seed)
    holdout_loader = DataLoader(
        G6AreaDataset(args.dataset, holdout_rows, augment=False, seed=args.seed),
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=True,
    )
    generator = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        generator=generator,
        num_workers=args.workers,
        pin_memory=True,
        drop_last=False,
    )
    model = G6BoundaryAwareAreaNet(args.base_channels).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.learning_rate, weight_decay=1e-4
    )
    scaler = torch.amp.GradScaler("cuda")
    checkpoint_path = args.output / "g6_area_shared.pt"
    history = []
    best_score = -float("inf")
    best_epoch = None
    for epoch in range(1, args.epochs + 1):
        model.train()
        train_dataset.seed = args.seed + epoch * 1000003
        total_loss = 0.0
        batches = 0
        for batch in train_loader:
            optimizer.zero_grad(set_to_none=True)
            features = batch["features"].to(device, non_blocking=True)
            targets = batch["targets"].to(device, non_blocking=True)
            boundaries = batch["boundaries"].to(device, non_blocking=True)
            negative = batch["negative"].to(device, non_blocking=True)
            with torch.amp.autocast("cuda", dtype=torch.float16):
                output = model(features)
                losses = g6_area_loss(output, targets, boundaries, negative)
            scaler.scale(losses["total"]).backward()
            scaler.step(optimizer)
            scaler.update()
            total_loss += float(losses["total"].detach().cpu())
            batches += 1
        metrics = evaluate(model, holdout_loader, device)
        score = (
            min(metrics["iou_by_class"].values())
            + 0.5 * metrics["boundary_f1"]
            - metrics["negative_area_fp_per_frame"]
        )
        record = {
            "epoch": epoch,
            "training_loss": total_loss / max(batches, 1),
            "selection_score": score,
            "holdout": metrics,
        }
        history.append(record)
        print(json.dumps(record, sort_keys=True), flush=True)
        if score > best_score:
            best_score = score
            best_epoch = epoch
            atomic_torch_save(
                {
                    "schema_version": 1,
                    "stage": "OPRV3-06-G6-AREA",
                    "checkpoint_status": "training_complete_candidate_not_frozen",
                    "model": "G6BoundaryAwareAreaNet",
                    "base_channels": args.base_channels,
                    "state_dict": model.state_dict(),
                    "selected_epoch": epoch,
                    "selection_score": score,
                    "G5_SEALED_FINAL_read": False,
                    "G5_V2_SEALED_FINAL_read": False,
                },
                checkpoint_path,
            )

    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model.load_state_dict(checkpoint["state_dict"])
    selected_metrics = evaluate(model, holdout_loader, device)
    onnx_records = export_onnx_and_check(model, args.output, device)
    report = {
        "schema_version": 1,
        "stage": "OPRV3-06-G6-AREA-TRAINING",
        "development_only": True,
        "G5_SEALED_FINAL_read": False,
        "G5_V2_SEALED_FINAL_read": False,
        "data": {
            "dataset": args.dataset.as_posix(),
            "fit_frames": len(fit_rows),
            "fit_worlds": sorted({row["world_id"] for row in fit_rows}),
            "selection_frames": len(holdout_rows),
            "selection_world": holdout_world,
            "taxonomy_balanced_hard_negatives": True,
        },
        "model": {
            "architecture": "shared_high_resolution_boundary_aware_encoder_decoder",
            "base_channels": args.base_channels,
            "input_shape": [1, 10, AREA_SIZE[1], AREA_SIZE[0]],
            "semantic_heads": list(AREA_CLASSES),
            "independent_boundary_heads": list(AREA_CLASSES),
            "checkpoint": checkpoint_path.name,
            "checkpoint_sha256": sha256(checkpoint_path),
            "checkpoint_status": "training_complete_candidate_not_frozen",
        },
        "training": {
            "epochs": args.epochs,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "seed": args.seed,
            "selected_epoch": best_epoch,
            "history": history,
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
