#!/usr/bin/env python3
"""Train ONLINE-X3 direct three-class FCOS without reading G5 or legacy D6."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import sys
import time

import numpy as np
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g4_data import (  # noqa: E402
    index_instance_records,
    load_frame_rows,
    load_instance_records,
)
from sanitation_learning.g4_direct_fcos import (  # noqa: E402
    DirectFCOSDataset,
    X3_ARCHITECTURE,
    build_direct_fcos,
    direct_fcos_collate,
    direct_predictions,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    discrete_metrics,
    match_discrete_predictions,
)
from sanitation_learning.g4_split_policy import stratified_row_sample  # noqa: E402
from sanitation_learning.g4_teacher import require_teacher_dataset_gate  # noqa: E402


SEED = 20260810
THRESHOLDS = tuple(round(value / 100, 2) for value in range(5, 96, 5))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def holdout_rows(rows, fraction):
    selected = []
    for row in rows:
        token = f"{row['world_id']}:{int(row['scene_seed'])}".encode()
        if hashlib.sha256(token).digest()[0] % 100 < int(fraction * 100):
            selected.append(row)
    return selected


def move_targets(targets, device):
    return [{key: value.to(device) for key, value in target.items()} for target in targets]


def ema_update(model, state, decay):
    current = model.state_dict()
    if state is None:
        return {key: value.detach().clone() for key, value in current.items()}
    with torch.no_grad():
        for key, value in current.items():
            if torch.is_floating_point(value):
                state[key].mul_(decay).add_(value.detach(), alpha=1.0 - decay)
            else:
                state[key].copy_(value.detach())
    return state


def threshold_sweep(model, rows, instances, device):
    raw = direct_predictions(
        model, rows, instances, device=device, score_threshold=0.01, batch_size=4
    )
    sweep = []
    for threshold in THRESHOLDS:
        filtered = []
        for frame in raw:
            items = [item for item in frame["predictions"] if item["score"] >= threshold]
            filtered.append({**frame, "predictions": items, "detections": items})
        metrics = discrete_metrics(match_discrete_predictions(filtered))
        gates = {
            "macro_precision_at_least_0_90": metrics["macro_precision"] >= 0.90,
            "macro_recall_at_least_0_90": metrics["macro_recall"] >= 0.90,
            "macro_f1_at_least_0_90": metrics["macro_f1"] >= 0.90,
            "false_candidates_per_min_at_most_2": metrics["false_candidates_per_min"] <= 2.0,
            "negative_fp_per_frame_at_most_0_05": metrics["negative_only_fp_per_frame"] <= 0.05,
            "paper_precision_at_least_0_80": metrics["paper_precision"] >= 0.80,
            "small_recall_at_least_0_70": metrics["small_object_recall"] >= 0.70,
        }
        distance = sum(
            max(0.0, target - metrics[name])
            for name, target in (
                ("macro_precision", 0.90),
                ("macro_recall", 0.90),
                ("macro_f1", 0.90),
                ("paper_precision", 0.80),
                ("small_object_recall", 0.70),
            )
        ) + max(0.0, metrics["false_candidates_per_min"] - 2.0)
        sweep.append(
            {
                "threshold": threshold,
                "metrics": metrics,
                "gates": gates,
                "all_pass": all(gates.values()),
                "constraint_distance": distance,
            }
        )
    selected = min(
        sweep,
        key=lambda item: (
            not item["all_pass"],
            item["constraint_distance"],
            -item["metrics"]["macro_f1"],
            -item["threshold"],
        ),
    )
    return selected, sweep


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--max-train-frames", type=int, default=600)
    parser.add_argument("--max-holdout-frames", type=int, default=100)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    qa = require_teacher_dataset_gate(args.evidence_dir)
    rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl",
        args.data_root,
        allowed_splits=("train", "val"),
    )
    train_all = [row for row in rows if row["split"] == "train"]
    holdout_raw = holdout_rows(train_all, 0.2)
    holdout_scenes = {(str(row["world_id"]), int(row["scene_seed"])) for row in holdout_raw}
    train_rows = [
        row for row in train_all
        if (str(row["world_id"]), int(row["scene_seed"])) not in holdout_scenes
    ]
    train_rows = stratified_row_sample(train_rows, args.max_train_frames, seed=SEED)
    holdout = stratified_row_sample(
        [{**row, "split": "train_world_holdout"} for row in holdout_raw],
        args.max_holdout_frames,
        seed=SEED + 1,
    )
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in rows}
    instances = index_instance_records(
        load_instance_records(
            args.evidence_dir / "g4_instance_records.jsonl", allowed_frame_keys=keys
        )
    )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal X3 training requires CUDA")
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.cuda.manual_seed_all(SEED)
    torch.use_deterministic_algorithms(True, warn_only=True)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    model = build_direct_fcos().to(device)
    loader = DataLoader(
        DirectFCOSDataset(train_rows, instances),
        batch_size=2,
        shuffle=True,
        num_workers=0,
        collate_fn=direct_fcos_collate,
        generator=torch.Generator().manual_seed(SEED),
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(args.epochs, 1), eta_min=5e-6
    )
    scaler = torch.amp.GradScaler("cuda", enabled=True)
    ema = None
    best = None
    best_state = None
    curves = []
    started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        model.train()
        loss_sum = 0.0
        steps = 0
        for images, targets, _ in loader:
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type="cuda", dtype=torch.float16, enabled=True):
                losses = model(
                    [image.to(device) for image in images], move_targets(targets, device)
                )
                total = sum(losses.values())
            scaler.scale(total).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            scaler.step(optimizer)
            scaler.update()
            ema = ema_update(model, ema, 0.999)
            loss_sum += float(total.detach().cpu())
            steps += 1
        scheduler.step()
        current = {key: value.detach().clone() for key, value in model.state_dict().items()}
        model.load_state_dict(ema)
        selected, sweep = threshold_sweep(model, holdout, instances, device)
        model.load_state_dict(current)
        curve = {
            "epoch": epoch,
            "loss": loss_sum / max(steps, 1),
            "selected": selected,
            "threshold_sweep": sweep,
        }
        curves.append(curve)
        if best is None or (
            not selected["all_pass"], selected["constraint_distance"], -selected["metrics"]["macro_f1"]
        ) < (
            not best["all_pass"], best["constraint_distance"], -best["metrics"]["macro_f1"]
        ):
            best = selected
            best_state = {key: value.detach().cpu().clone() for key, value in ema.items()}
        print(
            f"[X3] epoch={epoch} loss={curve['loss']:.4f} threshold={selected['threshold']:.2f} "
            f"f1={selected['metrics']['macro_f1']:.4f} fp_min={selected['metrics']['false_candidates_per_min']:.2f}",
            flush=True,
        )
    if best is None or best_state is None:
        raise RuntimeError("X3 training produced no checkpoint")
    checkpoint = args.output / "x3_fcos_r50_direct_3class.pt"
    torch.save(
        {
            "state_dict": best_state,
            "architecture": X3_ARCHITECTURE,
            "frozen_threshold_from_train_world_holdout": best["threshold"],
            "provenance": model.provenance,
            "checkpoint_status": "training_complete",
            "G5_SEALED_FINAL_read": False,
            "legacy_G4_D6_read": False,
        },
        checkpoint,
    )
    report = {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-02-X3-TRAIN",
        "route": "ONLINE-X3_TORCHVISION_FCOS_R50_DIRECT_3CLASS",
        "architecture": X3_ARCHITECTURE,
        "data_policy": {
            "train_frames": len(train_rows),
            "train_world_holdout_frames": len(holdout),
            "threshold_selection_split": "train_world_holdout",
            "VAL_read_for_training_or_selection": False,
            "G5_SEALED_FINAL_read": False,
            "legacy_G4_D6_read": False,
            "dataset_qa": qa,
        },
        "provenance": model.provenance,
        "training": {"duration_s": time.perf_counter() - started, "curves": curves},
        "selection": best,
        "checkpoint": {
            "path": checkpoint.name,
            "sha256": sha256(checkpoint),
            "bytes": checkpoint.stat().st_size,
        },
        "product_ready": False,
        "next_action": "run_full_VAL_D1_D5_static_gate",
    }
    (args.output / "X3_TRAIN_REPORT.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
