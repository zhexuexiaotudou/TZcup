#!/usr/bin/env python3
"""Train and gate the official FCOS ResNet50-FPN P2 reference teacher."""

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
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_data import (  # noqa: E402
    index_instance_records,
    load_frame_rows,
    load_instance_records,
)
from sanitation_learning.g4_teacher import (  # noqa: E402
    FCOSDiscoveryDataset,
    TEACHER_ARCHITECTURE,
    TEACHER_WEIGHT_SPEC,
    build_fcos_teacher,
    fcos_collate,
    filter_prediction_frames,
    require_teacher_dataset_gate,
    select_teacher_threshold,
    teacher_constraint_distance,
    teacher_gate,
    teacher_predictions,
)
from sanitation_learning.g4_split_policy import (  # noqa: E402
    stratified_row_sample,
)


SEED = 20260809


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _config_sha256(config: dict) -> str:
    payload = json.dumps(config, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _holdout_rows(rows: list[dict], fraction: float) -> list[dict]:
    if not 0.0 < fraction < 1.0:
        raise ValueError("holdout fraction must be in (0, 1)")
    return [
        row
        for row in rows
        if hashlib.sha256(
            f"{row['world_id']}:{int(row['scene_seed'])}".encode("utf-8")
        ).digest()[0]
        % 100
        < int(fraction * 100)
    ]


def _move_targets(targets, device):
    return [
        {key: value.to(device) for key, value in target.items()}
        for target in targets
    ]


def _ema_update(model, ema_state: dict | None, decay: float) -> dict:
    state = model.state_dict()
    if ema_state is None:
        return {key: value.detach().clone() for key, value in state.items()}
    with torch.no_grad():
        for key, value in state.items():
            if torch.is_floating_point(value):
                ema_state[key].mul_(decay).add_(
                    value.detach(), alpha=1.0 - decay
                )
            else:
                ema_state[key].copy_(value.detach())
    return ema_state


def _apply_state_temporarily(model, state: dict):
    current = {
        key: value.detach().clone() for key, value in model.state_dict().items()
    }
    model.load_state_dict(state)
    return current


def _selection_key(item: dict) -> tuple:
    metrics = item["metrics"]
    return (
        not item["all_pass"],
        item["constraint_distance"],
        -metrics["all_gt_candidate_recall"],
        metrics["false_candidates_per_min"],
        -item["threshold"],
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--eval-batch-size", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--ema-decay", type=float, default=0.999)
    parser.add_argument("--early-stopping-patience", type=int, default=4)
    parser.add_argument("--holdout-fraction", type=float, default=0.2)
    parser.add_argument("--max-train-frames", type=int, default=600)
    parser.add_argument("--max-eval-frames", type=int, default=100)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    dataset_qa = require_teacher_dataset_gate(args.evidence_dir)

    config = {
        "architecture": TEACHER_ARCHITECTURE,
        "weight_spec": TEACHER_WEIGHT_SPEC,
        "seed": SEED,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "eval_batch_size": args.eval_batch_size,
        "learning_rate": args.learning_rate,
        "weight_decay": args.weight_decay,
        "ema_decay": args.ema_decay,
        "early_stopping_patience": args.early_stopping_patience,
        "holdout_fraction": args.holdout_fraction,
        "max_train_frames": args.max_train_frames,
        "max_eval_frames": args.max_eval_frames,
        "amp": True,
        "gradient_clip_norm": 5.0,
        "dataset_qa_sha256": dataset_qa["sha256"],
    }
    rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl",
        args.data_root,
        allowed_splits=("train", "val"),
    )
    train_all = [row for row in rows if row["split"] == "train"]
    val_rows = [row for row in rows if row["split"] == "val"]
    holdout_raw = _holdout_rows(train_all, args.holdout_fraction)
    holdout_scenes = {
        (str(row["world_id"]), int(row["scene_seed"]))
        for row in holdout_raw
    }
    train_rows = [
        row
        for row in train_all
        if (str(row["world_id"]), int(row["scene_seed"]))
        not in holdout_scenes
    ]
    holdout_rows = [
        {**row, "split": "train_world_holdout"} for row in holdout_raw
    ]
    train_rows = stratified_row_sample(
        train_rows, args.max_train_frames, seed=SEED
    )
    holdout_rows = stratified_row_sample(
        holdout_rows, args.max_eval_frames, seed=SEED + 1
    )
    val_rows = stratified_row_sample(
        val_rows, args.max_eval_frames, seed=SEED + 2
    )
    if not train_rows or not holdout_rows or not val_rows:
        raise RuntimeError(
            "P2 teacher requires non-empty train/holdout/val rows"
        )
    allowed_keys = {
        (int(row["scene_seed"]), int(row["frame_index"])) for row in rows
    }
    records = load_instance_records(
        args.evidence_dir / "g4_instance_records.jsonl",
        allowed_frame_keys=allowed_keys,
    )
    instances_by_key = index_instance_records(records)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal P2 teacher training requires CUDA")
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.cuda.manual_seed_all(SEED)
    torch.use_deterministic_algorithms(True, warn_only=True)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    model = build_fcos_teacher().to(device)
    dataset = FCOSDiscoveryDataset(train_rows, instances_by_key)
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=0,
        collate_fn=fcos_collate,
        generator=torch.Generator().manual_seed(SEED),
    )
    optimizer = torch.optim.AdamW(
        [parameter for parameter in model.parameters() if parameter.requires_grad],
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer,
        T_max=max(args.epochs, 1),
        eta_min=args.learning_rate * 0.05,
    )
    scaler = torch.amp.GradScaler("cuda", enabled=True)
    ema_state = None
    best_state = None
    best_selection = None
    best_epoch = None
    curves = []
    epochs_without_improvement = 0
    started = time.perf_counter()
    print(
        f"[P2 teacher] train={len(train_rows)} holdout={len(holdout_rows)} "
        f"val={len(val_rows)} device={device} legacy_test=not_read",
        flush=True,
    )

    for epoch in range(1, args.epochs + 1):
        model.train()
        loss_totals: dict[str, float] = {}
        steps = 0
        for images, targets, _batch_rows in loader:
            images = [image.to(device) for image in images]
            targets = _move_targets(targets, device)
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(
                device_type="cuda", dtype=torch.float16, enabled=True
            ):
                losses = model(images, targets)
                total = sum(losses.values())
            scaler.scale(total).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            scaler.step(optimizer)
            scaler.update()
            ema_state = _ema_update(model, ema_state, args.ema_decay)
            loss_totals["total"] = loss_totals.get("total", 0.0) + float(
                total.detach().cpu()
            )
            for name, value in losses.items():
                loss_totals[name] = loss_totals.get(name, 0.0) + float(
                    value.detach().cpu()
                )
            steps += 1
        scheduler.step()
        if ema_state is None:
            raise RuntimeError("teacher EMA state was not initialized")
        current_state = _apply_state_temporarily(model, ema_state)
        try:
            holdout_frames = teacher_predictions(
                model,
                holdout_rows,
                instances_by_key,
                device=device,
                score_threshold=0.01,
                batch_size=args.eval_batch_size,
            )
            threshold, selection, sweep = select_teacher_threshold(
                holdout_frames
            )
        finally:
            model.load_state_dict(current_state)
            del current_state
        curve = {
            "epoch": epoch,
            "learning_rate": optimizer.param_groups[0]["lr"],
            "training_losses": {
                name: value / max(steps, 1)
                for name, value in loss_totals.items()
            },
            "holdout_selected_threshold": threshold,
            "holdout_selection": selection,
            "holdout_threshold_sweep": sweep,
        }
        curves.append(curve)
        improved = (
            best_selection is None
            or _selection_key(selection) < _selection_key(best_selection)
        )
        if improved:
            best_selection = selection
            best_epoch = epoch
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in ema_state.items()
            }
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1
        print(
            f"[P2 teacher] epoch={epoch} loss={curve['training_losses']['total']:.4f} "
            f"threshold={threshold:.2f} "
            f"recall={selection['metrics']['all_gt_candidate_recall']:.4f} "
            f"fp_min={selection['metrics']['false_candidates_per_min']:.2f} "
            f"pass={selection['all_pass']}",
            flush=True,
        )
        if (
            args.early_stopping_patience > 0
            and epochs_without_improvement >= args.early_stopping_patience
        ):
            break

    if best_state is None or best_selection is None or best_epoch is None:
        raise RuntimeError("teacher training produced no selectable checkpoint")
    model.load_state_dict(best_state)
    frozen_threshold = float(best_selection["threshold"])
    val_raw_frames = teacher_predictions(
        model,
        val_rows,
        instances_by_key,
        device=device,
        score_threshold=0.01,
        batch_size=args.eval_batch_size,
    )
    from sanitation_learning.g4_evaluation import discovery_metrics

    val_metrics = discovery_metrics(
        filter_prediction_frames(val_raw_frames, frozen_threshold)
    )
    val_gate = teacher_gate(val_metrics)
    checkpoint_path = args.output / "fcos_resnet50_fpn_teacher.pt"
    checkpoint_payload = {
        "state_dict": best_state,
        "best_epoch": best_epoch,
        "best_holdout_selection": best_selection,
        "frozen_threshold": frozen_threshold,
        "config": config,
        "config_sha256": _config_sha256(config),
        "provenance": model.provenance,
        "architecture_role": model.architecture_role,
        "product_deployable_default": False,
    }
    torch.save(checkpoint_payload, checkpoint_path)
    report = {
        "schema_version": 1,
        "stage": "PERCEPTION-P2_TEACHER",
        "architecture": TEACHER_ARCHITECTURE,
        "architecture_role": model.architecture_role,
        "teacher_purpose": "data_learnability_reference_and_distillation_source",
        "product_deployable_default": False,
        "data_policy": {
            "read_splits": ["train", "val"],
            "derived_split": "train_world_holdout",
            "legacy_G4_D6_diagnostic_read": False,
            "G5_SEALED_FINAL_read": False,
            "sampling": "deterministic_world_x_polarity_round_robin",
            "train_frames": len(train_rows),
            "holdout_frames": len(holdout_rows),
            "val_frames": len(val_rows),
            "dataset_qa": dataset_qa,
        },
        "config": config,
        "config_sha256": _config_sha256(config),
        "provenance": model.provenance,
        "training": {
            "epochs_completed": len(curves),
            "best_epoch": best_epoch,
            "early_stopped": len(curves) < args.epochs,
            "duration_s": time.perf_counter() - started,
            "curves": curves,
        },
        "frozen_threshold_from_train_world_holdout": frozen_threshold,
        "best_holdout_selection": best_selection,
        "cross_world_val_metrics": val_metrics,
        "cross_world_val_gate": val_gate,
        "teacher_data_learnability_pass": val_gate["all_pass"],
        "next_action": (
            "train_FCOS_lite_students"
            if val_gate["all_pass"]
            else "return_to_data_annotation_camera_scale"
        ),
        "checkpoint": {
            "path": checkpoint_path.name,
            "sha256": _sha256(checkpoint_path),
            "bytes": checkpoint_path.stat().st_size,
        },
    }
    report_path = args.output / "P2_TEACHER_REPORT.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "report": str(report_path),
                "best_epoch": best_epoch,
                "threshold": frozen_threshold,
                "val_metrics": val_metrics,
                "teacher_data_learnability_pass": val_gate["all_pass"],
                "next_action": report["next_action"],
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
