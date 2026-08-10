#!/usr/bin/env python3
"""Train the final MRV2-C P2 detector from TRAIN-only teacher geometry."""

from __future__ import annotations

import argparse
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
sys.path.insert(0, str(ROOT / "scripts"))

from perception_mrv2_a_train import (  # noqa: E402
    MRV2AugmentedDataset,
    SEED,
    build_training_catalogs,
    ema_update,
    holdout_rows,
    move_targets,
    sha256,
    threshold_sweep,
)
from sanitation_learning.g4_data import (  # noqa: E402
    discrete_boxes_for_frame,
    index_instance_records,
    load_frame_rows,
    load_instance_records,
    read_rgb,
)
from sanitation_learning.g4_direct_fcos import (  # noqa: E402
    MRV2_C_P2_ARCHITECTURE,
    build_p2_direct_fcos,
    direct_fcos_collate,
    load_direct_state_into_p2,
)
from sanitation_learning.g4_split_policy import stratified_row_sample  # noqa: E402
from sanitation_learning.g4_teacher import (  # noqa: E402
    build_fcos_teacher,
    require_teacher_dataset_gate,
    teacher_predictions,
)
from sanitation_learning.mrv2_sampling import build_mrv2_epoch_rows, row_key  # noqa: E402
from sanitation_learning.mrv2_teacher import select_small_teacher_pseudo_labels  # noqa: E402


INPUT_SIZE = (960, 720)


def load_teacher(report_path: Path, device):
    report = json.loads(report_path.read_text(encoding="utf-8"))
    checkpoint_path = report_path.parent / report["checkpoint"]["path"]
    if sha256(checkpoint_path) != report["checkpoint"]["sha256"]:
        raise RuntimeError("MRV2-C teacher checkpoint SHA-256 mismatch")
    input_scale = int(report["config"]["input_scale"])
    model = build_fcos_teacher(input_scale=input_scale).to(device)
    payload = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model.load_state_dict(payload["state_dict"], strict=True)
    model.eval()
    return model, report, checkpoint_path, input_scale


def native_truth_catalog(rows, instances):
    output = {}
    for row in rows:
        rgb = read_rgb(row)
        native_size = (int(rgb.shape[1]), int(rgb.shape[0]))
        output[row_key(row)] = discrete_boxes_for_frame(
            row, instances, native_size=native_size, model_size=native_size
        )
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--mrv2-a-checkpoint", type=Path, required=True)
    parser.add_argument("--teacher-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--epoch-frames", type=int, default=600)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    qa = require_teacher_dataset_gate(args.evidence_dir)
    all_rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl",
        args.data_root,
        allowed_splits=("train", "val"),
    )
    train_all = [row for row in all_rows if row["split"] == "train"]
    holdout_raw = holdout_rows(train_all)
    holdout_scenes = {
        (str(row["world_id"]), int(row["scene_seed"])) for row in holdout_raw
    }
    train_pool = [
        row for row in train_all
        if (str(row["world_id"]), int(row["scene_seed"])) not in holdout_scenes
    ]
    holdout = stratified_row_sample(
        [{**row, "split": "train_world_holdout"} for row in holdout_raw],
        100,
        seed=SEED + 1,
    )
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in all_rows}
    instances = index_instance_records(
        load_instance_records(
            args.evidence_dir / "g4_instance_records.jsonl", allowed_frame_keys=keys
        )
    )
    small_keys, metal_keys, donors = build_training_catalogs(train_pool, instances)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal MRV2-C training requires CUDA")
    random.seed(SEED + 300)
    np.random.seed(SEED + 300)
    torch.manual_seed(SEED + 300)
    torch.cuda.manual_seed_all(SEED + 300)
    torch.use_deterministic_algorithms(True, warn_only=True)

    teacher, teacher_report, teacher_checkpoint, input_scale = load_teacher(
        args.teacher_report, device
    )
    teacher_threshold = float(
        teacher_report["frozen_threshold_from_train_world_holdout"]
    )
    teacher_frames = teacher_predictions(
        teacher,
        train_pool,
        instances,
        device=device,
        score_threshold=teacher_threshold,
        batch_size=4,
        input_scale=input_scale,
    )
    pseudo_by_key, pseudo_report = select_small_teacher_pseudo_labels(
        teacher_frames,
        native_truth_catalog(train_pool, instances),
        score_threshold=teacher_threshold,
    )
    if pseudo_report["pseudo_label_count"] == 0:
        raise RuntimeError("MRV2-C teacher produced no eligible TRAIN-only pseudo labels")
    del teacher, teacher_frames
    torch.cuda.empty_cache()

    parent = torch.load(args.mrv2_a_checkpoint, map_location=device, weights_only=False)
    model = build_p2_direct_fcos(input_size=INPUT_SIZE).to(device)
    transplant = load_direct_state_into_p2(model, parent["state_dict"])
    optimizer = torch.optim.AdamW(model.parameters(), lr=4e-5, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(args.epochs, 1), eta_min=2e-6
    )
    scaler = torch.amp.GradScaler("cuda", enabled=True)
    ema = None
    best = None
    best_state = None
    curves = []
    started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        epoch_rows, sampling = build_mrv2_epoch_rows(
            train_pool,
            small_keys=small_keys,
            metal_keys=metal_keys,
            frame_count=args.epoch_frames,
            seed=SEED + 30_000 + epoch,
        )
        dataset = MRV2AugmentedDataset(
            epoch_rows,
            instances,
            input_size=INPUT_SIZE,
            donors=donors,
            seed=SEED + 300_000_019 + epoch * 10_000_019,
            teacher_pseudo_by_key=pseudo_by_key,
        )
        loader = DataLoader(
            dataset,
            batch_size=1,
            shuffle=False,
            num_workers=0,
            collate_fn=direct_fcos_collate,
        )
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
            ema = ema_update(model, ema)
            loss_sum += float(total.detach().cpu())
            steps += 1
        scheduler.step()
        current = {
            key: value.detach().clone() for key, value in model.state_dict().items()
        }
        model.load_state_dict(ema)
        selected, sweep = threshold_sweep(
            model, holdout, instances, device, INPUT_SIZE
        )
        model.load_state_dict(current)
        curve = {
            "epoch": epoch,
            "loss": loss_sum / max(steps, 1),
            "sampling": sampling,
            "selected": selected,
            "threshold_sweep": sweep,
        }
        curves.append(curve)
        rank = (
            not selected["all_pass"],
            selected["constraint_distance"],
            -selected["metrics"]["macro_f1"],
        )
        if best is None or rank < best[0]:
            best = (rank, selected)
            best_state = {
                key: value.detach().cpu().clone() for key, value in ema.items()
            }
        print(
            f"[MRV2-C P2] epoch={epoch} loss={curve['loss']:.4f} "
            f"threshold={selected['threshold']:.2f} "
            f"small={selected['metrics']['small_object_recall']:.4f} "
            f"metal={selected['metal_can_recall']:.4f} "
            f"f1={selected['metrics']['macro_f1']:.4f}",
            flush=True,
        )

    selected = best[1]
    checkpoint = args.output / "mrv2_c_p2_r960.pt"
    torch.save(
        {
            "state_dict": best_state,
            "architecture": MRV2_C_P2_ARCHITECTURE,
            "route": "MRV2-C",
            "input_size": INPUT_SIZE,
            "frozen_threshold_from_train_world_holdout": selected["threshold"],
            "checkpoint_status": "training_complete_candidate_not_frozen",
            "parent_mrv2_a_sha256": sha256(args.mrv2_a_checkpoint),
            "teacher_checkpoint_sha256": sha256(teacher_checkpoint),
            "G5_SEALED_FINAL_read": False,
            "legacy_G4_D6_read": False,
        },
        checkpoint,
    )
    report = {
        "schema_version": 1,
        "stage": "MRV2-C-TRAIN",
        "route": "MRV2-C",
        "architecture": MRV2_C_P2_ARCHITECTURE,
        "input_size": INPUT_SIZE,
        "data_policy": {
            "teacher_inference_splits": ["TRAIN"],
            "pseudo_labels_used_for": ["TRAIN"],
            "VAL_read_for_training_or_selection": False,
            "G5_SEALED_FINAL_read": False,
            "legacy_G4_D6_read": False,
            "dataset_qa": qa,
        },
        "teacher": {
            "report": args.teacher_report.as_posix(),
            "report_sha256": sha256(args.teacher_report),
            "checkpoint": teacher_checkpoint.as_posix(),
            "checkpoint_sha256": sha256(teacher_checkpoint),
            "pseudo_labels": pseudo_report,
        },
        "parent_mrv2_a": {
            "path": args.mrv2_a_checkpoint.as_posix(),
            "sha256": sha256(args.mrv2_a_checkpoint),
            "transplant": transplant,
        },
        "training": {
            "epochs": args.epochs,
            "epoch_frames": args.epoch_frames,
            "duration_s": time.perf_counter() - started,
            "curves": curves,
        },
        "selection": selected,
        "checkpoint": {
            "path": checkpoint.name,
            "bytes": checkpoint.stat().st_size,
            "sha256": sha256(checkpoint),
        },
        "product_ready": False,
        "next_action": "run fixed VAL and D1-D5 MRV2-C static gate",
    }
    (args.output / "MRV2_C_TRAIN_REPORT.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
