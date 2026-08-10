#!/usr/bin/env python3
"""Train the bounded MRV2-A small-object and metal-can recovery candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import sys
import time

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g4_data import (  # noqa: E402
    DISCRETE_NAMES,
    discrete_boxes_for_frame,
    index_instance_records,
    load_frame_rows,
    load_instance_records,
    read_rgb,
)
from sanitation_learning.g4_direct_fcos import (  # noqa: E402
    X3_ARCHITECTURE,
    build_direct_fcos,
    direct_fcos_collate,
    direct_predictions,
)
from sanitation_learning.g4_evaluation import discrete_metrics, match_discrete_predictions  # noqa: E402
from sanitation_learning.g4_teacher import require_teacher_dataset_gate  # noqa: E402
from sanitation_learning.mrv2_sampling import build_mrv2_epoch_rows, row_key  # noqa: E402
from sanitation_learning.g4_split_policy import stratified_row_sample  # noqa: E402


SEED = 20260810
THRESHOLDS = tuple(round(value / 100.0, 2) for value in range(5, 96, 5))
RESOLUTIONS = {640: (640, 480), 960: (960, 720), 1280: (1280, 960)}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def holdout_rows(rows, fraction=0.2):
    return [
        row for row in rows
        if hashlib.sha256(f"{row['world_id']}:{int(row['scene_seed'])}".encode()).digest()[0] % 100
        < int(fraction * 100)
    ]


def build_training_catalogs(rows: list[dict], instances_by_key: dict) -> tuple[set, set, list]:
    small_keys = set()
    metal_keys = set()
    row_lookup = {row_key(row): row for row in rows}
    donors = []
    for key, records in instances_by_key.items():
        matching_rows = [value for value in row_lookup if value[1:] == key]
        if not matching_rows:
            continue
        full_key = matching_rows[0]
        for record in records:
            class_name = str(record.get("semantic_class"))
            if class_name == "metal_can":
                metal_keys.add(full_key)
            if class_name in DISCRETE_NAMES and float(record.get("bbox_shortest_side_px", 0.0)) < 18:
                small_keys.add(full_key)
                donors.append((row_lookup[full_key], record))
    return small_keys, metal_keys, donors


def _clip_boxes(boxes: list[dict], width: int, height: int) -> list[dict]:
    output = []
    for item in boxes:
        x1, y1, x2, y2 = (float(value) for value in item["bbox_xyxy"])
        x1, y1 = max(0.0, x1), max(0.0, y1)
        x2, y2 = min(float(width), x2), min(float(height), y2)
        if x2 - x1 >= 2 and y2 - y1 >= 2:
            output.append({**item, "bbox_xyxy": [x1, y1, x2, y2]})
    return output


class MRV2AugmentedDataset:
    """TRAIN-only crop-scale, copy-paste and metal photometric augmentation."""

    def __init__(self, rows, instances_by_key, *, input_size, donors, seed):
        self.rows = list(rows)
        self.instances_by_key = instances_by_key
        self.input_size = tuple(input_size)
        self.donors = list(donors)
        self.seed = int(seed)

    def __len__(self):
        return len(self.rows)

    def _native_truth(self, row, native_size):
        return discrete_boxes_for_frame(
            row, self.instances_by_key, native_size=native_size, model_size=native_size
        )

    def _target_crop(self, rgb, boxes, rng, bucket):
        height, width = rgb.shape[:2]
        if bucket != "small_object" or not boxes:
            return rgb, boxes
        small = [item for item in boxes if min(
            item["bbox_xyxy"][2] - item["bbox_xyxy"][0],
            item["bbox_xyxy"][3] - item["bbox_xyxy"][1],
        ) < 18]
        if not small:
            return rgb, boxes
        target = rng.choice(small)
        scale = rng.uniform(0.60, 0.90)
        crop_width, crop_height = int(width * scale), int(height * scale)
        x1, y1, x2, y2 = target["bbox_xyxy"]
        center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
        left = int(np.clip(center_x - crop_width * rng.uniform(0.35, 0.65), 0, width - crop_width))
        top = int(np.clip(center_y - crop_height * rng.uniform(0.35, 0.65), 0, height - crop_height))
        cropped = rgb[top : top + crop_height, left : left + crop_width]
        transformed = []
        for item in boxes:
            bx1, by1, bx2, by2 = item["bbox_xyxy"]
            center_inside = left <= (bx1 + bx2) / 2 <= left + crop_width and top <= (by1 + by2) / 2 <= top + crop_height
            if center_inside:
                transformed.append({**item, "bbox_xyxy": [bx1 - left, by1 - top, bx2 - left, by2 - top]})
        return cropped, _clip_boxes(transformed, crop_width, crop_height)

    def _copy_paste_small(self, rgb, boxes, rng):
        if not self.donors or rng.random() >= 0.40:
            return rgb, boxes, False
        donor_row, record = rng.choice(self.donors)
        donor_rgb = read_rgb(donor_row)
        instance = np.load(donor_row["instance_path"], allow_pickle=False)
        instance_id = int(record["instance_id"])
        mask = instance == instance_id
        ys, xs = np.where(mask)
        if not len(xs):
            return rgb, boxes, False
        x1, x2, y1, y2 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
        patch, alpha = donor_rgb[y1:y2, x1:x2], mask[y1:y2, x1:x2]
        scale = rng.uniform(0.85, 1.35)
        new_width = max(3, int(patch.shape[1] * scale))
        new_height = max(3, int(patch.shape[0] * scale))
        patch = cv2.resize(patch, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        alpha = cv2.resize(alpha.astype(np.uint8), (new_width, new_height), interpolation=cv2.INTER_NEAREST).astype(bool)
        height, width = rgb.shape[:2]
        if new_width >= width or new_height >= height:
            return rgb, boxes, False
        paste_x = rng.randint(0, width - new_width)
        paste_y = rng.randint(max(0, height // 2), height - new_height)
        result = rgb.copy()
        region = result[paste_y : paste_y + new_height, paste_x : paste_x + new_width]
        region[alpha] = patch[alpha]
        result[paste_y : paste_y + new_height, paste_x : paste_x + new_width] = region
        class_name = str(record["semantic_class"])
        boxes = [*boxes, {"semantic_class": class_name, "bbox_xyxy": [paste_x, paste_y, paste_x + new_width, paste_y + new_height]}]
        return result, boxes, True

    def _metal_photometric(self, rgb, boxes, rng, bucket):
        if bucket not in ("metal_can", "small_object") or not any(item["semantic_class"] == "metal_can" for item in boxes):
            return rgb
        image = rgb.astype(np.float32)
        contrast = rng.uniform(0.70, 1.30)
        brightness = rng.uniform(-25.0, 25.0)
        image = np.clip((image - 127.5) * contrast + 127.5 + brightness, 0, 255)
        for item in boxes:
            if item["semantic_class"] != "metal_can" or rng.random() >= 0.50:
                continue
            x1, y1, x2, y2 = (int(value) for value in item["bbox_xyxy"])
            if x2 > x1 and y2 > y1:
                overlay = image[y1:y2, x1:x2]
                stripe = max(1, (x2 - x1) // 5)
                overlay[:, :stripe] = np.clip(overlay[:, :stripe] * 0.35 + 160, 0, 255)
        return image.astype(np.uint8)

    def __getitem__(self, index):
        row = self.rows[index]
        rng = random.Random(self.seed + index * 1_000_003)
        rgb = read_rgb(row)
        native_size = (int(rgb.shape[1]), int(rgb.shape[0]))
        boxes = self._native_truth(row, native_size)
        bucket = str(row["mrv2_sampling_bucket"])
        rgb, boxes = self._target_crop(rgb, boxes, rng, bucket)
        rgb, boxes, _ = self._copy_paste_small(rgb, boxes, rng) if bucket == "small_object" else (rgb, boxes, False)
        rgb = self._metal_photometric(rgb, boxes, rng, bucket)
        if rng.random() < 0.5:
            rgb = np.ascontiguousarray(rgb[:, ::-1])
            width = rgb.shape[1]
            boxes = [{**item, "bbox_xyxy": [width - item["bbox_xyxy"][2], item["bbox_xyxy"][1], width - item["bbox_xyxy"][0], item["bbox_xyxy"][3]]} for item in boxes]
        source_width, source_height = rgb.shape[1], rgb.shape[0]
        resized = cv2.resize(rgb, self.input_size, interpolation=cv2.INTER_CUBIC)
        sx, sy = self.input_size[0] / source_width, self.input_size[1] / source_height
        scaled_boxes = [[item["bbox_xyxy"][0] * sx, item["bbox_xyxy"][1] * sy, item["bbox_xyxy"][2] * sx, item["bbox_xyxy"][3] * sy] for item in boxes]
        image = torch.from_numpy(np.ascontiguousarray(resized.transpose(2, 0, 1), dtype=np.float32) / 255.0)
        target = {
            "boxes": torch.as_tensor(scaled_boxes, dtype=torch.float32).reshape(-1, 4),
            "labels": torch.as_tensor([DISCRETE_NAMES.index(item["semantic_class"]) for item in boxes], dtype=torch.int64),
        }
        return image, target, row


def move_targets(targets, device):
    return [{key: value.to(device) for key, value in item.items()} for item in targets]


def ema_update(model, state, decay=0.999):
    current = model.state_dict()
    if state is None:
        return {key: value.detach().clone() for key, value in current.items()}
    with torch.no_grad():
        for key, value in current.items():
            if torch.is_floating_point(value):
                state[key].mul_(decay).add_(value.detach(), alpha=1 - decay)
            else:
                state[key].copy_(value.detach())
    return state


def threshold_sweep(model, rows, instances, device, input_size):
    raw = direct_predictions(model, rows, instances, device=device, score_threshold=0.01, batch_size=2, input_size=input_size, top_k=100)
    sweep = []
    for threshold in THRESHOLDS:
        filtered = []
        for frame in raw:
            items = [item for item in frame["predictions"] if item["score"] >= threshold][:16]
            filtered.append({**frame, "predictions": items, "detections": items})
        metrics = discrete_metrics(match_discrete_predictions(filtered))
        metal_recall = metrics["per_class"]["metal_can"]["recall"]
        gates = {
            "macro_precision_at_least_0_90": metrics["macro_precision"] >= 0.90,
            "macro_recall_at_least_0_90": metrics["macro_recall"] >= 0.90,
            "small_recall_at_least_0_70": metrics["small_object_recall"] >= 0.70,
            "metal_can_recall_at_least_0_90": metal_recall >= 0.90,
            "false_candidates_per_min_at_most_2": metrics["false_candidates_per_min"] <= 2.0,
            "negative_fp_per_frame_at_most_0_05": metrics["negative_only_fp_per_frame"] <= 0.05,
        }
        deficits = (
            max(0.0, 0.90 - metrics["macro_precision"])
            + max(0.0, 0.90 - metrics["macro_recall"])
            + max(0.0, 0.70 - metrics["small_object_recall"])
            + max(0.0, 0.90 - metal_recall)
            + max(0.0, metrics["false_candidates_per_min"] - 2.0)
            + max(0.0, metrics["negative_only_fp_per_frame"] - 0.05)
        )
        sweep.append({"threshold": threshold, "metrics": metrics, "metal_can_recall": metal_recall, "gates": gates, "all_pass": all(gates.values()), "constraint_distance": deficits})
    selected = min(sweep, key=lambda item: (not item["all_pass"], item["constraint_distance"], -item["metrics"]["macro_f1"], -item["threshold"]))
    return selected, sweep


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--x3-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=int, choices=sorted(RESOLUTIONS), required=True)
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--epoch-frames", type=int, default=600)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    qa = require_teacher_dataset_gate(args.evidence_dir)
    all_rows = load_frame_rows(args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root, allowed_splits=("train", "val"))
    train_all = [row for row in all_rows if row["split"] == "train"]
    holdout_raw = holdout_rows(train_all)
    holdout_scenes = {(str(row["world_id"]), int(row["scene_seed"])) for row in holdout_raw}
    train_pool = [row for row in train_all if (str(row["world_id"]), int(row["scene_seed"])) not in holdout_scenes]
    holdout = stratified_row_sample(
        [{**row, "split": "train_world_holdout"} for row in holdout_raw],
        100,
        seed=SEED + 1,
    )
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in all_rows}
    instances = index_instance_records(load_instance_records(args.evidence_dir / "g4_instance_records.jsonl", allowed_frame_keys=keys))
    small_keys, metal_keys, donors = build_training_catalogs(train_pool, instances)
    input_size = RESOLUTIONS[args.resolution]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal MRV2-A training requires CUDA")
    random.seed(SEED); np.random.seed(SEED); torch.manual_seed(SEED); torch.cuda.manual_seed_all(SEED)
    torch.use_deterministic_algorithms(True, warn_only=True)
    model = build_direct_fcos(input_size=input_size).to(device)
    x3 = torch.load(args.x3_checkpoint, map_location=device, weights_only=False)
    model.load_state_dict(x3["state_dict"], strict=True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=5e-5, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(args.epochs, 1), eta_min=3e-6)
    scaler = torch.amp.GradScaler("cuda", enabled=True)
    ema = None; best = None; best_state = None; curves = []; started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        epoch_rows, sampling = build_mrv2_epoch_rows(train_pool, small_keys=small_keys, metal_keys=metal_keys, frame_count=args.epoch_frames, seed=SEED + epoch)
        dataset = MRV2AugmentedDataset(epoch_rows, instances, input_size=input_size, donors=donors, seed=SEED + epoch * 10_000_019)
        loader = DataLoader(dataset, batch_size=2 if args.resolution == 640 else 1, shuffle=False, num_workers=0, collate_fn=direct_fcos_collate)
        model.train(); loss_sum = 0.0; steps = 0
        for images, targets, _ in loader:
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type="cuda", dtype=torch.float16, enabled=True):
                losses = model([image.to(device) for image in images], move_targets(targets, device))
                total = sum(losses.values())
            scaler.scale(total).backward(); scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            scaler.step(optimizer); scaler.update(); ema = ema_update(model, ema)
            loss_sum += float(total.detach().cpu()); steps += 1
        scheduler.step()
        current = {key: value.detach().clone() for key, value in model.state_dict().items()}
        model.load_state_dict(ema); selected, sweep = threshold_sweep(model, holdout, instances, device, input_size); model.load_state_dict(current)
        curve = {"epoch": epoch, "loss": loss_sum / max(steps, 1), "sampling": sampling, "selected": selected, "threshold_sweep": sweep}
        curves.append(curve)
        rank = (not selected["all_pass"], selected["constraint_distance"], -selected["metrics"]["macro_f1"])
        if best is None or rank < best[0]:
            best = (rank, selected); best_state = {key: value.detach().cpu().clone() for key, value in ema.items()}
        print(f"[MRV2-A R{args.resolution}] epoch={epoch} loss={curve['loss']:.4f} threshold={selected['threshold']:.2f} small={selected['metrics']['small_object_recall']:.4f} metal={selected['metal_can_recall']:.4f} f1={selected['metrics']['macro_f1']:.4f}", flush=True)
    selected = best[1]
    checkpoint = args.output / f"mrv2_a_r{args.resolution}.pt"
    torch.save({
        "state_dict": best_state, "architecture": X3_ARCHITECTURE,
        "route": "MRV2-A", "input_size": input_size,
        "frozen_threshold_from_train_world_holdout": selected["threshold"],
        "checkpoint_status": "training_complete_candidate_not_frozen",
        "parent_x3_sha256": sha256(args.x3_checkpoint),
        "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
    }, checkpoint)
    report = {
        "schema_version": 1, "stage": "MRV2-A-TRAIN", "route": "MRV2-A",
        "resolution": args.resolution, "input_size": input_size,
        "data_policy": {
            "train_pool_frames": len(train_pool), "holdout_frames": len(holdout),
            "small_unique_frames": len(small_keys), "metal_unique_frames": len(metal_keys),
            "small_copy_paste_donors": len(donors), "quota_policy": "30_small_20_negative_15_metal_35_general",
            "crop_scale_augmentation": True, "TRAIN_only_copy_paste": True,
            "VAL_read_for_training_or_selection": False, "G5_SEALED_FINAL_read": False,
            "legacy_G4_D6_read": False, "dataset_qa": qa,
        },
        "parent_x3": {"path": args.x3_checkpoint.as_posix(), "sha256": sha256(args.x3_checkpoint)},
        "training": {"epochs": args.epochs, "epoch_frames": args.epoch_frames, "duration_s": time.perf_counter() - started, "curves": curves},
        "selection": selected,
        "checkpoint": {"path": checkpoint.name, "bytes": checkpoint.stat().st_size, "sha256": sha256(checkpoint)},
        "product_ready": False, "next_action": "run fixed VAL and D1-D5 MRV2-A detector gate",
    }
    (args.output / f"MRV2_A_R{args.resolution}_TRAIN_REPORT.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
