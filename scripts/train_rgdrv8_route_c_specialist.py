#!/usr/bin/env python3
"""Train the final RGDRV8 G8-only class-agnostic small-litter specialist."""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import random
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.g6_small_specialist import (  # noqa: E402
    SmallSpecialistDataset,
    _best_tile,
    build_small_specialist,
    rgdrv8_ground_roi_tiles,
    small_specialist_collate,
)

SEED = 20260813
THRESHOLDS = tuple(round(value / 100, 2) for value in range(5, 96, 5))
NAMES = ("plastic_bottle", "metal_can", "paper_litter")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_split(coco_path: Path, root: Path) -> tuple[list[dict], dict]:
    payload = json.loads(coco_path.read_text())
    images = {int(row["id"]): row for row in payload["images"]}
    indexed = defaultdict(list)
    categories = {int(row["id"]): row["name"] for row in payload["categories"]}
    for raw in payload["annotations"]:
        x, y, width, height = raw["bbox"]
        image = images[int(raw["image_id"])]
        indexed[(int(image["scene_seed"]), int(image["frame_index"]))].append(
            {
                "class_id": categories[int(raw["category_id"])],
                "bbox_short_side_px": int(raw["bbox_short_side_px"]),
                "bbox_xyxy": [x, y, x + width, y + height],
                "short_side_bucket": (
                    "lt8" if int(raw["bbox_short_side_px"]) < 8
                    else "8_12" if int(raw["bbox_short_side_px"]) < 12
                    else "12_18"
                ),
            }
        )
    rows = [
        {
            **row,
            "rgb_path": root / row["file_name"],
            "split": coco_path.stem,
            "negative_area_taxonomies": ["G8_negative_only"] if row.get("negative_only") else [],
        }
        for row in payload["images"]
    ]
    return rows, indexed


def samples(rows: list[dict], indexed: dict, *, negative_stride: int = 2) -> list[dict]:
    tiles = rgdrv8_ground_roi_tiles()
    output = []
    occupied = set()
    for position, row in enumerate(rows):
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        assigned = defaultdict(list)
        for record in indexed.get(key, []):
            if record["class_id"] not in NAMES or int(record["bbox_short_side_px"]) >= 18:
                continue
            tile_index = _best_tile(record["bbox_xyxy"], tiles)
            if tile_index is None:
                raise RuntimeError(f"small target outside fixed RGDRV8 ROI: {key}")
            assigned[tile_index].append(record)
        for tile_index, targets in assigned.items():
            occupied.add((*key, tile_index))
            output.append({"rgb_path": row["rgb_path"], "scene_seed": key[0], "frame_index": key[1], "split": row["split"], "tile_index": tile_index, "tile": tiles[tile_index], "targets": targets, "hard_negative": False})
        if row.get("negative_only") and position % negative_stride == 0:
            for tile_index in range(len(tiles)):
                if (*key, tile_index) not in occupied:
                    output.append({"rgb_path": row["rgb_path"], "scene_seed": key[0], "frame_index": key[1], "split": row["split"], "tile_index": tile_index, "tile": tiles[tile_index], "targets": [], "hard_negative": True})
    return output


def iou(left, right) -> float:
    x0, y0, x1, y1 = max(left[0], right[0]), max(left[1], right[1]), min(left[2], right[2]), min(left[3], right[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    a = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    b = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    return inter / max(a + b - inter, 1e-12)


def score(model, rows, device, batch_size):
    import torch
    from torch.utils.data import DataLoader
    loader = DataLoader(SmallSpecialistDataset(rows), batch_size=batch_size, shuffle=False, num_workers=0, collate_fn=small_specialist_collate)
    result = []
    model.eval()
    with torch.inference_mode():
        for images, targets, raw in loader:
            predictions = model([image.to(device) for image in images])
            for prediction, target, sample in zip(predictions, targets, raw):
                result.append({"truth": target["boxes"].tolist(), "records": sample["targets"], "negative": not sample["targets"], "predictions": [{"box": box, "score": float(value)} for box, value in zip(prediction["boxes"].cpu().tolist(), prediction["scores"].cpu().tolist())]})
    return result


def metrics(scored, threshold):
    truth = matched = false = negative_false = 0
    per_class = {name: [0, 0] for name in NAMES}
    for row in scored:
        predictions = [item for item in row["predictions"] if item["score"] >= threshold]
        unused = set(range(len(predictions)))
        truth += len(row["truth"])
        for box, record in zip(row["truth"], row["records"]):
            per_class[record["class_id"]][0] += 1
            choices = sorted(((iou(box, predictions[index]["box"]), index) for index in unused), reverse=True)
            if choices and choices[0][0] >= 0.5:
                unused.remove(choices[0][1]); matched += 1; per_class[record["class_id"]][1] += 1
        false += len(unused)
        if row["negative"]: negative_false += len(unused)
    return {"threshold": threshold, "truth": truth, "matched": matched, "proposal_recall": matched / max(truth, 1), "proposal_precision_diagnostic": matched / max(matched + false, 1), "negative_false_proposals": negative_false, "per_class_recall": {name: value[1] / max(value[0], 1) for name, value in per_class.items()}}


def main() -> int:
    import torch
    from torch.utils.data import DataLoader
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=3)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--max-train-samples", type=int, default=3000)
    parser.add_argument("--max-holdout-samples", type=int, default=1400)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    qa = json.loads((args.prepared / "G8_DATASET_QA.json").read_text())
    if not qa["G8_REAL_GAZEBO_DATA_PASS"]: raise RuntimeError("G8 QA not passed")
    train_rows, train_index = load_split(args.prepared / "fit.json", args.prepared)
    holdout_rows, holdout_index = load_split(args.prepared / "holdout.json", args.prepared)
    train, holdout = samples(train_rows, train_index), samples(holdout_rows, holdout_index)
    rng = random.Random(SEED); rng.shuffle(train); rng.shuffle(holdout)
    train, holdout = train[: args.max_train_samples], holdout[: args.max_holdout_samples]
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda": raise RuntimeError("formal Route C specialist requires CUDA")
    torch.manual_seed(SEED); torch.cuda.manual_seed_all(SEED)
    model = build_small_specialist(args.base_checkpoint).to(device)
    loader = DataLoader(SmallSpecialistDataset(train), batch_size=args.batch_size, shuffle=True, num_workers=0, collate_fn=small_specialist_collate, generator=torch.Generator().manual_seed(SEED))
    optimizer = torch.optim.AdamW(model.parameters(), lr=5e-5, weight_decay=1e-4)
    scaler = torch.amp.GradScaler("cuda")
    args.output.mkdir(parents=True)
    history = []; best = None; started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        model.train(); total = steps = 0
        for images, targets, _ in loader:
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast("cuda", dtype=torch.float16):
                losses = model([image.to(device) for image in images], [{key: value.to(device) for key, value in target.items()} for target in targets]); loss = sum(losses.values())
            scaler.scale(loss).backward(); scaler.step(optimizer); scaler.update(); total += float(loss.detach()); steps += 1
        scored = score(model, holdout, device, args.batch_size)
        sweep = [metrics(scored, threshold) for threshold in THRESHOLDS]
        selected = max(sweep, key=lambda row: (row["proposal_recall"] >= 0.97, row["proposal_recall"], row["proposal_precision_diagnostic"], row["threshold"]))
        history.append({"epoch": epoch, "loss": total / max(steps, 1), "selected": selected})
        rank = (selected["proposal_recall"] >= 0.97, selected["proposal_recall"], selected["proposal_precision_diagnostic"])
        if best is None or rank > best[0]:
            best = (rank, history[-1]); torch.save({"state_dict": model.state_dict(), "epoch": epoch, "objectness_threshold": selected["threshold"], "base_checkpoint_sha256": sha256(args.base_checkpoint)}, args.output / "specialist.pt")
        print(f"[Route C specialist] epoch={epoch} loss={history[-1]['loss']:.4f} recall={selected['proposal_recall']:.4f} precision={selected['proposal_precision_diagnostic']:.4f}", flush=True)
    selected = best[1]["selected"]
    report = {"schema_version": 1, "stage": "RGDRV8-04-ROUTE-C-SMALL-SPECIALIST", "architecture": "P2_FCOS_R50_CLASS_AGNOSTIC_3x3_FIXED_GROUND_TILES", "roi_contract": {"native_size": [640, 480], "tiles": rgdrv8_ground_roi_tiles(), "tile_scale_xy": [2.0, 2.0], "GT_based_selection": False}, "data_policy": {"TRAIN": "G8_TRAIN_NEW_only", "HOLDOUT": "G8_HOLDOUT_NEW_only", "VAL_NEW_read": False, "G5_V2_read": False, "train_samples": len(train), "holdout_samples": len(holdout)}, "history": history, "selected_holdout": selected, "checkpoint_sha256": sha256(args.output / "specialist.pt"), "ROUTE_C_SPECIALIST_HOLDOUT_PASS": selected["proposal_recall"] >= 0.97 and all(value >= 0.95 for value in selected["per_class_recall"].values()), "duration_s": time.perf_counter() - started}
    (args.output / "ROUTE_C_SPECIALIST_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"selected": selected, "pass": report["ROUTE_C_SPECIALIST_HOLDOUT_PASS"]}, indent=2))
    return 0 if report["ROUTE_C_SPECIALIST_HOLDOUT_PASS"] else 4


if __name__ == "__main__": raise SystemExit(main())
