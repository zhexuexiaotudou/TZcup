#!/usr/bin/env python3
"""Evaluate the frozen V10 classifier on the CRCRV11 five-view matrix."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from audit_crcrv11_classifier_contract import candidate_key, expand, remap_path
from evaluate_trcrv10_proposals import iou
from prepare_trcrv10_classifier_holdout import size_bucket


CLASSES = ("plastic_bottle", "metal_can", "paper_litter", "background_or_unknown")
TARGETS = CLASSES[:3]
COCO_CLASSES = {1: TARGETS[0], 2: TARGETS[1], 3: TARGETS[2]}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def jitter(box: list[float]) -> list[float]:
    x1, y1, x2, y2 = box
    width, height = x2 - x1, y2 - y1
    return [x1 - .06 * width, y1 + .04 * height, x2 + .03 * width, y2 - .02 * height]


def crop_array(image: np.ndarray, box: list[float]) -> np.ndarray:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = box
    x1 = max(0, min(width - 1, int(round(x1))))
    y1 = max(0, min(height - 1, int(round(y1))))
    x2 = max(1, min(width, int(round(x2))))
    y2 = max(1, min(height, int(round(y2))))
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"empty crop: {box}")
    return image[y1:y2, x1:x2]


def entropy(probabilities: np.ndarray) -> float:
    clipped = np.clip(probabilities, 1e-12, 1.0)
    return float(-(clipped * np.log(clipped)).sum())


def metrics(records: list[dict], labels: tuple[str, ...]) -> dict:
    confusion = np.zeros((len(labels), len(CLASSES)), dtype=np.int64)
    truth_index = {name: position for position, name in enumerate(labels)}
    predicted_index = {name: position for position, name in enumerate(CLASSES)}
    valid = [row for row in records if row["truth"] in truth_index and row["predicted"] in predicted_index]
    for row in valid:
        confusion[truth_index[row["truth"]], predicted_index[row["predicted"]]] += 1
    per_class, f1s = {}, []
    for position, name in enumerate(labels):
        prediction_column = predicted_index[name]
        tp = int(confusion[position, prediction_column])
        fp = int(confusion[:, prediction_column].sum() - tp)
        fn = int(confusion[position, :].sum() - tp)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[name] = {"precision": precision, "recall": recall, "f1": f1,
                           "support": int(confusion[position].sum())}
        f1s.append(f1)
    return {
        "samples": len(valid),
        "accuracy": sum(confusion[position, predicted_index[name]] for position, name in enumerate(labels)) / max(confusion.sum(), 1),
        "macro_f1": float(np.mean(f1s)) if f1s else None,
        "per_class": per_class, "confusion_rows": list(labels), "confusion_columns": list(CLASSES),
        "confusion": confusion.tolist(),
        "mean_predicted_confidence": float(np.mean([row["confidence"] for row in valid])) if valid else None,
        "mean_entropy": float(np.mean([row["entropy"] for row in valid])) if valid else None,
    }


def background_metrics(records: list[dict]) -> dict:
    return {
        "samples": len(records),
        "background_specificity": sum(row["predicted"] == CLASSES[-1] for row in records) / max(len(records), 1),
        "mean_predicted_confidence": float(np.mean([row["confidence"] for row in records])) if records else None,
        "mean_entropy": float(np.mean([row["entropy"] for row in records])) if records else None,
        "predicted_counts": {name: sum(row["predicted"] == name for row in records) for name in CLASSES},
    }


def add_prediction_metadata(item: dict, probabilities: np.ndarray) -> dict:
    predicted_index = int(np.argmax(probabilities))
    return {
        **item, "predicted": CLASSES[predicted_index],
        "confidence": float(probabilities[predicted_index]),
        "entropy": entropy(probabilities),
        "class_probabilities": {name: float(probabilities[index]) for index, name in enumerate(CLASSES)},
    }


def fuse(first: dict, second: dict, view: str) -> dict:
    probabilities = np.asarray([first["class_probabilities"][name] for name in CLASSES])
    probabilities += np.asarray([second["class_probabilities"][name] for name in CLASSES])
    probabilities /= 2.0
    return add_prediction_metadata({
        "candidate_id": first["candidate_id"], "truth": first["truth"], "view": view,
    }, probabilities)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--raw-inference", type=Path, required=True)
    parser.add_argument("--holdout", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--threshold", type=float, required=True)
    parser.add_argument("--min-reliable-short-side", type=int, default=18)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--path-map", action="append", default=[])
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--control-limit", type=int, default=300)
    args = parser.parse_args()

    import torch
    from torch import nn
    import torchvision
    from torchvision.transforms import v2

    if not torch.cuda.is_available():
        raise RuntimeError("CRCRV11 five-view evaluation requires CUDA")
    coco, raw = load_json(args.coco), load_json(args.raw_inference)
    manifest_path = args.holdout / "CLASSIFIER_HOLDOUT_CROP_MANIFEST.json"
    manifest = load_json(manifest_path)
    if {row["source_split"] for row in manifest["rows"]} != {"G10_HOLDOUT"}:
        raise ValueError("five-view evaluation accepts G10_HOLDOUT only")
    if any(manifest.get(name) is True for name in ("G10_DEV_VAL_SEALED_read", "VAL_NEW_read", "G5_V2_read")):
        raise RuntimeError("sealed boundary is already consumed")
    mappings = []
    for value in args.path_map:
        source, destination = value.split("=", 1)
        mappings.append((source, Path(destination)))

    images = {int(row["id"]): row for row in coco["images"]}
    image_key = {(row["scene"], int(row["frame_index"])): int(row["id"]) for row in coco["images"]}
    annotations: dict[int, list[dict]] = defaultdict(list)
    for row in coco["annotations"]:
        x, y, width, height = row["bbox"]
        annotations[int(row["image_id"])].append({
            "class_id": COCO_CLASSES[int(row["category_id"])],
            "bbox": [x, y, x + width, y + height],
            "short_side": min(width, height),
        })
    raw_by_image = {int(row["image_id"]): row["detections"] for row in raw["frames"]}
    manifest_pairs: dict[str, dict[str, dict]] = defaultdict(dict)
    for row in manifest["rows"]:
        manifest_pairs[candidate_key(row)][row["view"]] = row

    image_cache: dict[int, np.ndarray] = {}

    def image_for(image_id: int) -> np.ndarray:
        if image_id not in image_cache:
            path = remap_path(images[image_id]["file_name"], mappings)
            loaded = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if loaded is None:
                raise ValueError(f"unreadable RGB: {path}")
            image_cache[image_id] = loaded
        return image_cache[image_id]

    samples: list[dict] = []
    pair_ids: dict[str, dict[str, str]] = defaultdict(dict)
    for key, pair in sorted(manifest_pairs.items()):
        tight = pair.get("tight")
        context = pair.get("context")
        if not tight or not context or tight["class_id"] not in TARGETS:
            continue
        image_id = image_key[(tight["scene"], int(tight["frame_index"]))]
        proposal_box = tight["source_bbox_xyxy"]
        matches = sorted(
            ((iou(proposal_box, target["bbox"]), target) for target in annotations.get(image_id, [])),
            key=lambda item: item[0], reverse=True,
        )
        if not matches or matches[0][0] < .5:
            continue
        target = matches[0][1]
        boxes = {
            "A_GT_TIGHT": target["bbox"], "B_GT_CONTEXT": expand(target["bbox"]),
            "C_PROPOSAL_TIGHT": proposal_box, "D_PROPOSAL_CONTEXT": expand(proposal_box),
            "E_DETECTOR_JITTER": jitter(target["bbox"]),
        }
        for view, box in boxes.items():
            sample_id = f"{view}:{key}"
            pair_ids[key][view] = sample_id
            samples.append({
                "sample_id": sample_id, "candidate_id": key, "truth": target["class_id"],
                "view": view, "image_id": image_id, "box": box,
                "crop": crop_array(image_for(image_id), box),
            })

    for key, pair in sorted(manifest_pairs.items()):
        tight, context = pair.get("tight"), pair.get("context")
        if not tight or not context or tight["class_id"] != CLASSES[-1]:
            continue
        image_id = image_key[(tight["scene"], int(tight["frame_index"]))]
        for view, box in (("F_BACKGROUND_PROPOSAL_TIGHT", tight["source_bbox_xyxy"]),
                          ("G_BACKGROUND_PROPOSAL_CONTEXT", expand(tight["source_bbox_xyxy"]))):
            sample_id = f"{view}:{key}"
            pair_ids[key][view] = sample_id
            samples.append({
                "sample_id": sample_id, "candidate_id": key, "truth": CLASSES[-1],
                "view": view, "image_id": image_id, "box": box,
                "crop": crop_array(image_for(image_id), box),
            })

    control_count = 0
    for image_id, meta in sorted(images.items()):
        if not meta.get("negative_only") or control_count >= args.control_limit:
            continue
        width, height = int(meta["width"]), int(meta["height"])
        box = [.24 * width, .55 * height, .44 * width, .78 * height]
        key = f"{meta['scene']}:{meta['frame_index']}:clean_ground"
        samples.append({
            "sample_id": f"H_RANDOM_CLEAN_GROUND:{key}", "candidate_id": key,
            "truth": CLASSES[-1], "view": "H_RANDOM_CLEAN_GROUND", "image_id": image_id,
            "box": box, "crop": crop_array(image_for(image_id), box),
        })
        control_count += 1

    hard_count = 0
    for image_id, detections in sorted(raw_by_image.items()):
        if hard_count >= args.control_limit:
            break
        targets = annotations.get(image_id, [])
        for proposal_index, proposal in enumerate(detections):
            score, box = float(proposal["score"]), proposal["bbox_xyxy"]
            short_side = min(box[2] - box[0], box[3] - box[1])
            best_iou = max((iou(box, target["bbox"]) for target in targets), default=0.0)
            if not (.05 <= score < args.threshold and best_iou < .20 and short_side >= args.min_reliable_short_side):
                continue
            key = f"{images[image_id]['scene']}:{images[image_id]['frame_index']}:{proposal_index}:hard"
            samples.append({
                "sample_id": f"I_HARD_NEGATIVE_CATEGORY:{key}", "candidate_id": key,
                "truth": CLASSES[-1], "view": "I_HARD_NEGATIVE_CATEGORY", "image_id": image_id,
                "box": box, "score": score, "source_class_label": proposal.get("source_class_label"),
                "crop": crop_array(image_for(image_id), box),
            })
            hard_count += 1
            if hard_count >= args.control_limit:
                break

    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    if checkpoint.get("model_name") != "convnext_tiny" or tuple(checkpoint.get("classes", ())) != CLASSES:
        raise ValueError("five-view evaluator requires the frozen four-class ConvNeXt-Tiny checkpoint")
    model = torchvision.models.convnext_tiny(weights=None)
    model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, len(CLASSES))
    model.load_state_dict(checkpoint["model"])
    model.cuda().eval()
    transform = v2.Compose([
        v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True),
        v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)),
    ])
    records = []
    with torch.inference_mode():
        for start in range(0, len(samples), args.batch_size):
            chunk = samples[start:start + args.batch_size]
            tensors = []
            for item in chunk:
                rgb = cv2.cvtColor(item["crop"], cv2.COLOR_BGR2RGB)
                tensors.append(transform(torch.from_numpy(rgb).permute(2, 0, 1)))
            probabilities = model(torch.stack(tensors).cuda(non_blocking=True)).softmax(1).cpu().numpy()
            for item, probability in zip(chunk, probabilities):
                compact = {key: value for key, value in item.items() if key != "crop"}
                records.append(add_prediction_metadata(compact, probability))

    by_sample = {row["sample_id"]: row for row in records}
    target_views = ("A_GT_TIGHT", "B_GT_CONTEXT", "C_PROPOSAL_TIGHT", "D_PROPOSAL_CONTEXT", "E_DETECTOR_JITTER")
    background_views = ("F_BACKGROUND_PROPOSAL_TIGHT", "G_BACKGROUND_PROPOSAL_CONTEXT", "H_RANDOM_CLEAN_GROUND", "I_HARD_NEGATIVE_CATEGORY")
    view_metrics = {view: metrics([row for row in records if row["view"] == view], TARGETS) for view in target_views}
    background = {view: background_metrics([row for row in records if row["view"] == view]) for view in background_views}

    fused_records = []
    target_disagreement = background_disagreement = target_complementary = 0
    target_pairs = background_pairs = 0
    for key, ids in pair_ids.items():
        if "C_PROPOSAL_TIGHT" in ids and "D_PROPOSAL_CONTEXT" in ids:
            first, second = by_sample[ids["C_PROPOSAL_TIGHT"]], by_sample[ids["D_PROPOSAL_CONTEXT"]]
            fused_records.append(fuse(first, second, "C_D_CANDIDATE_FUSED"))
            target_pairs += 1
            target_disagreement += first["predicted"] != second["predicted"]
            target_complementary += (first["predicted"] == first["truth"]) != (second["predicted"] == second["truth"])
        if "F_BACKGROUND_PROPOSAL_TIGHT" in ids and "G_BACKGROUND_PROPOSAL_CONTEXT" in ids:
            first, second = by_sample[ids["F_BACKGROUND_PROPOSAL_TIGHT"]], by_sample[ids["G_BACKGROUND_PROPOSAL_CONTEXT"]]
            fused_records.append(fuse(first, second, "F_G_CANDIDATE_FUSED"))
            background_pairs += 1
            background_disagreement += first["predicted"] != second["predicted"]
    fused_metrics = metrics(fused_records, CLASSES)
    disagreement = {
        "target_pairs": target_pairs,
        "target_tight_context_disagreement_rate": target_disagreement / max(target_pairs, 1),
        "target_complementary_correctness_rate": target_complementary / max(target_pairs, 1),
        "background_pairs": background_pairs,
        "background_tight_context_disagreement_rate": background_disagreement / max(background_pairs, 1),
        "R3_COMPLEMENTARY_EVIDENCE": target_complementary / max(target_pairs, 1) >= .05,
    }
    degradation = {
        "gt_tight_to_proposal_tight_macro_f1": view_metrics["C_PROPOSAL_TIGHT"]["macro_f1"] - view_metrics["A_GT_TIGHT"]["macro_f1"],
        "gt_context_to_proposal_context_macro_f1": view_metrics["D_PROPOSAL_CONTEXT"]["macro_f1"] - view_metrics["B_GT_CONTEXT"]["macro_f1"],
        **disagreement,
    }
    matrix = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-02-FROZEN-FIVE-VIEW",
        "source_split": "G10_HOLDOUT", "checkpoint_sha256": sha256(args.checkpoint),
        "holdout_manifest_sha256": sha256(manifest_path), "raw_inference_sha256": sha256(args.raw_inference),
        "threshold": args.threshold, "views": view_metrics, "background_controls": background,
        "candidate_fused": fused_metrics, "disagreement": disagreement,
        "records": records, "fused_records": fused_records,
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output / "V11_FIVE_VIEW_MATRIX.json", matrix)
    write_json(args.output / "V11_VIEW_DEGRADATION.json", {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-02-VIEW-DEGRADATION",
        **degradation, "checkpoint_sha256": matrix["checkpoint_sha256"],
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
    })
    print(json.dumps({
        "views": view_metrics, "background_controls": background,
        "candidate_fused": fused_metrics, "degradation": degradation,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
