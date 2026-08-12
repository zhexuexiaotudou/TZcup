#!/usr/bin/env python3
"""Mine GA1 low-score false/wrong-class proposals from TRAIN only."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def iou(a, b) -> float:
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    aa, bb = max(0, a[2] - a[0]) * max(0, a[3] - a[1]), max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return intersection / max(aa + bb - intersection, 1e-12)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum", type=int, default=2000)
    parser.add_argument("--score", type=float, default=0.03)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if sha256(args.checkpoint) != args.expected_sha256:
        raise RuntimeError("GA1 checkpoint SHA mismatch")
    payload = json.loads((args.source_root / "source_train.json").read_text(encoding="utf-8"))
    truths = defaultdict(list)
    for row in payload["annotations"]:
        x, y, w, h = row["bbox"]
        truths[int(row["image_id"])].append({"bbox": [x, y, x + w, y + h], "label": int(row["category_id"]) - 1})
    patch_mmdet_cuda_nms()
    from mmdet.apis import inference_detector, init_detector
    model = init_detector(str(args.config), str(args.checkpoint), device="cuda:0")
    candidates = []
    for offset in range(0, len(payload["images"]), args.batch_size):
        batch = payload["images"][offset:offset + args.batch_size]
        paths = [args.source_root / row["file_name"] for row in batch]
        outputs = inference_detector(model, [str(path) for path in paths])
        if not isinstance(outputs, list):
            outputs = [outputs]
        for image, path, output in zip(batch, paths, outputs):
            pred = output.pred_instances.to("cpu")
            frame_truth = truths[int(image["id"])]
            for box, score, label in zip(pred.bboxes.tolist(), pred.scores.tolist(), pred.labels.tolist()):
                if score < args.score:
                    continue
                overlaps = sorted(((iou(box, row["bbox"]), row) for row in frame_truth), reverse=True, key=lambda item: item[0])
                best = overlaps[0] if overlaps else None
                if best and best[0] >= 0.5 and int(label) == best[1]["label"]:
                    continue
                taxonomy = "wrong_class" if best and best[0] >= 0.5 else "near_iou_confuser" if best and best[0] >= 0.1 else "unmatched_background"
                candidates.append({"source": path, "source_image_id": image["id"], "bbox": box, "score": float(score), "predicted_label": int(label) + 1, "taxonomy": taxonomy})
    candidates.sort(key=lambda row: (-row["score"], row["source_image_id"], row["bbox"]))
    selected = candidates[: max(args.minimum, min(len(candidates), args.minimum * 2))]
    args.output.mkdir(parents=True)
    images = []
    taxonomy = Counter()
    for index, row in enumerate(selected, 1):
        image = cv2.imread(str(row["source"]), cv2.IMREAD_COLOR)
        x1, y1, x2, y2 = row["bbox"]
        width, height = x2 - x1, y2 - y1
        pad_x, pad_y = max(4, width * 0.2), max(4, height * 0.2)
        left, top = max(0, int(x1 - pad_x)), max(0, int(y1 - pad_y))
        right, bottom = min(image.shape[1], int(x2 + pad_x + 1)), min(image.shape[0], int(y2 + pad_y + 1))
        crop = image[top:bottom, left:right]
        if crop.size == 0:
            continue
        target = args.output / "images" / f"proposal_{index:06d}.png"
        target.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(target), crop):
            raise RuntimeError(target)
        taxonomy[row["taxonomy"]] += 1
        images.append({"id": len(images) + 1, "file_name": target.relative_to(args.output).as_posix(), "width": crop.shape[1], "height": crop.shape[0], "negative_only": True, "proposal_taxonomy": row["taxonomy"], "source_image_id": row["source_image_id"], "source_rgb_sha256": sha256(row["source"]), "score": row["score"]})
    coco = {"info": {"description": "RGDRV8 Route A TRAIN-only GA1 proposal hard negatives", "TRAIN_ONLY": True, "HOLDOUT_NEW_read": False, "VAL_NEW_read": False, "G5_V2_read": False}, "images": images, "annotations": [], "categories": payload["categories"]}
    (args.output / "proposals.json").write_text(json.dumps(coco, indent=2) + "\n", encoding="utf-8")
    report = {"schema_version": 1, "stage": "RGDRV8-02-ROUTE-A-HARD-NEGATIVE-MINING", "checkpoint_sha256": args.expected_sha256, "source_train_sha256": sha256(args.source_root / "source_train.json"), "score_threshold": args.score, "candidate_count": len(candidates), "crop_count": len(images), "taxonomy": dict(taxonomy), "TRAIN_ONLY": True, "HOLDOUT_NEW_read": False, "VAL_NEW_read": False, "G5_V2_read": False, "G8_HARD_NEGATIVE_PROPOSALS_PASS": len(images) >= args.minimum}
    (args.output / "G8_HARD_NEGATIVE_PROPOSALS.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["G8_HARD_NEGATIVE_PROPOSALS_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
