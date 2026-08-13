#!/usr/bin/env python3
"""Select a bounded T2/T3 checkpoint using the frozen G9 product protocol."""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

import evaluate_tgarv9_t1 as evaluator  # noqa: E402
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402


POLICIES = [
    {"name": "weighted_log_probability", "config": {"observation_threshold": 0.05, "confirmation_probability": 0.90, "minimum_observations": 3, "maximum_map_scatter_m": 0.30}},
    {"name": "weighted_log_conservative", "config": {"observation_threshold": 0.10, "confirmation_probability": 0.95, "minimum_observations": 3, "maximum_map_scatter_m": 0.20}},
    {"name": "weighted_log_reobserve", "config": {"observation_threshold": 0.05, "confirmation_probability": 0.97, "minimum_observations": 4, "maximum_map_scatter_m": 0.25}},
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rank(result: dict) -> tuple:
    metrics = result["selected_metrics"]
    if result["pass"]:
        # Once all product constraints pass, AP50:95 is the only optimizer.
        return (1, result["detector_diagnostics"]["AP50_95"])
    return (
        0,
        min(metrics["eventual_correct_class_recall"], metrics["confirmed_actionable_precision"]),
        metrics["small_eventual_correct_class_recall"],
        -metrics["clean_opportunity_miss"],
    )


def detector_diagnostics(coco_payload: dict, raw_frames: list[dict], score_threshold: float = 0.05) -> dict:
    """Return COCO AP plus deterministic one-to-one IoU=.5 diagnostics."""
    from pycocotools.coco import COCO
    from pycocotools.cocoeval import COCOeval

    category_ids = sorted(int(row["id"]) for row in coco_payload["categories"])
    annotations_by_image: dict[int, list[dict]] = {}
    for annotation in coco_payload["annotations"]:
        annotations_by_image.setdefault(int(annotation["image_id"]), []).append(annotation)
    predictions = []
    true_positive = false_positive = false_negative = wrong_class_match = 0
    per_class = {category_id: {"true_positive": 0, "false_positive": 0, "false_negative": 0} for category_id in category_ids}
    size_recall = {
        "lt_18px": {"matched": 0, "total": 0},
        "18_to_32px": {"matched": 0, "total": 0},
        "gt_32px": {"matched": 0, "total": 0},
    }
    def size_bin(annotation: dict) -> str:
        short_side = min(float(annotation["bbox"][2]), float(annotation["bbox"][3]))
        if short_side < 18.0:
            return "lt_18px"
        if short_side <= 32.0:
            return "18_to_32px"
        return "gt_32px"
    for frame in raw_frames:
        image_id = int(frame["image_id"])
        detections = sorted(frame["detections"], key=lambda row: float(row["score"]), reverse=True)
        for row in detections:
            box = row["bbox_xyxy"]
            predictions.append({
                "image_id": image_id,
                "category_id": int(row["label"]) + 1,
                "bbox": [box[0], box[1], box[2] - box[0], box[3] - box[1]],
                "score": float(row["score"]),
            })
        ground_truth = annotations_by_image.get(image_id, [])
        for annotation in ground_truth:
            size_recall[size_bin(annotation)]["total"] += 1
        matched: set[int] = set()
        for row in (item for item in detections if float(item["score"]) >= score_threshold):
            predicted_category = int(row["label"]) + 1
            candidates = [(index, evaluator.iou(row["bbox_xyxy"], annotation["bbox"])) for index, annotation in enumerate(ground_truth) if index not in matched]
            candidates = [item for item in candidates if item[1] >= 0.5]
            if not candidates:
                false_positive += 1
                per_class[predicted_category]["false_positive"] += 1
                continue
            index, _ = max(candidates, key=lambda item: item[1])
            matched.add(index)
            actual_category = int(ground_truth[index]["category_id"])
            if actual_category == predicted_category:
                true_positive += 1
                per_class[actual_category]["true_positive"] += 1
                size_recall[size_bin(ground_truth[index])]["matched"] += 1
            else:
                wrong_class_match += 1
                false_positive += 1
                false_negative += 1
                per_class[predicted_category]["false_positive"] += 1
                per_class[actual_category]["false_negative"] += 1
        for index, annotation in enumerate(ground_truth):
            if index not in matched:
                false_negative += 1
                per_class[int(annotation["category_id"])]["false_negative"] += 1

    precision = true_positive / max(true_positive + false_positive, 1)
    recall = true_positive / max(true_positive + false_negative, 1)
    with tempfile.TemporaryDirectory() as temporary:
        ground_truth_path = Path(temporary) / "ground_truth.json"
        predictions_path = Path(temporary) / "predictions.json"
        ground_truth_path.write_text(json.dumps(coco_payload))
        predictions_path.write_text(json.dumps(predictions))
        coco = COCO(str(ground_truth_path))
        results = coco.loadRes(str(predictions_path))
        evaluation = COCOeval(coco, results, "bbox")
        evaluation.evaluate()
        evaluation.accumulate()
        evaluation.summarize()
        ap_50_95, ap_50 = float(evaluation.stats[0]), float(evaluation.stats[1])
    category_names = {int(row["id"]): row["name"] for row in coco_payload["categories"]}
    per_class_metrics = {}
    for category_id, counts in per_class.items():
        class_precision = counts["true_positive"] / max(counts["true_positive"] + counts["false_positive"], 1)
        class_recall = counts["true_positive"] / max(counts["true_positive"] + counts["false_negative"], 1)
        per_class_metrics[category_names[category_id]] = {
            **counts,
            "precision": class_precision,
            "recall": class_recall,
            "f1": 2.0 * class_precision * class_recall / max(class_precision + class_recall, 1e-12),
        }
    return {
        "score_threshold": score_threshold,
        "iou_threshold": 0.5,
        "precision": precision,
        "recall": recall,
        "f1": 2.0 * precision * recall / max(precision + recall, 1e-12),
        "AP50": ap_50,
        "AP50_95": ap_50_95,
        "wrong_class_match_count": wrong_class_match,
        "per_class": per_class_metrics,
        "size_recall": {
            name: {**counts, "recall": counts["matched"] / max(counts["total"], 1)}
            for name, counts in size_recall.items()
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", choices=("T2", "T3"), required=True)
    parser.add_argument("--g9", type=Path, required=True)
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-checkpoints", type=int, required=True)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--windows-root", required=True)
    parser.add_argument("--container-root", required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    checkpoints = sorted(args.checkpoint_dir.glob("epoch_*.pth"), key=lambda path: int(path.stem.split("_")[-1]))
    if len(checkpoints) != args.expected_checkpoints:
        raise RuntimeError(f"expected {args.expected_checkpoints} checkpoints, found {len(checkpoints)}")

    manifest = json.loads((args.g9 / "G9_HOLDOUT_MANIFEST.json").read_text())
    frame_rows = json.loads((args.g9 / "G9_PRODUCT_FRAME_STREAM.json").read_text())["frames"]
    for frame in frame_rows:
        for key in ("rgb_path", "depth_path", "camera_info_path", "tf_path"):
            frame[key] = frame[key].replace(args.windows_root, args.container_root).replace("\\", "/")
    frames = {row["frame_ref"]: row for row in frame_rows}
    tubes = json.loads((args.g9 / "G9_TARGET_TUBES.json").read_text())["tubes"]
    negatives = {row["mission_id"] for row in manifest["missions"] if row["negative_only"]}
    coco = json.loads(args.coco.read_text())

    patch_mmdet_cuda_nms()
    import mmcv.ops.multi_scale_deform_attn as deform_attn
    deform_attn.IS_CUDA_AVAILABLE = False
    import torch
    from mmdet.apis import inference_detector, init_detector

    args.output.mkdir(parents=True)
    candidates = []
    for checkpoint in checkpoints:
        candidate_dir = args.output / checkpoint.stem
        candidate_dir.mkdir()
        checkpoint_hash = sha256(checkpoint)
        model = init_detector(str(args.config), str(checkpoint), device="cuda:0")
        raw_frames = []
        for offset in range(0, len(coco["images"]), args.batch_size):
            batch = coco["images"][offset : offset + args.batch_size]
            paths = [row["file_name"].replace(args.windows_root, args.container_root).replace("\\", "/") for row in batch]
            outputs = inference_detector(model, paths)
            outputs = outputs if isinstance(outputs, list) else [outputs]
            for image, output in zip(batch, outputs):
                pred = output.pred_instances.to("cpu")
                raw_frames.append({
                    "image_id": int(image["id"]),
                    "mission_id": image["mission_id"],
                    "frame_index": int(image["frame_index"]),
                    "negative_only": bool(image["negative_only"]),
                    "detections": [
                        {"bbox_xyxy": [float(value) for value in box], "score": float(score), "label": int(label)}
                        for box, score, label in zip(pred.bboxes.tolist(), pred.scores.tolist(), pred.labels.tolist())
                    ],
                })
        raw = {row["image_id"]: row for row in raw_frames}
        raw_report = {
            "schema_version": 1,
            "protocol": "TGARV9",
            "stage": f"{args.stage}_G9_RAW_INFERENCE",
            "checkpoint": checkpoint.name,
            "checkpoint_sha256": checkpoint_hash,
            "config_sha256": sha256(args.config),
            "frame_count": len(raw_frames),
            "frames": raw_frames,
            "VAL_NEW_read": False,
            "G5_V2_read": False,
        }
        (candidate_dir / "RAW_INFERENCE.json").write_text(json.dumps(raw_report, indent=2) + "\n")
        policies = [evaluator.run(policy, tubes, frames, raw, negatives) for policy in POLICIES]
        selected = max(policies, key=lambda row: (row["pass"], min(row["metrics"]["eventual_correct_class_recall"], row["metrics"]["confirmed_actionable_precision"]), row["metrics"]["small_eventual_correct_class_recall"]))
        candidate = {
            "checkpoint": checkpoint.name,
            "checkpoint_sha256": checkpoint_hash,
            "algorithm_results": policies,
            "selected_algorithm": selected["algorithm"],
            "selected_metrics": selected["metrics"],
            "selected_gates": selected["gates"],
            "detector_diagnostics": detector_diagnostics(coco, raw_frames),
            "pass": selected["pass"],
        }
        (candidate_dir / "G9_REPORT.json").write_text(json.dumps(candidate, indent=2) + "\n")
        candidates.append(candidate)
        del model, raw, raw_frames
        gc.collect()
        torch.cuda.empty_cache()

    selected = max(candidates, key=rank)
    report = {
        "schema_version": 1,
        "protocol": "TGARV9",
        "stage": f"{args.stage}_G9_HOLDOUT",
        "selection_policy": "all bounded epoch checkpoints; shared three-policy temporal/geometry evaluator",
        "checkpoint_count": len(candidates),
        "candidates": candidates,
        "selected_checkpoint": selected["checkpoint"],
        "selected_checkpoint_sha256": selected["checkpoint_sha256"],
        "selected_algorithm": selected["selected_algorithm"],
        "selected_metrics": selected["selected_metrics"],
        "selected_gates": selected["selected_gates"],
        f"TGARV9_{args.stage}_HOLDOUT_PASS": selected["pass"],
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    (args.output / f"{args.stage}_G9_HOLDOUT_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    if not selected["pass"]:
        metrics = selected["selected_metrics"]
        taxonomy = {
            "schema_version": 1,
            "stage": args.stage,
            "detector_separation_limited": metrics["eventual_correct_class_recall"] < 0.95 or metrics["wrong_confirmed_actionable_rate"] > 0.01,
            "temporal_limited": metrics["eventual_observation_recall"] >= 0.97 and metrics["eventual_correct_class_recall"] < 0.95,
            "geometry_limited": not selected["selected_gates"]["clean_opportunity_miss"],
            "track_fragmentation_or_false_confirmation_limited": metrics["false_CLEAN_NOW"] > 0 or metrics["confirmed_actionable_precision"] < 0.95,
            "paper_specific": metrics["per_class_correct_recall"].get("paper_litter", 0.0) < 0.95,
            "small_specific": metrics["small_eventual_correct_class_recall"] < 0.90,
            "failed_gates": [name for name, passed in selected["selected_gates"].items() if not passed],
        }
        (args.output / f"{args.stage}_FAILURE_TAXONOMY.json").write_text(json.dumps(taxonomy, indent=2) + "\n")
    return 0 if selected["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
