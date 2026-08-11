#!/usr/bin/env python3
"""Select OPR-C on TRAIN-world holdout, then screen untouched G6 VAL."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.opr_c_rtmdet import CLASS_NAMES, patch_mmdet_cuda_nms  # noqa: E402
from train_opr_a_specialist import iou  # noqa: E402


THRESHOLDS = tuple(round(value / 100, 2) for value in range(10, 96, 5))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_truth(annotation_path: Path, g6_root: Path) -> list[dict]:
    payload = json.loads(annotation_path.read_text())
    annotations: dict[int, list[dict]] = {}
    for annotation in payload["annotations"]:
        x, y, width, height = annotation["bbox"]
        annotations.setdefault(annotation["image_id"], []).append(
            {
                "bbox_xyxy": [x, y, x + width, y + height],
                "label": int(annotation["category_id"]),
            }
        )
    return [
        {
            "image_id": image["id"],
            "image_path": g6_root / image["file_name"],
            "truth": annotations.get(image["id"], []),
        }
        for image in payload["images"]
    ]


def infer(model, frames: list[dict], batch_size: int) -> list[dict]:
    from mmdet.apis import inference_detector

    raw = []
    for offset in range(0, len(frames), batch_size):
        batch = frames[offset : offset + batch_size]
        outputs = inference_detector(model, [str(frame["image_path"]) for frame in batch])
        if not isinstance(outputs, list):
            outputs = [outputs]
        for frame, output in zip(batch, outputs):
            predictions = output.pred_instances.to("cpu")
            raw.append(
                {
                    "truth": frame["truth"],
                    "predictions": [
                        {
                            "bbox_xyxy": box,
                            "score": float(score),
                            "label": int(label) + 1,
                        }
                        for box, score, label in zip(
                            predictions.bboxes.tolist(),
                            predictions.scores.tolist(),
                            predictions.labels.tolist(),
                        )
                    ],
                }
            )
    return raw


def metrics(raw: list[dict], threshold: float) -> dict:
    truth_count = matched = false_positive = negative_fp = negative_frames = 0
    per_class = {name: {"truth": 0, "matched": 0} for name in CLASS_NAMES}
    for frame in raw:
        predictions = [item for item in frame["predictions"] if item["score"] >= threshold]
        unused = set(range(len(predictions)))
        truth_count += len(frame["truth"])
        for truth in frame["truth"]:
            class_name = CLASS_NAMES[truth["label"] - 1]
            per_class[class_name]["truth"] += 1
            ranked = sorted(
                (
                    (iou(truth["bbox_xyxy"], predictions[index]["bbox_xyxy"]), index)
                    for index in unused
                    if predictions[index]["label"] == truth["label"]
                ),
                reverse=True,
            )
            if ranked and ranked[0][0] >= 0.5:
                unused.remove(ranked[0][1])
                matched += 1
                per_class[class_name]["matched"] += 1
        false_positive += len(unused)
        if not frame["truth"]:
            negative_frames += 1
            negative_fp += len(predictions)
    return {
        "threshold": threshold,
        "truth_count": truth_count,
        "matched_correct_class": matched,
        "recall": matched / max(truth_count, 1),
        "precision": matched / max(matched + false_positive, 1),
        "false_positive_per_frame": false_positive / max(len(raw), 1),
        "negative_false_positive_per_frame": negative_fp / max(negative_frames, 1),
        "per_class_recall": {
            name: value["matched"] / value["truth"] if value["truth"] else None
            for name, value in per_class.items()
        },
    }


def select(raw: list[dict]) -> tuple[dict, list[dict]]:
    sweep = []
    for threshold in THRESHOLDS:
        item = metrics(raw, threshold)
        item["gates"] = {
            "recall_at_least_0_95": item["recall"] >= 0.95,
            "precision_at_least_0_95": item["precision"] >= 0.95,
            "unmatched_fp_per_frame_at_most_0_05": item["false_positive_per_frame"] <= 0.05,
        }
        item["all_pass"] = all(item["gates"].values())
        item["distance"] = (
            max(0.0, 0.95 - item["recall"])
            + max(0.0, 0.95 - item["precision"])
            + max(0.0, item["false_positive_per_frame"] - 0.05)
        )
        sweep.append(item)
    selected = min(
        sweep,
        key=lambda item: (not item["all_pass"], item["distance"], -item["recall"], -item["precision"]),
    )
    return selected, sweep


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--training", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    best = sorted(args.training.glob("best_coco_bbox_mAP_epoch_*.pth"))
    if len(best) != 1:
        raise RuntimeError(f"expected one holdout-selected best checkpoint, found {len(best)}")
    patch_mmdet_cuda_nms()
    from mmdet.apis import init_detector

    config = args.training / "opr_c_rtmdet_s_config.py"
    model = init_detector(str(config), str(best[0]), device="cuda:0")
    started = time.perf_counter()
    holdout_raw = infer(model, load_truth(args.prepared / "holdout.json", args.g6_root), args.batch_size)
    selected, sweep = select(holdout_raw)
    val_raw = infer(model, load_truth(args.prepared / "val.json", args.g6_root), args.batch_size)
    val_result = metrics(val_raw, selected["threshold"])
    gates = {
        "VAL_recall_at_least_0_95": val_result["recall"] >= 0.95,
        "VAL_precision_at_least_0_95": val_result["precision"] >= 0.95,
        "VAL_unmatched_fp_per_frame_at_most_0_05": val_result["false_positive_per_frame"] <= 0.05,
        "each_class_recall_at_least_0_90": all(
            value is not None and value >= 0.90 for value in val_result["per_class_recall"].values()
        ),
    }
    report = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-C-SCREEN",
        "route": "OPR-C_OFFICIAL_RTMDET_S",
        "checkpoint": {"path": best[0].name, "sha256": sha256(best[0])},
        "selection_policy": "best checkpoint by TRAIN-world holdout COCO mAP, threshold by same holdout; G6 VAL untouched until final screen",
        "selected_holdout_operating_point": selected,
        "holdout_threshold_sweep": sweep,
        "VAL": val_result,
        "gates": gates,
        "duration_s": time.perf_counter() - started,
        "G5_SEALED_FINAL_read": False,
        "OPR_C_PASS": all(gates.values()),
        "route_exhaustion": "OPR-C is final; no OPR-D/E/F" if not all(gates.values()) else None,
        "next_action": "integrate_online_development_candidate" if all(gates.values()) else "record_internal_model_blocker_and_minimum_next_research_need",
    }
    (args.output / "OPR_C_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["OPR_C_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
