#!/usr/bin/env python3
"""Select DDRV4-D1 on G7 holdout, then open G7 VAL exactly once."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import G7_DATASET_ID, require_ddrv4_selection_inputs  # noqa: E402
from sanitation_learning.opr_c_rtmdet import CLASS_NAMES, patch_mmdet_cuda_nms  # noqa: E402


THRESHOLDS = tuple(round(value / 100, 2) for value in range(5, 96))
GATES = {
    "recall": (">=", 0.95),
    "precision": (">=", 0.95),
    "macro_f1": (">=", 0.95),
    "metal_can_recall": (">=", 0.90),
    "paper_litter_precision": (">=", 0.95),
    "false_positive_per_frame": ("<=", 0.05),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def iou(first: list[float], second: list[float]) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    return intersection / max(first_area + second_area - intersection, 1e-12)


def load_truth(annotation_path: Path, g7_root: Path) -> list[dict]:
    payload = json.loads(annotation_path.read_text(encoding="utf-8"))
    by_image: dict[int, list[dict]] = {}
    for annotation in payload["annotations"]:
        x, y, width, height = (float(value) for value in annotation["bbox"])
        by_image.setdefault(int(annotation["image_id"]), []).append(
            {
                "bbox_xyxy": [x, y, x + width, y + height],
                "label": int(annotation["category_id"]),
                "small_lt18": float(annotation.get("bbox_short_side_px", min(width, height))) < 18.0,
            }
        )
    return [
        {
            "image_id": int(image["id"]),
            "image_path": g7_root / image["file_name"],
            "truth": by_image.get(int(image["id"]), []),
        }
        for image in payload["images"]
    ]


def infer(model, frames: list[dict], batch_size: int) -> list[dict]:
    from mmdet.apis import inference_detector

    raw: list[dict] = []
    for offset in range(0, len(frames), batch_size):
        batch = frames[offset : offset + batch_size]
        outputs = inference_detector(model, [str(frame["image_path"]) for frame in batch])
        if not isinstance(outputs, list):
            outputs = [outputs]
        for frame, output in zip(batch, outputs):
            predictions = output.pred_instances.to("cpu")
            raw.append(
                {
                    "image_id": frame["image_id"],
                    "truth": frame["truth"],
                    "predictions": [
                        {"bbox_xyxy": box, "score": float(score), "label": int(label) + 1}
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
    per_class = {name: {"truth": 0, "predicted": 0, "matched": 0} for name in CLASS_NAMES}
    truth_total = matched_total = predicted_total = small_total = small_matched = 0
    for frame in raw:
        truths = frame["truth"]
        predictions = sorted(
            (item for item in frame["predictions"] if item["score"] >= threshold),
            key=lambda item: item["score"],
            reverse=True,
        )
        unused_truth = set(range(len(truths)))
        truth_total += len(truths)
        predicted_total += len(predictions)
        for truth in truths:
            class_name = CLASS_NAMES[truth["label"] - 1]
            per_class[class_name]["truth"] += 1
            if truth["small_lt18"]:
                small_total += 1
        for prediction in predictions:
            class_name = CLASS_NAMES[prediction["label"] - 1]
            per_class[class_name]["predicted"] += 1
            ranked = sorted(
                (
                    (iou(prediction["bbox_xyxy"], truths[index]["bbox_xyxy"]), index)
                    for index in unused_truth
                    if truths[index]["label"] == prediction["label"]
                ),
                reverse=True,
            )
            if ranked and ranked[0][0] >= 0.5:
                truth_index = ranked[0][1]
                unused_truth.remove(truth_index)
                matched_total += 1
                per_class[class_name]["matched"] += 1
                if truths[truth_index]["small_lt18"]:
                    small_matched += 1
    false_positives = predicted_total - matched_total
    recall = matched_total / max(truth_total, 1)
    precision = matched_total / max(predicted_total, 1)
    class_metrics: dict[str, dict] = {}
    f1_values: list[float] = []
    for name, counts in per_class.items():
        class_recall = counts["matched"] / max(counts["truth"], 1)
        class_precision = counts["matched"] / max(counts["predicted"], 1)
        class_f1 = 2 * class_recall * class_precision / max(class_recall + class_precision, 1e-12)
        f1_values.append(class_f1)
        class_metrics[name] = {
            **counts,
            "recall": class_recall,
            "precision": class_precision,
            "f1": class_f1,
        }
    return {
        "threshold": threshold,
        "frame_count": len(raw),
        "truth_count": truth_total,
        "prediction_count": predicted_total,
        "matched_correct_class_iou_0_5": matched_total,
        "false_positive_count": false_positives,
        "recall": recall,
        "precision": precision,
        "macro_f1": sum(f1_values) / len(f1_values),
        "per_class": class_metrics,
        "metal_can_recall": class_metrics["metal_can"]["recall"],
        "paper_litter_precision": class_metrics["paper_litter"]["precision"],
        "false_positive_per_frame": false_positives / max(len(raw), 1),
        "small_lt18": {
            "truth": small_total,
            "matched": small_matched,
            "recall": small_matched / max(small_total, 1),
            "target": 0.75,
            "target_pass": small_matched / max(small_total, 1) >= 0.75,
        },
    }


def apply_gates(item: dict) -> dict:
    gates = {
        "recall_at_least_0_95": item["recall"] >= 0.95,
        "precision_at_least_0_95": item["precision"] >= 0.95,
        "macro_f1_at_least_0_95": item["macro_f1"] >= 0.95,
        "each_class_recall_at_least_0_90": all(row["recall"] >= 0.90 for row in item["per_class"].values()),
        "metal_can_recall_at_least_0_90": item["metal_can_recall"] >= 0.90,
        "paper_litter_precision_at_least_0_95": item["paper_litter_precision"] >= 0.95,
        "false_positive_per_frame_at_most_0_05": item["false_positive_per_frame"] <= 0.05,
    }
    distance = (
        max(0.0, 0.95 - item["recall"])
        + max(0.0, 0.95 - item["precision"])
        + max(0.0, 0.95 - item["macro_f1"])
        + sum(max(0.0, 0.90 - row["recall"]) for row in item["per_class"].values())
        + max(0.0, 0.90 - item["metal_can_recall"])
        + max(0.0, 0.95 - item["paper_litter_precision"])
        + max(0.0, item["false_positive_per_frame"] - 0.05)
    )
    return {**item, "gates": gates, "all_required_gates_pass": all(gates.values()), "gate_distance": distance}


def select_threshold(raw: list[dict]) -> tuple[dict, list[dict]]:
    sweep = [apply_gates(metrics(raw, threshold)) for threshold in THRESHOLDS]
    selected = min(
        sweep,
        key=lambda item: (
            not item["all_required_gates_pass"],
            item["gate_distance"],
            -item["macro_f1"],
            -item["small_lt18"]["recall"],
            -item["recall"],
            item["threshold"],
        ),
    )
    return selected, sweep


def best_checkpoint(route_dir: Path) -> Path:
    checkpoints = sorted(route_dir.glob("best_coco_bbox_mAP_epoch_*.pth"))
    if len(checkpoints) != 1:
        raise RuntimeError(f"expected one holdout-selected checkpoint in {route_dir}, found {len(checkpoints)}")
    return checkpoints[0]


def init_route(route_dir: Path):
    from mmdet.apis import init_detector

    return init_detector(
        str(route_dir / "ddrv4_d1_rtmdet_s_config.py"),
        str(best_checkpoint(route_dir)),
        device="cuda:0",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g7-root", required=True, type=Path)
    parser.add_argument("--prepared", required=True, type=Path)
    parser.add_argument("--training-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    require_ddrv4_selection_inputs([G7_DATASET_ID])
    if args.output.exists():
        raise FileExistsError(f"D1 screen output exists: {args.output}")
    prep_report = json.loads((args.prepared / "D1_PREP_REPORT.json").read_text(encoding="utf-8"))
    if prep_report.get("dataset_id") != G7_DATASET_ID or prep_report.get("untouched_val_used_for_selection") is not False:
        raise RuntimeError("D1 prepared boundary is invalid")
    args.output.mkdir(parents=True, exist_ok=False)
    patch_mmdet_cuda_nms()
    started = time.perf_counter()

    # Stage 1: both routes and all thresholds are compared using holdout only.
    holdout_frames = load_truth(args.prepared / "holdout.json", args.g7_root)
    routes: dict[str, dict] = {}
    for route, directory in (("D1-A", "d1-a"), ("D1-B", "d1-b")):
        route_dir = args.training_root / directory
        checkpoint = best_checkpoint(route_dir)
        model = init_route(route_dir)
        route_selected, sweep = select_threshold(infer(model, holdout_frames, args.batch_size))
        del model
        routes[route] = {
            "checkpoint": {"path": checkpoint.name, "sha256": sha256(checkpoint)},
            "selected_operating_point": route_selected,
            "threshold_sweep": sweep,
        }
    selected_route = min(
        routes,
        key=lambda route: (
            not routes[route]["selected_operating_point"]["all_required_gates_pass"],
            routes[route]["selected_operating_point"]["gate_distance"],
            -routes[route]["selected_operating_point"]["macro_f1"],
            -routes[route]["selected_operating_point"]["small_lt18"]["recall"],
            route,
        ),
    )
    selection_path = args.output / "D1_SELECTION.json"
    selection = {
        "schema_version": 1,
        "stage": "DDRV4-03-D1-SELECTION",
        "selection_data": "G7_IN_DOMAIN_HOLDOUT_ONLY",
        "selection_annotation_sha256": sha256(args.prepared / "holdout.json"),
        "G7_VAL_read_before_selection_freeze": False,
        "G6_used": False,
        "G5_read": False,
        "G5_V2_read": False,
        "route_results": routes,
        "selected_route": selected_route,
        "selected_threshold": routes[selected_route]["selected_operating_point"]["threshold"],
        "selection_frozen_unix_ns": time.time_ns(),
    }
    atomic_json(selection_path, selection)
    frozen_selection = json.loads(selection_path.read_text(encoding="utf-8"))
    if frozen_selection != selection:
        raise RuntimeError("D1 selection freeze failed round-trip verification")

    # Stage 2: after the selection freeze, only the selected route sees VAL once.
    selected_directory = "d1-a" if selected_route == "D1-A" else "d1-b"
    selected_model = init_route(args.training_root / selected_directory)
    val_frames = load_truth(args.prepared / "val.json", args.g7_root)
    val_metrics = apply_gates(
        metrics(
            infer(selected_model, val_frames, args.batch_size),
            float(selection["selected_threshold"]),
        )
    )
    report = {
        "schema_version": 1,
        "stage": "DDRV4-03-D1-STATIC-GATE",
        "route": selected_route,
        "checkpoint": routes[selected_route]["checkpoint"],
        "threshold": selection["selected_threshold"],
        "selection_freeze": {"path": selection_path.name, "sha256": sha256(selection_path)},
        "selection_policy": "route and threshold selected only on G7 in-domain holdout; selected route evaluated once on untouched G7 cross-world VAL",
        "G7_VAL_candidate_count": 1,
        "G7_VAL_evaluation_count": 1,
        "G6_used": False,
        "G5_read": False,
        "G5_V2_read": False,
        "VAL": val_metrics,
        "D1_STATIC_PASS": val_metrics["all_required_gates_pass"],
        "small_target_is_reported_not_required": True,
        "duration_s": time.perf_counter() - started,
        "next_action": "freeze_static_candidate_and_start_online_development" if val_metrics["all_required_gates_pass"] else "start_DDRV4_D2",
    }
    atomic_json(args.output / "D1_STATIC_REPORT.json", report)
    return 0 if report["D1_STATIC_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
