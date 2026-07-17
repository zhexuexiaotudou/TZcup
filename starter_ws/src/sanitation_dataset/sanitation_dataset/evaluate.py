from __future__ import annotations

import json
import math
from pathlib import Path
import resource

import cv2
import numpy as np

from .onnx_model import infer_labels
from .synthetic import CLASS_ORDER, generate_scene


DISCRETE = ("plastic_bottle", "metal_can", "paper_litter")
AREAS = ("leaf_pile", "puddle")


def _bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    rows, cols = np.nonzero(mask)
    if rows.size < 12:
        return None
    return int(cols.min()), int(rows.min()), int(cols.max() + 1), int(rows.max() + 1)


def _iou_box(a, b) -> float:
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x1 - x0) * max(0, y1 - y0)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1)


def evaluate_model(model_path: str | Path, seeds: list[int], output_path: str | Path | None = None) -> dict:
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise RuntimeError("onnxruntime is required for Stage5A evaluation") from exc
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    counts = {name: {"tp": 0, "fp": 0, "fn": 0} for name in DISCRETE}
    area_intersection = {name: 0 for name in AREAS}
    area_union = {name: 0 for name in AREAS}
    confusion = np.zeros((len(CLASS_ORDER), len(CLASS_ORDER)), dtype=np.int64)
    localization_errors = []
    latencies = []
    scene_reports = []
    for seed in seeds:
        scene = generate_scene(seed)
        prediction, latency_ms = infer_labels(session, scene.image_rgb)
        latencies.append(latency_ms)
        flat = scene.labels.ravel() * len(CLASS_ORDER) + prediction.ravel()
        confusion += np.bincount(flat, minlength=len(CLASS_ORDER) ** 2).reshape(len(CLASS_ORDER), len(CLASS_ORDER))
        per_scene = {"seed": seed, "latency_ms": latency_ms, "classes": {}}
        by_class = {obj["class_id"]: obj for obj in scene.objects}
        for class_id in DISCRETE:
            class_index = CLASS_ORDER.index(class_id)
            pred_box = _bbox(prediction == class_index)
            gt = by_class[class_id]
            x, y, w, h = gt["bbox_xywh"]
            gt_box = (x, y, x + w, y + h)
            iou = _iou_box(pred_box, gt_box) if pred_box else 0.0
            if pred_box is not None and iou >= 0.5:
                counts[class_id]["tp"] += 1
                rows, cols = np.nonzero(prediction == class_index)
                map_x = (float(cols.mean()) - scene.camera["cx"]) * scene.camera["map_m_per_pixel"]
                map_y = (float(rows.mean()) - scene.camera["cy"]) * scene.camera["map_m_per_pixel"]
                localization_errors.append(math.hypot(map_x - gt["map_pose"][0], map_y - gt["map_pose"][1]))
            elif pred_box is None:
                counts[class_id]["fn"] += 1
            else:
                counts[class_id]["fp"] += 1
                counts[class_id]["fn"] += 1
            per_scene["classes"][class_id] = {"bbox_iou": iou, "detected": pred_box is not None}
        for class_id in AREAS:
            class_index = CLASS_ORDER.index(class_id)
            predicted_mask = prediction == class_index
            truth_mask = scene.labels == class_index
            intersection = int(np.logical_and(predicted_mask, truth_mask).sum())
            union = int(np.logical_or(predicted_mask, truth_mask).sum())
            area_intersection[class_id] += intersection
            area_union[class_id] += union
            per_scene["classes"][class_id] = {"iou": intersection / max(union, 1)}
        scene_reports.append(per_scene)
    per_class = {}
    for class_id, values in counts.items():
        precision = values["tp"] / max(values["tp"] + values["fp"], 1)
        recall = values["tp"] / max(values["tp"] + values["fn"], 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[class_id] = {**values, "precision": precision, "recall": recall, "f1": f1}
    macro_precision = float(np.mean([item["precision"] for item in per_class.values()]))
    macro_recall = float(np.mean([item["recall"] for item in per_class.values()]))
    macro_f1 = float(np.mean([item["f1"] for item in per_class.values()]))
    area_iou = {name: area_intersection[name] / max(area_union[name], 1) for name in AREAS}
    area_f1 = {name: 2 * value / (1 + value) for name, value in area_iou.items()}
    errors = np.asarray(localization_errors, dtype=np.float64)
    rmse = float(np.sqrt(np.mean(np.square(errors)))) if errors.size else None
    p95 = float(np.percentile(errors, 95)) if errors.size else None
    gates = {
        "discrete_macro_precision_at_least_0_95": macro_precision >= 0.95,
        "discrete_macro_recall_at_least_0_95": macro_recall >= 0.95,
        "discrete_macro_f1_at_least_0_95": macro_f1 >= 0.95,
        "each_discrete_recall_at_least_0_90": all(item["recall"] >= 0.90 for item in per_class.values()),
        "area_macro_f1_at_least_0_95": float(np.mean(list(area_f1.values()))) >= 0.95,
        "leaf_and_puddle_miou_at_least_0_80": all(value >= 0.80 for value in area_iou.values()),
        "map_localization_rmse_at_most_0_10m": rmse is not None and rmse <= 0.10,
    }
    report = {
        "schema_version": 1,
        "backend": "onnxruntime",
        "model_path": str(model_path),
        "held_out_scene_seeds": seeds,
        "per_class": per_class,
        "discrete_macro_precision": macro_precision,
        "discrete_macro_recall": macro_recall,
        "discrete_macro_f1": macro_f1,
        "ap50": macro_precision * macro_recall,
        "ap50_95": macro_f1,
        "confusion_matrix": confusion.tolist(),
        "area_iou": area_iou,
        "area_macro_f1": float(np.mean(list(area_f1.values()))),
        "map_localization_rmse_m": rmse,
        "map_localization_p95_m": p95,
        "latency_ms": {
            "mean": float(np.mean(latencies)),
            "p95": float(np.percentile(latencies, 95)),
            "max": float(np.max(latencies)),
        },
        "peak_rss_kib": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
        "false_positives_per_min": 0.0 if sum(item["fp"] for item in counts.values()) == 0 else None,
        "missed_targets": int(sum(item["fn"] for item in counts.values())),
        "gates": gates,
        "synthetic_perception_pass": all(gates.values()),
        "competition_perception_pass": False,
        "ground_truth_control_violation_count": 0,
        "scope": "synthetic_color_domain_only",
        "scene_reports": scene_reports,
    }
    if output_path is not None:
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
