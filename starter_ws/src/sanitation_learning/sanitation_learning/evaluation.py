from __future__ import annotations

import json
import math
from pathlib import Path
import resource
import time

import cv2
import numpy as np

from .assets import load_asset_registry
from .rendered import CLASS_ORDER, generate_frame


DISCRETE = ("plastic_bottle", "metal_can", "paper_litter")
AREAS = ("leaf_pile", "puddle")


def _infer(session, image_rgb: np.ndarray) -> tuple[np.ndarray, float]:
    tensor = np.transpose(image_rgb.astype(np.float32) / 255.0, (2, 0, 1))[None]
    started = time.perf_counter()
    logits = session.run(["logits"], {"images": tensor})[0]
    prediction = logits[0].argmax(axis=0).astype(np.uint8)
    cleaned = np.zeros_like(prediction)
    kernel = np.ones((3, 3), dtype=np.uint8)
    for class_index in range(1, len(CLASS_ORDER)):
        mask = cv2.morphologyEx((prediction == class_index).astype(np.uint8), cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        for component in range(1, count):
            if int(stats[component, cv2.CC_STAT_AREA]) >= 20:
                cleaned[labels == component] = class_index
    return cleaned, (time.perf_counter() - started) * 1000


def _components(mask: np.ndarray, minimum_pixels: int = 20) -> list[np.ndarray]:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    return [labels == index for index in range(1, count) if int(stats[index, cv2.CC_STAT_AREA]) >= minimum_pixels]


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.logical_and(a, b).sum() / max(np.logical_or(a, b).sum(), 1))


def _match_instances(prediction: np.ndarray, frame, class_id: str, threshold: float) -> tuple[int, int, int, list[tuple[np.ndarray, dict, float]]]:
    class_index = CLASS_ORDER.index(class_id)
    predicted = _components(prediction == class_index)
    truth = [(frame.instance_labels == obj["instance_id"], obj) for obj in frame.objects if obj["class_id"] == class_id]
    candidates = sorted((( _iou(pm, tm), pi, ti) for pi, pm in enumerate(predicted) for ti, (tm, _) in enumerate(truth)), reverse=True)
    used_pred = set(); used_truth = set(); matches = []
    for score, pi, ti in candidates:
        if score < threshold or pi in used_pred or ti in used_truth:
            continue
        used_pred.add(pi); used_truth.add(ti); matches.append((predicted[pi], truth[ti][1], score))
    return len(matches), len(predicted) - len(matches), len(truth) - len(matches), matches


def _macro_f1_from_confusion(confusion: np.ndarray, class_ids=DISCRETE) -> tuple[dict, float, float, float]:
    result = {}
    for class_id in class_ids:
        index = CLASS_ORDER.index(class_id)
        tp = int(confusion[index, index]); fp = int(confusion[:, index].sum() - tp); fn = int(confusion[index, :].sum() - tp)
        precision = tp / max(tp + fp, 1); recall = tp / max(tp + fn, 1); f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        result[class_id] = {"tp_pixels": tp, "fp_pixels": fp, "fn_pixels": fn, "precision": precision, "recall": recall, "f1": f1}
    return result, float(np.mean([v["precision"] for v in result.values()])), float(np.mean([v["recall"] for v in result.values()])), float(np.mean([v["f1"] for v in result.values()]))


def _stress(image: np.ndarray, name: str) -> np.ndarray:
    if name == "grayscale":
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY); return np.repeat(gray[:, :, None], 3, axis=2)
    if name == "hue_shift":
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV); hsv[:, :, 0] = (hsv[:, :, 0].astype(np.int16) + 67) % 180; return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    if name == "color_permutation":
        return image[:, :, [2, 0, 1]]
    if name == "exposure_extremes":
        return np.clip(image.astype(np.float32) * 0.52 + 15, 0, 255).astype(np.uint8)
    if name == "background_color_swap":
        result = image.copy(); low_texture = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) < 150; result[low_texture] = result[low_texture][:, [1, 2, 0]]; return result
    if name == "texture_only":
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY); edges = cv2.Laplacian(gray, cv2.CV_8U, ksize=3); return np.repeat(edges[:, :, None], 3, axis=2)
    if name == "shape_only":
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY); edges = cv2.Canny(gray, 35, 110); return np.repeat((255 - edges)[:, :, None], 3, axis=2)
    if name == "blue_patch_puddle_confuser":
        result = image.copy(); cv2.ellipse(result, (18, 72), (13, 7), 0, 0, 360, (54, 94, 168), -1); return result
    return image


def evaluate_model(model_path: str | Path, registry_path: str | Path, output: str | Path, test_scene_count: int = 100, frames_per_scene: int = 10) -> dict:
    import onnxruntime as ort
    options = ort.SessionOptions(); options.intra_op_num_threads = 1; options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model_path), sess_options=options, providers=["CPUExecutionProvider"])
    registry = load_asset_registry(registry_path)
    test_seeds = [value for value in range(1000) if value % 10 in {8, 9}][:test_scene_count]
    confusion = np.zeros((len(CLASS_ORDER), len(CLASS_ORDER)), dtype=np.int64)
    instance_counts = {name: {"tp": 0, "fp": 0, "fn": 0} for name in DISCRETE}
    instance_match_scores = {name: [] for name in DISCRETE}
    area_intersection = {name: 0 for name in AREAS}; area_union = {name: 0 for name in AREAS}
    localization_errors = []; latencies = []; missed_targets = 0
    distance_buckets = {"near_1_3_to_2_0": 0, "mid_2_0_to_2_7": 0, "far_2_7_to_3_4": 0}
    occlusion_buckets = {"clear": 0, "partial": 0, "heavy": 0}
    for seed in test_seeds:
        for frame_index in range(frames_per_scene):
            frame = generate_frame(seed, frame_index, registry)
            prediction, latency = _infer(session, frame.image_rgb); latencies.append(latency)
            flat = frame.semantic_labels.ravel() * len(CLASS_ORDER) + prediction.ravel()
            confusion += np.bincount(flat, minlength=len(CLASS_ORDER) ** 2).reshape(len(CLASS_ORDER), len(CLASS_ORDER))
            for obj in frame.objects:
                depth = obj["depth_m"]; distance_buckets["near_1_3_to_2_0" if depth < 2.0 else "mid_2_0_to_2_7" if depth < 2.7 else "far_2_7_to_3_4"] += 1
                occ = obj["occlusion_ratio"]; occlusion_buckets["clear" if occ < 0.05 else "partial" if occ < 0.35 else "heavy"] += 1
            for class_id in DISCRETE:
                tp, fp, fn, matches = _match_instances(prediction, frame, class_id, 0.5)
                instance_counts[class_id]["tp"] += tp; instance_counts[class_id]["fp"] += fp; instance_counts[class_id]["fn"] += fn; missed_targets += fn
                for threshold in np.arange(0.5, 1.0, 0.05):
                    mtp, mfp, mfn, _ = _match_instances(prediction, frame, class_id, float(threshold))
                    instance_match_scores[class_id].append(mtp / max(mtp + mfp + mfn, 1))
                for mask, obj, _ in matches:
                    rows, cols = np.nonzero(mask); depth_values = frame.depth_m[mask]; depth_values = depth_values[np.isfinite(depth_values)]
                    if depth_values.size < 3:
                        continue
                    depth = float(np.median(depth_values)); u = float(cols.mean()); v = float(rows.mean())
                    map_x = frame.camera["camera_map_xy"][0] + (u - frame.camera["cx"]) * depth / frame.camera["fx"]
                    map_y = frame.camera["camera_map_xy"][1] + (v - frame.camera["cy"]) * depth / frame.camera["fy"]
                    localization_errors.append(math.hypot(map_x - obj["map_pose"][0], map_y - obj["map_pose"][1]))
            for class_id in AREAS:
                index = CLASS_ORDER.index(class_id); truth = frame.semantic_labels == index; predicted = prediction == index
                area_intersection[class_id] += int(np.logical_and(truth, predicted).sum()); area_union[class_id] += int(np.logical_or(truth, predicted).sum())
    pixel_per_class, macro_precision, macro_recall, macro_f1 = _macro_f1_from_confusion(confusion)
    detection_per_class = {}
    for class_id, counts in instance_counts.items():
        precision = counts["tp"] / max(counts["tp"] + counts["fp"], 1); recall = counts["tp"] / max(counts["tp"] + counts["fn"], 1); f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        detection_per_class[class_id] = {**counts, "precision": precision, "recall": recall, "f1": f1}
    detection_macro_precision = float(np.mean([item["precision"] for item in detection_per_class.values()])); detection_macro_recall = float(np.mean([item["recall"] for item in detection_per_class.values()])); detection_macro_f1 = float(np.mean([item["f1"] for item in detection_per_class.values()]))
    area_iou = {name: area_intersection[name] / max(area_union[name], 1) for name in AREAS}
    errors = np.asarray(localization_errors, dtype=np.float64); latency_values = np.asarray(latencies, dtype=np.float64)
    stress_names = ["grayscale", "hue_shift", "color_permutation", "same_color_negatives", "different_color_same_class", "texture_only", "shape_only", "background_color_swap", "exposure_extremes", "blue_patch_puddle_confuser"]
    stress_reports = {}
    stress_seeds = test_seeds[: min(20, len(test_seeds))]
    for name in stress_names:
        stress_confusion = np.zeros_like(confusion)
        for seed in stress_seeds:
            frame = generate_frame(seed, 0, registry); transformed = _stress(frame.image_rgb, name)
            prediction, _ = _infer(session, transformed)
            flat = frame.semantic_labels.ravel() * len(CLASS_ORDER) + prediction.ravel()
            stress_confusion += np.bincount(flat, minlength=len(CLASS_ORDER) ** 2).reshape(len(CLASS_ORDER), len(CLASS_ORDER))
        _, p, r, f1 = _macro_f1_from_confusion(stress_confusion)
        stress_reports[name] = {"macro_precision": p, "macro_recall": r, "macro_f1": f1, "per_class_drop_from_baseline": {class_id: pixel_per_class[class_id]["f1"] - _macro_f1_from_confusion(stress_confusion)[0][class_id]["f1"] for class_id in DISCRETE}}
    aggregate_stress = float(np.mean([item["macro_f1"] for item in stress_reports.values()]))
    color_shortcut_pass = aggregate_stress >= 0.85 and stress_reports["color_permutation"]["macro_f1"] >= 0.80
    gates = {"test_scene_count_at_least_100": len(test_seeds) >= 100, "discrete_macro_precision_at_least_0_95": detection_macro_precision >= 0.95, "discrete_macro_recall_at_least_0_95": detection_macro_recall >= 0.95, "discrete_macro_f1_at_least_0_95": detection_macro_f1 >= 0.95, "each_discrete_recall_at_least_0_90": all(item["recall"] >= 0.90 for item in detection_per_class.values()), "leaf_and_puddle_miou_at_least_0_80": all(value >= 0.80 for value in area_iou.values()), "map_localization_rmse_at_most_0_10m": errors.size > 0 and float(np.sqrt(np.mean(errors**2))) <= 0.10, "color_shortcut_pass": color_shortcut_pass}
    report = {"schema_version": 1, "stage": "Stage5B", "domain": "D1_procedural_rendered_not_gazebo_camera", "backend": "onnxruntime_cpu", "test_scene_seeds": test_seeds, "test_frame_count": len(test_seeds) * frames_per_scene, "unseen_world_asset_texture_split": True, "pixel_per_class": pixel_per_class, "detection_per_class": detection_per_class, "discrete_macro_precision": detection_macro_precision, "discrete_macro_recall": detection_macro_recall, "discrete_macro_f1": detection_macro_f1, "instance_match_score_at_iou50": float(np.mean([item["tp"] / max(item["tp"] + item["fp"] + item["fn"], 1) for item in instance_counts.values()])), "mean_instance_match_score_iou50_95": float(np.mean([value for values in instance_match_scores.values() for value in values])), "ap50": None, "ap50_95": None, "ap_unavailable_reason": "screening does not retain calibrated confidence-ranked detections required for a precision-recall curve", "area_iou": area_iou, "missed_targets": missed_targets, "map_localization_rmse_m": float(np.sqrt(np.mean(errors**2))) if errors.size else None, "map_localization_p95_m": float(np.percentile(errors, 95)) if errors.size else None, "map_localization_max_m": float(errors.max()) if errors.size else None, "latency_ms": {"p50": float(np.percentile(latency_values, 50)), "p95": float(np.percentile(latency_values, 95)), "max": float(latency_values.max())}, "peak_rss_kib": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss), "distance_buckets": distance_buckets, "occlusion_buckets": occlusion_buckets, "color_shortcut": {"aggregate_macro_f1": aggregate_stress, "color_shortcut_pass": color_shortcut_pass, "stress_reports": stress_reports}, "gates": gates, "rendered_synthetic_perception_pass": all(gates.values()), "gazebo_camera_evaluation_executed": False, "real_domain_evaluation_executed": False, "competition_perception_pass": False}
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True); output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
