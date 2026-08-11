#!/usr/bin/env python3
"""Run frozen OPR-C and produce the DDRV4-02 detector failure taxonomy."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.ddrv4_boundary import G6_DATASET_ID, G7_DATASET_ID, require_ddrv4_selection_inputs  # noqa: E402
from sanitation_learning.opr_c_rtmdet import CLASS_NAMES, patch_mmdet_cuda_nms  # noqa: E402


CATEGORIES = (
    "NO_PROPOSAL", "SCORE_BELOW_THRESHOLD", "BOX_IOU_FAIL", "WRONG_CLASS",
    "DUPLICATE_NMS", "BACKGROUND_CONFUSION", "OCCLUDED",
    "OUT_OF_EFFECTIVE_RANGE", "ANNOTATION_QA_FAILURE",
)
OPERATING_THRESHOLD = 0.30
PROPOSAL_THRESHOLD = 0.001


def iou(first: list[float], second: list[float]) -> float:
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, float(first[2]) - float(first[0])) * max(0.0, float(first[3]) - float(first[1]))
    second_area = max(0.0, float(second[2]) - float(second[0])) * max(0.0, float(second[3]) - float(second[1]))
    return intersection / max(first_area + second_area - intersection, 1e-12)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def instance_index(rows: list[dict]) -> dict[tuple[int, int], list[dict]]:
    result: dict[tuple[int, int], list[dict]] = defaultdict(list)
    for row in rows:
        if row.get("class_id") not in CLASS_NAMES:
            continue
        result[(int(row["scene_seed"]), int(row["frame_index"]))].append(row)
    return dict(result)


def normalize_truth(item: dict) -> dict:
    short = int(item["bbox_short_side_px"])
    distance = float(item.get("distance_m", item.get("depth_m", 0.0)))
    occlusion = item.get("occlusion_metadata")
    if isinstance(occlusion, dict):
        partial = bool(occlusion.get("partial", False))
    else:
        partial = bool(occlusion)
    label = item.get("class_index")
    if label is None:
        label = CLASS_NAMES.index(item["class_id"]) + 1
    return {
        "bbox_xyxy": [float(value) for value in item["bbox_xyxy"]],
        "label": int(label),
        "class_id": item["class_id"],
        "size": "small_lt18" if short < 18 else ("medium_18_48" if short <= 48 else "large_gt48"),
        "distance": "near_0_3m" if distance <= 3 else ("mid_3_5m" if distance <= 5 else "far_gt5m"),
        "distance_m": distance,
        "material": item.get("material", item.get("material_id", item.get("metal_domain", "unknown"))),
        "lighting": item.get("lighting", item.get("lighting_id", "unknown")),
        "world": item["world_id"],
        "occluded": partial,
        "annotation_valid": len(item.get("bbox_xyxy", [])) == 4 and int(item.get("visible_pixels", item.get("mask_area_px", 0))) > 0,
    }


def classify_truth(truth: dict, predictions: list[dict], threshold: float = OPERATING_THRESHOLD) -> str:
    if not truth["annotation_valid"]:
        return "ANNOTATION_QA_FAILURE"
    scored = [item for item in predictions if float(item["score"]) >= PROPOSAL_THRESHOLD]
    correct = [(iou(truth["bbox_xyxy"], item["bbox_xyxy"]), item) for item in scored if int(item["label"]) == truth["label"]]
    any_class = [(iou(truth["bbox_xyxy"], item["bbox_xyxy"]), item) for item in scored]
    best_correct = max(correct, default=(0.0, None), key=lambda pair: pair[0])
    best_any = max(any_class, default=(0.0, None), key=lambda pair: pair[0])
    if best_correct[0] >= 0.5 and float(best_correct[1]["score"]) >= threshold:
        return "MATCH"
    if best_correct[0] >= 0.5:
        return "SCORE_BELOW_THRESHOLD"
    if best_any[0] >= 0.5 and int(best_any[1]["label"]) != truth["label"]:
        return "WRONG_CLASS"
    if best_correct[0] >= 0.1:
        return "BOX_IOU_FAIL"
    if truth["occluded"]:
        return "OCCLUDED"
    if truth["distance_m"] > 6.0 or (truth["size"] == "small_lt18" and truth["distance_m"] > 5.0):
        return "OUT_OF_EFFECTIVE_RANGE"
    return "NO_PROPOSAL"


def aggregate_frame(truths: list[dict], predictions: list[dict], backgrounds: list[str], threshold: float = OPERATING_THRESHOLD) -> dict:
    events = []
    matched_prediction_ids: set[int] = set()
    for truth_index, truth in enumerate(truths):
        category = classify_truth(truth, predictions, threshold)
        events.append({"kind": "truth", "truth_index": truth_index, "category": category, **{key: truth[key] for key in ("class_id", "size", "distance", "material", "lighting", "world")}})
        if category == "MATCH":
            candidates = sorted(((iou(truth["bbox_xyxy"], item["bbox_xyxy"]), index) for index, item in enumerate(predictions) if item["score"] >= threshold and item["label"] == truth["label"]), reverse=True)
            if candidates and candidates[0][0] >= 0.5:
                matched_prediction_ids.add(candidates[0][1])
    for index, prediction in enumerate(predictions):
        if prediction["score"] < threshold or index in matched_prediction_ids:
            continue
        overlaps = [iou(truth["bbox_xyxy"], prediction["bbox_xyxy"]) for truth in truths]
        events.append({"kind": "prediction", "category": "DUPLICATE_NMS" if overlaps and max(overlaps) >= 0.5 else "BACKGROUND_CONFUSION", "background_taxonomy": backgrounds or ["unclassified_background"]})
    return {"events": events}


def summarize(dataset_id: str, split: str, frames: list[dict]) -> dict:
    truth_total = matches = prediction_total = false_predictions = proposal_matches = 0
    taxonomy = Counter({name: 0 for name in CATEGORIES})
    dimensions = {name: defaultdict(lambda: Counter({item: 0 for item in CATEGORIES})) for name in ("class_id", "size", "distance", "material", "lighting", "world", "background_taxonomy")}
    for frame in frames:
        truths, predictions = frame["truth"], frame["predictions"]
        result = aggregate_frame(truths, predictions, frame.get("background_taxonomy", []))
        truth_total += len(truths)
        prediction_total += sum(item["score"] >= OPERATING_THRESHOLD for item in predictions)
        for truth in truths:
            if any(item["score"] >= PROPOSAL_THRESHOLD and iou(truth["bbox_xyxy"], item["bbox_xyxy"]) >= 0.5 for item in predictions):
                proposal_matches += 1
        for event in result["events"]:
            category = event["category"]
            if event["kind"] == "truth":
                if category == "MATCH":
                    matches += 1
                    continue
                taxonomy[category] += 1
                for dimension in ("class_id", "size", "distance", "material", "lighting", "world"):
                    dimensions[dimension][str(event[dimension])][category] += 1
            else:
                false_predictions += 1
                taxonomy[category] += 1
                for value in event["background_taxonomy"]:
                    dimensions["background_taxonomy"][str(value)][category] += 1
    precision = matches / max(matches + false_predictions, 1)
    recall = matches / max(truth_total, 1)
    return {
        "dataset_id": dataset_id, "split": split, "frame_count": len(frames),
        "truth_count": truth_total, "prediction_count_at_threshold": prediction_total,
        "matched_correct_class": matches, "precision": precision, "recall": recall,
        "f1": 2 * precision * recall / max(precision + recall, 1e-12),
        "proposal_recall_at_0_001": proposal_matches / max(truth_total, 1),
        "false_prediction_count": false_predictions,
        "false_positive_per_frame": false_predictions / max(len(frames), 1),
        "failure_taxonomy": dict(taxonomy),
        "dimensions": {name: {key: dict(value) for key, value in sorted(rows.items())} for name, rows in dimensions.items()},
    }


def infer_split(model, root: Path, frame_manifest: str, instance_manifest: str, split: str, batch_size: int) -> list[dict]:
    from mmdet.apis import inference_detector

    rows = [item for item in load_jsonl(root / frame_manifest) if item["split"] == split]
    instances = instance_index(load_jsonl(root / instance_manifest))
    output = []
    for offset in range(0, len(rows), batch_size):
        batch = rows[offset : offset + batch_size]
        results = inference_detector(model, [str(root / row["rgb_path"]) for row in batch])
        if not isinstance(results, list):
            results = [results]
        for row, result in zip(batch, results):
            prediction = result.pred_instances.to("cpu")
            key = (int(row["scene_seed"]), int(row["frame_index"]))
            output.append({
                "truth": [normalize_truth(item) for item in instances.get(key, [])],
                "predictions": [{"bbox_xyxy": box, "score": float(score), "label": int(label) + 1} for box, score, label in zip(prediction.bboxes.tolist(), prediction.scores.tolist(), prediction.labels.tolist())],
                "background_taxonomy": row.get("negative_taxonomies", row.get("negative_area_taxonomies", [])),
            })
    return output


def hypotheses(results: dict[str, dict]) -> dict:
    g6, holdout, val = results["G6_HISTORICAL_VAL"], results["G7_IN_DOMAIN_HOLDOUT"], results["G7_CROSS_WORLD_VAL"]
    metal = val["dimensions"]["class_id"].get("metal_can", {})
    metal_total = sum(metal.values())
    score_share = metal.get("SCORE_BELOW_THRESHOLD", 0) / max(metal_total, 1)
    background = val["dimensions"]["background_taxonomy"]
    named = sum(row.get("BACKGROUND_CONFUSION", 0) for name, row in background.items() if name != "unclassified_background")
    all_confusions = val["failure_taxonomy"].get("BACKGROUND_CONFUSION", 0)
    small_misses = sum(val["dimensions"]["size"].get("small_lt18", {}).values())
    object_misses = sum(val["failure_taxonomy"].get(name, 0) for name in CATEGORIES if name not in {"BACKGROUND_CONFUSION", "DUPLICATE_NMS"})
    return {
        "H1_domain_mismatch": {"status": "supported" if holdout["recall"] - val["recall"] >= 0.05 or g6["recall"] - val["recall"] >= 0.05 else "inconclusive", "evidence": {"G6_VAL_recall": g6["recall"], "G7_holdout_recall": holdout["recall"], "G7_VAL_recall": val["recall"]}},
        "H2_metal_score_calibration_appearance_shift": {"status": "supported" if metal_total and score_share >= 0.5 else ("rejected" if metal_total else "inconclusive"), "evidence": {"metal_failure_count": metal_total, "score_below_threshold_share": score_share}},
        "H3_false_positives_wet_specular_paint_clutter": {"status": "supported" if all_confusions and named / all_confusions >= 0.5 else ("rejected" if all_confusions else "inconclusive"), "evidence": {"named_taxonomy_confusions": named, "all_background_confusions": all_confusions}},
        "H4_small_object_secondary": {"status": "supported" if 0 < small_misses < object_misses - small_misses else "inconclusive", "evidence": {"small_misses": small_misses, "non_small_misses": object_misses - small_misses}},
        "H5_rtmdet_capacity_data_first": {"status": "supported" if val["proposal_recall_at_0_001"] >= 0.95 and g6["recall"] >= 0.85 else ("rejected" if val["proposal_recall_at_0_001"] < 0.80 else "inconclusive"), "evidence": {"G7_VAL_raw_proposal_recall": val["proposal_recall_at_0_001"], "G6_VAL_operating_recall": g6["recall"]}},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--g6-root", required=True, type=Path)
    parser.add_argument("--g7-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"taxonomy output already exists: {args.output}")
    require_ddrv4_selection_inputs([G7_DATASET_ID])
    expected = "833e6148f566aed60c27378c4c1f832bb0e3f7532dae780d12ce5424579e2dfa"
    if sha256(args.checkpoint) != expected:
        raise RuntimeError("OPR-C checkpoint SHA-256 mismatch")
    patch_mmdet_cuda_nms()
    from mmdet.apis import init_detector

    model = init_detector(str(args.config), str(args.checkpoint), device="cuda:0")
    started = time.perf_counter()
    raw = {
        "G6_HISTORICAL_VAL": infer_split(model, args.g6_root, "G6_FRAME_MANIFEST.jsonl", "G6_INSTANCE_RECORDS.jsonl", "val", args.batch_size),
        "G7_IN_DOMAIN_HOLDOUT": infer_split(model, args.g7_root, "G7_FRAME_MANIFEST.jsonl", "G7_INSTANCE_RECORDS.jsonl", "IN_DOMAIN_HOLDOUT", args.batch_size),
        "G7_CROSS_WORLD_VAL": infer_split(model, args.g7_root, "G7_FRAME_MANIFEST.jsonl", "G7_INSTANCE_RECORDS.jsonl", "CROSS_WORLD_VAL", args.batch_size),
    }
    results = {
        "G6_HISTORICAL_VAL": summarize(G6_DATASET_ID, "val", raw["G6_HISTORICAL_VAL"]),
        "G7_IN_DOMAIN_HOLDOUT": summarize(G7_DATASET_ID, "IN_DOMAIN_HOLDOUT", raw["G7_IN_DOMAIN_HOLDOUT"]),
        "G7_CROSS_WORLD_VAL": summarize(G7_DATASET_ID, "CROSS_WORLD_VAL", raw["G7_CROSS_WORLD_VAL"]),
    }
    report = {
        "schema_version": 1, "stage": "DDRV4-02", "route": "OPR-C_FROZEN_HISTORICAL_DIAGNOSTIC",
        "checkpoint": {"sha256": expected}, "operating_threshold": OPERATING_THRESHOLD,
        "proposal_threshold": PROPOSAL_THRESHOLD, "categories": list(CATEGORIES),
        "results": results, "hypotheses": hypotheses(results),
        "data_policy": {"G6": "historical regression only; never DDRV4 selection", "G7_holdout": "diagnostic input for frozen historical OPR-C", "G7_VAL": "old-model taxonomy only; forbidden for D1 selection", "G5_read": False, "G5_V2_read": False},
        "duration_s": time.perf_counter() - started, "DDRV4_02_COMPLETE": True,
        "next_action": "run DDRV4-D1 A/B using G7 TRAIN and IN_DOMAIN_HOLDOUT only",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
