#!/usr/bin/env python3
"""Quantify the three MODEL-RECOVERY-V2 blockers without training.

This development-only audit reads TRAIN, the deterministic train-world
holdout, VAL and D1-D5.  It never opens G5 or legacy D6.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import platform
from pathlib import Path
import statistics
import sys
import time

import cv2
import numpy as np
import yaml

try:
    import torch
except ImportError:  # Pure audit contracts run on the Windows host without Torch.
    torch = None


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.auto04_contract import box_iou  # noqa: E402
from sanitation_learning.g4_data import (  # noqa: E402
    DISCRETE_NAMES,
    index_instance_records,
    load_frame_rows,
    load_instance_records,
    load_scene_manifests,
    mask_boundary,
)
from sanitation_learning.g4_direct_fcos import (  # noqa: E402
    DirectFCOSDataset,
    build_direct_fcos,
    direct_predictions,
)
from sanitation_learning.g4_evaluation import area_predictions  # noqa: E402
from sanitation_learning.g4_split_policy import stratified_row_sample  # noqa: E402
from perception_prod_x1_full_pipeline import (  # noqa: E402
    combine_area,
    load_checkpoint_model,
    sha256,
)
SEED = 20260810
SIZE_BINS = ("lt_8", "8_to_12", "12_to_18", "18_to_32", "32_to_48", "ge_48")
AREA_CLASSES = ("leaf_pile", "puddle")
CURRENT_AREA_THRESHOLDS = (0.85, 0.85)
DEVELOPMENT_THRESHOLDS = (0.70, 0.75, 0.80, 0.85, 0.90, 0.95)
MORPHOLOGIES = (
    "none",
    "open3",
    "close3",
    "open_close3",
    "dilate3",
    "dilate5",
    "erode3",
)


def holdout_rows(rows, fraction):
    """Exact deterministic scene selection used by perception_prod_x3_train."""
    selected = []
    for row in rows:
        token = f"{row['world_id']}:{int(row['scene_seed'])}".encode()
        if hashlib.sha256(token).digest()[0] % 100 < int(fraction * 100):
            selected.append(row)
    return selected


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def size_bin(value: float) -> str:
    if value < 8:
        return "lt_8"
    if value < 12:
        return "8_to_12"
    if value < 18:
        return "12_to_18"
    if value < 32:
        return "18_to_32"
    if value < 48:
        return "32_to_48"
    return "ge_48"


def apply_morphology(mask: np.ndarray, name: str) -> np.ndarray:
    source = mask.astype(np.uint8)
    kernel3 = np.ones((3, 3), dtype=np.uint8)
    if name == "none":
        return source.astype(bool)
    if name == "open3":
        result = cv2.morphologyEx(source, cv2.MORPH_OPEN, kernel3)
    elif name == "close3":
        result = cv2.morphologyEx(source, cv2.MORPH_CLOSE, kernel3)
    elif name == "open_close3":
        result = cv2.morphologyEx(
            cv2.morphologyEx(source, cv2.MORPH_OPEN, kernel3),
            cv2.MORPH_CLOSE,
            kernel3,
        )
    elif name == "dilate3":
        result = cv2.dilate(source, kernel3)
    elif name == "dilate5":
        result = cv2.dilate(source, np.ones((5, 5), dtype=np.uint8))
    elif name == "erode3":
        result = cv2.erode(source, kernel3)
    else:
        raise ValueError(f"unknown morphology: {name}")
    return result.astype(bool)


def has_area_candidate(mask: np.ndarray, minimum_pixels: int = 20) -> bool:
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    return bool(count > 1 and int(stats[1:, cv2.CC_STAT_AREA].max()) >= minimum_pixels)


def load_asset_metadata(path: Path) -> dict[str, dict]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    result = {}
    for class_name, records in payload.get("classes", {}).items():
        for record in records or ():
            result[str(record["id"])] = {"registry_class": class_name, **record}
    return result


def scene_object(manifest: dict, class_name: str) -> dict:
    matches = [item for item in manifest.get("objects", ()) if item.get("class_id") == class_name]
    return matches[0] if matches else {}


def metadata_for_instance(
    record: dict, manifest: dict, assets: dict[str, dict]
) -> dict:
    obj = scene_object(manifest, str(record["semantic_class"]))
    asset = assets.get(str(obj.get("asset_id", "")), {})
    palette = asset.get("palette") or ()
    return {
        "world": str(record.get("world_id") or manifest.get("world_id") or "unknown"),
        "asset": str(obj.get("asset_id") or "unknown"),
        "material": str(asset.get("material_family") or "unknown"),
        "geometry": str(asset.get("geometry_family") or "unknown"),
        "texture": str(asset.get("texture_family") or "unknown"),
        "palette": "+".join(str(value) for value in palette) or "unknown",
        "lighting": str(manifest.get("lighting_executed_by_world") or "unknown"),
        "ground": str(manifest.get("ground_material_executed_by_world") or "unknown"),
        "distance": str(obj.get("distance_bucket_m") or "unknown"),
        "distance_m": float(obj.get("distance_m", -1.0)),
        "trajectory": str(manifest.get("trajectory_id") or "unknown"),
        "occlusion": str(obj.get("occlusion_bucket") or "unknown"),
        "specular_proxy": "specular_or_glossy"
        if any(token in str(asset.get("material_family", "")).lower() for token in ("gloss", "aluminum", "metal"))
        else "matte_or_unknown",
        "background_contrast_proxy": f"{manifest.get('ground_material_executed_by_world', 'unknown')}__vs__{'+'.join(str(value) for value in palette) or 'unknown'}",
    }


def init_count_record() -> dict:
    return {name: 0 for name in SIZE_BINS}


def summarize_small_partition(
    rows: list[dict], records_by_key: dict, manifests: dict, assets: dict
) -> dict:
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in rows}
    by_class = {name: init_count_record() for name in DISCRETE_NAMES}
    dimensions = {
        name: defaultdict(init_count_record)
        for name in ("world", "asset", "material", "lighting", "distance", "trajectory")
    }
    small_frames = set()
    total = 0
    small = 0
    for key in keys:
        manifest = manifests.get(key[0], {})
        for record in records_by_key.get(key, ()):
            class_name = str(record.get("semantic_class"))
            if class_name not in DISCRETE_NAMES:
                continue
            total += 1
            shortest = float(record.get("bbox_shortest_side_px", 0.0))
            bucket = size_bin(shortest)
            by_class[class_name][bucket] += 1
            if shortest < 18:
                small += 1
                small_frames.add(key)
            metadata = metadata_for_instance(record, manifest, assets)
            for dimension in dimensions:
                dimensions[dimension][metadata[dimension]][bucket] += 1
    return {
        "frames": len(rows),
        "objects": total,
        "small_lt_18_objects": small,
        "small_lt_18_frames": len(small_frames),
        "small_frame_ratio": len(small_frames) / max(len(rows), 1),
        "by_class": by_class,
        "by_dimension": {
            dimension: dict(sorted(values.items()))
            for dimension, values in dimensions.items()
        },
    }


def filter_detections(frame: dict, threshold: float, top_k: int) -> dict:
    items = sorted(frame["predictions"], key=lambda item: float(item["score"]), reverse=True)
    items = [item for item in items if float(item["score"]) >= threshold][:top_k]
    return {**frame, "predictions": items, "detections": items}


def object_recall(frames: list[dict], *, small_only: bool = False) -> dict:
    matched = 0
    total = 0
    by_class = {name: {"matched": 0, "total": 0} for name in DISCRETE_NAMES}
    for frame in frames:
        detections = frame["detections"]
        for truth in frame["truth"]:
            if small_only and float(truth.get("native_short_side_px", 0.0)) >= 18:
                continue
            class_name = str(truth["semantic_class"])
            total += 1
            by_class[class_name]["total"] += 1
            hit = any(
                item["class_name"] == class_name
                and box_iou(tuple(truth["bbox_xyxy"]), tuple(item["bbox_xyxy"])) >= 0.5
                for item in detections
            )
            matched += int(hit)
            by_class[class_name]["matched"] += int(hit)
    for value in by_class.values():
        value["recall"] = value["matched"] / max(value["total"], 1)
    return {
        "matched": matched,
        "total": total,
        "recall": matched / max(total, 1),
        "by_class": by_class,
    }


def detection_truncation_audit(raw_frames: list[dict], threshold: float) -> dict:
    variants = {
        "raw_score_0_01_top100": [filter_detections(frame, 0.01, 100) for frame in raw_frames],
        "raw_score_0_01_top16": [filter_detections(frame, 0.01, 16) for frame in raw_frames],
        "frozen_threshold_top100": [filter_detections(frame, threshold, 100) for frame in raw_frames],
        "frozen_threshold_top16": [filter_detections(frame, threshold, 16) for frame in raw_frames],
    }
    return {
        name: {
            "all_object": object_recall(frames),
            "small_lt_18": object_recall(frames, small_only=True),
        }
        for name, frames in variants.items()
    }


def classify_metal_truth(truth: dict, raw: list[dict], selected: list[dict], threshold: float) -> str:
    def iou(item):
        return box_iou(tuple(truth["bbox_xyxy"]), tuple(item["bbox_xyxy"]))

    if any(item["class_name"] == "metal_can" and iou(item) >= 0.5 for item in selected):
        return "correct"
    if any(item["class_name"] != "metal_can" and iou(item) >= 0.5 for item in selected):
        return "detector_found_but_wrong_class"
    same = [item for item in raw if item["class_name"] == "metal_can"]
    localized = [item for item in same if iou(item) >= 0.5]
    if localized:
        best = max(localized, key=lambda item: float(item["score"]))
        if float(best["score"]) < threshold:
            return "score_below_threshold"
        return "top_k_truncated"
    if same and max(iou(item) for item in same) >= 0.1:
        return "box_iou_below_0_5"
    return "detector_missed"


def aggregate_outcomes(records: list[dict], dimension: str) -> dict:
    output = {}
    for value in sorted({str(item[dimension]) for item in records}):
        subset = [item for item in records if str(item[dimension]) == value]
        outcomes = {name: sum(item["outcome"] == name for item in subset) for name in (
            "correct", "detector_missed", "detector_found_but_wrong_class",
            "score_below_threshold", "box_iou_below_0_5", "top_k_truncated",
        )}
        output[value] = {
            "total": len(subset),
            "recall": outcomes["correct"] / max(len(subset), 1),
            "outcomes": outcomes,
        }
    return output


def metal_audit_for_frames(
    split: str, raw_frames: list[dict], threshold: float, manifests: dict, assets: dict
) -> tuple[dict, list[dict]]:
    records = []
    for frame in raw_frames:
        selected = filter_detections(frame, threshold, 16)["detections"]
        manifest = manifests.get(int(frame["scene_seed"]), {})
        for truth in frame["truth"]:
            if truth["semantic_class"] != "metal_can":
                continue
            metadata = metadata_for_instance(
                {"semantic_class": "metal_can", "world_id": frame["world_id"]},
                manifest,
                assets,
            )
            records.append({
                "split": split,
                "pixel_size": size_bin(float(truth.get("native_short_side_px", 0.0))),
                "native_short_side_px": float(truth.get("native_short_side_px", 0.0)),
                "outcome": classify_metal_truth(truth, frame["detections"], selected, threshold),
                **metadata,
            })
    dimensions = (
        "split", "world", "asset", "material", "geometry", "texture", "palette",
        "specular_proxy", "distance", "pixel_size", "occlusion", "lighting",
        "ground", "background_contrast_proxy",
    )
    return {
        "objects": len(records),
        "recall": sum(item["outcome"] == "correct" for item in records) / max(len(records), 1),
        "by_dimension": {name: aggregate_outcomes(records, name) for name in dimensions},
    }, records


def edge_stats(predicted: np.ndarray, truth: np.ndarray) -> tuple[int, int]:
    pred_edge = mask_boundary(predicted.astype(bool)) > 0
    truth_edge = mask_boundary(truth.astype(bool)) > 0
    return int((pred_edge & truth_edge).sum()), int((pred_edge | truth_edge).sum())


def empty_area_accumulator() -> dict:
    return {
        "intersection": [0, 0], "union": [0, 0],
        "boundary_intersection": [0, 0], "boundary_union": [0, 0],
        "raw_boundary_intersection": [0, 0], "raw_boundary_union": [0, 0],
        "truth_pixels": [0, 0],
        "negative_frames": 0, "negative_fp_frames": 0,
    }


def update_area_accumulator(acc: dict, frame: dict, masks: list[np.ndarray]) -> None:
    truth = frame["truth"].astype(bool)
    for channel in range(2):
        predicted = masks[channel].astype(bool)
        acc["truth_pixels"][channel] += int(truth[channel].sum())
        acc["intersection"][channel] += int((predicted & truth[channel]).sum())
        acc["union"][channel] += int((predicted | truth[channel]).sum())
        intersection, union = edge_stats(predicted, truth[channel])
        acc["boundary_intersection"][channel] += intersection
        acc["boundary_union"][channel] += union
        raw_edge = frame["boundary_probabilities"][channel] >= 0.5
        truth_edge = mask_boundary(truth[channel]) > 0
        intersection = int((raw_edge & truth_edge).sum())
        union = int((raw_edge | truth_edge).sum())
        acc["raw_boundary_intersection"][channel] += intersection
        acc["raw_boundary_union"][channel] += union
    if frame["negative_only"]:
        acc["negative_frames"] += 1
        acc["negative_fp_frames"] += int(any(has_area_candidate(mask) for mask in masks))


def finalize_area(acc: dict) -> dict:
    iou = [acc["intersection"][i] / max(acc["union"][i], 1) for i in range(2)]
    boundary = [
        2 * acc["boundary_intersection"][i]
        / max(acc["boundary_intersection"][i] + acc["boundary_union"][i], 1)
        for i in range(2)
    ]
    raw_boundary = [
        2 * acc["raw_boundary_intersection"][i]
        / max(acc["raw_boundary_intersection"][i] + acc["raw_boundary_union"][i], 1)
        for i in range(2)
    ]
    return {
        "iou_by_class": dict(zip(AREA_CLASSES, iou)),
        "macro_miou": statistics.fmean(iou),
        "postprocessed_mask_boundary_f1_by_class": dict(zip(AREA_CLASSES, boundary)),
        "postprocessed_mask_boundary_f1": statistics.fmean(boundary),
        "raw_network_boundary_head_f1_by_class": dict(zip(AREA_CLASSES, raw_boundary)),
        "raw_network_boundary_head_f1": statistics.fmean(raw_boundary),
        "positive_truth_pixels_by_class": dict(zip(AREA_CLASSES, acc["truth_pixels"])),
        "positive_metric_eligibility_by_class": {
            AREA_CLASSES[i]: "evaluated" if acc["truth_pixels"][i] else "not_applicable_no_positive_truth"
            for i in range(2)
        },
        "negative_only_frames": acc["negative_frames"],
        "negative_only_fp_frames": acc["negative_fp_frames"],
        "negative_area_fp_per_frame": acc["negative_fp_frames"] / max(acc["negative_frames"], 1),
        "pixel_totals": acc,
    }


def area_masks(frame: dict, thresholds, morphologies) -> list[np.ndarray]:
    return [
        apply_morphology(frame["probabilities"][channel] >= thresholds[channel], morphologies[channel])
        for channel in range(2)
    ]


def truth_shape_groups(mask: np.ndarray) -> dict[str, str]:
    ys, xs = np.where(mask)
    if not len(xs):
        return {}
    shortest = min(int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1))
    width = "lt_32px" if shortest < 32 else "32_to_64px" if shortest < 64 else "ge_64px"
    area = int(mask.sum())
    perimeter = int((mask_boundary(mask) > 0).sum())
    compactness = (perimeter * perimeter) / max(4.0 * math.pi * area, 1.0)
    shape = "compact" if compactness < 2.0 else "irregular" if compactness < 5.0 else "highly_irregular"
    return {"width": width, "shape": shape}


def update_group_boundary(acc: dict, predicted: np.ndarray, truth: np.ndarray, raw_edge: np.ndarray) -> None:
    intersection, union = edge_stats(predicted, truth)
    acc["boundary_intersection"] += intersection
    acc["boundary_union"] += union
    truth_edge = mask_boundary(truth) > 0
    acc["raw_intersection"] += int((raw_edge & truth_edge).sum())
    acc["raw_union"] += int((raw_edge | truth_edge).sum())
    acc["frames"] += 1


def finalize_group_boundary(groups: dict) -> dict:
    result = {}
    for dimension, buckets in groups.items():
        result[dimension] = {}
        for bucket, acc in sorted(buckets.items()):
            post_f1 = 2 * acc["boundary_intersection"] / max(acc["boundary_intersection"] + acc["boundary_union"], 1)
            raw_f1 = 2 * acc["raw_intersection"] / max(acc["raw_intersection"] + acc["raw_union"], 1)
            result[dimension][bucket] = {
                "frames": acc["frames"],
                "postprocessed_boundary_f1": post_f1,
                "postprocessed_boundary_error": 1.0 - post_f1,
                "raw_network_boundary_f1": raw_f1,
                "raw_network_boundary_error": 1.0 - raw_f1,
            }
    return result


def select_area_config(sweep: dict) -> dict:
    selected = {}
    for channel, class_name in enumerate(AREA_CLASSES):
        options = []
        for key, metrics in sweep.items():
            threshold_text, morphology = key.split("__", 1)
            candidate = {
                "threshold": float(threshold_text),
                "morphology": morphology,
                "iou": metrics["iou_by_class"][class_name],
                "boundary_f1": metrics["postprocessed_mask_boundary_f1_by_class"][class_name],
                "negative_area_fp_per_frame": metrics["negative_area_fp_per_frame"],
            }
            candidate["development_target_pass"] = (
                candidate["iou"] >= 0.75 and candidate["boundary_f1"] >= 0.75
                and candidate["negative_area_fp_per_frame"] <= 0.05
            )
            options.append(candidate)
        selected[class_name] = max(
            options,
            key=lambda item: (
                item["development_target_pass"],
                item["boundary_f1"],
                item["iou"],
                -item["negative_area_fp_per_frame"],
            ),
        )
    return selected


def evaluate_area_rows(
    leaf, puddle, rows, device, configs: list[tuple], *, manifests=None,
    collect_shape: bool = True, chunk_size=10,
):
    accumulators = {name: empty_area_accumulator() for name, _, _ in configs}
    negative_taxonomy = {name: defaultdict(lambda: {"frames": 0, "fp_frames": 0}) for name, _, _ in configs}
    shape_groups = {
        name: [
            {dimension: defaultdict(lambda: {
                "boundary_intersection": 0, "boundary_union": 0,
                "raw_intersection": 0, "raw_union": 0, "frames": 0,
            }) for dimension in ("width", "shape")}
            for _ in AREA_CLASSES
        ]
        for name, _, _ in configs
    }
    for start in range(0, len(rows), chunk_size):
        chunk = rows[start : start + chunk_size]
        leaf_frames = area_predictions(leaf, chunk, device=device, thresholds=CURRENT_AREA_THRESHOLDS, task="leaf")
        puddle_frames = area_predictions(puddle, chunk, device=device, thresholds=CURRENT_AREA_THRESHOLDS, task="puddle")
        for frame in combine_area(leaf_frames, puddle_frames):
            taxonomies = frame["row"].get("taxonomies") or ("unclassified_negative",)
            for name, thresholds, morphologies in configs:
                masks = area_masks(frame, thresholds, morphologies)
                update_area_accumulator(accumulators[name], frame, masks)
                if collect_shape:
                    for channel in range(2):
                        truth = frame["truth"][channel].astype(bool)
                        for dimension, bucket in truth_shape_groups(truth).items():
                            update_group_boundary(
                                shape_groups[name][channel][dimension][bucket],
                                masks[channel], truth,
                                frame["boundary_probabilities"][channel] >= 0.5,
                            )
                if frame["negative_only"]:
                    fp = any(has_area_candidate(mask) for mask in masks)
                    scene = (manifests or {}).get(int(frame["row"]["scene_seed"]), {})
                    scene_taxonomies = [
                        str(item.get("taxonomy"))
                        for item in scene.get("objects", ())
                        if item.get("class_id") == "background" and item.get("taxonomy")
                    ]
                    domain_taxonomies = [
                        f"ground:{scene.get('ground_material_executed_by_world')}",
                        f"lighting:{scene.get('lighting_executed_by_world')}",
                    ] if scene else []
                    effective_taxonomies = sorted(set(scene_taxonomies + domain_taxonomies)) or taxonomies
                    for taxonomy in effective_taxonomies:
                        negative_taxonomy[name][str(taxonomy)]["frames"] += 1
                        negative_taxonomy[name][str(taxonomy)]["fp_frames"] += int(fp)
    reports = {}
    for name in accumulators:
        report = finalize_area(accumulators[name])
        report["boundary_error_by_width_shape"] = (
            {
                AREA_CLASSES[channel]: finalize_group_boundary(shape_groups[name][channel])
                for channel in range(2)
            }
            if collect_shape else "not_collected_for_development_sweep"
        )
        report["negative_fp_taxonomy"] = {
            taxonomy: {
                **values,
                "fp_per_frame": values["fp_frames"] / max(values["frames"], 1),
            }
            for taxonomy, values in sorted(negative_taxonomy[name].items())
        }
        reports[name] = report
    return reports


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else 0.0


def profile_callable(name: str, call, *, warmup=3, repeats=20) -> dict:
    for _ in range(warmup):
        call()
    torch.cuda.synchronize()
    torch.cuda.reset_peak_memory_stats()
    samples = []
    for _ in range(repeats):
        started = time.perf_counter()
        call()
        torch.cuda.synchronize()
        samples.append((time.perf_counter() - started) * 1000.0)
    return {
        "component": name,
        "samples": repeats,
        "latency_ms_p50": percentile(samples, 50),
        "latency_ms_p95": percentile(samples, 95),
        "latency_ms_max": max(samples),
        "peak_allocated_vram_bytes": int(torch.cuda.max_memory_allocated()),
        "measurement_scope": "single full frame including repository preprocessing and file IO",
    }


def artifact_manifest(output: Path) -> dict:
    records = []
    for path in sorted(output.glob("*.json")):
        if path.name == "artifact_manifest.json":
            continue
        records.append({"path": path.name, "bytes": path.stat().st_size, "sha256": sha256(path)})
    return {"schema_version": 1, "artifacts": records}


def main() -> int:
    if torch is None:
        raise RuntimeError("formal MRV2-00 audit requires PyTorch and CUDA")
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--factorized-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--asset-registry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--area-only", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal MRV2-00 audit requires CUDA")
    assets = load_asset_metadata(args.asset_registry)
    payload = torch.load(args.checkpoint, map_location=device, weights_only=False)
    if payload.get("G5_SEALED_FINAL_read") is not False or payload.get("legacy_G4_D6_read") is not False:
        raise RuntimeError("X3 checkpoint violates sealed-data boundary")
    threshold = float(payload["frozen_threshold_from_train_world_holdout"])
    detector = build_direct_fcos().to(device)
    detector.load_state_dict(payload["state_dict"], strict=True)
    detector.eval()
    leaf, leaf_record = load_checkpoint_model("leaf", args.model_dir / "leaf.pt", device)
    puddle, puddle_record = load_checkpoint_model("puddle", args.model_dir / "puddle.pt", device)

    main_rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root, allowed_splits=("train", "val")
    )
    train_all = [row for row in main_rows if row["split"] == "train"]
    val_rows = [row for row in main_rows if row["split"] == "val"]
    holdout_raw = holdout_rows(train_all, 0.2)
    holdout_scenes = {(str(row["world_id"]), int(row["scene_seed"])) for row in holdout_raw}
    train_pool = [row for row in train_all if (str(row["world_id"]), int(row["scene_seed"])) not in holdout_scenes]
    train_batch = stratified_row_sample(train_pool, 600, seed=SEED)
    holdout = stratified_row_sample(
        [{**row, "split": "train_world_holdout"} for row in holdout_raw], 100, seed=SEED + 1
    )
    main_keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in main_rows}
    main_records = load_instance_records(args.evidence_dir / "g4_instance_records.jsonl", allowed_frame_keys=main_keys)
    main_by_key = index_instance_records(main_records)
    main_manifests = load_scene_manifests(args.data_root, main_rows)

    partitions = {
        "TRAIN_full": (train_all, main_by_key, main_manifests),
        "TRAIN_effective_X3_batch": (train_batch, main_by_key, main_manifests),
        "train_world_holdout": (holdout, main_by_key, main_manifests),
        "VAL": (val_rows, main_by_key, main_manifests),
    }
    factorized = {}
    for index in range(1, 6):
        split = f"D{index}"
        root = args.factorized_root / split
        data_root = root / "g4_screening_native"
        evidence = root / "evidence/raw_g4_qa"
        rows = load_frame_rows(evidence / "g4_frame_manifest.jsonl", data_root)
        keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in rows}
        records = load_instance_records(evidence / "g4_instance_records.jsonl", allowed_frame_keys=keys)
        by_key = index_instance_records(records)
        manifests = load_scene_manifests(data_root, rows)
        partitions[split] = (rows, by_key, manifests)
        factorized[split] = (rows, by_key, manifests)

    small_partitions = {
        name: summarize_small_partition(rows, by_key, manifests, assets)
        for name, (rows, by_key, manifests) in partitions.items()
    }
    dataset = DirectFCOSDataset(train_batch, main_by_key)
    sample_image, _, _ = dataset[0]
    backbone_levels = ["P3", "P4", "P5", "P6", "P7"]
    small_audit = {
        "schema_version": 1,
        "stage": "MRV2-00-SMALL-OBJECT-AUDIT",
        "source_commit": "81746ed",
        "partitions": small_partitions,
        "exact_X3_batch_reconstruction": {
            "seed": SEED, "train_pool_frames_after_scene_holdout": len(train_pool),
            "selected_frames": len(train_batch), "holdout_frames": len(holdout),
            "algorithm": "perception_prod_x3_train.holdout_rows + stratified_row_sample",
        },
        "input_and_assignment": {
            "native_width_height": [640, 480],
            "model_tensor_width_height": [int(sample_image.shape[2]), int(sample_image.shape[1])],
            "additional_resize_shrink": False,
            "training_augmentation": "none in DirectFCOSDataset",
            "torchvision_fcos_backbone_levels": backbone_levels,
            "minimum_detection_stride": 8,
            "P2_stride4_present": False,
            "assignment": "torchvision FCOS center sampling on P3-P7; <18px targets rely on minimum stride 8",
        },
        "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
    }

    detector_reports = {}
    metal_records = []
    inference_partitions = {
        "TRAIN_effective_X3_batch": (train_batch, main_by_key, main_manifests),
        "train_world_holdout": (holdout, main_by_key, main_manifests),
        "VAL": (val_rows, main_by_key, main_manifests),
        **factorized,
    }
    if not args.area_only:
        for name, (rows, by_key, manifests) in inference_partitions.items():
            print(f"[MRV2 audit] detector {name}: {len(rows)} frames", flush=True)
            raw = direct_predictions(detector, rows, by_key, device=device, score_threshold=0.01, batch_size=4, top_k=100)
            detector_reports[name] = detection_truncation_audit(raw, threshold)
            metal_report, records = metal_audit_for_frames(name, raw, threshold, manifests, assets)
            detector_reports[name]["metal_can"] = metal_report
            metal_records.extend(records)
        small_audit["top_k_threshold_effect"] = {
            name: report for name, report in detector_reports.items()
        }
        write_json(args.output / "SMALL_OBJECT_AUDIT.json", small_audit)

        metal_report = {
            "schema_version": 1,
            "stage": "MRV2-00-METAL-CAN-DOMAIN-AUDIT",
            "frozen_threshold": threshold,
            "fixed_top_k": 16,
            "outcome_taxonomy": [
                "correct", "detector_missed", "detector_found_but_wrong_class",
                "score_below_threshold", "box_iou_below_0_5", "top_k_truncated",
            ],
            "background_contrast_and_specular_are_registry_proxies": True,
            "aggregate": {dimension: aggregate_outcomes(metal_records, dimension) for dimension in (
                "split", "world", "asset", "material", "geometry", "texture", "palette",
                "specular_proxy", "distance", "pixel_size", "occlusion", "lighting", "ground",
                "background_contrast_proxy",
            )},
            "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
        }
        write_json(args.output / "METAL_CAN_DOMAIN_AUDIT.json", metal_report)

    val_sweep_configs = [
        (f"{threshold_value:.2f}__{morphology}", (threshold_value, threshold_value), (morphology, morphology))
        for threshold_value in DEVELOPMENT_THRESHOLDS for morphology in MORPHOLOGIES
    ]
    print(f"[MRV2 audit] area VAL development sweep: {len(val_rows)} frames", flush=True)
    val_sweep = evaluate_area_rows(
        leaf, puddle, val_rows, device, val_sweep_configs,
        manifests=main_manifests, collect_shape=False,
    )
    selected = select_area_config(val_sweep)
    selected_thresholds = tuple(selected[name]["threshold"] for name in AREA_CLASSES)
    selected_morphologies = tuple(selected[name]["morphology"] for name in AREA_CLASSES)
    comparison_configs = [
        ("current", CURRENT_AREA_THRESHOLDS, ("none", "none")),
        ("development_selected_postprocess", selected_thresholds, selected_morphologies),
    ]
    area_splits = {}
    print("[MRV2 audit] area VAL current/selected comparison", flush=True)
    area_splits["VAL"] = evaluate_area_rows(
        leaf, puddle, val_rows, device, comparison_configs, manifests=main_manifests
    )
    for name, (rows, _, manifests) in factorized.items():
        print(f"[MRV2 audit] area {name}: {len(rows)} frames", flush=True)
        area_splits[name] = evaluate_area_rows(
            leaf, puddle, rows, device, comparison_configs, manifests=manifests
        )
    area_report = {
        "schema_version": 1,
        "stage": "MRV2-00-AREA-BOUNDARY-AUDIT",
        "metric_semantics": {
            "raw_network_boundary_head_f1": "boundary_logits sigmoid >= 0.5 compared with truth boundary",
            "postprocessed_mask_boundary_f1": "boundary extracted from thresholded semantic mask",
            "historical_X3_boundary_semantics": "postprocessed_mask_boundary_f1 only; boundary_logits were unused",
        },
        "models": {"leaf": leaf_record, "puddle": puddle_record},
        "current_config": {"thresholds": CURRENT_AREA_THRESHOLDS, "morphologies": ["none", "none"]},
        "development_sweep_VAL_only": val_sweep,
        "development_selected_config": {
            "by_class": selected, "thresholds": selected_thresholds,
            "morphologies": selected_morphologies,
        },
        "splits": area_splits,
        "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
    }
    write_json(args.output / "AREA_BOUNDARY_AUDIT.json", area_report)

    perf_rows = val_rows[:1]
    perf_by_key = main_by_key
    performance = {
        "schema_version": 1,
        "stage": "MRV2-00-PERFORMANCE-BUDGET",
        "device": {"gpu": torch.cuda.get_device_name(0), "torch": torch.__version__, "platform": platform.platform()},
        "components": {},
        "classifier": {"status": "not_applicable", "reason": "X3 is a direct closed-set three-class FCOS"},
    }
    if args.area_only and (args.output / "PERFORMANCE_BUDGET.json").is_file():
        prior = json.loads((args.output / "PERFORMANCE_BUDGET.json").read_text(encoding="utf-8"))
        performance["components"]["detector_full_frame"] = prior["components"]["detector_full_frame"]
    else:
        performance["components"]["detector_full_frame"] = profile_callable(
            "detector_full_frame",
            lambda: direct_predictions(detector, perf_rows, perf_by_key, device=device, score_threshold=threshold, batch_size=1, top_k=16),
        )
    performance["components"]["leaf_full_frame"] = profile_callable(
        "leaf_full_frame", lambda: area_predictions(leaf, perf_rows, device=device, thresholds=CURRENT_AREA_THRESHOLDS, task="leaf")
    )
    performance["components"]["puddle_full_frame"] = profile_callable(
        "puddle_full_frame", lambda: area_predictions(puddle, perf_rows, device=device, thresholds=CURRENT_AREA_THRESHOLDS, task="puddle")
    )
    p95_sum = sum(item["latency_ms_p95"] for item in performance["components"].values())
    performance["end_to_end_serial_estimate_ms_p95"] = p95_sum
    performance["headroom_to_200ms"] = 200.0 - p95_sum
    performance["measurement_warning"] = "serial component sum; projection/tracking/scheduler excluded and must be measured at product gate"
    write_json(args.output / "PERFORMANCE_BUDGET.json", performance)

    index = {
        "schema_version": 1, "stage": "MRV2-00-AUDIT-INDEX",
        "source_commit": "81746ed", "checkpoint": {"path": args.checkpoint.as_posix(), "sha256": sha256(args.checkpoint)},
        "frozen_threshold": threshold,
        "environment": {"gpu": torch.cuda.get_device_name(0), "cuda": torch.version.cuda, "torch": torch.__version__},
        "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
        "files": ["SMALL_OBJECT_AUDIT.json", "METAL_CAN_DOMAIN_AUDIT.json", "AREA_BOUNDARY_AUDIT.json", "PERFORMANCE_BUDGET.json"],
    }
    write_json(args.output / "MRV2_00_AUDIT_INDEX.json", index)
    write_json(args.output / "artifact_manifest.json", artifact_manifest(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
