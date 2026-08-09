"""Frozen one-shot G5 evaluator for AUTO-05R product models.

This module is loaded only by ``run_sealed_final_test.py`` after the atomic
first-access record has been created.  It derives truth directly from the
captured semantic/instance tensors, verifies every frozen ONNX hash and runs
the fixed thresholds from ``MODEL_FREEZE.json`` without calibration or tuning.
"""

from __future__ import annotations

from collections import defaultdict
import json
from pathlib import Path
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.auto04_contract import box_iou  # noqa: E402
from sanitation_learning.g4_data import index_instance_records  # noqa: E402
from sanitation_learning.g4_evaluation import (  # noqa: E402
    area_metrics,
    area_predictions,
    classify_detections,
    discrete_metrics,
    discovery_predictions,
    match_discrete_predictions,
)
from sanitation_learning.g4_manifest import file_sha256  # noqa: E402
from sanitation_learning.g5_dataset import _dataset_tree_digest  # noqa: E402


LABEL_TO_CLASS = {
    1: "plastic_bottle",
    2: "metal_can",
    3: "paper_litter",
    4: "leaf_pile",
    5: "puddle",
}
DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")


class _OrtModel:
    """Small adapter exposing the dict-output contract used by evaluation."""

    def __init__(self, task: str, path: Path):
        import onnxruntime as ort

        available = ort.get_available_providers()
        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if "CUDAExecutionProvider" in available
            else ["CPUExecutionProvider"]
        )
        self.task = task
        self.session = ort.InferenceSession(str(path), providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.providers = self.session.get_providers()

    def eval(self):
        return self

    def __call__(self, tensor):
        import torch

        value = np.ascontiguousarray(tensor.detach().cpu().numpy())
        flat = torch.from_numpy(self.session.run(None, {self.input_name: value})[0])
        if self.task == "discovery":
            return {
                "objectness_logits": flat[:, :3],
                "offset": flat[:, 3:9],
                "bbox_size": flat[:, 9:15],
            }
        if self.task == "classifier":
            return flat
        return {"logits": flat[:, :1], "boundary_logits": flat[:, 1:2]}


def _load_dataset_rows(dataset_root: Path) -> tuple[list[dict], list[dict], dict]:
    rows: list[dict] = []
    instances: list[dict] = []
    scene_metadata: dict[tuple[int, int], dict] = {}
    for scene_dir in sorted((dataset_root / "scenes").glob("scene_*")):
        scene = json.loads((scene_dir / "scene_manifest.json").read_text())
        capture = json.loads((scene_dir / "capture_report.json").read_text())
        if scene.get("split") != "G5_SEALED_FINAL":
            raise ValueError(f"non-sealed scene in G5 root: {scene_dir.name}")
        seed = int(scene["scene_seed"])
        taxonomies = sorted({
            item["taxonomy"]
            for item in scene.get("objects", [])
            if item.get("class_id") == "background" and item.get("taxonomy")
        })
        target_objects = [
            item for item in scene.get("objects", [])
            if item.get("class_id") in LABEL_TO_CLASS.values()
        ]
        for record in capture.get("records", []):
            frame_index = int(record["frame_index"])
            paths = {
                name: scene_dir / relative
                for name, relative in record["paths"].items()
            }
            row = {
                "split": "G5_SEALED_FINAL",
                "world_id": scene["world_id"],
                "scene_seed": seed,
                "frame_index": frame_index,
                "rgb_path": paths["rgb"],
                "depth_path": paths["depth"],
                "semantic_path": paths["semantic"],
                "instance_path": paths["instance"],
                "negative_only": bool(scene.get("negative_only", False)),
            }
            rows.append(row)
            scene_metadata[(seed, frame_index)] = {
                "world_id": scene["world_id"],
                "negative_taxonomies": taxonomies,
                "distance_buckets": sorted({
                    "_".join(str(value) for value in item["distance_bucket_m"])
                    for item in target_objects
                    if item.get("distance_bucket_m")
                }),
                "size_buckets": sorted({
                    str(item["size_bucket"])
                    for item in target_objects
                    if item.get("size_bucket")
                }),
            }
            semantic = np.load(paths["semantic"], allow_pickle=False)
            instance = np.load(paths["instance"], allow_pickle=False)
            for instance_id in (int(value) for value in np.unique(instance) if int(value)):
                mask = instance == instance_id
                labels = semantic[mask].astype(np.int64)
                label = int(np.bincount(labels, minlength=6).argmax())
                class_name = LABEL_TO_CLASS.get(label)
                if class_name not in DISCRETE_CLASSES:
                    continue
                ys, xs = np.nonzero(mask & (semantic == label))
                if xs.size == 0:
                    continue
                width = int(xs.max() - xs.min() + 1)
                height = int(ys.max() - ys.min() + 1)
                instances.append({
                    "scene_seed": seed,
                    "frame_index": frame_index,
                    "semantic_class": class_name,
                    "bbox_xyxy_px": [
                        int(xs.min()), int(ys.min()), int(xs.max() + 1),
                        int(ys.max() + 1),
                    ],
                    "bbox_shortest_side_px": min(width, height),
                    "mask_area_px": int(xs.size),
                })
    return rows, instances, scene_metadata


def _combine_area(leaf_predictions: list[dict], puddle_predictions: list[dict]) -> list[dict]:
    combined = []
    for leaf, puddle in zip(leaf_predictions, puddle_predictions):
        item = dict(leaf)
        item["probabilities"] = np.stack(
            (leaf["probabilities"][0], puddle["probabilities"][1]), axis=0
        )
        item["boundary_probabilities"] = np.stack(
            (
                leaf["boundary_probabilities"][0],
                puddle["boundary_probabilities"][1],
            ),
            axis=0,
        )
        combined.append(item)
    return combined


def _average_precision(frames: list[dict], iou_threshold: float) -> float:
    class_scores = []
    for class_name in DISCRETE_CLASSES:
        truth_total = sum(
            sum(item["semantic_class"] == class_name for item in frame["truth"])
            for frame in frames
        )
        ranked = []
        for frame_index, frame in enumerate(frames):
            for prediction in frame.get("predictions", []):
                if prediction.get("class_name") == class_name:
                    ranked.append((float(prediction["score"]), frame_index, prediction))
        ranked.sort(key=lambda item: item[0], reverse=True)
        used: dict[int, set[int]] = defaultdict(set)
        true_positive = []
        false_positive = []
        for _, frame_index, prediction in ranked:
            frame = frames[frame_index]
            best_index, best_iou = -1, 0.0
            for truth_index, truth in enumerate(frame["truth"]):
                if truth["semantic_class"] != class_name or truth_index in used[frame_index]:
                    continue
                iou = box_iou(
                    tuple(float(value) for value in prediction["bbox_xyxy"]),
                    tuple(float(value) for value in truth["bbox_xyxy"]),
                )
                if iou > best_iou:
                    best_index, best_iou = truth_index, iou
            matched = best_index >= 0 and best_iou >= iou_threshold
            if matched:
                used[frame_index].add(best_index)
            true_positive.append(int(matched))
            false_positive.append(int(not matched))
        if truth_total == 0:
            class_scores.append(0.0)
            continue
        tp = np.cumsum(true_positive)
        fp = np.cumsum(false_positive)
        recall = tp / truth_total
        precision = tp / np.maximum(tp + fp, 1)
        interpolated = [
            float(precision[recall >= level].max()) if np.any(recall >= level) else 0.0
            for level in np.linspace(0.0, 1.0, 101)
        ]
        class_scores.append(float(np.mean(interpolated)))
    return float(np.mean(class_scores))


def _subset_metrics(matched: list[dict], area: list[dict], indices: list[int]) -> dict:
    return {
        "frames": len(indices),
        "discrete": discrete_metrics([matched[index] for index in indices]),
        "area": area_metrics([area[index] for index in indices]),
    }


def preflight_frozen_evaluator(*, freeze: dict) -> dict:
    """Verify frozen graphs and providers without touching the sealed data."""
    model_root = Path(__file__).resolve().parent
    providers = {}
    for task in ("discovery", "classifier", "leaf", "puddle"):
        filename = freeze["model_config"][task]["onnx"]
        path = model_root / filename
        if not path.is_file():
            raise FileNotFoundError(f"frozen ONNX artifact missing for {task}: {path}")
        if file_sha256(path) != freeze["model_artifact_hashes"][task]:
            raise ValueError(f"frozen ONNX SHA-256 mismatch for {task}")
        providers[task] = _OrtModel(task, path).providers
    return {"passed": True, "providers": providers}


def evaluate_sealed_final(*, dataset_root, freeze: dict, sealed_manifest: dict) -> dict:
    import torch

    dataset_root = Path(dataset_root)
    declared_content = sealed_manifest.get("dataset_content", {})
    actual_content = _dataset_tree_digest(dataset_root)
    if actual_content.get("sha256") != declared_content.get("sha256"):
        raise ValueError("G5 dataset tree SHA-256 does not match sealed manifest")
    model_root = Path(__file__).resolve().parent
    models = {}
    for task in ("discovery", "classifier", "leaf", "puddle"):
        filename = freeze["model_config"][task]["onnx"]
        path = model_root / filename
        if file_sha256(path) != freeze["model_artifact_hashes"][task]:
            raise ValueError(f"frozen ONNX SHA-256 mismatch for {task}")
        models[task] = _OrtModel(task, path)

    rows, instance_records, metadata = _load_dataset_rows(dataset_root)
    if len(rows) != int(sealed_manifest["frames"]):
        raise ValueError("G5 evaluator frame count does not match sealed manifest")
    instances = index_instance_records(instance_records)
    thresholds = freeze["thresholds"]
    device = torch.device("cpu")
    candidates = discovery_predictions(
        models["discovery"], rows, instances, device=device,
        threshold=float(thresholds["discovery"]["score"]),
    )
    classified = classify_detections(
        models["classifier"], candidates, device=device,
        class_threshold=float(thresholds["classifier"]["score"]),
    )
    matched = match_discrete_predictions(classified)
    area_thresholds = (
        float(thresholds["leaf"]["mask"]),
        float(thresholds["puddle"]["mask"]),
    )
    leaf = area_predictions(
        models["leaf"], rows, device=device, thresholds=area_thresholds, task="leaf"
    )
    puddle = area_predictions(
        models["puddle"], rows, device=device, thresholds=area_thresholds, task="puddle"
    )
    area = _combine_area(leaf, puddle)
    discrete = discrete_metrics(matched)
    area_summary = area_metrics(area)

    per_world = {}
    for world_id in sorted({row["world_id"] for row in rows}):
        indices = [i for i, row in enumerate(rows) if row["world_id"] == world_id]
        per_world[world_id] = _subset_metrics(matched, area, indices)
    breakdowns = {}
    for field in ("distance_buckets", "size_buckets", "negative_taxonomies"):
        values = sorted({
            value for key in metadata for value in metadata[key].get(field, [])
        })
        breakdowns[field] = {
            value: _subset_metrics(
                matched,
                area,
                [
                    index for index, row in enumerate(rows)
                    if value in metadata[(int(row["scene_seed"]), int(row["frame_index"]))][field]
                ],
            )
            for value in values
        }

    negative_indices = [i for i, row in enumerate(rows) if row["negative_only"]]
    negative_frames_with_hallucination = sum(
        bool([item for item in matched[index].get("predictions", []) if item["class_index"] > 0])
        for index in negative_indices
    )
    same_color_specificity = 1.0 - (
        negative_frames_with_hallucination / max(len(negative_indices), 1)
    )
    missing_class_events = 0
    possible_missing_events = 0
    for frame in matched:
        present = {item["semantic_class"] for item in frame["truth"]}
        predicted = {
            item["class_name"]
            for item in frame.get("predictions", [])
            if item["class_index"] > 0
        }
        absent = set(DISCRETE_CLASSES) - present
        missing_class_events += len(predicted & absent)
        possible_missing_events += len(absent)
    ap50 = _average_precision(matched, 0.50)
    ap50_95 = float(np.mean([
        _average_precision(matched, threshold)
        for threshold in np.arange(0.50, 0.96, 0.05)
    ]))
    world_manifest = json.loads(
        (dataset_root / "worlds" / "g5_world_manifest.json").read_text()
    )
    world_profiles = {
        item["world_id"]: {
            "lighting": item["lighting_family"],
            "material": item["material_id"],
            "layout": item["layout_family"],
        }
        for item in world_manifest["worlds"]
    }
    return {
        "schema_version": 1,
        "dataset_id": "G5_SEALED_FINAL",
        "frames": len(rows),
        "discrete": discrete,
        "per_class_recall": min(
            item["recall"] for item in discrete["per_class"].values()
        ),
        "AP50": ap50,
        "AP50_95": ap50_95,
        "area": area_summary,
        "same_color_specificity": same_color_specificity,
        "missing_class_hallucination": (
            missing_class_events / max(possible_missing_events, 1)
        ),
        "per_world": per_world,
        "per_distance": breakdowns["distance_buckets"],
        "per_size": breakdowns["size_buckets"],
        "per_negative_taxonomy": breakdowns["negative_taxonomies"],
        "per_lighting_material_layout": {
            world_id: {**world_profiles[world_id], **per_world[world_id]}
            for world_id in sorted(per_world)
        },
        "breakdown_aggregation": "scene_contains_bucket_or_taxonomy",
        "runtime": {
            task: {"providers": model.providers}
            for task, model in models.items()
        },
        "dataset_content_sha256": actual_content["sha256"],
    }


__all__ = ["evaluate_sealed_final", "preflight_frozen_evaluator"]
