"""Bounded OPR-C RTMDet data and runtime compatibility helpers."""

from __future__ import annotations

from collections import defaultdict
import json
from pathlib import Path
import random
from typing import Any, Iterable


CLASS_NAMES = ("plastic_bottle", "metal_can", "paper_litter")
CLASS_TO_ID = {name: index + 1 for index, name in enumerate(CLASS_NAMES)}


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def bounded_frames(
    frames: Iterable[dict[str, Any]],
    instances_by_frame: dict[tuple[int, int], list[dict[str, Any]]],
    maximum: int,
    seed: int,
) -> list[dict[str, Any]]:
    """Select deterministic full frames while retaining 20% hard negatives."""
    items = list(frames)
    if maximum <= 0 or len(items) <= maximum:
        return sorted(items, key=lambda row: (row["scene_seed"], row["frame_index"]))
    rng = random.Random(seed)
    positives = [
        row
        for row in items
        if instances_by_frame.get((int(row["scene_seed"]), int(row["frame_index"])))
    ]
    negatives = [row for row in items if row not in positives]
    rng.shuffle(positives)
    rng.shuffle(negatives)
    negative_count = min(len(negatives), max(1, maximum // 5))
    selected = positives[: maximum - negative_count] + negatives[:negative_count]
    rng.shuffle(selected)
    return selected


def to_coco(
    frames: Iterable[dict[str, Any]],
    instances_by_frame: dict[tuple[int, int], list[dict[str, Any]]],
) -> dict[str, Any]:
    images: list[dict[str, Any]] = []
    annotations: list[dict[str, Any]] = []
    annotation_id = 1
    for image_id, row in enumerate(frames, start=1):
        images.append(
            {
                "id": image_id,
                "file_name": row["rgb_path"].replace("\\", "/"),
                "width": 640,
                "height": 480,
                "world_id": row["world_id"],
                "scene_seed": row["scene_seed"],
                "frame_index": row["frame_index"],
                "negative_only": bool(row["negative_only"]),
            }
        )
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        for instance in instances_by_frame.get(key, []):
            x1, y1, x2, y2 = (float(value) for value in instance["bbox_xyxy"])
            width = x2 - x1
            height = y2 - y1
            if width <= 0 or height <= 0:
                raise ValueError(f"invalid bbox for {key}: {instance['bbox_xyxy']}")
            annotations.append(
                {
                    "id": annotation_id,
                    "image_id": image_id,
                    "category_id": CLASS_TO_ID[instance["class_id"]],
                    "bbox": [x1, y1, width, height],
                    "area": width * height,
                    "iscrowd": 0,
                    "bbox_short_side_px": instance["bbox_short_side_px"],
                }
            )
            annotation_id += 1
    return {
        "info": {"description": "G6 OPR-C development split; not sealed final"},
        "licenses": [],
        "categories": [
            {"id": category_id, "name": name, "supercategory": "litter"}
            for name, category_id in CLASS_TO_ID.items()
        ],
        "images": images,
        "annotations": annotations,
    }


def index_instances(instances: Iterable[dict[str, Any]]) -> dict[tuple[int, int], list[dict[str, Any]]]:
    result: dict[tuple[int, int], list[dict[str, Any]]] = defaultdict(list)
    for instance in instances:
        if instance.get("visible", True) and instance["class_id"] in CLASS_TO_ID:
            result[(int(instance["scene_seed"]), int(instance["frame_index"]))].append(instance)
    return dict(result)


def patch_mmdet_cuda_nms() -> None:
    """Use Torchvision CUDA NMS with the official MMDetection RTMDet head."""
    import torch
    from torchvision.ops import batched_nms as torchvision_batched_nms
    import mmdet.models.dense_heads.base_dense_head as base_dense_head

    def compatible_batched_nms(boxes, scores, labels, nms_cfg, class_agnostic=False):
        iou_threshold = float(nms_cfg.get("iou_threshold", 0.5))
        effective_labels = torch.zeros_like(labels) if class_agnostic else labels
        keep = torchvision_batched_nms(boxes, scores, effective_labels, iou_threshold)
        dets = torch.cat((boxes[keep], scores[keep, None]), dim=1)
        return dets, keep

    base_dense_head.batched_nms = compatible_batched_nms


__all__ = [
    "CLASS_NAMES",
    "CLASS_TO_ID",
    "bounded_frames",
    "index_instances",
    "load_jsonl",
    "patch_mmdet_cuda_nms",
    "to_coco",
]
