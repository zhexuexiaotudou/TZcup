"""G4 dataset loading and task-specific torch datasets.

This module bridges the formal G4 QA manifests to the AUTO-05R-2/3 model
families.  It intentionally keeps all file access in one place so training and
screening use the same paths, target encoding, crop contract and scale fields.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import random
from typing import Iterable

import cv2
import numpy as np

from .auto04_contract import box_iou, encode_centernet_targets
from .g4_geometry import (
    bbox_native_to_model,
    flip_bbox_horizontal,
    remap_flipped_box,
)
from .ground_geometry import GroundGeometryEstimator


DISCRETE_NAMES = ("plastic_bottle", "metal_can", "paper_litter")
AREA_NAMES = ("leaf_pile", "puddle")
CLASSIFIER_CLASSES = ("background", *DISCRETE_NAMES)
SEMANTIC_TO_DISCRETE = {"plastic_bottle": 1, "metal_can": 2, "paper_litter": 3}
DISCRETE_TO_CLASS = {value: index for index, value in enumerate(DISCRETE_NAMES)}

DISCOVERY_MODEL_SIZE = (640, 480)  # width, height
AREA_MODEL_SIZE = (512, 384)  # width, height
CLASSIFIER_MODEL_SIZE = (192, 192)
DISCOVERY_STRIDE = 4
DISCOVERY_PYRAMID_STRIDES = (4, 8, 16)
AREA_FEATURE_COUNT = 10


DISCOVERY_LEVEL_MAX_SIDE = {4: 48.0, 8: 80.0, 16: float("inf")}


def _discovery_stride_for_box(box: dict) -> int:
    x1, y1, x2, y2 = (float(value) for value in box["bbox_xyxy"])
    max_side = max(x2 - x1, y2 - y1)
    for stride in DISCOVERY_PYRAMID_STRIDES:
        if max_side <= DISCOVERY_LEVEL_MAX_SIDE[stride]:
            return stride
    raise AssertionError("discovery size assignment has no terminal level")


def encode_discovery_pyramid_targets(
    boxes: list[dict], *, assign_by_scale: bool = False
) -> dict[str, np.ndarray]:
    """Encode boxes on P3/P4/P5, optionally assigning each to one level."""
    result: dict[str, np.ndarray] = {}
    for stride in DISCOVERY_PYRAMID_STRIDES:
        level_boxes = (
            [box for box in boxes if _discovery_stride_for_box(box) == stride]
            if assign_by_scale
            else boxes
        )
        encoded = encode_centernet_targets(
            level_boxes,
            input_width=DISCOVERY_MODEL_SIZE[0],
            input_height=DISCOVERY_MODEL_SIZE[1],
            stride=stride,
            class_count=1,
        )
        for name in ("heatmap", "offset", "size", "regression_mask"):
            result[f"{name}_s{stride}"] = encoded[name]
    return result


def encode_teacher_quality_pyramid(
    detections: list[dict], *, assign_by_scale: bool = True
) -> dict[str, np.ndarray]:
    """Encode frozen-teacher scores as soft quality maps for distillation."""
    result: dict[str, np.ndarray] = {}
    for stride in DISCOVERY_PYRAMID_STRIDES:
        output = np.zeros(
            (
                1,
                DISCOVERY_MODEL_SIZE[1] // stride,
                DISCOVERY_MODEL_SIZE[0] // stride,
            ),
            dtype=np.float32,
        )
        for detection in detections:
            if assign_by_scale and _discovery_stride_for_box(detection) != stride:
                continue
            encoded = encode_centernet_targets(
                [{"class_index": 0, "bbox_xyxy": detection["bbox_xyxy"]}],
                input_width=DISCOVERY_MODEL_SIZE[0],
                input_height=DISCOVERY_MODEL_SIZE[1],
                stride=stride,
                class_count=1,
            )["heatmap"]
            score = float(np.clip(detection.get("score", 1.0), 0.0, 1.0))
            output = np.maximum(output, encoded * score)
        result[f"teacher_quality_s{stride}"] = output
    return result


def normalize_depth(depth: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0)
    normalized = np.zeros_like(depth, dtype=np.float32)
    normalized[valid] = np.clip(
        np.log1p(depth[valid]) / math.log(11.0), 0.0, 1.0
    )
    return normalized


def load_camera_info(row: dict) -> dict:
    """Load the fixed CameraInfo JSON adjacent to the frame's RGB image."""
    rgb_path = Path(row["rgb_path"])
    scene_dir = rgb_path.resolve().parent.parent
    camera_path = scene_dir / "camera" / f"frame_{int(row['frame_index']):02d}.json"
    data = json.loads(camera_path.read_text(encoding="utf-8"))
    k = data["k"]
    return {
        "width": int(data["width"]),
        "height": int(data["height"]),
        "fx": float(k[0]),
        "fy": float(k[4]),
        "cx": float(k[2]),
        "cy": float(k[5]),
    }


def load_frame_rows(
    manifest_path: str | Path,
    data_root: str | Path | None = None,
    *,
    allowed_splits: Iterable[str] | None = None,
) -> list[dict]:
    root = Path(data_root) if data_root is not None else None
    allowed = set(allowed_splits) if allowed_splits is not None else None
    rows: list[dict] = []
    for line in Path(manifest_path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        if allowed is not None and row.get("split") not in allowed:
            continue
        if root is not None:
            for key in ("rgb_path", "depth_path", "semantic_path", "instance_path"):
                row[key] = root / row[key]
        rows.append(row)
    return rows


def load_instance_records(
    instance_path: str | Path,
    *,
    allowed_frame_keys: Iterable[tuple[int, int]] | None = None,
) -> list[dict]:
    allowed = (
        set(allowed_frame_keys)
        if allowed_frame_keys is not None
        else None
    )
    records: list[dict] = []
    for line in Path(instance_path).read_text(encoding="utf-8").splitlines():
        if line.strip():
            record = json.loads(line)
            key = (
                int(record["scene_seed"]),
                int(record["frame_index"]),
            )
            if allowed is None or key in allowed:
                records.append(record)
    return records


def index_instance_records(
    records: Iterable[dict],
) -> dict[tuple[int, int], list[dict]]:
    indexed: dict[tuple[int, int], list[dict]] = {}
    for record in records:
        indexed.setdefault(
            (int(record["scene_seed"]), int(record["frame_index"])), []
        ).append(record)
    return indexed


def load_scene_manifests(
    data_root: str | Path, rows: Iterable[dict]
) -> dict[int, dict]:
    root = Path(data_root)
    manifests: dict[int, dict] = {}
    for row in rows:
        scene_seed = int(row["scene_seed"])
        if scene_seed in manifests:
            continue
        path = root / "scenes" / f"scene_{scene_seed:04d}" / "scene_manifest.json"
        if path.is_file():
            manifests[scene_seed] = json.loads(path.read_text(encoding="utf-8"))
        else:
            manifests[scene_seed] = {
                "scene_seed": scene_seed,
                "world_id": row["world_id"],
                "split": row["split"],
                "negative_only": bool(row.get("negative_only", False)),
            }
    return manifests


def read_frame(
    row: dict,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rgb = cv2.cvtColor(cv2.imread(str(row["rgb_path"])), cv2.COLOR_BGR2RGB)
    depth = np.load(row["depth_path"], allow_pickle=False).astype(np.float32)
    semantic = np.load(row["semantic_path"], allow_pickle=False)
    instance = np.load(row["instance_path"], allow_pickle=False)
    return rgb, depth, semantic, instance


def read_rgb(row: dict) -> np.ndarray:
    return cv2.cvtColor(cv2.imread(str(row["rgb_path"])), cv2.COLOR_BGR2RGB)


def discrete_boxes_for_frame(
    row: dict,
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    native_size: tuple[int, int] = (640, 480),
    model_size: tuple[int, int] = DISCOVERY_MODEL_SIZE,
) -> list[dict]:
    if row.get("negative_only"):
        return []
    records = instances_by_key.get(
        (int(row["scene_seed"]), int(row["frame_index"])), []
    )
    boxes: list[dict] = []
    for record in records:
        if record.get("semantic_class") not in DISCRETE_NAMES:
            continue
        native_bbox = tuple(float(value) for value in record["bbox_xyxy_px"])
        model_bbox = bbox_native_to_model(
            native_bbox, native_size, model_size
        )
        boxes.append(
            {
                "class_index": DISCRETE_TO_CLASS[record["semantic_class"]],
                "semantic_class": record["semantic_class"],
                "native_bbox_xyxy": list(native_bbox),
                "model_bbox_xyxy": list(model_bbox),
                "bbox_xyxy": list(model_bbox),
                "native_short_side_px": float(record.get("bbox_shortest_side_px", 0.0)),
                "mask_area_px": int(record.get("mask_area_px", 0)),
            }
        )
    return boxes


def mask_boundary(mask: np.ndarray) -> np.ndarray:
    mask = (np.asarray(mask) > 0).astype(np.uint8)
    kernel = np.ones((3, 3), dtype=np.uint8)
    eroded = cv2.erode(mask, kernel)
    return ((mask - eroded) > 0).astype(np.float32)


def _resize_area_image_crop(
    array: np.ndarray, crop: tuple[int, int, int, int]
) -> np.ndarray:
    left, top, right, bottom = crop
    return np.stack(
        [
            cv2.resize(
                array[top:bottom, left:right, channel],
                AREA_MODEL_SIZE,
                interpolation=cv2.INTER_AREA,
            )
            for channel in range(array.shape[2])
        ],
        axis=-1,
    )


def _resize_area_mask_crop(
    array: np.ndarray, crop: tuple[int, int, int, int]
) -> np.ndarray:
    left, top, right, bottom = crop
    return np.stack(
        [
            cv2.resize(
                array[channel, top:bottom, left:right],
                AREA_MODEL_SIZE,
                interpolation=cv2.INTER_NEAREST,
            )
            for channel in range(array.shape[0])
        ],
        axis=0,
    )


def _positive_area_crop(
    targets: np.ndarray,
    channel: int | None,
) -> tuple[int, int, int, int] | None:
    active = channel if channel is not None else 0
    ys, xs = np.where(targets[active] > 0)
    if ys.size == 0:
        return None
    height, width = targets.shape[1:]
    center_y = float((int(ys.min()) + int(ys.max())) * 0.5)
    center_x = float((int(xs.min()) + int(xs.max())) * 0.5)
    side = max(160, int(max(ys.max() - ys.min(), xs.max() - xs.min()) * 2.0))
    side = min(side, min(width, height))
    left = int(round(center_x - side * 0.5))
    top = int(round(center_y - side * 0.5))
    left = max(0, min(width - side, left))
    top = max(0, min(height - side, top))
    return left, top, left + side, top + side


def build_area_input(
    rgb: np.ndarray,
    depth: np.ndarray,
    size: tuple[int, int] = AREA_MODEL_SIZE,
    task: str = "leaf",
    camera_info: dict | None = None,
) -> np.ndarray:
    if task not in ("leaf", "puddle"):
        raise ValueError(f"unknown area input task {task}")
    if camera_info is None:
        camera_info = {
            "width": 640,
            "height": 480,
            "fx": 343.15907310693535,
            "fy": 343.1590731069353,
            "cx": 320.0,
            "cy": 240.0,
        }
    estimator = GroundGeometryEstimator(camera_info)
    try:
        geometry = estimator.estimate(depth)
        valid = geometry["valid_depth_mask"].astype(np.float32)
        height = np.where(
            geometry["valid_depth_mask"],
            geometry["height_above_ground"],
            0.0,
        ).astype(np.float32)
        gradient = np.nan_to_num(
            geometry["depth_gradient_magnitude"].astype(np.float32),
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        normal = geometry["local_surface_normal"].astype(np.float32)
    except ValueError:
        valid = (np.isfinite(depth) & (depth > 0.0)).astype(np.float32)
        height = np.zeros_like(depth, dtype=np.float32)
        gradient = np.nan_to_num(
            estimator.depth_gradient_magnitude(depth, valid),
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        normal = np.full(depth.shape + (3,), np.nan, dtype=np.float32)
    resized_rgb = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA).astype(
        np.float32
    ) / 255.0
    resized_depth = cv2.resize(
        depth.astype(np.float32), size, interpolation=cv2.INTER_NEAREST
    )
    normalized_depth = normalize_depth(resized_depth)
    resized_valid = cv2.resize(
        valid, size, interpolation=cv2.INTER_NEAREST
    ).astype(np.float32)
    resized_height = cv2.resize(
        height, size, interpolation=cv2.INTER_NEAREST
    ).astype(np.float32)
    if task == "leaf":
        resized_gradient = cv2.resize(
            gradient, size, interpolation=cv2.INTER_NEAREST
        ).astype(np.float32)
        resized_normal = np.stack(
            [
                cv2.resize(
                    normal[:, :, channel],
                    size,
                    interpolation=cv2.INTER_NEAREST,
                )
                for channel in range(3)
            ],
            axis=-1,
        ).astype(np.float32)
        normal_features = np.nan_to_num(
            (resized_normal + 1.0) * 0.5,
            nan=0.0,
            posinf=1.0,
            neginf=0.0,
        )
        channels = [
            resized_rgb,
            normalized_depth[:, :, None],
            resized_valid[:, :, None],
            resized_height[:, :, None],
            resized_gradient[:, :, None],
            normal_features,
        ]
    else:
        hsv = cv2.cvtColor(
            np.clip(resized_rgb, 0.0, 1.0).astype(np.float32) * 255.0,
            cv2.COLOR_RGB2HSV,
        ).astype(np.float32)
        hsv[:, :, 0] /= 180.0
        hsv[:, :, 1:] /= 255.0
        gray = cv2.cvtColor(
            np.clip(resized_rgb, 0.0, 1.0).astype(np.float32) * 255.0,
            cv2.COLOR_RGB2GRAY,
        )
        texture = np.abs(cv2.Laplacian(gray, cv2.CV_32F))
        texture = np.clip(texture / 80.0, 0.0, 1.0).astype(np.float32)
        channels = [
            resized_rgb,
            hsv,
            normalized_depth[:, :, None],
            resized_valid[:, :, None],
            resized_height[:, :, None],
            texture[:, :, None],
        ]
    return np.concatenate(channels, axis=2).astype(np.float32)


def square_crop(
    width: int,
    height: int,
    bbox: tuple[float, float, float, float],
    scale: float,
    minimum_side: int = 64,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    center_x = (x1 + x2) * 0.5
    center_y = (y1 + y2) * 0.5
    side = max(float(minimum_side), max(x2 - x1, y2 - y1) * scale)
    side = min(side, float(min(width, height)))
    left = int(round(center_x - side * 0.5))
    top = int(round(center_y - side * 0.5))
    left = max(0, min(width - int(side), left))
    top = max(0, min(height - int(side), top))
    return left, top, left + int(side), top + int(side)


def _random_background_crop(
    width: int,
    height: int,
    boxes: list[dict],
    rng: random.Random,
    side: int = 96,
) -> tuple[int, int, int, int]:
    for _ in range(40):
        side = min(side, width, height)
        left = rng.randint(0, max(0, width - side))
        top = rng.randint(0, max(0, height - side))
        crop_box = (left, top, left + side, top + side)
        overlaps = [
            box_iou(
                tuple(float(value) for value in item["native_bbox_xyxy"]),
                crop_box,
            )
            for item in boxes
        ]
        if not overlaps or max(overlaps) < 0.1:
            return crop_box
    return (0, 0, side, side)


def build_classifier_samples(
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    positive_per_class: int = 60,
    background_per_positive: int = 2,
    negative_only_per_frame: int = 4,
    background_limit: int | None = None,
    seed: int = 20260806,
) -> list[dict]:
    rng = random.Random(seed)
    selected: dict[str, list[dict]] = {name: [] for name in DISCRETE_NAMES}
    negatives: list[dict] = []
    used_crops: set[tuple] = set()
    for row in rows:
        rgb_path = row["rgb_path"]
        boxes = discrete_boxes_for_frame(row, instances_by_key)
        height, width = 480, 640
        if boxes:
            for box in boxes:
                class_name = box["semantic_class"]
                if len(selected[class_name]) >= positive_per_class:
                    continue
                native = tuple(float(value) for value in box["native_bbox_xyxy"])
                shortest = float(box["native_short_side_px"])
                bucket = 0 if shortest < 8 else 1 if shortest < 18 else 2
                crop = square_crop(
                    width,
                    height,
                    native,
                    scale=(9.0, 6.0, 4.0)[bucket],
                )
                key = (str(rgb_path), crop, class_name)
                if key in used_crops:
                    continue
                used_crops.add(key)
                selected[class_name].append(
                    {
                        "rgb_path": rgb_path,
                        "crop": crop,
                        "label": CLASSIFIER_CLASSES.index(class_name),
                        "class_name": class_name,
                        "split": row.get("split", "train"),
                        "scene_seed": int(row.get("scene_seed", 0)),
                        "frame_index": int(row.get("frame_index", 0)),
                        "hard_negative": False,
                    }
                )
        crop_count = background_per_positive if boxes else negative_only_per_frame
        for _ in range(crop_count):
            if background_limit is not None and len(negatives) >= background_limit:
                break
            crop = _random_background_crop(width, height, boxes, rng)
            key = (str(rgb_path), crop, "background")
            if key in used_crops:
                continue
            used_crops.add(key)
            negatives.append(
                {
                    "rgb_path": rgb_path,
                    "crop": crop,
                    "label": 0,
                    "class_name": "background",
                    "split": row.get("split", "train"),
                    "scene_seed": int(row.get("scene_seed", 0)),
                    "frame_index": int(row.get("frame_index", 0)),
                    "hard_negative": bool(
                        row.get("paper_like_hard_negative", False)
                    ),
                }
            )
    samples = negatives
    for name in DISCRETE_NAMES:
        samples.extend(selected[name])
    return samples


def _transform_boxes_to_crop(
    boxes: list[dict],
    crop: tuple[int, int, int, int],
    model_size: tuple[int, int] = DISCOVERY_MODEL_SIZE,
) -> list[dict]:
    left, top, right, bottom = crop
    scale_x = model_size[0] / max(right - left, 1)
    scale_y = model_size[1] / max(bottom - top, 1)
    transformed: list[dict] = []
    for item in boxes:
        x1, y1, x2, y2 = item["native_bbox_xyxy"]
        clipped = (
            max(left, x1),
            max(top, y1),
            min(right, x2),
            min(bottom, y2),
        )
        if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
            continue
        visible = (clipped[2] - clipped[0]) * (clipped[3] - clipped[1]) / max(
            (x2 - x1) * (y2 - y1), 1.0
        )
        if visible < 0.7:
            continue
        transformed.append(
            {
                "class_index": 0,
                "bbox_xyxy": [
                    (clipped[0] - left) * scale_x,
                    (clipped[1] - top) * scale_y,
                    (clipped[2] - left) * scale_x,
                    (clipped[3] - top) * scale_y,
                ],
            }
        )
    return transformed


def build_discovery_crop_samples(
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    positive_frame_limit: int = 40,
    max_positive_samples: int = 120,
    negative_count: int = 20,
    seed: int = 20260806,
) -> list[dict]:
    rng = random.Random(seed)
    positives: list[dict] = []
    negatives: list[dict] = []
    used_frames: set[tuple[int, int]] = set()
    samples_per_frame = max(1, int(max_positive_samples / max(positive_frame_limit, 1)))
    for row in rows:
        if len(used_frames) >= positive_frame_limit or len(positives) >= max_positive_samples:
            break
        frame_key = (int(row["scene_seed"]), int(row["frame_index"]))
        if frame_key in used_frames:
            continue
        boxes = discrete_boxes_for_frame(row, instances_by_key)
        if not boxes and row.get("negative_only"):
            negatives.append(row)
            continue
        if not boxes:
            continue
        used_frames.add(frame_key)
        selected_boxes = sorted(
            boxes,
            key=lambda item: (
                float(item["native_short_side_px"]),
                item["semantic_class"],
            ),
        )[:samples_per_frame]
        for box in selected_boxes:
            if len(positives) >= max_positive_samples:
                break
            native = tuple(float(value) for value in box["native_bbox_xyxy"])
            shortest = float(box["native_short_side_px"])
            bucket = 0 if shortest < 8 else 1 if shortest < 18 else 2
            crop = square_crop(
                640,
                480,
                native,
                scale=(16.0, 6.0, 4.0)[bucket],
                minimum_side=64 if bucket == 0 else 128,
            )
            transformed = _transform_boxes_to_crop(
                discrete_boxes_for_frame(row, instances_by_key), crop
            )
            positives.append(
                {
                    "rgb_path": row["rgb_path"],
                    "crop": crop,
                    "boxes": transformed,
                    "negative_only": False,
                    "scene_seed": int(row["scene_seed"]),
                    "frame_index": int(row["frame_index"]),
                    "split": row["split"],
                }
            )
    rng.shuffle(positives)
    positives = positives[:max_positive_samples]
    negative_samples: list[dict] = []
    for row in negatives:
        if len(negative_samples) >= negative_count:
            break
        crop = (160, 120, 480, 360)
        negative_samples.append(
            {
                "rgb_path": row["rgb_path"],
                "crop": crop,
                "boxes": [],
                "negative_only": True,
                "scene_seed": int(row["scene_seed"]),
                "frame_index": int(row["frame_index"]),
                "split": row["split"],
            }
        )
    return positives + negative_samples


class G4DiscoveryCropDataset:
    """Object-level crop dataset used for the discovery micro-overfit gate."""

    def __init__(
        self,
        samples: list[dict],
        *,
        augment: bool = False,
        seed: int = 20260806,
    ):
        self.samples = samples
        self.augment = augment
        self.seed = seed

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int):
        torch = _torch()
        sample = self.samples[index]
        rgb = cv2.cvtColor(cv2.imread(str(sample["rgb_path"])), cv2.COLOR_BGR2RGB)
        left, top, right, bottom = sample["crop"]
        crop = rgb[top:bottom, left:right]
        resized = cv2.resize(
            crop, DISCOVERY_MODEL_SIZE, interpolation=cv2.INTER_AREA
        ).astype(np.float32) / 255.0
        if self.augment:
            rng = random.Random(self.seed + index * 1009)
            hsv = cv2.cvtColor(resized, cv2.COLOR_RGB2HSV)
            hsv[:, :, 0] = np.mod(hsv[:, :, 0] + rng.uniform(-30.0, 30.0), 360.0)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * rng.uniform(0.75, 1.25), 0.0, 1.0)
            resized = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
            resized = np.clip(
                resized * rng.uniform(0.80, 1.20) + rng.uniform(-0.06, 0.06),
                0.0,
                1.0,
            ).astype(np.float32)
        targets = encode_centernet_targets(
            sample["boxes"],
            input_width=DISCOVERY_MODEL_SIZE[0],
            input_height=DISCOVERY_MODEL_SIZE[1],
            stride=DISCOVERY_STRIDE,
            class_count=1,
        )
        tensor = torch.from_numpy(
            np.ascontiguousarray(resized.transpose(2, 0, 1), dtype=np.float32)
        )
        pyramid = encode_discovery_pyramid_targets(sample["boxes"])
        return (
            tensor,
            {
                "heatmap": torch.from_numpy(targets["heatmap"]),
                "offset": torch.from_numpy(targets["offset"]),
                "size": torch.from_numpy(targets["size"]),
                "regression_mask": torch.from_numpy(targets["regression_mask"]),
                **{
                    name: torch.from_numpy(value)
                    for name, value in pyramid.items()
                },
            },
        )


def _torch():
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for G4 torch datasets") from exc
    return torch


class G4DiscoveryDataset:
    """Full-frame class-agnostic discovery dataset."""

    def __init__(
        self,
        rows: list[dict],
        instances_by_key: dict[tuple[int, int], list[dict]],
        *,
        augment: bool = False,
        epoch: int = 0,
        seed: int = 20260806,
        assign_pyramid_by_scale: bool = False,
        teacher_detections_by_key: dict[
            tuple[int, int], list[dict]
        ] | None = None,
    ):
        self.rows = rows
        self.instances_by_key = instances_by_key
        self.augment = augment
        self.epoch = epoch
        self.seed = seed
        self.assign_pyramid_by_scale = bool(assign_pyramid_by_scale)
        self.teacher_detections_by_key = teacher_detections_by_key

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        torch = _torch()
        row = self.rows[index]
        # Discovery is RGB-only. Loading depth/semantic/instance arrays here
        # added three unused disk reads per sample and made formal full-frame
        # training I/O-bound without changing a single target.
        rgb = read_rgb(row)
        native_size = (int(rgb.shape[1]), int(rgb.shape[0]))
        boxes = discrete_boxes_for_frame(
            row,
            self.instances_by_key,
            native_size=native_size,
            model_size=DISCOVERY_MODEL_SIZE,
        )
        rng = random.Random(self.seed + index * 7919 + self.epoch * 104729)
        flip = self.augment and rng.random() < 0.5
        if flip:
            rgb = np.ascontiguousarray(rgb[:, ::-1])
            boxes = [
                remap_flipped_box(
                    box,
                    native_size=native_size,
                    model_size=DISCOVERY_MODEL_SIZE,
                )
                for box in boxes
            ]
        teacher_detections = None
        if self.teacher_detections_by_key is not None:
            key = (int(row["scene_seed"]), int(row["frame_index"]))
            teacher_detections = [
                dict(item)
                for item in self.teacher_detections_by_key.get(key, ())
            ]
            if flip:
                teacher_detections = [
                    {
                        **item,
                        "bbox_xyxy": list(
                            flip_bbox_horizontal(
                                item["bbox_xyxy"], DISCOVERY_MODEL_SIZE[0]
                            )
                        ),
                    }
                    for item in teacher_detections
                ]
        resized = cv2.resize(rgb, DISCOVERY_MODEL_SIZE, interpolation=cv2.INTER_AREA)
        image = resized.astype(np.float32) / 255.0
        if self.augment:
            hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            hsv[:, :, 0] = np.mod(hsv[:, :, 0] + rng.uniform(-45.0, 45.0), 360.0)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * rng.uniform(0.65, 1.35), 0.0, 1.0)
            hsv[:, :, 2] = np.clip(hsv[:, :, 2] ** rng.uniform(0.70, 1.35), 0.0, 1.0)
            image = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
            image = np.clip(
                image * rng.uniform(0.75, 1.25) + rng.uniform(-0.08, 0.08),
                0.0,
                1.0,
            ).astype(np.float32)
        targets = encode_centernet_targets(
            [{"class_index": 0, "bbox_xyxy": box["bbox_xyxy"]} for box in boxes],
            input_width=DISCOVERY_MODEL_SIZE[0],
            input_height=DISCOVERY_MODEL_SIZE[1],
            stride=DISCOVERY_STRIDE,
            class_count=1,
        )
        tensor = torch.from_numpy(
            np.ascontiguousarray(image.transpose(2, 0, 1), dtype=np.float32)
        )
        pyramid = encode_discovery_pyramid_targets(
            [{"class_index": 0, "bbox_xyxy": box["bbox_xyxy"]} for box in boxes],
            assign_by_scale=self.assign_pyramid_by_scale,
        )
        teacher_quality = (
            encode_teacher_quality_pyramid(
                teacher_detections,
                assign_by_scale=self.assign_pyramid_by_scale,
            )
            if teacher_detections is not None
            else {}
        )
        return (
            tensor,
            {
                "heatmap": torch.from_numpy(targets["heatmap"]),
                "offset": torch.from_numpy(targets["offset"]),
                "size": torch.from_numpy(targets["size"]),
                "regression_mask": torch.from_numpy(targets["regression_mask"]),
                **{
                    name: torch.from_numpy(value)
                    for name, value in {**pyramid, **teacher_quality}.items()
                },
            },
        )


class G4AreaDataset:
    """Full-frame independent binary area dataset for leaf/puddle."""

    def __init__(
        self,
        rows: list[dict],
        *,
        augment: bool = False,
        epoch: int = 0,
        seed: int = 20260807,
        channel: int | None = None,
        cache_frames: bool = False,
        crop_mode: str = "full",
    ):
        self.rows = rows
        self.augment = augment
        self.epoch = epoch
        self.seed = seed
        self.channel = channel
        self.cache_frames = cache_frames
        self.crop_mode = crop_mode
        self._frame_cache: dict[
            int, tuple[np.ndarray, np.ndarray, np.ndarray]
        ] = {}

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        torch = _torch()
        row = self.rows[index]
        if self.cache_frames and index in self._frame_cache:
            inputs, targets, boundaries = self._frame_cache[index]
        else:
            rgb, depth, semantic, _ = read_frame(row)
            inputs = build_area_input(
                rgb,
                depth,
                AREA_MODEL_SIZE,
                task="leaf" if self.channel in (None, 0) else "puddle",
                camera_info=load_camera_info(row),
            )
            semantic_model = cv2.resize(
                semantic, AREA_MODEL_SIZE, interpolation=cv2.INTER_NEAREST
            )
            targets = np.stack(
                (semantic_model == 4, semantic_model == 5), axis=0
            ).astype(np.float32)
            boundaries = np.stack(
                (
                    mask_boundary(semantic_model == 4),
                    mask_boundary(semantic_model == 5),
                ),
                axis=0,
            ).astype(np.float32)
            if row.get("negative_only"):
                targets = np.zeros_like(targets)
                boundaries = np.zeros_like(boundaries)
            if self.crop_mode == "positive_crop" and not row.get("negative_only"):
                crop = _positive_area_crop(targets, self.channel)
                if crop is not None:
                    inputs = _resize_area_image_crop(inputs, crop)
                    targets = _resize_area_mask_crop(targets, crop)
                    boundaries = _resize_area_mask_crop(boundaries, crop)
            if self.channel is not None:
                targets = targets[self.channel : self.channel + 1]
                boundaries = boundaries[self.channel : self.channel + 1]
            if self.cache_frames:
                self._frame_cache[index] = (inputs, targets, boundaries)
        rng = random.Random(self.seed + index * 1009 + self.epoch * 7919)
        flip = self.augment and rng.random() < 0.5
        if flip:
            inputs = np.ascontiguousarray(inputs[:, ::-1])
            targets = np.ascontiguousarray(targets[:, :, ::-1])
            boundaries = np.ascontiguousarray(boundaries[:, :, ::-1])
        tensor = torch.from_numpy(
            np.ascontiguousarray(inputs.transpose(2, 0, 1), dtype=np.float32)
        )
        return (
            tensor,
            torch.from_numpy(targets),
            torch.from_numpy(boundaries),
        )


def load_classifier_crop(
    sample: dict, size: tuple[int, int] = CLASSIFIER_MODEL_SIZE
) -> np.ndarray:
    rgb = cv2.cvtColor(cv2.imread(str(sample["rgb_path"])), cv2.COLOR_BGR2RGB)
    left, top, right, bottom = sample["crop"]
    crop = rgb[top:bottom, left:right]
    return cv2.resize(crop, size, interpolation=cv2.INTER_AREA)


class G4ClassifierDataset:
    def __init__(
        self,
        samples: list[dict],
        *,
        augment: bool = False,
        seed: int = 20260808,
        cache_crops: bool = False,
    ):
        self.samples = samples
        self.augment = augment
        self.seed = seed
        self.cache_crops = cache_crops
        self._crop_cache: dict[int, np.ndarray] = {}

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int):
        torch = _torch()
        sample = self.samples[index]
        if self.cache_crops and index in self._crop_cache:
            crop = self._crop_cache[index].copy()
        else:
            crop = load_classifier_crop(sample)
            if self.cache_crops:
                self._crop_cache[index] = crop.copy()
        image = crop.astype(np.float32) / 255.0
        if self.augment:
            rng = random.Random(self.seed + index * 1009)
            hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            hsv[:, :, 0] = np.mod(hsv[:, :, 0] + rng.uniform(-30.0, 30.0), 360.0)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * rng.uniform(0.75, 1.25), 0.0, 1.0)
            image = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
            image = np.clip(
                image * rng.uniform(0.80, 1.20) + rng.uniform(-0.06, 0.06),
                0.0,
                1.0,
            )
        tensor = torch.from_numpy(
            np.ascontiguousarray(image.transpose(2, 0, 1), dtype=np.float32)
        )
        return tensor, torch.tensor(int(sample["label"]), dtype=torch.long)


__all__ = [
    "AREA_FEATURE_COUNT",
    "AREA_MODEL_SIZE",
    "AREA_NAMES",
    "CLASSIFIER_CLASSES",
    "CLASSIFIER_MODEL_SIZE",
    "DISCOVERY_MODEL_SIZE",
    "DISCOVERY_STRIDE",
    "DISCRETE_NAMES",
    "G4AreaDataset",
    "G4ClassifierDataset",
    "G4DiscoveryCropDataset",
    "G4DiscoveryDataset",
    "build_area_input",
    "build_classifier_samples",
    "build_discovery_crop_samples",
    "discrete_boxes_for_frame",
    "index_instance_records",
    "load_classifier_crop",
    "load_camera_info",
    "load_frame_rows",
    "load_instance_records",
    "load_scene_manifests",
    "mask_boundary",
    "normalize_depth",
    "bbox_native_to_model",
    "flip_bbox_horizontal",
    "remap_flipped_box",
    "read_frame",
    "read_rgb",
    "square_crop",
]
