"""OPR-A class-agnostic small-object specialist data and fusion contracts."""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from .g4_data import DISCRETE_NAMES
from .g4_direct_fcos import build_p2_direct_fcos
from .g6_dataset import load_jsonl


TILE_INPUT_SIZE = (640, 480)


def ground_roi_tiles(width: int = 640, height: int = 480) -> list[tuple[int, int, int, int]]:
    """Six overlapping 320x240 native tiles over the lower 75% ground ROI."""
    if (width, height) != (640, 480):
        raise ValueError("OPR-A v1 tile contract requires native 640x480 input")
    return [
        (x, y, x + 320, y + 240)
        for y in (120, 240)
        for x in (0, 160, 320)
    ]


def map_tile_box_to_native(
    box: list[float], tile: tuple[int, int, int, int]
) -> list[float]:
    x0, y0, x1, y1 = tile
    sx = (x1 - x0) / TILE_INPUT_SIZE[0]
    sy = (y1 - y0) / TILE_INPUT_SIZE[1]
    return [
        x0 + float(box[0]) * sx,
        y0 + float(box[1]) * sy,
        x0 + float(box[2]) * sx,
        y0 + float(box[3]) * sy,
    ]


def _contains(tile: tuple[int, int, int, int], box: list[int]) -> bool:
    return (
        box[0] >= tile[0]
        and box[1] >= tile[1]
        and box[2] <= tile[2]
        and box[3] <= tile[3]
    )


def _best_tile(box: list[int], tiles: list[tuple[int, int, int, int]]) -> int | None:
    eligible = []
    for index, tile in enumerate(tiles):
        if _contains(tile, box):
            margin = min(
                box[0] - tile[0],
                box[1] - tile[1],
                tile[2] - box[2],
                tile[3] - box[3],
            )
            eligible.append((margin, -index, index))
    return max(eligible)[2] if eligible else None


def load_g6_rows(
    root: str | Path, allowed_splits: tuple[str, ...]
) -> tuple[list[dict], dict[tuple[int, int], list[dict]]]:
    root = Path(root)
    allowed = set(allowed_splits)
    rows = [
        {**row, "rgb_path": root / row["rgb_path"]}
        for row in load_jsonl(root / "G6_FRAME_MANIFEST.jsonl")
        if row["split"] in allowed
    ]
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in rows}
    indexed: dict[tuple[int, int], list[dict]] = {}
    for record in load_jsonl(root / "G6_INSTANCE_RECORDS.jsonl"):
        key = (int(record["scene_seed"]), int(record["frame_index"]))
        if key in keys:
            indexed.setdefault(key, []).append(record)
    return rows, indexed


def build_small_specialist_samples(
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    negative_stride: int = 5,
) -> list[dict]:
    """Assign every native <18px object to one best tile plus hard negatives."""
    if negative_stride <= 0:
        raise ValueError("negative_stride must be positive")
    tiles = ground_roi_tiles()
    samples: list[dict] = []
    seen: set[tuple[int, int, int]] = set()
    for row_position, row in enumerate(rows):
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        assigned: dict[int, list[dict]] = {}
        for record in instances_by_key.get(key, []):
            if (
                record["class_id"] not in DISCRETE_NAMES
                or int(record["bbox_short_side_px"]) >= 18
            ):
                continue
            tile_index = _best_tile(record["bbox_xyxy"], tiles)
            if tile_index is not None:
                assigned.setdefault(tile_index, []).append(record)
        for tile_index, targets in assigned.items():
            sample_key = (*key, tile_index)
            seen.add(sample_key)
            samples.append(
                {
                    "rgb_path": row["rgb_path"],
                    "scene_seed": key[0],
                    "frame_index": key[1],
                    "split": row["split"],
                    "tile_index": tile_index,
                    "tile": tiles[tile_index],
                    "targets": targets,
                    "hard_negative": False,
                }
            )
        if row["negative_area_taxonomies"] and row_position % negative_stride == 0:
            tile_index = row_position % len(tiles)
            sample_key = (*key, tile_index)
            if sample_key not in seen:
                samples.append(
                    {
                        "rgb_path": row["rgb_path"],
                        "scene_seed": key[0],
                        "frame_index": key[1],
                        "split": row["split"],
                        "tile_index": tile_index,
                        "tile": tiles[tile_index],
                        "targets": [],
                        "hard_negative": True,
                    }
                )
    return samples


class SmallSpecialistDataset:
    def __init__(self, samples: list[dict], *, class_agnostic: bool = True):
        self.samples = list(samples)
        self.class_agnostic = class_agnostic

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int):
        import torch

        sample = self.samples[index]
        image = cv2.imread(str(sample["rgb_path"]), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"unable to read G6 RGB: {sample['rgb_path']}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        x0, y0, x1, y1 = sample["tile"]
        crop = cv2.resize(
            image[y0:y1, x0:x1], TILE_INPUT_SIZE, interpolation=cv2.INTER_CUBIC
        )
        boxes = []
        for record in sample["targets"]:
            bx0, by0, bx1, by1 = record["bbox_xyxy"]
            boxes.append(
                [
                    (bx0 - x0) * 2.0,
                    (by0 - y0) * 2.0,
                    (bx1 - x0) * 2.0,
                    (by1 - y0) * 2.0,
                ]
            )
        tensor = torch.from_numpy(
            np.ascontiguousarray(crop.transpose(2, 0, 1), dtype=np.float32) / 255.0
        )
        target = {
            "boxes": torch.as_tensor(boxes, dtype=torch.float32).reshape(-1, 4),
            "labels": torch.as_tensor(
                [
                    0
                    if self.class_agnostic
                    else DISCRETE_NAMES.index(record["class_id"]) + 1
                    for record in sample["targets"]
                ],
                dtype=torch.int64,
            ),
        }
        return tensor, target, sample


def small_specialist_collate(batch):
    images, targets, samples = zip(*batch)
    return list(images), list(targets), list(samples)


def build_small_specialist(base_checkpoint: str | Path | None = None):
    """Build a P2 FCOS objectness head initialized from the MRV2-C detector."""
    import torch

    model = build_p2_direct_fcos(input_size=TILE_INPUT_SIZE)
    provenance = None
    if base_checkpoint is not None:
        payload = torch.load(base_checkpoint, map_location="cpu", weights_only=False)
        state = payload["state_dict"]
        model.load_state_dict(state, strict=True)
        provenance = {
            "source": Path(base_checkpoint).as_posix(),
            "architecture": payload.get("architecture"),
        }
    head = model.head.classification_head
    previous = head.cls_logits
    replacement = torch.nn.Conv2d(
        previous.in_channels,
        1,
        kernel_size=previous.kernel_size,
        stride=previous.stride,
        padding=previous.padding,
    )
    with torch.no_grad():
        replacement.weight.copy_(previous.weight.mean(dim=0, keepdim=True))
        replacement.bias.copy_(previous.bias.mean().reshape(1))
    head.cls_logits = replacement
    head.num_classes = 1
    model.model_id = "opr_a_p2_fcos_small_objectness_v1"
    model.architecture_role = "OPR-A_small_specialist_not_frozen"
    model.opr_a_provenance = provenance
    return model


def class_agnostic_nms(
    items: list[dict], iou_threshold: float = 0.5
) -> list[dict]:
    if not items:
        return []
    ordered = sorted(
        items,
        key=lambda item: (-float(item["objectness"]), tuple(item["bbox_xyxy"])),
    )
    kept = []
    for item in ordered:
        if all(_box_iou(item["bbox_xyxy"], other["bbox_xyxy"]) < iou_threshold for other in kept):
            kept.append(item)
    return kept


def _box_iou(left: list[float], right: list[float]) -> float:
    x0 = max(float(left[0]), float(right[0]))
    y0 = max(float(left[1]), float(right[1]))
    x1 = min(float(left[2]), float(right[2]))
    y1 = min(float(left[3]), float(right[3]))
    intersection = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    left_area = max(0.0, float(left[2]) - float(left[0])) * max(
        0.0, float(left[3]) - float(left[1])
    )
    right_area = max(0.0, float(right[2]) - float(right[0])) * max(
        0.0, float(right[3]) - float(right[1])
    )
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0


def _class_aware_nms(items: list[dict], iou_threshold: float) -> list[dict]:
    output = []
    for class_name in DISCRETE_NAMES:
        group = [item for item in items if item["class_name"] == class_name]
        ordered = sorted(group, key=lambda item: -float(item["score"]))
        kept = []
        for item in ordered:
            if all(
                _box_iou(item["bbox_xyxy"], other["bbox_xyxy"]) < iou_threshold
                for other in kept
            ):
                kept.append(item)
        output.extend(kept)
    return sorted(output, key=lambda item: -float(item["score"]))


def fuse_opr_a(
    general: list[dict],
    specialist: list[dict],
    classify,
    *,
    classifier_threshold: float,
    nms_iou: float = 0.5,
) -> list[dict]:
    """Classify specialist boxes, then fuse all boxes in native coordinates."""
    classified = []
    for candidate in class_agnostic_nms(specialist, nms_iou):
        result = classify(candidate)
        if result is None or float(result["class_score"]) < classifier_threshold:
            continue
        class_name = str(result["class_name"])
        if class_name not in DISCRETE_NAMES:
            continue
        classified.append(
            {
                "bbox_xyxy": [float(value) for value in candidate["bbox_xyxy"]],
                "class_name": class_name,
                "class_index": DISCRETE_NAMES.index(class_name) + 1,
                "score": float(candidate["objectness"]) * float(result["class_score"]),
                "objectness": float(candidate["objectness"]),
                "class_score": float(result["class_score"]),
                "proposal_source": "OPR-A_small_specialist",
            }
        )
    normalized_general = [
        {**item, "bbox_xyxy": [float(value) for value in item["bbox_xyxy"]]}
        for item in general
    ]
    return _class_aware_nms(normalized_general + classified, nms_iou)


__all__ = [
    "SmallSpecialistDataset",
    "TILE_INPUT_SIZE",
    "build_small_specialist",
    "build_small_specialist_samples",
    "class_agnostic_nms",
    "fuse_opr_a",
    "ground_roi_tiles",
    "load_g6_rows",
    "map_tile_box_to_native",
    "small_specialist_collate",
]
