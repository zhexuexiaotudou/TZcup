"""Deterministic ground-ROI tiled refinement for MRV2-B."""

from __future__ import annotations

import cv2
import numpy as np

from .g4_data import DISCRETE_NAMES, discrete_boxes_for_frame, read_rgb


def ground_tile_specs(width: int, height: int, mode: str) -> list[tuple[int, int, int, int]]:
    ground_top = int(round(height * 0.30))
    if mode == "ground3":
        tile_width = int(round(width * 0.50))
        starts = (0, int(round(width * 0.25)), width - tile_width)
        return [(start, ground_top, start + tile_width, height) for start in starts]
    if mode == "ground2x2":
        tile_width = int(round(width * 0.58))
        tile_height = int(round((height - ground_top) * 0.62))
        xs = (0, width - tile_width)
        ys = (ground_top, height - tile_height)
        return [(x, y, x + tile_width, y + tile_height) for y in ys for x in xs]
    raise ValueError(f"unknown tile mode: {mode}")


def map_tile_box(
    box: list[float], tile: tuple[int, int, int, int],
    tile_input_size: tuple[int, int], output_size: tuple[int, int],
    native_size: tuple[int, int],
) -> list[float]:
    tx1, ty1, tx2, ty2 = tile
    tile_width, tile_height = tx2 - tx1, ty2 - ty1
    native_width, native_height = native_size
    output_width, output_height = output_size
    x1 = tx1 + float(box[0]) * tile_width / tile_input_size[0]
    y1 = ty1 + float(box[1]) * tile_height / tile_input_size[1]
    x2 = tx1 + float(box[2]) * tile_width / tile_input_size[0]
    y2 = ty1 + float(box[3]) * tile_height / tile_input_size[1]
    return [
        x1 * output_width / native_width,
        y1 * output_height / native_height,
        x2 * output_width / native_width,
        y2 * output_height / native_height,
    ]


def class_aware_nms(items: list[dict], iou_threshold: float = 0.5) -> list[dict]:
    if not items:
        return []
    import torch
    from torchvision.ops import nms

    output = []
    for class_name in DISCRETE_NAMES:
        group = [item for item in items if item["class_name"] == class_name]
        if not group:
            continue
        boxes = torch.as_tensor([item["bbox_xyxy"] for item in group], dtype=torch.float32)
        scores = torch.as_tensor([item["score"] for item in group], dtype=torch.float32)
        kept = nms(boxes, scores, iou_threshold).tolist()
        output.extend(group[index] for index in kept)
    return sorted(
        output,
        key=lambda item: (-float(item["score"]), item["class_name"], tuple(item["bbox_xyxy"])),
    )


def _tensor(rgb, input_size):
    import torch
    resized = cv2.resize(rgb, input_size, interpolation=cv2.INTER_CUBIC)
    return torch.from_numpy(
        np.ascontiguousarray(resized.transpose(2, 0, 1), dtype=np.float32) / 255.0
    )


def _model_items(model, image, device, score_floor):
    import torch
    with torch.no_grad():
        output = model([image.to(device)])[0]
    items = []
    for box, score, label in zip(
        output["boxes"].cpu().tolist(),
        output["scores"].cpu().tolist(),
        output["labels"].cpu().tolist(),
    ):
        if float(score) < score_floor:
            continue
        items.append({
            "class_index": int(label) + 1,
            "class_name": DISCRETE_NAMES[int(label)],
            "score": float(score),
            "bbox_xyxy": [float(value) for value in box],
        })
    return items


def tiled_direct_predictions(
    full_model, tile_model, rows, instances_by_key, *, device,
    full_input_size=(960, 720), tile_input_size=(640, 480),
    mode="ground3", score_floor=0.01, tile_score_scale=0.95,
    top_k=100, nms_iou=0.5,
):
    full_model.eval(); tile_model.eval(); frames = []
    for row in rows:
        rgb = read_rgb(row)
        native_size = (int(rgb.shape[1]), int(rgb.shape[0]))
        items = _model_items(full_model, _tensor(rgb, full_input_size), device, score_floor)
        for tile in ground_tile_specs(*native_size, mode):
            x1, y1, x2, y2 = tile
            crop = rgb[y1:y2, x1:x2]
            for item in _model_items(tile_model, _tensor(crop, tile_input_size), device, score_floor):
                items.append({
                    **item,
                    "score": float(item["score"]) * tile_score_scale,
                    "bbox_xyxy": map_tile_box(
                        item["bbox_xyxy"], tile, tile_input_size,
                        full_input_size, native_size,
                    ),
                    "proposal_source": mode,
                })
        items = class_aware_nms(items, nms_iou)[:top_k]
        truth = discrete_boxes_for_frame(row, instances_by_key, model_size=full_input_size)
        frames.append({
            "row": row, "scene_seed": int(row["scene_seed"]),
            "frame_index": int(row["frame_index"]), "split": row["split"],
            "world_id": row["world_id"], "negative_only": bool(row.get("negative_only", False)),
            "detections": [dict(item) for item in items],
            "predictions": [dict(item) for item in items], "truth": truth,
        })
    return frames


__all__ = ["class_aware_nms", "ground_tile_specs", "map_tile_box", "tiled_direct_predictions"]
