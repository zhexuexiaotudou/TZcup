"""Official Torchvision FCOS-R50 direct three-class X3 candidate."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .g4_data import DISCRETE_NAMES, discrete_boxes_for_frame, read_rgb
from .g4_pretrained import provenance_record, torchvision_cache_path
from .g4_teacher import teacher_input_size


X3_ARCHITECTURE = "torchvision_fcos_resnet50_fpn_direct_3class"
X3_WEIGHT_SPEC = "fcos_resnet50_fpn_coco"


def _torch():
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for direct FCOS") from exc
    return torch


def build_direct_fcos(input_scale: int = 1):
    """Build a three-class detector from exact official FCOS COCO weights."""
    torch = _torch()
    try:
        import torchvision
    except ImportError as exc:
        raise RuntimeError("torchvision is required for direct FCOS") from exc

    input_size = teacher_input_size(input_scale)
    weights = torchvision.models.detection.FCOS_ResNet50_FPN_Weights.COCO_V1
    model = torchvision.models.detection.fcos_resnet50_fpn(
        weights=weights,
        min_size=input_size[1],
        max_size=input_size[0],
        box_score_thresh=0.01,
        box_detections_per_img=100,
        topk_candidates=1000,
    )
    classification = model.head.classification_head
    replacement = torch.nn.Conv2d(
        256, len(DISCRETE_NAMES), kernel_size=3, stride=1, padding=1
    )
    torch.nn.init.normal_(replacement.weight, std=0.01)
    torch.nn.init.constant_(
        replacement.bias, -math.log((1.0 - 0.01) / 0.01)
    )
    classification.cls_logits = replacement
    classification.num_classes = len(DISCRETE_NAMES)
    model.provenance = provenance_record(
        X3_WEIGHT_SPEC,
        cache_path=torchvision_cache_path(X3_WEIGHT_SPEC),
        torchvision_version=torchvision.__version__,
    )
    model.architecture_role = "x86_product_candidate_not_frozen"
    model.model_id = "x3_fcos_resnet50_fpn_direct_3class_v1"
    model.input_size = input_size
    return model


class DirectFCOSDataset:
    def __init__(self, rows, instances_by_key, *, input_scale: int = 1):
        self.rows = list(rows)
        self.instances_by_key = instances_by_key
        self.input_size = teacher_input_size(input_scale)

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        torch = _torch()
        row = self.rows[index]
        rgb = read_rgb(row)
        native_size = (int(rgb.shape[1]), int(rgb.shape[0]))
        truth = discrete_boxes_for_frame(
            row,
            self.instances_by_key,
            native_size=native_size,
            model_size=self.input_size,
        )
        resized = cv2.resize(
            rgb,
            self.input_size,
            interpolation=cv2.INTER_AREA if self.input_size == native_size else cv2.INTER_CUBIC,
        )
        image = torch.from_numpy(
            np.ascontiguousarray(resized.transpose(2, 0, 1), dtype=np.float32) / 255.0
        )
        boxes = torch.as_tensor(
            [item["bbox_xyxy"] for item in truth], dtype=torch.float32
        ).reshape(-1, 4)
        labels = torch.as_tensor(
            [DISCRETE_NAMES.index(item["semantic_class"]) for item in truth],
            dtype=torch.int64,
        )
        return image, {"boxes": boxes, "labels": labels}, row


def direct_fcos_collate(batch):
    images, targets, rows = zip(*batch)
    return list(images), list(targets), list(rows)


def direct_predictions(
    model,
    rows,
    instances_by_key,
    *,
    device,
    score_threshold: float,
    batch_size: int = 4,
    input_scale: int = 1,
    top_k: int = 16,
):
    torch = _torch()
    from torch.utils.data import DataLoader

    dataset = DirectFCOSDataset(rows, instances_by_key, input_scale=input_scale)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=direct_fcos_collate,
    )
    frames = []
    model.eval()
    with torch.no_grad():
        for images, _targets, batch_rows in loader:
            outputs = model([image.to(device) for image in images])
            for output, row in zip(outputs, batch_rows):
                items = []
                for box, score, label in zip(
                    output["boxes"].cpu().tolist(),
                    output["scores"].cpu().tolist(),
                    output["labels"].cpu().tolist(),
                ):
                    if float(score) < score_threshold:
                        continue
                    class_zero = int(label)
                    if not 0 <= class_zero < len(DISCRETE_NAMES):
                        raise RuntimeError(f"direct FCOS produced invalid class {class_zero}")
                    items.append(
                        {
                            "class_index": class_zero + 1,
                            "class_name": DISCRETE_NAMES[class_zero],
                            "score": float(score),
                            "bbox_xyxy": [float(value) for value in box],
                        }
                    )
                items = sorted(items, key=lambda item: item["score"], reverse=True)[:top_k]
                truth = discrete_boxes_for_frame(
                    row, instances_by_key, model_size=teacher_input_size(input_scale)
                )
                frames.append(
                    {
                        "row": row,
                        "scene_seed": int(row["scene_seed"]),
                        "frame_index": int(row["frame_index"]),
                        "split": row["split"],
                        "world_id": row["world_id"],
                        "negative_only": bool(row.get("negative_only", False)),
                        "detections": [dict(item) for item in items],
                        "predictions": [dict(item) for item in items],
                        "truth": truth,
                    }
                )
    return frames


__all__ = [
    "DirectFCOSDataset",
    "X3_ARCHITECTURE",
    "X3_WEIGHT_SPEC",
    "build_direct_fcos",
    "direct_fcos_collate",
    "direct_predictions",
]
