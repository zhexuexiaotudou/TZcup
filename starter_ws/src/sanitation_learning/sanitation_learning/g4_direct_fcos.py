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
MRV2_C_P2_ARCHITECTURE = "torchvision_fcos_resnet50_fpn_p2_direct_3class"


def _torch():
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for direct FCOS") from exc
    return torch


def direct_input_size(
    input_scale: int = 1, input_size: tuple[int, int] | None = None
) -> tuple[int, int]:
    """Resolve an explicit width/height while preserving the X3 scale API."""
    if input_size is None:
        return teacher_input_size(input_scale)
    width, height = (int(value) for value in input_size)
    if width <= 0 or height <= 0 or width % 8 or height % 8:
        raise ValueError("direct FCOS input dimensions must be positive multiples of 8")
    return width, height


def build_direct_fcos(
    input_scale: int = 1, *, input_size: tuple[int, int] | None = None
):
    """Build a three-class detector from exact official FCOS COCO weights."""
    torch = _torch()
    try:
        import torchvision
    except ImportError as exc:
        raise RuntimeError("torchvision is required for direct FCOS") from exc

    input_size = direct_input_size(input_scale, input_size)
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


def build_p2_direct_fcos(*, input_size: tuple[int, int] = (960, 720)):
    """Build the MRV2-C six-level FCOS variant with an explicit stride-4 P2.

    The ordinary Torchvision FCOS factory starts at stride 8.  MRV2-C keeps
    the same closed three-class head but exposes ResNet layer1 through the FPN
    and assigns a 4-pixel anchor to the new first level.
    """
    _torch()
    try:
        from torchvision.models.detection import FCOS
        from torchvision.models.detection.anchor_utils import AnchorGenerator
        from torchvision.models.detection.backbone_utils import resnet_fpn_backbone
        from torchvision.ops.feature_pyramid_network import LastLevelP6P7
    except ImportError as exc:
        raise RuntimeError("torchvision is required for P2 direct FCOS") from exc

    input_size = direct_input_size(input_size=input_size)
    backbone = resnet_fpn_backbone(
        backbone_name="resnet50",
        weights=None,
        trainable_layers=5,
        returned_layers=[1, 2, 3, 4],
        extra_blocks=LastLevelP6P7(256, 256),
    )
    anchors = AnchorGenerator(
        sizes=((4,), (8,), (16,), (32,), (64,), (128,)),
        aspect_ratios=((1.0,),) * 6,
    )
    model = FCOS(
        backbone,
        num_classes=len(DISCRETE_NAMES),
        anchor_generator=anchors,
        min_size=input_size[1],
        max_size=input_size[0],
        box_score_thresh=0.01,
        box_detections_per_img=100,
        topk_candidates=1000,
    )
    model.architecture_role = "x86_product_candidate_not_frozen"
    model.model_id = "mrv2_c_fcos_resnet50_fpn_p2_direct_3class_v1"
    model.input_size = input_size
    return model


def load_direct_state_into_p2(model, direct_state: dict) -> dict:
    """Transplant a trained five-level direct FCOS into the six-level P2 model.

    ResNet body, detection heads, and P3-P7 FPN weights are preserved.  Only
    the newly introduced P2 lateral/output convolutions remain initialized by
    Torchvision.
    """
    destination = model.state_dict()
    loaded = {}
    skipped = []
    for source_name, value in direct_state.items():
        target_name = source_name
        for prefix in ("backbone.fpn.inner_blocks.", "backbone.fpn.layer_blocks."):
            if source_name.startswith(prefix):
                suffix = source_name[len(prefix):]
                index, separator, rest = suffix.partition(".")
                if index.isdigit() and separator:
                    target_name = f"{prefix}{int(index) + 1}.{rest}"
                break
        if target_name in destination and destination[target_name].shape == value.shape:
            loaded[target_name] = value
        else:
            skipped.append(source_name)
    missing, unexpected = model.load_state_dict(loaded, strict=False)
    allowed_missing = {
        name for name in missing
        if name.startswith("backbone.fpn.inner_blocks.0.")
        or name.startswith("backbone.fpn.layer_blocks.0.")
    }
    unresolved = sorted(set(missing) - allowed_missing)
    if unresolved or unexpected:
        raise RuntimeError(
            f"P2 transplant unresolved missing={unresolved} unexpected={unexpected}"
        )
    return {
        "loaded_tensor_count": len(loaded),
        "new_p2_tensor_names": sorted(allowed_missing),
        "source_tensor_count": len(direct_state),
        "skipped_source_tensor_names": sorted(skipped),
    }


class DirectFCOSDataset:
    def __init__(
        self, rows, instances_by_key, *, input_scale: int = 1,
        input_size: tuple[int, int] | None = None,
    ):
        self.rows = list(rows)
        self.instances_by_key = instances_by_key
        self.input_size = direct_input_size(input_scale, input_size)

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
    input_size: tuple[int, int] | None = None,
    top_k: int = 16,
):
    torch = _torch()
    from torch.utils.data import DataLoader

    resolved_input_size = direct_input_size(input_scale, input_size)
    dataset = DirectFCOSDataset(
        rows, instances_by_key, input_scale=input_scale, input_size=resolved_input_size
    )
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
                    row, instances_by_key, model_size=resolved_input_size
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
    "direct_input_size",
    "X3_ARCHITECTURE",
    "X3_WEIGHT_SPEC",
    "build_direct_fcos",
    "direct_fcos_collate",
    "direct_predictions",
]
