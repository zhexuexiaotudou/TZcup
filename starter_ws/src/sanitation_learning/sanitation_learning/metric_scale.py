"""AUTO-05R-0 native/model-input metric scale contract.

The AUTO-05 detector matches predictions on model-input-scale boxes while the
machine-evaluable and small-object buckets must be decided on native-scale
fields.  This module makes that distinction explicit and machine checkable.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

import cv2
import numpy as np


class MetricScale(Enum):
    NATIVE_SCALE = "native"
    MODEL_INPUT_SCALE = "model_input"


NATIVE_SCALE_FIELDS = (
    "native_bbox_xyxy",
    "native_short_side_px",
    "native_mask_area_px",
)
MODEL_SCALE_FIELDS = (
    "model_bbox_xyxy",
    "model_short_side_px",
    "model_mask_area_px",
)
ALL_SCALE_FIELDS = NATIVE_SCALE_FIELDS + MODEL_SCALE_FIELDS


@dataclass(frozen=True)
class InstanceScaleRecord:
    native_bbox_xyxy: tuple[float, float, float, float]
    native_short_side_px: float
    native_mask_area_px: int
    model_bbox_xyxy: tuple[float, float, float, float]
    model_short_side_px: float
    model_mask_area_px: int

    def to_dict(self) -> dict[str, object]:
        return {
            "native_bbox_xyxy": list(self.native_bbox_xyxy),
            "native_short_side_px": self.native_short_side_px,
            "native_mask_area_px": self.native_mask_area_px,
            "model_bbox_xyxy": list(self.model_bbox_xyxy),
            "model_short_side_px": self.model_short_side_px,
            "model_mask_area_px": self.model_mask_area_px,
        }


def _validate_size(size: tuple[int, int] | list[int] | tuple[float, float]) -> tuple[float, float]:
    if len(size) != 2:
        raise ValueError("size must be (width, height)")
    width, height = float(size[0]), float(size[1])
    if width <= 0.0 or height <= 0.0:
        raise ValueError("size must be positive")
    return width, height


def _coerce_native(
    record: InstanceScaleRecord | Mapping[str, object],
) -> tuple[tuple[float, float, float, float], float, int]:
    if isinstance(record, InstanceScaleRecord):
        return record.native_bbox_xyxy, record.native_short_side_px, record.native_mask_area_px
    missing = [name for name in NATIVE_SCALE_FIELDS if name not in record]
    if missing:
        raise ValueError(f"native scale fields missing: {', '.join(missing)}")
    bbox = tuple(float(value) for value in record["native_bbox_xyxy"])  # type: ignore[index]
    if len(bbox) != 4:
        raise ValueError("native_bbox_xyxy must contain four values")
    return bbox, float(record["native_short_side_px"]), int(record["native_mask_area_px"])  # type: ignore[index]


def assert_scale_fields_present(record: InstanceScaleRecord | Mapping[str, object]) -> None:
    """Raise ValueError unless every native and model scale field exists."""
    if isinstance(record, InstanceScaleRecord):
        return
    missing = [name for name in ALL_SCALE_FIELDS if name not in record]
    if missing:
        raise ValueError(f"scale fields missing: {', '.join(missing)}")


def derive_model_scale_record(
    record: InstanceScaleRecord | Mapping[str, object],
    source_size: tuple[int, int],
    target_size: tuple[int, int],
    model_mask: np.ndarray | None = None,
) -> InstanceScaleRecord:
    """Derive model-input scale fields from a native-scale record.

    The bbox is linearly scaled and the short side is recomputed from the
    model-scale bbox.  When ``model_mask`` is provided the model mask area is
    the exact pixel count after ``cv2.resize(..., INTER_NEAREST)``; native mask
    area is never used as a substitute in that path.  Without a mask the area
    falls back to proportional area scaling and callers should prefer passing
    the mask whenever it is available.
    """
    source_width, source_height = _validate_size(source_size)
    target_width, target_height = _validate_size(target_size)
    native_bbox, native_short_side, native_mask_area = _coerce_native(record)
    scale_x = target_width / source_width
    scale_y = target_height / source_height
    model_bbox = (
        native_bbox[0] * scale_x,
        native_bbox[1] * scale_y,
        native_bbox[2] * scale_x,
        native_bbox[3] * scale_y,
    )
    model_short_side = float(min(model_bbox[2] - model_bbox[0], model_bbox[3] - model_bbox[1]))
    if model_mask is not None:
        mask = np.asarray(model_mask, dtype=np.uint8)
        if mask.ndim != 2:
            raise ValueError("model_mask must be a 2D boolean/uint8 array")
        if (mask.shape[1], mask.shape[0]) != (int(source_width), int(source_height)):
            raise ValueError(
                "model_mask shape must match source resolution "
                f"{(int(source_width), int(source_height))}, got {mask.shape}"
            )
        resized = cv2.resize(
            mask,
            (int(target_width), int(target_height)),
            interpolation=cv2.INTER_NEAREST,
        )
        model_mask_area = int(np.count_nonzero(resized))
    else:
        model_mask_area = int(round(native_mask_area * scale_x * scale_y))
    return InstanceScaleRecord(
        native_bbox_xyxy=native_bbox,
        native_short_side_px=native_short_side,
        native_mask_area_px=native_mask_area,
        model_bbox_xyxy=model_bbox,
        model_short_side_px=model_short_side,
        model_mask_area_px=model_mask_area,
    )


def _fields_for_scale(
    record: InstanceScaleRecord | Mapping[str, object],
    scale: MetricScale,
) -> tuple[float, int]:
    if scale is MetricScale.NATIVE_SCALE:
        if isinstance(record, InstanceScaleRecord):
            return record.native_short_side_px, record.native_mask_area_px
        missing = [name for name in NATIVE_SCALE_FIELDS if name not in record]
        if missing:
            raise ValueError(f"native scale fields missing: {', '.join(missing)}")
        return float(record["native_short_side_px"]), int(record["native_mask_area_px"])  # type: ignore[index]
    if scale is MetricScale.MODEL_INPUT_SCALE:
        if isinstance(record, InstanceScaleRecord):
            return record.model_short_side_px, record.model_mask_area_px
        missing = [name for name in MODEL_SCALE_FIELDS if name not in record]
        if missing:
            raise ValueError(f"model scale fields missing: {', '.join(missing)}")
        return float(record["model_short_side_px"]), int(record["model_mask_area_px"])  # type: ignore[index]
    raise ValueError(f"unknown metric scale: {scale!r}")


def machine_evaluable_bucket(
    record: InstanceScaleRecord | Mapping[str, object],
    scale: MetricScale = MetricScale.NATIVE_SCALE,
    min_short_side_px: float = 8.0,
    min_mask_area_px: float = 20.0,
) -> bool:
    """Return True when the record falls into the machine-evaluable bucket."""
    short_side, mask_area = _fields_for_scale(record, scale)
    return (
        float(short_side) >= float(min_short_side_px)
        and float(mask_area) >= float(min_mask_area_px)
    )


def small_object_bucket(
    record: InstanceScaleRecord | Mapping[str, object],
    scale: MetricScale = MetricScale.NATIVE_SCALE,
    max_short_side_px: float = 18.0,
) -> bool:
    """Return True when the record falls into the small-object bucket."""
    short_side, _ = _fields_for_scale(record, scale)
    return float(short_side) < float(max_short_side_px)


def bbox_short_side_px(bbox_xyxy: tuple[float, float, float, float]) -> float:
    """Short side of an xyxy bbox; convenience used by record construction."""
    return float(min(bbox_xyxy[2] - bbox_xyxy[0], bbox_xyxy[3] - bbox_xyxy[1]))
