"""Frozen DOSOD square-pad geometry shared by PC and board consumers."""
from __future__ import annotations

from dataclasses import dataclass
from math import ceil, floor, isfinite
from typing import Iterable


@dataclass(frozen=True)
class DosodGeometry:
    source_width: int
    source_height: int
    model_size: int
    scale: float
    pad_x: int
    pad_y: int


def square_pad_geometry(source_width: int, source_height: int, model_size: int = 640) -> DosodGeometry:
    if not all(isinstance(value, int) and not isinstance(value, bool) and value > 0 for value in (source_width, source_height, model_size)):
        raise ValueError("DOSOD source/model dimensions invalid")
    side = max(source_width, source_height)
    return DosodGeometry(
        source_width, source_height, model_size, model_size / float(side),
        (side - source_width) // 2, (side - source_height) // 2,
    )


def inverse_model_roi(
    box: Iterable[float], geometry: DosodGeometry, *, integer_roi: bool = False
) -> tuple[tuple[float, float, float, float] | tuple[int, int, int, int] | None, str]:
    """Clip a model-space half-open ROI then map it back to source pixels."""
    try:
        x1, y1, x2, y2 = (float(value) for value in box)
    except (TypeError, ValueError):
        return None, "invalid_box"
    if not all(isfinite(value) for value in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        return None, "invalid_box"
    valid_x1 = geometry.pad_x * geometry.scale
    valid_y1 = geometry.pad_y * geometry.scale
    valid_x2 = (geometry.pad_x + geometry.source_width) * geometry.scale
    valid_y2 = (geometry.pad_y + geometry.source_height) * geometry.scale
    clipped = (max(x1, valid_x1), max(y1, valid_y1), min(x2, valid_x2), min(y2, valid_y2))
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        return None, "padding_only_drop"
    status = "partial_clip" if clipped != (x1, y1, x2, y2) else "kept"
    source = (
        max(0.0, min(float(geometry.source_width), clipped[0] / geometry.scale - geometry.pad_x)),
        max(0.0, min(float(geometry.source_height), clipped[1] / geometry.scale - geometry.pad_y)),
        max(0.0, min(float(geometry.source_width), clipped[2] / geometry.scale - geometry.pad_x)),
        max(0.0, min(float(geometry.source_height), clipped[3] / geometry.scale - geometry.pad_y)),
    )
    if source[2] <= source[0] or source[3] <= source[1]:
        return None, "padding_only_drop"
    if integer_roi:
        result = (floor(source[0]), floor(source[1]), ceil(source[2]), ceil(source[3]))
        if result[2] <= result[0] or result[3] <= result[1]:
            return None, "padding_only_drop"
        return result, status
    return source, status
