"""Native<->model bbox geometry helpers for the G4 discovery task.

The discovery detector resizes full RGB frames from a native capture
resolution (for example 640x480) to ``DISCOVERY_MODEL_SIZE``.  Every bbox
conversion in this module derives its scale factors from the actual frame
shape and the requested model size; there is intentionally no hard-coded
``384``/``512`` fallback.  All helpers are pure Python so the round-trip
contract can be exercised by the fast, ROS-independent test suite.
"""

from __future__ import annotations

from typing import Iterable


def validate_native_size(native_size) -> tuple[int, int]:
    """Return ``(width, height)`` after strict validation."""
    try:
        width, height = int(native_size[0]), int(native_size[1])
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError(
            f"native_size must be a (width, height) pair, got {native_size!r}"
        ) from exc
    if width < 1 or height < 1:
        raise ValueError(
            f"native_size must be positive, got {(width, height)!r}"
        )
    return width, height


def validate_model_size(model_size) -> tuple[int, int]:
    """Return ``(width, height)`` after strict validation."""
    try:
        width, height = int(model_size[0]), int(model_size[1])
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError(
            f"model_size must be a (width, height) pair, got {model_size!r}"
        ) from exc
    if width < 1 or height < 1:
        raise ValueError(
            f"model_size must be positive, got {(width, height)!r}"
        )
    return width, height


def validate_bbox(bbox) -> tuple[float, float, float, float]:
    """Validate an xyxy box and return it as floats.

    Boxes must have strictly positive area and ordered coordinates.  This
    prevents silently inverted or zero-area targets from entering training.
    """
    try:
        values = tuple(float(value) for value in bbox)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"bbox must be four numbers, got {bbox!r}") from exc
    if len(values) != 4:
        raise ValueError(f"bbox must be four numbers, got {bbox!r}")
    x1, y1, x2, y2 = values
    if not (x2 > x1 and y2 > y1):
        raise ValueError(f"bbox must be ordered with positive area: {bbox!r}")
    return x1, y1, x2, y2


def _clamp_ordered(
    bbox: tuple[float, float, float, float],
    width: int,
    height: int,
) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = bbox
    x1 = max(0.0, min(float(width), x1))
    y1 = max(0.0, min(float(height), y1))
    x2 = max(0.0, min(float(width), x2))
    y2 = max(0.0, min(float(height), y2))
    x2 = min(x2, float(width))
    y2 = min(y2, float(height))
    if x2 <= x1 or y2 <= y1:
        raise ValueError("bbox collapses after clamping to image bounds")
    return x1, y1, x2, y2


def _require_bounded(bbox, size, space: str) -> None:
    width, height = validate_native_size(size)
    x1, y1, x2, y2 = validate_bbox(bbox)
    if not (
        0.0 <= x1 < x2 <= float(width)
        and 0.0 <= y1 < y2 <= float(height)
    ):
        raise ValueError(
            f"{space} bbox {bbox!r} is outside {(width, height)!r}"
        )


def bbox_native_to_model(
    bbox,
    native_size,
    model_size,
) -> tuple[float, float, float, float]:
    """Scale a native-space xyxy box into model space.

    Scale factors are always derived from the supplied sizes; a caller may
    never rely on a fixed-resolution assumption.
    """
    native_width, native_height = validate_native_size(native_size)
    model_width, model_height = validate_model_size(model_size)
    x1, y1, x2, y2 = validate_bbox(bbox)
    _require_bounded((x1, y1, x2, y2), (native_width, native_height), "native")
    scale_x = model_width / native_width
    scale_y = model_height / native_height
    scaled = (
        x1 * scale_x,
        y1 * scale_y,
        x2 * scale_x,
        y2 * scale_y,
    )
    return _clamp_ordered(scaled, model_width, model_height)


def bbox_model_to_native(
    bbox,
    native_size,
    model_size,
) -> tuple[float, float, float, float]:
    """Scale a model-space xyxy box back into native space."""
    native_width, native_height = validate_native_size(native_size)
    model_width, model_height = validate_model_size(model_size)
    x1, y1, x2, y2 = validate_bbox(bbox)
    _require_bounded((x1, y1, x2, y2), (model_width, model_height), "model")
    scale_x = native_width / model_width
    scale_y = native_height / model_height
    scaled = (
        x1 * scale_x,
        y1 * scale_y,
        x2 * scale_x,
        y2 * scale_y,
    )
    return _clamp_ordered(scaled, native_width, native_height)


def flip_bbox_horizontal(bbox, width) -> tuple[float, float, float, float]:
    """Mirror a native-space xyxy box across the image vertical axis."""
    x1, y1, x2, y2 = validate_bbox(bbox)
    frame_width = validate_native_size((width, 1))[0]
    if not (0.0 <= x1 <= x2 <= frame_width):
        raise ValueError(
            f"box {bbox!r} is outside the native frame width {frame_width}"
        )
    flipped = (frame_width - x2, y1, frame_width - x1, y2)
    return flipped


def remap_flipped_box(
    box: dict,
    native_size,
    model_size,
) -> dict:
    """Return an updated box after a horizontal flip.

    The flipped native bbox is derived first and the model bbox is regenerated
    from the unified native->model utility, so the model coordinates can never
    drift from the native coordinates.
    """
    if "native_bbox_xyxy" not in box:
        raise ValueError("box must carry native_bbox_xyxy for flip remapping")
    native_width, _ = validate_native_size(native_size)
    flipped_native = flip_bbox_horizontal(
        box["native_bbox_xyxy"], native_width
    )
    model_bbox = bbox_native_to_model(
        flipped_native, native_size, model_size
    )
    updated = dict(box)
    updated["native_bbox_xyxy"] = list(flipped_native)
    updated["model_bbox_xyxy"] = list(model_bbox)
    updated["bbox_xyxy"] = list(model_bbox)
    return updated


def max_coordinate_error(
    first,
    second,
) -> float:
    """Maximum per-coordinate absolute error between two xyxy boxes."""
    a = tuple(float(value) for value in first)
    b = tuple(float(value) for value in second)
    if len(a) != 4 or len(b) != 4:
        raise ValueError("both boxes must have four coordinates")
    return max(abs(av - bv) for av, bv in zip(a, b))


def bbox_is_bounded(bbox, size) -> bool:
    """Return whether an xyxy box is ordered and inside ``(width, height)``."""
    width, height = validate_native_size(size)
    x1, y1, x2, y2 = validate_bbox(bbox)
    return (
        0.0 <= x1 < x2 <= float(width)
        and 0.0 <= y1 < y2 <= float(height)
    )


def boxes_round_trip_error(
    boxes: Iterable[dict],
    native_size,
    model_size,
) -> float:
    """Maximum native round-trip error for a list of box dicts."""
    worst = 0.0
    for box in boxes:
        model_box = bbox_native_to_model(
            box["native_bbox_xyxy"], native_size, model_size
        )
        restored = bbox_model_to_native(
            model_box, native_size, model_size
        )
        worst = max(
            worst,
            max_coordinate_error(restored, box["native_bbox_xyxy"]),
        )
    return float(worst)


__all__ = [
    "bbox_is_bounded",
    "bbox_model_to_native",
    "bbox_native_to_model",
    "boxes_round_trip_error",
    "flip_bbox_horizontal",
    "max_coordinate_error",
    "remap_flipped_box",
    "validate_bbox",
    "validate_model_size",
    "validate_native_size",
]
