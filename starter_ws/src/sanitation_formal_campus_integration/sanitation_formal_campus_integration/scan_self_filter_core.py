"""Pure geometry contract for the formal UTM-30LX self-return mask."""

from __future__ import annotations

import math
from typing import Iterable, Sequence


def parse_masks(values: Sequence[float]) -> tuple[tuple[float, float, float], ...]:
    """Validate flattened ``start_angle, end_angle, max_range`` triplets."""
    if len(values) % 3:
        raise ValueError("angular range masks must contain complete triplets")
    masks = []
    for index in range(0, len(values), 3):
        start, end, maximum = (float(value) for value in values[index:index + 3])
        if not all(math.isfinite(value) for value in (start, end, maximum)):
            raise ValueError("angular range masks must be finite")
        if start > end or maximum <= 0.0:
            raise ValueError("angular range mask bounds are invalid")
        masks.append((start, end, maximum))
    if any(left[1] >= right[0] for left, right in zip(masks, masks[1:])):
        raise ValueError("angular range masks must be ordered and non-overlapping")
    return tuple(masks)


def is_self_return(
    *,
    angle_rad: float,
    range_m: float,
    masks: Iterable[tuple[float, float, float]],
) -> bool:
    """Return true only for a finite hit inside a mesh-derived self mask."""
    if not math.isfinite(angle_rad) or not math.isfinite(range_m):
        return False
    return any(
        start <= angle_rad <= end and 0.0 <= range_m <= maximum
        for start, end, maximum in masks
    )


def filter_ranges(
    *,
    angle_min: float,
    angle_increment: float,
    ranges: Sequence[float],
    masks: Iterable[tuple[float, float, float]],
    range_max: float | None = None,
    normalize_positive_infinity: bool = False,
    no_return_replacement_m: float | None = None,
) -> tuple[list[float], int, int]:
    """Mask self returns and normalize physical no-return rays.

    NaN is deliberate: an infinity value can be interpreted as observed free
    space when a costmap has ``inf_is_valid`` enabled.  A ray physically
    blocked by the vehicle says nothing about the space behind that vehicle.

    A positive infinity from the formal lidar has the opposite meaning: no
    obstacle returned anywhere inside the sensor's physical range.  When
    enabled, it becomes exactly the formal SLAM raster range threshold.  Karto
    includes values at the threshold in the scan bounding box, but marks an
    endpoint occupied only when the value is strictly below the threshold by
    its tolerance.  The equality therefore expands an open-field map with a
    free-space ray without creating a false occupied endpoint ring.
    """
    if normalize_positive_infinity and (
        range_max is None or not math.isfinite(range_max) or range_max <= 0.0
    ):
        raise ValueError("positive-infinity normalization requires finite range_max")
    if normalize_positive_infinity and (
        no_return_replacement_m is None
        or not math.isfinite(no_return_replacement_m)
        or no_return_replacement_m <= 0.0
        or no_return_replacement_m >= float(range_max)
    ):
        raise ValueError(
            "no-return replacement must lie strictly inside the physical range"
        )
    mask_rows = tuple(masks)
    filtered = list(ranges)
    self_return_count = 0
    no_return_count = 0
    for index, distance in enumerate(filtered):
        if is_self_return(
            angle_rad=angle_min + index * angle_increment,
            range_m=distance,
            masks=mask_rows,
        ):
            filtered[index] = math.nan
            self_return_count += 1
        elif normalize_positive_infinity and math.isinf(distance) and distance > 0.0:
            filtered[index] = float(no_return_replacement_m)
            no_return_count += 1
    return filtered, self_return_count, no_return_count
