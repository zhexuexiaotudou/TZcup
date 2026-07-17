from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DiscObject:
    name: str
    x_m: float
    y_m: float
    radius_m: float
    is_target: bool = True


def visible_targets(camera_xy: tuple[float, float], objects: list[DiscObject]) -> dict[str, dict]:
    """Simple auditable 2-D ray occlusion fallback for semantic truth.

    A closer disc fully suppresses a farther target when its angular interval covers
    the target interval. Partial overlap reports a conservative occlusion ratio.
    """
    cx, cy = camera_xy
    polar = []
    for obj in objects:
        dx, dy = obj.x_m - cx, obj.y_m - cy
        distance = math.hypot(dx, dy)
        if distance <= obj.radius_m:
            continue
        angle = math.atan2(dy, dx)
        half_width = math.asin(min(obj.radius_m / distance, 0.999))
        polar.append((distance, angle, half_width, obj))
    polar.sort(key=lambda item: item[0])
    occluder_intervals: list[tuple[float, float]] = []
    result = {}
    for _, angle, half_width, obj in polar:
        low, high = angle - half_width, angle + half_width
        span = max(high - low, 1e-9)
        overlap = 0.0
        for occ_low, occ_high in occluder_intervals:
            overlap += max(0.0, min(high, occ_high) - max(low, occ_low))
        ratio = min(overlap / span, 1.0)
        if obj.is_target and ratio < 0.999:
            result[obj.name] = {"visibility": 1.0 - ratio, "occlusion_ratio": ratio}
        occluder_intervals.append((low, high))
    return result
