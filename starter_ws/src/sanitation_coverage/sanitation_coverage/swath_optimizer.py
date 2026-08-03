"""Deterministic angle and spacing optimizers for rectangular/polygonal fields."""

from dataclasses import dataclass
import math
from typing import Iterable


Point = tuple[float, float]


def _rotate(point: Point, angle_rad: float) -> Point:
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return (
        point[0] * cosine + point[1] * sine,
        -point[0] * sine + point[1] * cosine,
    )


@dataclass(frozen=True)
class AngleCandidate:
    angle_deg: float
    swath_count: int
    primary_length_m: float
    connector_lower_bound_m: float
    score: float


def evaluate_swath_angle(
    polygon: Iterable[Point], angle_deg: float, spacing_m: float
) -> AngleCandidate:
    points = [_rotate(point, math.radians(angle_deg)) for point in polygon]
    if len(points) < 3 or spacing_m <= 0.0:
        raise ValueError("polygon and positive spacing_m are required")
    width = max(p[0] for p in points) - min(p[0] for p in points)
    height = max(p[1] for p in points) - min(p[1] for p in points)
    count = max(1, int(math.ceil(height / spacing_m)))
    primary = count * width
    connectors = max(0, count - 1) * spacing_m
    # Path length dominates; a small component penalty breaks near-ties toward
    # fewer turns, which is especially important for skid-steer vehicles.
    score = primary + 2.0 * connectors + 0.20 * max(0, count - 1)
    return AngleCandidate(angle_deg % 180.0, count, primary, connectors, score)


def optimize_swath_angle(
    polygon: Iterable[Point], spacing_m: float, step_deg: int = 5
) -> tuple[AngleCandidate, list[AngleCandidate]]:
    candidates = [
        evaluate_swath_angle(polygon, angle, spacing_m)
        for angle in range(0, 180, step_deg)
    ]
    best = min(candidates, key=lambda item: (round(item.score, 9), item.angle_deg))
    return best, candidates


@dataclass(frozen=True)
class SpacingCandidate:
    spacing_m: float
    coverage_rate: float
    repeat_rate: float
    path_length_m: float
    valid: bool
    score: float


def optimize_swath_spacing(
    observations: Iterable[dict],
    coverage_threshold: float = 0.995,
    candidates: tuple[float, ...] = (0.42, 0.46, 0.48, 0.50, 0.52),
    legacy_fallback_m: float = 0.35,
) -> tuple[float, list[SpacingCandidate], bool]:
    by_spacing = {round(float(item["spacing_m"]), 3): item for item in observations}
    ranked = []
    for spacing in candidates:
        item = by_spacing.get(round(spacing, 3), {})
        coverage = float(item.get("coverage_rate", 0.0))
        repeat = float(item.get("repeat_rate", 1.0))
        length = float(item.get("path_length_m", math.inf))
        valid = coverage >= coverage_threshold and math.isfinite(length)
        score = length * (1.0 + max(0.0, repeat)) if valid else math.inf
        ranked.append(SpacingCandidate(spacing, coverage, repeat, length, valid, score))
    valid = [item for item in ranked if item.valid]
    if not valid:
        return legacy_fallback_m, ranked, True
    choice = min(valid, key=lambda item: (item.score, -item.spacing_m))
    return choice.spacing_m, ranked, False
