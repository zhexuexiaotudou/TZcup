"""Prediction-derived multi-region projection into auditable map polygons."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from sanitation_perception.projection import ProjectionError, project_pixel_to_map, robust_depth


@dataclass(frozen=True)
class ProjectedAreaRegion:
    region_id: int
    polygon_xy_m: tuple[tuple[float, float], ...]
    physical_area_m2: float
    confidence: float
    covariance_xy: tuple[tuple[float, float], tuple[float, float]]
    pixel_area: int


def _shoelace(points: list[tuple[float, float]]) -> float:
    return abs(
        sum(
            first[0] * second[1] - second[0] * first[1]
            for first, second in zip(points, points[1:] + points[:1])
        )
    ) * 0.5


def _vertex_depth(
    depth: np.ndarray, component: np.ndarray, u: int, v: int, radius: int = 2
) -> float:
    top, bottom = max(0, v - radius), min(depth.shape[0], v + radius + 1)
    left, right = max(0, u - radius), min(depth.shape[1], u + radius + 1)
    local = depth[top:bottom, left:right]
    valid_component = component[top:bottom, left:right]
    return robust_depth(local[valid_component].reshape(-1), maximum_m=15.0)


def mask_regions_to_map(
    mask: np.ndarray,
    probability: np.ndarray,
    depth_m: np.ndarray,
    camera: dict,
    transform_map_camera: np.ndarray,
    *,
    minimum_pixels: int = 20,
    minimum_physical_area_m2: float = 0.0,
    contour_epsilon_ratio: float = 0.02,
) -> list[ProjectedAreaRegion]:
    """Project every valid predicted region; no registry rectangle is used."""
    binary = np.asarray(mask, dtype=np.uint8)
    probability = np.asarray(probability, dtype=np.float32)
    depth_m = np.asarray(depth_m, dtype=np.float32)
    if binary.ndim != 2 or probability.shape != binary.shape or depth_m.shape != binary.shape:
        raise ProjectionError("mask, probability and depth must share one HxW shape")
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    regions = []
    for region_id in range(1, component_count):
        pixel_area = int(stats[region_id, cv2.CC_STAT_AREA])
        if pixel_area < minimum_pixels:
            continue
        component = labels == region_id
        contours, _ = cv2.findContours(
            component.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        epsilon = max(1.0, contour_epsilon_ratio * cv2.arcLength(contour, True))
        vertices = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
        if len(vertices) < 3:
            continue
        polygon = []
        covariances = []
        try:
            for u, v in vertices:
                sample_depth = _vertex_depth(depth_m, component, int(u), int(v))
                xyz, covariance = project_pixel_to_map(
                    float(u),
                    float(v),
                    sample_depth,
                    camera,
                    transform_map_camera,
                )
                polygon.append((float(xyz[0]), float(xyz[1])))
                covariances.append(covariance[:2, :2])
        except ProjectionError:
            continue
        physical_area = _shoelace(polygon)
        if physical_area <= 0.0 or physical_area < minimum_physical_area_m2:
            continue
        covariance_xy = np.mean(covariances, axis=0)
        regions.append(
            ProjectedAreaRegion(
                region_id=region_id,
                polygon_xy_m=tuple(polygon),
                physical_area_m2=float(physical_area),
                confidence=float(probability[component].mean()),
                covariance_xy=(
                    (float(covariance_xy[0, 0]), float(covariance_xy[0, 1])),
                    (float(covariance_xy[1, 0]), float(covariance_xy[1, 1])),
                ),
                pixel_area=pixel_area,
            )
        )
    return regions
