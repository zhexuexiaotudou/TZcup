from __future__ import annotations

import math
from typing import Iterable

import numpy as np


class ProjectionError(ValueError):
    pass


def robust_depth(values: Iterable[float], minimum_m: float = 0.2, maximum_m: float = 30.0) -> float:
    samples = np.asarray(list(values), dtype=np.float64)
    samples = samples[np.isfinite(samples)]
    samples = samples[(samples >= minimum_m) & (samples <= maximum_m)]
    if samples.size < 3:
        raise ProjectionError("insufficient valid depth samples")
    median = float(np.median(samples))
    mad = float(np.median(np.abs(samples - median)))
    if mad > 0:
        samples = samples[np.abs(samples - median) <= 3.5 * 1.4826 * mad]
    if samples.size < 3:
        raise ProjectionError("depth samples rejected as edge/outliers")
    return float(np.median(samples))


def project_pixel_to_map(
    u: float,
    v: float,
    depth_m: float,
    camera: dict,
    transform_map_camera: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if not math.isfinite(depth_m) or depth_m <= 0:
        raise ProjectionError("depth must be finite and positive")
    required = {"fx", "fy", "cx", "cy"}
    if required - set(camera):
        raise ProjectionError("camera calibration is incomplete")
    fx, fy = float(camera["fx"]), float(camera["fy"])
    if fx <= 0 or fy <= 0:
        raise ProjectionError("camera focal lengths must be positive")
    transform = np.asarray(transform_map_camera, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ProjectionError("map-camera transform must be a finite 4x4 matrix")
    point_camera = np.array(
        [
            (float(u) - float(camera["cx"])) * depth_m / fx,
            (float(v) - float(camera["cy"])) * depth_m / fy,
            depth_m,
            1.0,
        ],
        dtype=np.float64,
    )
    point_map = transform @ point_camera
    if abs(point_map[3]) < 1e-12:
        raise ProjectionError("invalid homogeneous transform result")
    xyz = point_map[:3] / point_map[3]
    pixel_sigma = float(camera.get("pixel_sigma", 0.5))
    depth_sigma = float(camera.get("depth_sigma_m", 0.02))
    jacobian = np.array(
        [
            [depth_m / fx, 0.0, (float(u) - float(camera["cx"])) / fx],
            [0.0, depth_m / fy, (float(v) - float(camera["cy"])) / fy],
            [0.0, 0.0, 1.0],
        ]
    )
    source_cov = np.diag([pixel_sigma**2, pixel_sigma**2, depth_sigma**2])
    camera_cov = jacobian @ source_cov @ jacobian.T
    rotation = transform[:3, :3]
    map_cov = rotation @ camera_cov @ rotation.T
    return xyz, map_cov


def mask_bounds(mask: np.ndarray, minimum_pixels: int = 12) -> tuple[int, int, int, int]:
    rows, cols = np.nonzero(mask)
    if rows.size < minimum_pixels:
        raise ProjectionError("mask visibility below minimum component size")
    return int(cols.min()), int(rows.min()), int(cols.max() + 1), int(rows.max() + 1)
