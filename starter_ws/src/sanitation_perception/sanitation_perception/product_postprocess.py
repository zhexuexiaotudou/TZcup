"""Prediction-only map projection for the frozen product pipeline."""

from __future__ import annotations

import cv2
import numpy as np

from sanitation_perception.detector_runtime import MODEL_SIZE
from sanitation_perception.map_projection_v2 import mask_regions_to_map
from sanitation_perception.projection import (
    ProjectionError,
    project_pixel_to_map,
    robust_depth,
)


def transform_to_matrix(transform) -> np.ndarray:
    translation = transform.transform.translation
    quaternion = transform.transform.rotation
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if not np.isfinite(norm) or norm <= 0.0:
        raise ProjectionError("TF quaternion is invalid")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = (
        float(translation.x),
        float(translation.y),
        float(translation.z),
    )
    return matrix


def project_discrete_predictions(
    predictions: list[dict],
    depth_m: np.ndarray,
    camera: dict,
    transform_map_camera: np.ndarray,
) -> list[dict]:
    height, width = depth_m.shape
    scale_x, scale_y = width / MODEL_SIZE[0], height / MODEL_SIZE[1]
    projected = []
    for prediction in predictions:
        x1, y1, x2, y2 = prediction["bbox_xyxy"]
        native = (
            max(0, int(np.floor(x1 * scale_x))),
            max(0, int(np.floor(y1 * scale_y))),
            min(width, int(np.ceil(x2 * scale_x))),
            min(height, int(np.ceil(y2 * scale_y))),
        )
        if native[2] <= native[0] or native[3] <= native[1]:
            continue
        inset_x = max(1, int((native[2] - native[0]) * 0.2))
        inset_y = max(1, int((native[3] - native[1]) * 0.2))
        left, right = native[0] + inset_x, native[2] - inset_x
        top, bottom = native[1] + inset_y, native[3] - inset_y
        if right <= left or bottom <= top:
            left, top, right, bottom = native
        try:
            depth = robust_depth(depth_m[top:bottom, left:right].reshape(-1))
            u = (native[0] + native[2]) * 0.5
            v = (native[1] + native[3]) * 0.5
            xyz, covariance = project_pixel_to_map(
                u, v, depth, camera, transform_map_camera
            )
        except ProjectionError:
            continue
        projected.append(
            {
                **prediction,
                "bbox_xyxy": tuple(float(value) for value in native),
                "x_m": float(xyz[0]),
                "y_m": float(xyz[1]),
                "z_m": float(xyz[2]),
                "covariance_trace": float(np.trace(covariance)),
                "source_backend": "onnxruntime",
                "target_type": "DISCRETE",
            }
        )
    return projected


def project_area_predictions(
    areas: dict[str, dict],
    depth_m: np.ndarray,
    camera: dict,
    transform_map_camera: np.ndarray,
    *,
    minimum_pixels: int,
) -> list[dict]:
    projected = []
    for task, class_id in (("leaf", "leaf_pile"), ("puddle", "puddle")):
        output = areas[task]
        binary = np.asarray(output["mask"], dtype=np.uint8)
        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            binary, 8
        )
        regions = mask_regions_to_map(
            binary,
            output["probability"],
            depth_m,
            camera,
            transform_map_camera,
            minimum_pixels=minimum_pixels,
        )
        for region in regions:
            if region.region_id >= component_count:
                raise ProjectionError("area component identity mismatch")
            x = int(stats[region.region_id, cv2.CC_STAT_LEFT])
            y = int(stats[region.region_id, cv2.CC_STAT_TOP])
            width = int(stats[region.region_id, cv2.CC_STAT_WIDTH])
            height = int(stats[region.region_id, cv2.CC_STAT_HEIGHT])
            center = np.mean(np.asarray(region.polygon_xy_m), axis=0)
            projected.append(
                {
                    "class_id": class_id,
                    "class_probabilities": {
                        class_id: region.confidence,
                        "background": 1.0 - region.confidence,
                    },
                    "confidence": region.confidence,
                    "bbox_xyxy": (x, y, x + width, y + height),
                    "x_m": float(center[0]),
                    "y_m": float(center[1]),
                    "z_m": 0.0,
                    "covariance_trace": float(
                        region.covariance_xy[0][0]
                        + region.covariance_xy[1][1]
                    ),
                    "polygon_xy_m": region.polygon_xy_m,
                    "physical_area_m2": region.physical_area_m2,
                    "pixel_area": region.pixel_area,
                    "source_backend": "onnxruntime",
                    "target_type": "AREA",
                }
            )
    return projected


__all__ = [
    "project_area_predictions",
    "project_discrete_predictions",
    "transform_to_matrix",
]
