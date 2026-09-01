"""Project RGB-D product observations into the public occupancy-grid frame."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class PublicGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    # Optional public /map values are diagnostic context only. Product output
    # remains byte-for-byte compatible with the established geometry-only
    # projection until a separately accepted product change enables an ROI.
    occupancy: np.ndarray | None = None


@dataclass(frozen=True)
class ProjectedTarget:
    detection_index: int
    xyz: tuple[float, float, float]


def _depth_meters(depth: np.ndarray) -> np.ndarray:
    values = np.asarray(depth)
    if values.dtype == np.uint16:
        return values.astype(np.float32) * 0.001
    return values.astype(np.float32)


def _project_pixels(
    pixels_uv: np.ndarray,
    depths: np.ndarray,
    camera: CameraIntrinsics,
    map_from_camera: np.ndarray,
) -> np.ndarray:
    u = pixels_uv[:, 0]
    v = pixels_uv[:, 1]
    z = depths
    camera_points = np.stack(
        ((u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z), axis=1
    )
    homogeneous = np.concatenate(
        (camera_points, np.ones((camera_points.shape[0], 1), dtype=np.float32)), axis=1
    )
    return (np.asarray(map_from_camera, dtype=np.float64) @ homogeneous.T).T[:, :3]


def project_rgbd_observation(
    depth: np.ndarray,
    camera: CameraIntrinsics,
    map_from_camera: np.ndarray,
    grid: PublicGrid,
    *,
    boxes_xyxy: np.ndarray,
    class_ids: list[str],
    masks: list[np.ndarray],
    confidences: list[float],
    sample_stride: int = 4,
    ground_z: float = 0.0,
    ground_tolerance: float = 0.25,
    diagnostics_out: dict | None = None,
) -> tuple[np.ndarray, list[ProjectedTarget]]:
    """Return map-frame 0/1/2..255 dirt raster and projected litter targets."""

    depth_m = _depth_meters(depth)
    if depth_m.ndim != 2 or not np.isfinite(map_from_camera).all():
        raise ValueError("depth and transform must be finite and dimensionally valid")
    if camera.fx <= 0 or camera.fy <= 0 or grid.resolution <= 0:
        raise ValueError("camera focal lengths and grid resolution must be positive")
    if not (len(class_ids) == len(masks) == len(confidences) == len(boxes_xyxy)):
        raise ValueError("detection projection arrays must have equal lengths")
    height, width = depth_m.shape
    output = np.zeros((grid.height, grid.width), dtype=np.uint8)
    per_class_rasters = {
        class_id: np.zeros_like(output)
        for class_id in sorted(set(class_ids))
        if class_id != "litter_cube"
    }
    ys, xs = np.mgrid[0:height:sample_stride, 0:width:sample_stride]
    sampled_depth = depth_m[ys, xs]
    valid = np.isfinite(sampled_depth) & (sampled_depth > 0.05) & (sampled_depth < 50.0)
    trace_pixels = np.empty((0, 2), dtype=np.int32)
    trace_depth = np.empty((0,), dtype=np.float32)
    trace_points = np.empty((0, 3), dtype=np.float64)
    trace_ground = np.empty((0,), dtype=bool)
    trace_inside = np.empty((0,), dtype=bool)
    trace_public_free = np.empty((0,), dtype=bool)
    trace_rows_cols = np.empty((0, 2), dtype=np.int32)
    if valid.any():
        pixels = np.stack((xs[valid], ys[valid]), axis=1).astype(np.float32)
        valid_depths = sampled_depth[valid]
        points = _project_pixels(pixels, valid_depths, camera, map_from_camera)
        on_ground = np.abs(points[:, 2] - ground_z) <= ground_tolerance
        trace_pixels = pixels.astype(np.int32)
        trace_depth = valid_depths.astype(np.float32)
        trace_points = points.copy()
        trace_ground = on_ground.copy()
        trace_inside = np.zeros(len(points), dtype=bool)
        trace_public_free = np.zeros(len(points), dtype=bool)
        trace_rows_cols = np.full((len(points), 2), -1, dtype=np.int32)
        points = points[on_ground]
        pixels = pixels[on_ground].astype(np.int64)
        cols = np.floor((points[:, 0] - grid.origin_x) / grid.resolution).astype(np.int64)
        rows = np.floor((points[:, 1] - grid.origin_y) / grid.resolution).astype(np.int64)
        inside = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
        ground_indices = np.flatnonzero(on_ground)
        trace_inside[ground_indices] = inside
        trace_rows_cols[ground_indices, 0] = rows.astype(np.int32)
        trace_rows_cols[ground_indices, 1] = cols.astype(np.int32)
        occupancy = None
        if grid.occupancy is not None:
            occupancy = np.asarray(grid.occupancy).reshape(grid.height, grid.width)
            inside_indices = ground_indices[inside]
            trace_public_free[inside_indices] = occupancy[rows[inside], cols[inside]] == 0
        output[rows[inside], cols[inside]] = 1
        pixels, rows, cols = pixels[inside], rows[inside], cols[inside]
        for class_id, mask, confidence in zip(class_ids, masks, confidences):
            if class_id == "litter_cube":
                continue
            mask_array = np.asarray(mask, dtype=bool)
            dirty = mask_array[pixels[:, 1], pixels[:, 0]]
            value = int(np.clip(round(2.0 + max(0.0, confidence) * 253.0), 2, 255))
            output[rows[dirty], cols[dirty]] = np.maximum(output[rows[dirty], cols[dirty]], value)
            class_raster = per_class_rasters[class_id]
            class_raster[rows[dirty], cols[dirty]] = np.maximum(
                class_raster[rows[dirty], cols[dirty]], value
            )

    projected: list[ProjectedTarget] = []
    for index, (class_id, box, mask) in enumerate(zip(class_ids, boxes_xyxy, masks)):
        if class_id != "litter_cube":
            continue
        x1, y1, x2, y2 = np.asarray(box, dtype=np.float32)
        x1i, x2i = sorted((max(0, int(x1)), min(width, int(np.ceil(x2)))))
        y1i, y2i = sorted((max(0, int(y1)), min(height, int(np.ceil(y2)))))
        if x2i <= x1i or y2i <= y1i:
            continue
        region_mask = np.asarray(mask, dtype=bool)[y1i:y2i, x1i:x2i]
        region_depth = depth_m[y1i:y2i, x1i:x2i]
        valid = region_mask & np.isfinite(region_depth) & (region_depth > 0.05) & (region_depth < 50.0)
        if not valid.any():
            continue
        local_y, local_x = np.nonzero(valid)
        median = float(np.median(region_depth[valid]))
        nearest = int(np.argmin(np.abs(region_depth[valid] - median)))
        pixel = np.asarray([[local_x[nearest] + x1i, local_y[nearest] + y1i]], dtype=np.float32)
        point = _project_pixels(pixel, np.asarray([median], dtype=np.float32), camera, map_from_camera)[0]
        if np.isfinite(point).all():
            projected.append(ProjectedTarget(index, tuple(float(value) for value in point)))
    if diagnostics_out is not None:
        diagnostics_out.clear()
        diagnostics_out.update(
            {
                "sample_stride": int(sample_stride),
                "ground_z_m": float(ground_z),
                "ground_tolerance_m": float(ground_tolerance),
                "valid_depth_pixels_uv": trace_pixels,
                "valid_depth_m": trace_depth,
                "map_points_xyz": trace_points,
                "ground_mask": trace_ground,
                "in_grid_mask": trace_inside,
                "public_free_mask": trace_public_free,
                "map_rows_cols": trace_rows_cols,
                "public_free_applied_to_product_output": False,
                "per_class_rasters": per_class_rasters,
                "final_union_raster": output.copy(),
            }
        )
    return output, projected
