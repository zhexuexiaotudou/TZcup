"""Live depth-derived geometry features used by the frozen area models.

This is deliberately NumPy-only and accepts no semantic, instance or ground
truth plane input.  The fitted plane and every derived channel therefore use
the same RGB-D information available to the product node.
"""

from __future__ import annotations

import math

import numpy as np


class GroundGeometryRuntime:
    def __init__(self, camera: dict):
        self.width = int(camera["width"])
        self.height = int(camera["height"])
        self.fx = float(camera["fx"])
        self.fy = float(camera["fy"])
        self.cx = float(camera["cx"])
        self.cy = float(camera["cy"])
        if self.width < 1 or self.height < 1 or self.fx <= 0 or self.fy <= 0:
            raise ValueError("camera calibration is invalid")

    def valid_depth_mask(self, depth: np.ndarray) -> np.ndarray:
        value = np.asarray(depth, dtype=np.float32)
        if value.shape != (self.height, self.width):
            raise ValueError(
                f"depth shape {value.shape} does not match camera "
                f"{self.width}x{self.height}"
            )
        return np.isfinite(value) & (value > 0.0)

    def unproject(self, depth: np.ndarray, valid: np.ndarray) -> np.ndarray:
        yy, xx = np.mgrid[0 : self.height, 0 : self.width]
        points = np.full((self.height, self.width, 3), np.nan, np.float32)
        z = depth[valid]
        points[valid, 0] = (xx[valid] - self.cx) * z / self.fx
        points[valid, 1] = (yy[valid] - self.cy) * z / self.fy
        points[valid, 2] = z
        return points

    def fit_ground_plane(
        self, points: np.ndarray, valid: np.ndarray, max_points: int = 8192
    ) -> tuple[np.ndarray, float]:
        samples = points[valid]
        if samples.shape[0] < 3:
            raise ValueError("insufficient valid depth for ground plane")
        if samples.shape[0] > max_points:
            stride = int(math.ceil(samples.shape[0] / max_points))
            samples = samples[::stride][:max_points]
        centroid = samples.mean(axis=0)
        _, singular, vh = np.linalg.svd(samples - centroid, full_matrices=False)
        if singular.size < 3 or singular[1] <= 1e-6 * singular[0]:
            raise ValueError("degenerate depth ground plane")
        normal = vh[-1]
        if normal[1] > 0.0 or (
            abs(float(normal[1])) <= 1e-6 and normal[2] < 0.0
        ):
            normal = -normal
        return normal.astype(np.float64), -float(normal @ centroid)

    def depth_gradient(
        self, depth: np.ndarray, valid: np.ndarray
    ) -> np.ndarray:
        masked = np.where(valid, depth, np.nan)
        gradient_y, gradient_x = np.gradient(masked)
        magnitude = np.sqrt(gradient_x**2 + gradient_y**2)
        magnitude[~valid] = 0.0
        return np.nan_to_num(magnitude, nan=0.0).astype(np.float32)

    def local_surface_normal(
        self,
        depth: np.ndarray,
        valid: np.ndarray,
        reference: np.ndarray,
    ) -> np.ndarray:
        masked = np.where(valid, depth, np.nan)
        gz_dv, gz_du = np.gradient(masked)
        yy, xx = np.mgrid[0 : self.height, 0 : self.width]
        z = depth.astype(np.float32)
        tangent_u = np.stack(
            (
                (z + (xx - self.cx) * gz_du) / self.fx,
                (yy - self.cy) * gz_du / self.fy,
                gz_du,
            ),
            axis=-1,
        )
        tangent_v = np.stack(
            (
                (xx - self.cx) * gz_dv / self.fx,
                (z + (yy - self.cy) * gz_dv) / self.fy,
                gz_dv,
            ),
            axis=-1,
        )
        normal = np.cross(tangent_u, tangent_v)
        normal /= np.maximum(np.linalg.norm(normal, axis=-1, keepdims=True), 1e-6)
        reference = reference / np.linalg.norm(reference)
        normal[(normal @ reference) < 0.0] *= -1.0
        normal[~valid] = np.nan
        return normal.astype(np.float32)

    def estimate(self, depth: np.ndarray) -> dict:
        depth = np.asarray(depth, dtype=np.float32)
        valid = self.valid_depth_mask(depth)
        points = self.unproject(depth, valid)
        normal, offset = self.fit_ground_plane(points, valid)
        height = np.maximum(points @ normal + offset, 0.0).astype(np.float32)
        return {
            "valid_depth_mask": valid,
            "valid_depth_ratio": float(valid.mean()),
            "height_above_ground": np.where(valid, height, 0.0),
            "depth_gradient_magnitude": self.depth_gradient(depth, valid),
            "local_surface_normal": self.local_surface_normal(
                depth, valid, normal
            ),
            "ground_plane": {"normal": normal.tolist(), "d": offset},
        }


__all__ = ["GroundGeometryRuntime"]
