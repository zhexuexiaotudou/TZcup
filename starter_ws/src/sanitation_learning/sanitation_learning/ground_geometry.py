"""Ground geometry estimation shared by G4 training and ROS live inference.

``GroundGeometryEstimator`` is a single NumPy implementation that consumes
CameraInfo, depth and optional camera extrinsics and produces a valid-depth
mask, a deterministically fitted ground plane, height above ground, a local
surface-normal proxy and a depth-gradient proxy.

The same code path is used offline (training augmentation / evaluation) and
live (ROS node): there is deliberately no training-only GT-plane bypass.
``estimate`` either fits the plane from the very same depth input a live node
would consume, or reuses a plane that was itself estimated on a previous
frame; it never accepts a GT annotation as the plane source.
"""

from __future__ import annotations

import math

import numpy as np


def _extrinsic_translation(camera_extrinsics: dict | None) -> np.ndarray | None:
    if not camera_extrinsics:
        return None
    for key in (
        "base_to_camera_xyz_m",
        "base_to_camera_xyz",
        "translation_xyz_m",
        "translation_m",
    ):
        if key in camera_extrinsics:
            value = np.asarray(camera_extrinsics[key], dtype=np.float64)
            if value.shape != (3,):
                raise ValueError(f"camera extrinsic {key} must be a 3-vector")
            return value
    return None


class GroundGeometryEstimator:
    """NumPy-only ground geometry from CameraInfo + depth (+ extrinsics).

    Camera frame convention: ``x`` right, ``y`` down, ``z`` forward.  The
    ground plane is returned as ``normal``/``d`` with ``normal . p + d = 0``
    and the normal oriented upward (negative y) when possible.  Extrinsics are
    consumed as a pure translation (aligned axes): the base origin in camera
    coordinates is ``-base_to_camera_xyz_m``.
    """

    def __init__(self, camera_info: dict):
        info = dict(camera_info)
        self.width = int(info.get("width", 640))
        self.height = int(info.get("height", 480))
        self.fx = float(info["fx"])
        self.fy = float(info["fy"])
        self.cx = float(info["cx"])
        self.cy = float(info["cy"])
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera_info fx/fy must be positive")
        if self.width < 1 or self.height < 1:
            raise ValueError("camera_info width/height must be positive")

    def valid_depth_mask(self, depth: np.ndarray) -> np.ndarray:
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim != 2:
            raise ValueError("depth must be a 2D image")
        if depth.shape != (self.height, self.width):
            raise ValueError(
                f"depth shape {depth.shape} does not match camera_info "
                f"({self.width}x{self.height})"
            )
        return np.isfinite(depth) & (depth > 0.0)

    def unproject(
        self, depth: np.ndarray, valid_mask: np.ndarray | None = None
    ) -> np.ndarray:
        """Unproject depth into camera 3D points; invalid pixels are NaN."""
        depth = np.asarray(depth, dtype=np.float32)
        valid = (
            self.valid_depth_mask(depth)
            if valid_mask is None
            else np.asarray(valid_mask, dtype=bool)
        )
        if valid.ndim != 2 or valid.shape != (self.height, self.width):
            raise ValueError("valid_mask must be a bool image of camera_info size")
        yy, xx = np.mgrid[0 : self.height, 0 : self.width]
        points = np.full((self.height, self.width, 3), np.nan, dtype=np.float32)
        z = depth[valid]
        points[valid, 0] = (
            xx[valid].astype(np.float32) - self.cx
        ) * z / self.fx
        points[valid, 1] = (
            yy[valid].astype(np.float32) - self.cy
        ) * z / self.fy
        points[valid, 2] = z
        return points

    def fit_ground_plane(
        self,
        depth: np.ndarray,
        valid_mask: np.ndarray | None = None,
        max_points: int = 8192,
    ) -> dict:
        """Deterministic least-squares ground plane fit (SVD).

        Raises ``ValueError`` on degenerate input: too few valid depth points
        or collinear/constant point clouds.
        """
        depth = np.asarray(depth, dtype=np.float32)
        valid = (
            self.valid_depth_mask(depth)
            if valid_mask is None
            else np.asarray(valid_mask, dtype=bool)
        )
        if valid.ndim != 2 or valid.shape != (self.height, self.width):
            raise ValueError("valid_mask must be a bool image of camera_info size")
        points = self.unproject(depth, valid)
        samples = points[valid]
        count = samples.shape[0]
        if count < 3:
            raise ValueError(
                f"insufficient valid depth points for ground plane fit: {count}"
            )
        if count > max_points:
            if max_points < 3:
                raise ValueError("max_points must be at least 3")
            stride = int(math.ceil(count / max_points))
            samples = samples[::stride][:max_points]
        centroid = samples.mean(axis=0)
        _, singular, vh = np.linalg.svd(
            samples - centroid, full_matrices=False
        )
        if (
            singular.size < 3
            or singular[1] <= 1e-6 * singular[0]
        ):
            raise ValueError(
                "degenerate ground plane fit: valid depth points are "
                "collinear or constant"
            )
        normal = vh[-1]
        # Camera y points down; orient the normal upward (negative y).
        if normal[1] > 0.0:
            normal = -normal
        elif abs(normal[1]) <= 1e-6 and normal[2] < 0.0:
            normal = -normal
        d = -float(normal @ centroid)
        residual = float(np.sqrt(np.mean((samples @ normal + d) ** 2)))
        return {
            "normal": normal.astype(np.float64),
            "d": d,
            "residual": residual,
            "inlier_count": int(count),
            "sample_count": int(samples.shape[0]),
            "provided": False,
        }

    def depth_gradient_magnitude(
        self, depth: np.ndarray, valid_mask: np.ndarray | None = None
    ) -> np.ndarray:
        """Depth-gradient proxy magnitude; invalid pixels are zero."""
        depth = np.asarray(depth, dtype=np.float32)
        valid = (
            self.valid_depth_mask(depth)
            if valid_mask is None
            else np.asarray(valid_mask, dtype=bool)
        )
        masked = np.where(valid, depth, np.nan)
        gradient = np.gradient(masked)
        magnitude = np.sqrt(gradient[0] ** 2 + gradient[1] ** 2)
        magnitude[~valid] = 0.0
        return magnitude.astype(np.float32)

    def local_surface_normal(
        self,
        depth: np.ndarray,
        valid_mask: np.ndarray | None = None,
        reference_normal: np.ndarray | None = None,
    ) -> np.ndarray:
        """Local surface-normal proxy from depth gradients (H, W, 3).

        Invalid pixels are NaN.  When ``reference_normal`` is given the proxy
        is oriented so its dot product with the reference is non-negative,
        making it consistent with the fitted ground plane.
        """
        depth = np.asarray(depth, dtype=np.float32)
        valid = (
            self.valid_depth_mask(depth)
            if valid_mask is None
            else np.asarray(valid_mask, dtype=bool)
        )
        masked = np.where(valid, depth, np.nan)
        gz_dv, gz_du = np.gradient(masked)
        yy, xx = np.mgrid[0 : self.height, 0 : self.width]
        xx = xx.astype(np.float32)
        yy = yy.astype(np.float32)
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
        norm = np.linalg.norm(normal, axis=-1, keepdims=True)
        normal = normal / np.maximum(norm, 1e-6)
        if reference_normal is not None:
            reference = np.asarray(reference_normal, dtype=np.float64)
            reference = reference / np.linalg.norm(reference)
            flips = (normal @ reference) < 0.0
            normal[flips] = -normal[flips]
        normal[~valid] = np.nan
        return normal.astype(np.float32)

    def estimate(
        self,
        depth: np.ndarray,
        camera_info: dict | None = None,
        camera_extrinsics: dict | None = None,
        ground_plane: dict | None = None,
        max_points: int = 8192,
    ) -> dict:
        """Full estimate; raises ``ValueError`` on degenerate depth input."""
        depth = np.asarray(depth, dtype=np.float32)
        if camera_info is not None:
            self.__init__(camera_info)
        valid = self.valid_depth_mask(depth)
        points = self.unproject(depth, valid)
        if ground_plane is None:
            plane = self.fit_ground_plane(depth, valid, max_points=max_points)
        else:
            plane = dict(ground_plane)
            plane["provided"] = True
        normal = np.asarray(plane["normal"], dtype=np.float64)
        d = float(plane["d"])
        norm = float(np.linalg.norm(normal))
        if norm <= 0.0:
            raise ValueError("ground_plane normal must be non-zero")
        normal_unit = normal / norm
        d_unit = d / norm
        signed_height = points @ normal_unit + d_unit
        height_above_ground = np.maximum(signed_height, 0.0).astype(np.float32)
        camera_height = abs(d_unit)

        base_height = None
        translation = _extrinsic_translation(camera_extrinsics)
        if translation is not None:
            base_origin_camera = -translation
            base_height = float(
                np.clip(base_origin_camera @ normal_unit + d_unit, 0.0, None)
            )

        local_surface_normal = self.local_surface_normal(
            depth, valid, reference_normal=normal_unit
        )
        depth_gradient = self.depth_gradient_magnitude(depth, valid)
        return {
            "valid_depth_mask": valid,
            "ground_plane": {
                "normal": np.asarray(plane["normal"], dtype=np.float64).tolist(),
                "d": float(plane["d"]),
                "residual": float(plane.get("residual", 0.0)),
                "inlier_count": int(plane.get("inlier_count", 0)),
                "sample_count": int(plane.get("sample_count", 0)),
                "provided": bool(plane.get("provided", False)),
            },
            "height_above_ground": height_above_ground,
            "signed_height_from_plane": signed_height.astype(np.float32),
            "camera_height_above_ground": float(camera_height),
            "base_height_above_ground": base_height,
            "local_surface_normal": local_surface_normal,
            "depth_gradient_magnitude": depth_gradient,
            "point_count": int(valid.sum()),
            "camera_info": {
                "width": self.width,
                "height": self.height,
                "fx": self.fx,
                "fy": self.fy,
                "cx": self.cx,
                "cy": self.cy,
            },
        }


__all__ = ["GroundGeometryEstimator"]
