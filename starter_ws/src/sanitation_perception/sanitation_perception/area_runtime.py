"""Frozen leaf/puddle area-model preprocessing and mask decoding."""

from __future__ import annotations

import cv2
import numpy as np

from sanitation_perception.ground_geometry_runtime import GroundGeometryRuntime


AREA_SIZE = (512, 384)


def _normalized_depth(depth: np.ndarray) -> np.ndarray:
    value = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(value) & (value > 0.0)
    output = np.zeros_like(value, dtype=np.float32)
    output[valid] = np.clip(np.log1p(value[valid]) / np.log(11.0), 0.0, 1.0)
    return output


def preprocess_area(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    camera: dict,
    *,
    task: str,
    geometry: dict | None = None,
) -> tuple[np.ndarray, dict]:
    if task not in {"leaf", "puddle"}:
        raise ValueError(f"unsupported area task {task!r}")
    geometry_runtime = GroundGeometryRuntime(camera)
    try:
        geometry = geometry or geometry_runtime.estimate(depth_m)
        valid = geometry["valid_depth_mask"].astype(np.float32)
        height = geometry["height_above_ground"].astype(np.float32)
        gradient = geometry["depth_gradient_magnitude"].astype(np.float32)
        normal = geometry["local_surface_normal"].astype(np.float32)
    except ValueError:
        valid = (np.isfinite(depth_m) & (depth_m > 0.0)).astype(np.float32)
        if float(valid.mean()) < 0.05:
            raise ValueError("valid depth ratio is below 5%")
        height = np.zeros_like(depth_m, dtype=np.float32)
        gradient = geometry_runtime.depth_gradient(depth_m, valid.astype(bool))
        normal = np.full(depth_m.shape + (3,), np.nan, dtype=np.float32)
        geometry = {
            "valid_depth_mask": valid.astype(bool),
            "valid_depth_ratio": float(valid.mean()),
            "ground_plane": None,
            "height_above_ground": height,
            "depth_gradient_magnitude": gradient,
            "local_surface_normal": normal,
        }
    resized_rgb = cv2.resize(rgb, AREA_SIZE, interpolation=cv2.INTER_AREA).astype(
        np.float32
    ) / 255.0
    resized_depth = cv2.resize(
        np.asarray(depth_m, dtype=np.float32),
        AREA_SIZE,
        interpolation=cv2.INTER_NEAREST,
    )
    normalized_depth = _normalized_depth(resized_depth)
    resized_valid = cv2.resize(valid, AREA_SIZE, interpolation=cv2.INTER_NEAREST)
    resized_height = cv2.resize(height, AREA_SIZE, interpolation=cv2.INTER_NEAREST)
    if task == "leaf":
        resized_gradient = cv2.resize(
            gradient, AREA_SIZE, interpolation=cv2.INTER_NEAREST
        )
        resized_normal = np.stack(
            [
                cv2.resize(normal[:, :, index], AREA_SIZE, interpolation=cv2.INTER_NEAREST)
                for index in range(3)
            ],
            axis=-1,
        )
        normal_features = np.nan_to_num(
            (resized_normal + 1.0) * 0.5, nan=0.0, posinf=1.0, neginf=0.0
        )
        channels = [
            resized_rgb,
            normalized_depth[:, :, None],
            resized_valid[:, :, None],
            resized_height[:, :, None],
            resized_gradient[:, :, None],
            normal_features,
        ]
    else:
        hsv = cv2.cvtColor(
            np.clip(resized_rgb, 0.0, 1.0).astype(np.float32) * 255.0,
            cv2.COLOR_RGB2HSV,
        ).astype(np.float32)
        hsv[:, :, 0] /= 180.0
        hsv[:, :, 1:] /= 255.0
        gray = cv2.cvtColor(
            np.clip(resized_rgb, 0.0, 1.0).astype(np.float32) * 255.0,
            cv2.COLOR_RGB2GRAY,
        )
        texture = np.clip(
            np.abs(cv2.Laplacian(gray, cv2.CV_32F)) / 80.0, 0.0, 1.0
        )
        channels = [
            resized_rgb,
            hsv,
            normalized_depth[:, :, None],
            resized_valid[:, :, None],
            resized_height[:, :, None],
            texture[:, :, None],
        ]
    value = np.concatenate(channels, axis=2).astype(np.float32)
    if value.shape != (AREA_SIZE[1], AREA_SIZE[0], 10):
        raise AssertionError(f"area feature contract mismatch: {value.shape}")
    return np.ascontiguousarray(value.transpose(2, 0, 1)[None]), geometry


def decode_area(
    flat_output: np.ndarray,
    *,
    mask_threshold: float,
    native_size: tuple[int, int],
) -> dict:
    flat = np.asarray(flat_output, dtype=np.float32)
    if flat.shape != (1, 2, AREA_SIZE[1], AREA_SIZE[0]):
        raise ValueError(f"area output shape mismatch: {flat.shape}")
    probability = 1.0 / (1.0 + np.exp(-np.clip(flat[0, 0], -80.0, 80.0)))
    boundary = 1.0 / (1.0 + np.exp(-np.clip(flat[0, 1], -80.0, 80.0)))
    width, height = native_size
    probability_native = cv2.resize(
        probability, (width, height), interpolation=cv2.INTER_LINEAR
    ).astype(np.float32)
    boundary_native = cv2.resize(
        boundary, (width, height), interpolation=cv2.INTER_LINEAR
    ).astype(np.float32)
    return {
        "probability": probability_native,
        "boundary_probability": boundary_native,
        "mask": probability_native >= float(mask_threshold),
    }


__all__ = ["AREA_SIZE", "decode_area", "preprocess_area"]
