"""Deterministic RGB letterbox and NV12 emulation for Journey 6 parity."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class LetterboxGeometry:
    source_width: int
    source_height: int
    target_width: int
    target_height: int
    scale: float
    pad_left: int
    pad_top: int
    resized_width: int
    resized_height: int

    def restore_xyxy(self, box: tuple[float, float, float, float]) -> tuple[float, ...]:
        x1, y1, x2, y2 = box
        restored = (
            (x1 - self.pad_left) / self.scale,
            (y1 - self.pad_top) / self.scale,
            (x2 - self.pad_left) / self.scale,
            (y2 - self.pad_top) / self.scale,
        )
        return (
            max(0.0, min(float(self.source_width), restored[0])),
            max(0.0, min(float(self.source_height), restored[1])),
            max(0.0, min(float(self.source_width), restored[2])),
            max(0.0, min(float(self.source_height), restored[3])),
        )


def letterbox_rgb(
    rgb: np.ndarray,
    target_size: tuple[int, int] = (640, 640),
    *,
    pad_value: int = 114,
) -> tuple[np.ndarray, LetterboxGeometry]:
    source = np.asarray(rgb)
    if source.ndim != 3 or source.shape[2] != 3 or source.dtype != np.uint8:
        raise ValueError("Journey 6 RGB input must be uint8 HxWx3")
    target_width, target_height = (int(value) for value in target_size)
    if target_width <= 0 or target_height <= 0 or target_width % 2 or target_height % 2:
        raise ValueError("NV12 target dimensions must be positive and even")
    source_height, source_width = source.shape[:2]
    scale = min(target_width / source_width, target_height / source_height)
    resized_width = max(1, int(round(source_width * scale)))
    resized_height = max(1, int(round(source_height * scale)))
    pad_left = (target_width - resized_width) // 2
    pad_top = (target_height - resized_height) // 2
    resized = cv2.resize(source, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    output = np.full((target_height, target_width, 3), int(pad_value), dtype=np.uint8)
    output[pad_top : pad_top + resized_height, pad_left : pad_left + resized_width] = resized
    geometry = LetterboxGeometry(
        source_width=source_width,
        source_height=source_height,
        target_width=target_width,
        target_height=target_height,
        scale=scale,
        pad_left=pad_left,
        pad_top=pad_top,
        resized_width=resized_width,
        resized_height=resized_height,
    )
    return np.ascontiguousarray(output), geometry


def rgb_to_nv12(rgb: np.ndarray, *, matrix: str = "bt601", value_range: str = "limited") -> np.ndarray:
    """Return one contiguous H*3/2 by W NV12 plane.

    OpenCV's I420 conversion supplies the frozen BT.601 limited-range contract.
    BT.709 is implemented explicitly because OpenCV does not expose a matching
    uint8 NV12 conversion primitive.
    """
    source = np.asarray(rgb)
    if source.ndim != 3 or source.shape[2] != 3 or source.dtype != np.uint8:
        raise ValueError("NV12 conversion requires uint8 RGB")
    height, width = source.shape[:2]
    if height % 2 or width % 2:
        raise ValueError("NV12 input dimensions must be even")
    if matrix not in {"bt601", "bt709"} or value_range not in {"limited", "full"}:
        raise ValueError("unsupported NV12 color contract")

    pixels = source.astype(np.float32)
    red, green, blue = pixels[..., 0], pixels[..., 1], pixels[..., 2]
    if matrix == "bt601":
        kr, kb = 0.299, 0.114
    else:
        kr, kb = 0.2126, 0.0722
    kg = 1.0 - kr - kb
    luminance = kr * red + kg * green + kb * blue
    cb = (blue - luminance) / (2.0 * (1.0 - kb))
    cr = (red - luminance) / (2.0 * (1.0 - kr))
    if value_range == "limited":
        y_plane = 16.0 + luminance * (219.0 / 255.0)
        u_plane = 128.0 + cb * (224.0 / 255.0)
        v_plane = 128.0 + cr * (224.0 / 255.0)
    else:
        y_plane = luminance
        u_plane = 128.0 + cb
        v_plane = 128.0 + cr
    u_half = u_plane.reshape(height // 2, 2, width // 2, 2).mean(axis=(1, 3))
    v_half = v_plane.reshape(height // 2, 2, width // 2, 2).mean(axis=(1, 3))
    uv = np.empty((height // 2, width), dtype=np.uint8)
    uv[:, 0::2] = np.clip(np.rint(u_half), 0, 255).astype(np.uint8)
    uv[:, 1::2] = np.clip(np.rint(v_half), 0, 255).astype(np.uint8)
    return np.ascontiguousarray(
        np.vstack((np.clip(np.rint(y_plane), 0, 255).astype(np.uint8), uv))
    )


def nv12_to_rgb(nv12: np.ndarray, *, width: int, height: int) -> np.ndarray:
    plane = np.asarray(nv12, dtype=np.uint8)
    if plane.shape != (height * 3 // 2, width):
        raise ValueError("NV12 plane shape mismatch")
    return cv2.cvtColor(np.ascontiguousarray(plane), cv2.COLOR_YUV2RGB_NV12)


__all__ = ["LetterboxGeometry", "letterbox_rgb", "nv12_to_rgb", "rgb_to_nv12"]
