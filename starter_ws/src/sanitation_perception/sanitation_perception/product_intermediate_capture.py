"""Bounded atomic capture of product-input-only perception intermediates."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np


FORBIDDEN_ARTIFACT_TOKENS = ("truth", "evaluator")


def _json_bytes(value: object) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode(
        "utf-8"
    )


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _assert_product_only_metadata(value: object) -> None:
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True).lower()
    forbidden = [token for token in FORBIDDEN_ARTIFACT_TOKENS if token in encoded]
    if forbidden:
        raise ValueError(f"capture metadata contains forbidden private-input tokens: {forbidden}")


def _tree_size(root: Path) -> int:
    return sum(path.stat().st_size for path in root.rglob("*") if path.is_file())


def _remove_staged_directory(path: Path, expected_parent: Path) -> None:
    resolved = path.resolve()
    if resolved.parent != expected_parent.resolve() or not resolved.name.startswith("."):
        raise RuntimeError(f"refusing to remove unexpected capture staging path: {resolved}")
    shutil.rmtree(resolved)


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


class ProductIntermediateCapture:
    """Capture the first N fixed-rate front RGB-D product observations.

    The caller supplies only values already consumed or produced by the product
    node. Capture failures are raised to the caller so it can emit diagnostics;
    they never alter the product messages that were already computed.
    """

    def __init__(
        self,
        root: str | Path,
        *,
        max_frames: int = 12,
        minimum_interval_s: float = 1.0,
        max_bytes: int = 256 * 1024 * 1024,
    ) -> None:
        self.root = Path(root)
        self.max_frames = int(max_frames)
        self.minimum_interval_s = float(minimum_interval_s)
        self.max_bytes = int(max_bytes)
        if self.max_frames < 1:
            raise ValueError("capture max_frames must be positive")
        if not math.isfinite(self.minimum_interval_s) or self.minimum_interval_s <= 0.0:
            raise ValueError("capture interval must be finite and positive")
        if self.max_bytes < 1024 * 1024:
            raise ValueError("capture disk limit must be at least 1 MiB")
        self.frames_root = self.root / "frames"
        self.maps_root = self.root / "maps"
        self.disabled_reason: str | None = None
        self.frames_root.mkdir(parents=True, exist_ok=True)
        self.maps_root.mkdir(parents=True, exist_ok=True)
        committed = sorted(self.frames_root.glob("frame-[0-9][0-9][0-9][0-9]"))
        self.frame_count = len([path for path in committed if path.is_dir()])
        self.last_stamp_s: float | None = None
        if committed:
            metadata_path = committed[-1] / "metadata.json"
            if metadata_path.is_file():
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                self.last_stamp_s = float(metadata["rgb_stamp_s"])

    def wants_frame(self, sensor: str, stamp_s: float) -> bool:
        stamp_s = float(stamp_s)
        if (
            self.disabled_reason is not None
            or sensor != "front"
            or self.frame_count >= self.max_frames
            or not math.isfinite(stamp_s)
        ):
            return False
        if self.last_stamp_s is None:
            return True
        return stamp_s + 1e-9 >= self.last_stamp_s + self.minimum_interval_s

    def disable(self, reason: object) -> None:
        """Latch capture off after persistence failure without affecting product output."""

        encoded = str(reason).strip()
        self.disabled_reason = (encoded or "capture_disabled")[:512]

    def _write_bundle(self, final_directory: Path, arrays: dict, metadata: dict) -> dict:
        _assert_product_only_metadata(metadata)
        before_bytes = _tree_size(self.root)
        temporary = Path(tempfile.mkdtemp(prefix=f".{final_directory.name}-", dir=final_directory.parent))
        try:
            arrays_path = temporary / "arrays.npz"
            with arrays_path.open("wb") as stream:
                np.savez_compressed(stream, **arrays)
                stream.flush()
                os.fsync(stream.fileno())
            metadata_path = temporary / "metadata.json"
            metadata_bytes = _json_bytes(metadata)
            with metadata_path.open("wb") as stream:
                stream.write(metadata_bytes)
                stream.flush()
                os.fsync(stream.fileno())
            manifest = {
                "schema_version": 1,
                "files": {
                    "arrays.npz": _sha256_file(arrays_path),
                    "metadata.json": _sha256_file(metadata_path),
                },
            }
            _assert_product_only_metadata(manifest)
            manifest_path = temporary / "manifest.json"
            with manifest_path.open("wb") as stream:
                stream.write(_json_bytes(manifest))
                stream.flush()
                os.fsync(stream.fileno())
            staged_bytes = _tree_size(temporary)
            if before_bytes + staged_bytes > self.max_bytes:
                raise RuntimeError(
                    f"capture disk limit exceeded: current={before_bytes} "
                    f"staged={staged_bytes} maximum={self.max_bytes}"
                )
            if final_directory.exists():
                _remove_staged_directory(temporary, final_directory.parent)
                return json.loads((final_directory / "manifest.json").read_text(encoding="utf-8"))
            os.replace(temporary, final_directory)
            _fsync_directory(final_directory.parent)
            return manifest
        except Exception:
            if temporary.exists():
                _remove_staged_directory(temporary, final_directory.parent)
            raise

    def _write_map_snapshot(self, occupancy: np.ndarray, metadata: dict) -> tuple[str, str]:
        occupancy = np.asarray(occupancy, dtype=np.int8)
        _assert_product_only_metadata(metadata)
        content_hash = _sha256_bytes(_json_bytes(metadata) + occupancy.tobytes(order="C"))
        final_directory = self.maps_root / content_hash
        if not final_directory.is_dir():
            self._write_bundle(
                final_directory,
                {"occupancy": occupancy},
                {**metadata, "map_content_sha256": content_hash},
            )
        return content_hash, _sha256_file(final_directory / "manifest.json")

    def capture_frame(
        self,
        *,
        sensor: str,
        rgb_stamp_s: float,
        depth_stamp_s: float,
        rgb: np.ndarray,
        depth: np.ndarray,
        camera_info: dict,
        map_from_camera: np.ndarray,
        detections: list[dict],
        prompt_decisions: list[dict],
        prompt_detection_indices: np.ndarray,
        prompt_masks: list[np.ndarray],
        prompt_qualities: list[float],
        projection_diagnostics: dict,
        map_occupancy: np.ndarray,
        map_metadata: dict,
    ) -> bool:
        if not self.wants_frame(sensor, rgb_stamp_s):
            return False
        prompt_indices = np.asarray(prompt_detection_indices, dtype=np.int64).reshape(-1)
        if not (len(prompt_indices) == len(prompt_masks) == len(prompt_qualities)):
            raise ValueError("captured prompts, masks and qualities must align")
        occupancy = np.asarray(map_occupancy, dtype=np.int8)
        expected_shape = (int(map_metadata["height"]), int(map_metadata["width"]))
        if occupancy.shape != expected_shape:
            raise ValueError("captured /map dimensions disagree with metadata")
        map_hash, map_manifest_hash = self._write_map_snapshot(occupancy, map_metadata)

        arrays = {
            "rgb": np.asarray(rgb, dtype=np.uint8),
            "depth": np.asarray(depth),
            "camera_k": np.asarray(camera_info["k"], dtype=np.float64),
            "camera_d": np.asarray(camera_info.get("d", []), dtype=np.float64),
            "map_from_camera": np.asarray(map_from_camera, dtype=np.float64),
            "prompt_detection_indices": prompt_indices,
            "prompt_qualities": np.asarray(prompt_qualities, dtype=np.float32),
            "valid_depth_pixels_uv": np.asarray(
                projection_diagnostics["valid_depth_pixels_uv"], dtype=np.int32
            ),
            "valid_depth_m": np.asarray(projection_diagnostics["valid_depth_m"], dtype=np.float32),
            "map_points_xyz": np.asarray(
                projection_diagnostics["map_points_xyz"], dtype=np.float64
            ),
            "ground_mask": np.asarray(projection_diagnostics["ground_mask"], dtype=bool),
            "in_grid_mask": np.asarray(projection_diagnostics["in_grid_mask"], dtype=bool),
            "public_free_mask": np.asarray(
                projection_diagnostics["public_free_mask"], dtype=bool
            ),
            "map_rows_cols": np.asarray(
                projection_diagnostics["map_rows_cols"], dtype=np.int32
            ),
            "final_union_raster": np.asarray(
                projection_diagnostics["final_union_raster"], dtype=np.uint8
            ),
        }
        for index, mask in enumerate(prompt_masks):
            arrays[f"prompt_mask_{index:03d}"] = np.asarray(mask, dtype=bool)
        for class_id, raster in projection_diagnostics["per_class_rasters"].items():
            arrays[f"class_raster_{class_id}"] = np.asarray(raster, dtype=np.uint8)

        frame_index = self.frame_count
        metadata = {
            "schema_version": 1,
            "capture_policy": {
                "sensor": "front",
                "maximum_frames": self.max_frames,
                "minimum_stamp_interval_s": self.minimum_interval_s,
                "selection": "first_fixed_rate_product_frames",
            },
            "frame_index": frame_index,
            "sensor": sensor,
            "rgb_stamp_s": float(rgb_stamp_s),
            "depth_stamp_s": float(depth_stamp_s),
            "rgb_depth_skew_s": abs(float(rgb_stamp_s) - float(depth_stamp_s)),
            "camera_info": camera_info,
            "detections": detections,
            "prompt_decisions": prompt_decisions,
            "projection": {
                "sample_stride": int(projection_diagnostics["sample_stride"]),
                "ground_z_m": float(projection_diagnostics["ground_z_m"]),
                "ground_tolerance_m": float(projection_diagnostics["ground_tolerance_m"]),
                "public_free_applied_to_product_output": False,
            },
            "map_content_sha256": map_hash,
            "map_manifest_sha256": map_manifest_hash,
        }
        self._write_bundle(self.frames_root / f"frame-{frame_index:04d}", arrays, metadata)
        self.frame_count += 1
        self.last_stamp_s = float(rgb_stamp_s)
        return True
