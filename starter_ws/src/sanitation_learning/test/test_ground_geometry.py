from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.ground_geometry import GroundGeometryEstimator  # noqa: E402


def _camera_info(width: int = 640, height: int = 480) -> dict:
    return {
        "width": width,
        "height": height,
        "fx": 320.0,
        "fy": 320.0,
        "cx": 320.0,
        "cy": 240.0,
    }


def _synthetic_ground_depth(
    camera: dict, ground_y: float = 0.3, noise_std: float = 0.0
) -> np.ndarray:
    """Depth whose valid pixels lie exactly on the plane y=ground_y."""
    yy, _ = np.mgrid[0 : camera["height"], 0 : camera["width"]]
    with np.errstate(divide="ignore", invalid="ignore"):
        z = camera["fy"] * ground_y / (yy - camera["cy"])
    depth = np.where(yy > camera["cy"], z, 0.0).astype(np.float32)
    if noise_std > 0.0:
        rng = np.random.default_rng(20260805)
        depth = depth + rng.normal(0.0, noise_std, depth.shape).astype(
            np.float32
        )
        depth[depth <= 0.0] = 0.0
    return depth


def test_valid_depth_returns_nonempty_estimate() -> None:
    camera = _camera_info()
    estimator = GroundGeometryEstimator(camera)
    depth = _synthetic_ground_depth(camera)
    result = estimator.estimate(depth)
    assert set(
        {
            "valid_depth_mask",
            "ground_plane",
            "height_above_ground",
            "camera_height_above_ground",
            "local_surface_normal",
            "depth_gradient_magnitude",
        }
    ) <= set(result)
    assert result["point_count"] > 0
    assert result["valid_depth_mask"].shape == (480, 640)
    assert result["height_above_ground"].shape == (480, 640)
    assert result["local_surface_normal"].shape == (480, 640, 3)
    assert result["depth_gradient_magnitude"].shape == (480, 640)


def test_ground_plane_fit_recovers_synthetic_plane() -> None:
    camera = _camera_info()
    estimator = GroundGeometryEstimator(camera)
    depth = _synthetic_ground_depth(camera, ground_y=0.3)
    result = estimator.estimate(depth)
    normal = np.asarray(result["ground_plane"]["normal"])
    assert np.allclose(np.abs(normal), (0.0, 1.0, 0.0), atol=1e-3)
    assert abs(result["ground_plane"]["residual"]) < 1e-3
    assert abs(result["camera_height_above_ground"] - 0.3) < 1e-2
    interior = (
        result["valid_depth_mask"]
        & np.isfinite(result["signed_height_from_plane"])
        & (result["signed_height_from_plane"] < 0.5)
    )
    assert interior.sum() > 0
    heights = result["height_above_ground"][interior]
    assert float(np.mean(heights)) < 0.05


def test_local_surface_normal_consistent_with_plane() -> None:
    camera = _camera_info()
    estimator = GroundGeometryEstimator(camera)
    depth = _synthetic_ground_depth(camera)
    result = estimator.estimate(depth)
    normal = np.asarray(result["ground_plane"]["normal"])
    normal = normal / np.linalg.norm(normal)
    local = result["local_surface_normal"]
    valid_rows = np.where(result["valid_depth_mask"])[0]
    if valid_rows.size:
        first_valid = int(valid_rows.min())
    else:
        first_valid = 0
    finite = np.isfinite(local).all(axis=-1)
    # np.gradient is NaN for the first two rows next to the invalid horizon.
    finite[first_valid : first_valid + 2, :] = False
    assert finite.sum() > result["point_count"] * 0.9
    dots = local[finite] @ normal
    assert float(dots.min()) > -0.1
    assert float(dots.mean()) > 0.95


def test_deterministic_estimate() -> None:
    camera = _camera_info()
    estimator = GroundGeometryEstimator(camera)
    depth = _synthetic_ground_depth(camera, noise_std=0.005)
    first = estimator.estimate(depth)
    second = estimator.estimate(depth)
    assert np.allclose(
        first["height_above_ground"],
        second["height_above_ground"],
        equal_nan=True,
    )
    assert first["ground_plane"]["normal"] == second["ground_plane"]["normal"]
    assert first["ground_plane"]["d"] == second["ground_plane"]["d"]


def test_extrinsics_and_plane_reuse() -> None:
    camera = _camera_info()
    estimator = GroundGeometryEstimator(camera)
    depth = _synthetic_ground_depth(camera)
    result = estimator.estimate(
        depth,
        camera_extrinsics={"base_to_camera_xyz_m": [0.53, 0.0, 0.22]},
    )
    assert result["base_height_above_ground"] is not None
    assert np.isfinite(result["base_height_above_ground"])
    reused = estimator.estimate(
        depth, ground_plane=result["ground_plane"]
    )
    assert reused["ground_plane"]["provided"] is True
    assert np.allclose(
        reused["height_above_ground"],
        result["height_above_ground"],
        atol=1e-6,
        equal_nan=True,
    )


def test_degenerate_depth_raises_value_error() -> None:
    camera = _camera_info()
    estimator = GroundGeometryEstimator(camera)
    with pytest.raises(ValueError):
        estimator.fit_ground_plane(np.zeros((480, 640), np.float32))
    with pytest.raises(ValueError):
        estimator.estimate(np.zeros((480, 640), np.float32))
    tiny = np.zeros((480, 640), np.float32)
    tiny[240, 320] = 1.0
    tiny[241, 320] = 1.0
    with pytest.raises(ValueError):
        estimator.fit_ground_plane(tiny)


def test_invalid_camera_info_raises() -> None:
    camera = _camera_info()
    camera["fx"] = 0.0
    with pytest.raises(ValueError):
        GroundGeometryEstimator(camera)


def test_wrong_depth_shape_raises() -> None:
    estimator = GroundGeometryEstimator(_camera_info())
    with pytest.raises(ValueError):
        estimator.valid_depth_mask(np.zeros((32, 32), np.float32))
