import numpy as np
import pytest

from sanitation_perception.projection import ProjectionError, mask_bounds, project_pixel_to_map, robust_depth


def test_robust_depth_rejects_invalid_and_edge_outlier():
    assert robust_depth([0.0, float("nan"), 2.0, 2.01, 1.99, 8.0]) == pytest.approx(2.0, abs=0.02)
    with pytest.raises(ProjectionError):
        robust_depth([0.0, float("nan"), 2.0])


def test_projection_and_covariance_are_finite():
    camera = {"fx": 100.0, "fy": 100.0, "cx": 50.0, "cy": 40.0}
    transform = np.eye(4); transform[:3, 3] = [1.0, 2.0, 0.5]
    point, covariance = project_pixel_to_map(60.0, 40.0, 2.0, camera, transform)
    assert point == pytest.approx([1.2, 2.0, 2.5])
    assert np.linalg.eigvalsh(covariance).min() >= -1e-12


def test_low_visibility_mask_fails_closed():
    with pytest.raises(ProjectionError, match="visibility"):
        mask_bounds(np.eye(3, dtype=bool), minimum_pixels=5)
