import math

import pytest

from sanitation_spot_cleaning.node import (
    approach_pose_xyyaw,
    frustum_record_contains,
)


def test_approach_pose_places_physical_brush_over_target() -> None:
    x_m, y_m, yaw = approach_pose_xyyaw((0.0, 0.0), (2.0, 0.0), 0.55)
    assert (x_m, y_m, yaw) == pytest.approx((1.45, 0.0, 0.0))
    assert x_m + 0.55 * math.cos(yaw) == pytest.approx(2.0)
    assert y_m + 0.55 * math.sin(yaw) == pytest.approx(0.0)


def test_persisted_camera_frustum_is_required_for_absence_evidence() -> None:
    record = {
        "sweep_id": "sweep-1",
        "mission_id": "mission-1",
        "stamp_ns": 42,
        "camera_frame_id": "camera_optical",
        "image_frame_id": "image-42",
        "camera_x_m": 0.0,
        "camera_y_m": 0.0,
        "camera_yaw_rad": 0.0,
        "horizontal_fov_rad": math.pi / 2.0,
        "minimum_range_m": 0.1,
        "maximum_range_m": 4.0,
    }
    assert frustum_record_contains(record, 2.0, 0.0)
    assert not frustum_record_contains(record, -2.0, 0.0)
    assert not frustum_record_contains({}, 2.0, 0.0)
