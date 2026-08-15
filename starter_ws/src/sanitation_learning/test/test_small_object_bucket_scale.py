from __future__ import annotations

import pytest

from sanitation_learning.metric_scale import (
    MetricScale,
    derive_model_scale_record,
    small_object_bucket,
)


def _record(native_short_side: float, native_mask_area: int = 100) -> dict:
    half = native_short_side / 2.0
    return {
        "native_bbox_xyxy": [
            0.0,
            0.0,
            native_short_side,
            native_short_side,
        ],
        "native_short_side_px": native_short_side,
        "native_mask_area_px": native_mask_area,
    }


def test_small_object_bucket_native_scale_uses_native_fields() -> None:
    record = _record(native_short_side=16.0)
    assert small_object_bucket(record, MetricScale.NATIVE_SCALE) is True
    assert small_object_bucket(record) is True
    assert small_object_bucket(_record(18.0), MetricScale.NATIVE_SCALE) is False


def test_small_object_bucket_model_scale_differs_from_native() -> None:
    # 30 px native short side is not small, but 0.6x model scale (18 px) is
    # strictly below the 18 px boundary.
    native = _record(native_short_side=30.0)
    derived = derive_model_scale_record(native, (640, 480), (384, 288))
    assert small_object_bucket(native, MetricScale.NATIVE_SCALE) is False
    assert small_object_bucket(derived, MetricScale.MODEL_INPUT_SCALE) is False
    # 29 px native -> 17.4 px model scale, still below the boundary.
    smaller = derive_model_scale_record(
        _record(native_short_side=29.0), (640, 480), (384, 288)
    )
    assert small_object_bucket(smaller, MetricScale.MODEL_INPUT_SCALE) is True


def test_small_object_bucket_boundary_is_strict() -> None:
    record = _record(native_short_side=18.0)
    assert small_object_bucket(record, MetricScale.NATIVE_SCALE) is False
    record_just_below = _record(native_short_side=17.999)
    assert small_object_bucket(record_just_below, MetricScale.NATIVE_SCALE) is True


def test_small_object_bucket_fails_closed_on_missing_fields() -> None:
    with pytest.raises(ValueError, match="scale fields missing"):
        small_object_bucket({"short_side": 10.0})


def test_small_object_bucket_returns_bool() -> None:
    assert isinstance(small_object_bucket(_record(10.0)), bool)
