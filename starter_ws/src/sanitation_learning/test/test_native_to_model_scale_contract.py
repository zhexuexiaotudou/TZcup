from __future__ import annotations

import cv2
import numpy as np
import pytest

from sanitation_learning.metric_scale import (
    InstanceScaleRecord,
    assert_scale_fields_present,
    derive_model_scale_record,
)


def test_derive_model_scale_bbox_and_short_side() -> None:
    record = {
        "native_bbox_xyxy": [40.0, 30.0, 120.0, 90.0],
        "native_short_side_px": 50.0,
        "native_mask_area_px": 200,
    }
    derived = derive_model_scale_record(
        record, (640, 480), (384, 288), model_mask=np.ones((480, 640), dtype=bool)
    )
    assert derived.model_bbox_xyxy == pytest.approx((24.0, 18.0, 72.0, 54.0))
    assert derived.model_short_side_px == pytest.approx(36.0)
    assert derived.native_short_side_px == pytest.approx(50.0)


def test_model_mask_area_uses_nearest_resize_not_area_scaling() -> None:
    record = {
        "native_bbox_xyxy": [0.0, 0.0, 640.0, 480.0],
        "native_short_side_px": 480.0,
        "native_mask_area_px": 16,
    }
    mask = np.ones((480, 640), dtype=bool)
    derived = derive_model_scale_record(
        record, (640, 480), (384, 288), model_mask=mask
    )
    assert derived.model_mask_area_px == 384 * 288
    # The proportional fallback (16 * 0.6 * 0.6) is never allowed to replace
    # the INTER_NEAREST pixel count when a mask is provided.
    assert derived.model_mask_area_px != int(round(16 * 0.6 * 0.6))
    partial = np.zeros((480, 640), dtype=bool)
    partial[:4, :4] = True
    derived_partial = derive_model_scale_record(
        record, (640, 480), (384, 288), model_mask=partial
    )
    expected = int(
        np.count_nonzero(
            cv2.resize(
                partial.astype(np.uint8),
                (384, 288),
                interpolation=cv2.INTER_NEAREST,
            )
        )
    )
    assert derived_partial.model_mask_area_px == expected


def test_proportional_area_fallback_without_mask() -> None:
    record = {
        "native_bbox_xyxy": [0.0, 0.0, 40.0, 40.0],
        "native_short_side_px": 40.0,
        "native_mask_area_px": 1600,
    }
    derived = derive_model_scale_record(record, (640, 480), (384, 288))
    assert derived.model_mask_area_px == int(round(1600 * 0.6 * 0.6))


def test_derive_accepts_instance_scale_record_and_dict() -> None:
    native = {
        "native_bbox_xyxy": [20.0, 10.0, 100.0, 50.0],
        "native_short_side_px": 30.0,
        "native_mask_area_px": 300,
    }
    mask = np.ones((480, 640), dtype=bool)
    from_dict = derive_model_scale_record(
        native, (640, 480), (384, 288), model_mask=mask
    )
    record = InstanceScaleRecord(
        native_bbox_xyxy=(20.0, 10.0, 100.0, 50.0),
        native_short_side_px=30.0,
        native_mask_area_px=300,
        model_bbox_xyxy=(0.0, 0.0, 0.0, 0.0),
        model_short_side_px=0.0,
        model_mask_area_px=0,
    )
    from_record = derive_model_scale_record(
        record, (640, 480), (384, 288), model_mask=mask
    )
    assert from_dict == from_record
    assert isinstance(from_record, InstanceScaleRecord)


def test_assert_scale_fields_present_raises_on_missing_fields() -> None:
    native_only = {
        "native_bbox_xyxy": [0.0, 0.0, 10.0, 10.0],
        "native_short_side_px": 10.0,
        "native_mask_area_px": 100,
    }
    with pytest.raises(ValueError, match="scale fields missing"):
        assert_scale_fields_present(native_only)
    complete = derive_model_scale_record(native_only, (640, 480), (384, 288))
    assert_scale_fields_present(complete)
    assert_scale_fields_present(complete.to_dict())


def test_invalid_mask_shape_fails_closed() -> None:
    record = {
        "native_bbox_xyxy": [0.0, 0.0, 10.0, 10.0],
        "native_short_side_px": 10.0,
        "native_mask_area_px": 100,
    }
    with pytest.raises(ValueError, match="must match source resolution"):
        derive_model_scale_record(
            record,
            (640, 480),
            (384, 288),
            model_mask=np.ones((100, 100), dtype=bool),
        )
