from __future__ import annotations

import pytest

from sanitation_learning.metric_scale import (
    MetricScale,
    derive_model_scale_record,
    machine_evaluable_bucket,
)


def _record(native_short_side: float, native_mask_area: int) -> dict:
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


def test_machine_evaluable_native_boundaries_are_inclusive() -> None:
    assert machine_evaluable_bucket(_record(8.0, 20)) is True
    assert machine_evaluable_bucket(_record(8.0, 20)) is True
    assert machine_evaluable_bucket(_record(7.9, 20)) is False
    assert machine_evaluable_bucket(_record(8.0, 19)) is False


def test_machine_evaluable_scale_is_explicit_and_differs_between_scales() -> None:
    native = _record(native_short_side=14.0, native_mask_area=30)
    derived = derive_model_scale_record(native, (640, 480), (384, 288))
    assert machine_evaluable_bucket(native, MetricScale.NATIVE_SCALE) is True
    # 14 px * 0.6 = 8.4 px short side, but the 0.36 area ratio pushes the mask
    # area below the 20 px minimum, so the model-input view is not evaluable.
    assert machine_evaluable_bucket(derived, MetricScale.MODEL_INPUT_SCALE) is False


def test_machine_evaluable_custom_thresholds() -> None:
    record = _record(native_short_side=12.0, native_mask_area=80)
    assert (
        machine_evaluable_bucket(
            record,
            MetricScale.NATIVE_SCALE,
            min_short_side_px=12.0,
            min_mask_area_px=80.0,
        )
        is True
    )
    assert (
        machine_evaluable_bucket(
            record,
            MetricScale.NATIVE_SCALE,
            min_short_side_px=12.1,
            min_mask_area_px=80.0,
        )
        is False
    )


def test_machine_evaluable_fails_closed_on_missing_fields() -> None:
    with pytest.raises(ValueError, match="scale fields missing"):
        machine_evaluable_bucket({"short_side": 8.0, "mask_area": 20})


def test_machine_evaluable_returns_bool() -> None:
    assert isinstance(machine_evaluable_bucket(_record(9.0, 21)), bool)
