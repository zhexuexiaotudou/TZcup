from pathlib import Path

import pytest

from sanitation_learning.oprv3_geometry import class_window_kwargs, derive_product_geometry
from sanitation_learning.oprv3_online import ClassWindow


ROOT = Path(__file__).resolve().parents[4]


def test_actionable_windows_are_pre_model_geometry_and_nonempty():
    report = derive_product_geometry(ROOT)
    assert report["frozen_before_moving_model_measurement"] is True
    assert report["observation_semantics"]["actionable_window_eligibility_uses_model_result"] is False
    assert report["all_classes_have_nonempty_window"] is True
    windows = {
        name: ClassWindow(class_id=name, **values)
        for name, values in class_window_kwargs(report).items()
    }
    assert set(windows) == {"plastic_bottle", "metal_can", "paper_litter", "leaf_pile", "puddle"}
    assert all(window.minimum_visible_frames == 3 for window in windows.values())
    assert all(window.minimum_actionable_range_m > 0.55 for window in windows.values())
    assert all(
        isinstance(values["frames_in_actionable_window"], int)
        and values["frames_in_actionable_window"] >= values["minimum_visible_frames"]
        for values in report["class_actionable_windows"].values()
    )


def test_no_return_range_includes_latency_braking_and_brush_offset():
    report = derive_product_geometry(ROOT)
    action = report["vehicle_and_action"]
    expected = (
        action["brush_forward_offset_m"]
        + action["normal_product_speed_m_s"] * action["control_latency_s"]
        + action["normal_product_speed_m_s"] ** 2 / (2 * action["maximum_deceleration_m_s2"])
    )
    assert action["no_return_range_from_base_m"] == pytest.approx(expected)
