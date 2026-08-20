import math

import pytest

from sanitation_tasks.tf_continuity_core import (
    transform_jump,
    wrapped_angle_delta,
)


def test_wrapped_angle_delta_handles_pi_boundary():
    assert wrapped_angle_delta(3.1, -3.1) == pytest.approx(
        2.0 * (math.pi - 3.1)
    )


def test_transform_jump_is_diagnostic_not_a_frame_break_decision():
    small = transform_jump(
        (0.0, 0.0, 0.0),
        (0.2, 0.1, 0.05),
        translation_threshold_m=1.0,
        yaw_threshold_rad=0.35,
    )
    large = transform_jump(
        (0.0, 0.0, 0.0),
        (1.2, 0.0, 0.0),
        translation_threshold_m=1.0,
        yaw_threshold_rad=0.35,
    )
    assert small["exceeds_diagnostic_threshold"] is False
    assert large["exceeds_diagnostic_threshold"] is True
