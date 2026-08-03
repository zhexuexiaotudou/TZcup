import math

from sanitation_coverage.skid_steer_translation import (
    bounded_translation_command,
    translation_errors,
)


def test_translation_errors_are_resolved_in_chassis_travel_frame():
    along, cross, yaw, distance = translation_errors((0, 0.1, 0), (1, 0, 0), 0)
    assert along == 1.0
    assert abs(cross + 0.1) < 1e-9
    assert yaw == 0.0
    assert abs(distance - math.hypot(1, 0.1)) < 1e-9


def test_translation_command_supports_bounded_forward_and_backup():
    assert bounded_translation_command(1.0, 0.0, 0.0) == (0.55, 0.0)
    assert bounded_translation_command(-1.0, 0.0, 0.0) == (-0.30, 0.0)
    assert bounded_translation_command(1.0, 0.0, 0.5)[0] == 0.0
