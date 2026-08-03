import math

from sanitation_coverage.skid_steer_rotation import (
    bounded_angular_command,
    normalized_yaw_error,
)


def test_yaw_error_takes_short_path_across_pi_boundary():
    error = normalized_yaw_error(-math.pi + 0.05, math.pi - 0.05)
    assert abs(error - 0.10) < 1e-9


def test_angular_command_stops_inside_tolerance_and_is_bounded():
    assert bounded_angular_command(0.05) == 0.0
    assert bounded_angular_command(2.0) == 0.60
    assert bounded_angular_command(-0.08) == -0.12
