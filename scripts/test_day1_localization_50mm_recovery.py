import importlib.util
import math
from pathlib import Path
import sys

import pytest


SPEC = importlib.util.spec_from_file_location(
    "localization_recovery",
    Path(__file__).with_name("day1_localization_50mm_recovery.py"),
)
recovery = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = recovery
SPEC.loader.exec_module(recovery)


def sample(t, x=0.0, y=0.0, yaw=0.0):
    return recovery.Pose2D(t, x, y, yaw)


def test_causal_filter_does_not_use_future_values():
    past = [sample(0.0), sample(1.0, x=1.0)]
    future_a = [sample(2.0, x=10.0), sample(3.0, x=20.0)]
    future_b = [sample(2.0, x=-10.0), sample(3.0, x=-20.0)]
    output_a = recovery.causal_lowpass(past + future_a, tau_sec=1.5)
    output_b = recovery.causal_lowpass(past + future_b, tau_sec=1.5)
    assert output_a[:2] == output_b[:2]


def test_constant_pose_is_unchanged():
    samples = [sample(index * 0.02, x=0.25, y=-0.5, yaw=0.75) for index in range(20)]
    assert recovery.causal_lowpass(samples, tau_sec=1.5) == samples


def test_angle_wrap_uses_short_arc():
    output = recovery.causal_lowpass(
        [sample(0.0, yaw=math.pi - 0.01), sample(0.1, yaw=-math.pi + 0.01)],
        tau_sec=0.1,
    )
    assert abs(abs(output[-1].yaw_rad) - math.pi) < 0.05


def test_conflicting_same_timestamp_is_rejected():
    with pytest.raises(ValueError, match="conflicting pose"):
        recovery.validate_pose_stream(
            [sample(1.0, x=0.0), sample(1.0, x=1.0)], "fixture"
        )


def test_sample_at_or_before_never_uses_future_value():
    stream = [sample(1.0, x=1.0), sample(2.0, x=2.0)]
    assert recovery.sample_at_or_before(stream, 1.5).x_m == 1.0
    assert recovery.sample_at_or_before(stream, 0.5) is None


def test_p95_uses_linear_interpolation():
    metrics = recovery.summarize_errors([0.1, 0.2, 0.3, 0.4])
    assert metrics["samples"] == 4
    assert metrics["p95_m"] == pytest.approx(0.385)
    assert metrics["max_m"] == pytest.approx(0.4)


def test_compose_uses_explicit_map_to_base_transform():
    x, y = recovery.compose_map_pose(
        sample(0.0, x=1.0, y=2.0, yaw=math.pi / 2.0),
        sample(0.0, x=2.0, y=0.0),
    )
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(4.0)


def test_motion_classification_is_exclusive():
    assert recovery.motion_class(0.0, 0.0) == "stationary"
    assert recovery.motion_class(0.2, 0.0) == "straight"
    assert recovery.motion_class(0.2, 0.1) == "turning"
