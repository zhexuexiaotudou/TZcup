from __future__ import annotations

import copy

import pytest

from formal_water_motor_metrics import central_roller_duty_metrics, side_brush_duty_metrics


def _samples() -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    for index in range(61):
        stamp = index * 0.05
        startup = stamp < 0.50
        motors = []
        for name, sign in (("left_side_brush", 1.0), ("right_side_brush", -1.0)):
            motors.append({
                "name": name,
                "command": sign * 8.0,
                "measured_speed": sign * (4.0 if startup else 7.2),
                "current_a": 1.1 if startup else 0.5,
                "temperature_c": 31.0,
                "protection_active": False,
                "fault": "none",
            })
        motors.append({
            "name": "central_roller",
            "command": 12.0,
            "measured_speed": 6.0 if startup else 10.8,
            "current_a": 1.1 if startup else 0.5,
            "temperature_c": 32.0,
            "protection_active": False,
            "fault": "none",
        })
        samples.append({
            "sim_time_s": stamp,
            "physics_update_stale": False,
            "fault_active": False,
            "motors": motors,
        })
    return samples


def test_metrics_separate_startup_transient_from_steady_tracking() -> None:
    metrics = side_brush_duty_metrics(_samples())
    for side in ("left_side_brush", "right_side_brush"):
        row = metrics[side]
        assert row["steady_sample_count"] >= 20
        assert row["p05_tracking_ratio"] == pytest.approx(0.9)
        assert row["p50_tracking_ratio"] == pytest.approx(0.9)
        assert row["steady_peak_current_a"] == pytest.approx(0.5)
        assert row["maximum_contiguous_over_rated_s"] < 1.0
        assert row["maximum_contiguous_low_speed_s"] == pytest.approx(0.0)
        assert row["direction_matches_all_steady_samples"] is True
        assert row["fault_free_all_samples"] is True


def test_metrics_fail_closed_on_wrong_direction_fault_and_missing_speed() -> None:
    samples = copy.deepcopy(_samples())
    left = samples[-1]["motors"][0]  # type: ignore[index]
    left["measured_speed"] = -7.2
    left["fault"] = "stall"
    left["protection_active"] = True
    samples[-1]["fault_active"] = True
    right = samples[-1]["motors"][1]  # type: ignore[index]
    del right["measured_speed"]
    metrics = side_brush_duty_metrics(samples)
    assert metrics["left_side_brush"]["direction_matches_all_steady_samples"] is False
    assert metrics["left_side_brush"]["fault_free_all_samples"] is False
    assert metrics["right_side_brush"]["all_fields_finite"] is False


def test_metrics_report_sustained_low_speed_and_overcurrent() -> None:
    samples = _samples()
    for sample in samples:
        if float(sample["sim_time_s"]) < 1.0:
            continue
        for motor in sample["motors"]:  # type: ignore[union-attr]
            motor["measured_speed"] = 0.5 if motor["command"] > 0 else -0.5
            motor["current_a"] = 1.2
    metrics = side_brush_duty_metrics(samples)
    for row in metrics.values():
        assert row["p05_tracking_ratio"] < 0.8
        assert row["maximum_contiguous_low_speed_s"] > 1.0
        assert row["steady_peak_current_a"] > 0.75
        assert row["maximum_contiguous_over_rated_s"] > 1.0


def test_central_roller_uses_12_rad_s_command_and_same_strict_duty_gates() -> None:
    row = central_roller_duty_metrics(_samples())
    assert row["expected_command_rad_s"] == pytest.approx(12.0)
    assert row["steady_sample_count"] >= 20
    assert row["p05_tracking_ratio"] == pytest.approx(0.9)
    assert row["steady_peak_current_a"] == pytest.approx(0.5)
    assert row["maximum_contiguous_over_rated_s"] < 1.0
    assert row["peak_temperature_c"] == pytest.approx(32.0)
    assert row["fault_free_all_samples"] is True


def test_central_roller_fails_closed_on_sustained_rigid_contact_overload() -> None:
    samples = _samples()
    for sample in samples:
        if float(sample["sim_time_s"]) < 1.0:
            continue
        motor = sample["motors"][2]  # type: ignore[index]
        motor["measured_speed"] = 2.05
        motor["current_a"] = 2.05
        motor["temperature_c"] = 90.0
    row = central_roller_duty_metrics(samples)
    assert row["p05_tracking_ratio"] < 0.8
    assert row["maximum_contiguous_low_speed_s"] > 1.0
    assert row["steady_peak_current_a"] > 0.75
    assert row["maximum_contiguous_over_rated_s"] > 1.0
    assert row["peak_temperature_c"] >= 90.0
