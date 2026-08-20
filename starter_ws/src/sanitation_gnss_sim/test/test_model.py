import math
import statistics

from sanitation_gnss_sim.model import (
    GnssNoiseModel,
    PROFILES,
    local_xy_to_wgs84,
    wgs84_to_local_xy,
)


def test_fixed_profile_is_deterministic_and_within_expected_noise():
    assert PROFILES["rtk_fixed"].correlated_drift_standard_deviation_m == 0.01
    assert PROFILES["rtk_float"].correlated_drift_standard_deviation_m == 0.08
    first = GnssNoiseModel(PROFILES["rtk_fixed"], seed=23)
    second = GnssNoiseModel(PROFILES["rtk_fixed"], seed=23)
    first_samples = [first.sample(2.0, -1.0, 0.1) for _ in range(1000)]
    second_samples = [second.sample(2.0, -1.0, 0.1) for _ in range(1000)]
    assert first_samples == second_samples
    errors = [math.hypot(item.x_m - 2.0, item.y_m + 1.0) for item in first_samples]
    assert statistics.median(errors) < 0.04
    assert max(errors) < 0.12


def test_denied_profile_never_publishes():
    model = GnssNoiseModel(PROFILES["gnss_denied"], seed=1)
    assert not any(model.sample(0.0, 0.0, 0.1).publish for _ in range(100))


def test_fixed_profile_covariance_is_bounded_for_long_missions():
    profile = PROFILES["rtk_fixed"]
    model = GnssNoiseModel(profile, seed=1)
    first = model.sample(0.0, 0.0, 0.1)
    second = model.sample(0.0, 0.0, 9.9)
    base = profile.standard_deviation_m**2
    bias = profile.fixed_bias_standard_deviation_m**2
    drift = profile.correlated_drift_standard_deviation_m**2
    assert math.isclose(first.variance_m2, base + bias + drift)
    assert math.isclose(second.variance_m2, base + bias + drift)


def test_fixed_profile_does_not_diverge_over_hour_long_mission():
    profile = PROFILES["rtk_fixed"]
    model = GnssNoiseModel(profile, seed=2019)
    samples = [model.sample(0.0, 0.0, 0.1) for _ in range(36_000)]
    errors = [math.hypot(item.x_m, item.y_m) for item in samples]
    assert statistics.quantiles(errors, n=100)[94] < 0.06
    assert max(errors) < 0.13


def test_fixed_dual_antenna_heading_is_noisy_bounded_and_wrap_safe():
    profile = PROFILES["rtk_fixed"]
    model = GnssNoiseModel(profile, seed=2022)
    truth = math.pi - 0.002
    samples = [model.sample(0.0, 0.0, 0.1, truth) for _ in range(36_000)]
    errors = [
        abs(math.atan2(math.sin(item.heading_rad - truth), math.cos(item.heading_rad - truth)))
        for item in samples
    ]
    assert all(-math.pi <= item.heading_rad <= math.pi for item in samples)
    assert statistics.quantiles(errors, n=100)[94] < math.radians(0.8)
    assert max(errors) < math.radians(1.5)
    assert all(item.heading_variance_rad2 > 0.0 for item in samples)


def test_multipath_profile_injects_approximately_one_percent_outliers():
    model = GnssNoiseModel(PROFILES["multipath"], seed=8)
    samples = [model.sample(0.0, 0.0, 0.1) for _ in range(10_000)]
    multipath_count = sum(item.multipath for item in samples)
    assert 70 <= multipath_count <= 130
    assert max(math.hypot(item.x_m, item.y_m) for item in samples) > 0.4


def test_local_xy_wgs84_round_trip_scale():
    latitude, longitude = local_xy_to_wgs84(10.0, -4.0, 31.2304, 121.4737)
    recovered_x, recovered_y = wgs84_to_local_xy(
        latitude, longitude, 31.2304, 121.4737
    )
    assert math.isclose(recovered_x, 10.0, abs_tol=1e-6)
    assert math.isclose(recovered_y, -4.0, abs_tol=1e-6)
