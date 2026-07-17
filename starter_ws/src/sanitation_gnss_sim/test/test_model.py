import math
import statistics

from sanitation_gnss_sim.model import (
    GnssNoiseModel,
    PROFILES,
    local_xy_to_wgs84,
)


def test_fixed_profile_is_deterministic_and_within_expected_noise():
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


def test_fixed_profile_covariance_includes_bias_and_accumulated_random_walk():
    profile = PROFILES["rtk_fixed"]
    model = GnssNoiseModel(profile, seed=1)
    first = model.sample(0.0, 0.0, 0.1)
    second = model.sample(0.0, 0.0, 9.9)
    base = profile.standard_deviation_m**2
    bias = profile.fixed_bias_standard_deviation_m**2
    walk_rate = profile.random_walk_standard_deviation_m_sqrt_s**2
    assert math.isclose(first.variance_m2, base + bias + walk_rate * 0.1)
    assert math.isclose(second.variance_m2, base + bias + walk_rate * 10.0)
    assert second.variance_m2 > first.variance_m2


def test_multipath_profile_injects_approximately_one_percent_outliers():
    model = GnssNoiseModel(PROFILES["multipath"], seed=8)
    samples = [model.sample(0.0, 0.0, 0.1) for _ in range(10_000)]
    multipath_count = sum(item.multipath for item in samples)
    assert 70 <= multipath_count <= 130
    assert max(math.hypot(item.x_m, item.y_m) for item in samples) > 0.4


def test_local_xy_wgs84_round_trip_scale():
    latitude, longitude = local_xy_to_wgs84(10.0, -4.0, 31.2304, 121.4737)
    earth_radius = 6378137.0
    recovered_y = math.radians(latitude - 31.2304) * earth_radius
    recovered_x = (
        math.radians(longitude - 121.4737)
        * earth_radius
        * math.cos(math.radians(31.2304))
    )
    assert math.isclose(recovered_x, 10.0, abs_tol=1e-6)
    assert math.isclose(recovered_y, -4.0, abs_tol=1e-6)
