import math

import pytest

from sanitation_gnss_sim.dual_navsat import (
    NavSatPairBuffer,
    RawNavSatSample,
    solve_dual_navsat,
)
from sanitation_gnss_sim.model import local_xy_to_wgs84


ORIGIN = (31.2304, 121.4737)


def sample(stamp_ns: int, x_m: float, y_m: float) -> RawNavSatSample:
    latitude, longitude = local_xy_to_wgs84(x_m, y_m, *ORIGIN)
    return RawNavSatSample(stamp_ns, latitude, longitude, 5.0)


def test_dual_navsat_solution_uses_midpoint_and_baseline_heading() -> None:
    solution = solve_dual_navsat(
        sample(1_000_000_000, 3.4, -2.0),
        sample(1_000_000_000, 2.6, -2.0),
        origin_latitude_deg=ORIGIN[0],
        origin_longitude_deg=ORIGIN[1],
        minimum_baseline_m=0.75,
        maximum_baseline_m=0.85,
    )
    assert math.isclose(solution.center_x_m, 3.0, abs_tol=1e-6)
    assert math.isclose(solution.center_y_m, -2.0, abs_tol=1e-6)
    assert math.isclose(solution.heading_rad, 0.0, abs_tol=1e-6)
    assert math.isclose(solution.baseline_m, 0.8, abs_tol=1e-6)


def test_invalid_dual_navsat_baseline_fails_closed() -> None:
    with pytest.raises(ValueError, match="baseline out of range"):
        solve_dual_navsat(
            sample(1, 0.1, 0.0),
            sample(1, 0.0, 0.0),
            origin_latitude_deg=ORIGIN[0],
            origin_longitude_deg=ORIGIN[1],
            minimum_baseline_m=0.75,
            maximum_baseline_m=0.85,
        )


def test_pair_buffer_rejects_cross_epoch_pairs() -> None:
    pairs = NavSatPairBuffer(maximum_skew_ns=20_000_000)
    assert pairs.push("front", sample(100_000_000, 0.4, 0.0)) is None
    assert pairs.push("rear", sample(200_000_000, -0.4, 0.0)) is None
    paired = pairs.push("front", sample(205_000_000, 0.4, 0.0))
    assert paired is not None
    assert abs(paired[0].stamp_ns - paired[1].stamp_ns) == 5_000_000

    with pytest.raises(ValueError, match="unknown antenna"):
        pairs.push("middle", sample(300_000_000, 0.0, 0.0))
