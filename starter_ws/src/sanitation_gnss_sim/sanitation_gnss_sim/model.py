from dataclasses import dataclass
import math
import random


EARTH_RADIUS_M = 6378137.0


@dataclass(frozen=True)
class GnssProfile:
    name: str
    publish: bool
    rate_hz: float
    standard_deviation_m: float
    latency_s: float
    dropout_probability: float
    multipath_probability: float
    multipath_magnitude_m: float
    fixed_bias_standard_deviation_m: float = 0.005
    correlated_drift_standard_deviation_m: float = 0.02
    correlated_drift_time_constant_s: float = 120.0
    heading_standard_deviation_rad: float = math.radians(0.25)
    heading_fixed_bias_standard_deviation_rad: float = math.radians(0.10)
    heading_correlated_drift_standard_deviation_rad: float = math.radians(0.10)


PROFILES = {
    "rtk_fixed": GnssProfile(
        "rtk_fixed", True, 10.0, 0.02, 0.10, 0.0, 0.0, 0.0,
        fixed_bias_standard_deviation_m=0.005,
        correlated_drift_standard_deviation_m=0.01,
        correlated_drift_time_constant_s=120.0,
    ),
    "rtk_float": GnssProfile(
        "rtk_float", True, 10.0, 0.12, 0.10, 0.0, 0.0, 0.0,
        correlated_drift_standard_deviation_m=0.08,
        correlated_drift_time_constant_s=180.0,
    ),
    "gnss_denied": GnssProfile(
        "gnss_denied", False, 10.0, 0.0, 0.10, 1.0, 0.0, 0.0,
        correlated_drift_standard_deviation_m=0.0,
    ),
    "multipath": GnssProfile(
        "multipath", True, 10.0, 0.02, 0.10, 0.0, 0.01, 0.50,
        correlated_drift_standard_deviation_m=0.01,
        correlated_drift_time_constant_s=120.0,
    ),
}


@dataclass(frozen=True)
class GnssMeasurement:
    publish: bool
    x_m: float
    y_m: float
    variance_m2: float
    heading_rad: float
    heading_variance_rad2: float
    multipath: bool
    reason: str


class GnssNoiseModel:
    def __init__(self, profile: GnssProfile, seed: int):
        self.profile = profile
        self._random = random.Random(seed)
        self._bias_x = self._random.gauss(
            0.0, profile.fixed_bias_standard_deviation_m
        )
        self._bias_y = self._random.gauss(
            0.0, profile.fixed_bias_standard_deviation_m
        )
        # A fixed RTK solution has time-correlated error, but its uncertainty
        # does not grow without bound.  Start the first-order Gauss-Markov
        # state in its stationary distribution so short and long missions are
        # statistically comparable.
        self._drift_x = self._random.gauss(
            0.0, profile.correlated_drift_standard_deviation_m
        )
        self._drift_y = self._random.gauss(
            0.0, profile.correlated_drift_standard_deviation_m
        )
        # Keep heading noise on an independent deterministic stream so adding
        # the dual-antenna heading lane does not rewrite established XY seeds.
        self._heading_random = random.Random(seed ^ 0x47505348)
        self._heading_bias = self._heading_random.gauss(
            0.0, profile.heading_fixed_bias_standard_deviation_rad
        )
        self._heading_drift = self._heading_random.gauss(
            0.0, profile.heading_correlated_drift_standard_deviation_rad
        )

    @property
    def fixed_bias(self):
        return self._bias_x, self._bias_y

    def sample(
        self,
        truth_x_m: float,
        truth_y_m: float,
        dt_s: float,
        truth_heading_rad: float = 0.0,
    ) -> GnssMeasurement:
        if not self.profile.publish:
            return GnssMeasurement(
                False, 0.0, 0.0, 0.0, 0.0, 0.0, False, "profile_denied"
            )
        if self._random.random() < self.profile.dropout_probability:
            return GnssMeasurement(
                False, 0.0, 0.0, 0.0, 0.0, 0.0, False, "random_dropout"
            )

        dt = max(0.0, dt_s)
        time_constant = max(1e-6, self.profile.correlated_drift_time_constant_s)
        retention = math.exp(-dt / time_constant)
        innovation_sigma = self.profile.correlated_drift_standard_deviation_m * math.sqrt(
            max(0.0, 1.0 - retention * retention)
        )
        self._drift_x = retention * self._drift_x + self._random.gauss(
            0.0, innovation_sigma
        )
        self._drift_y = retention * self._drift_y + self._random.gauss(
            0.0, innovation_sigma
        )
        heading_innovation_sigma = (
            self.profile.heading_correlated_drift_standard_deviation_rad
            * math.sqrt(max(0.0, 1.0 - retention * retention))
        )
        self._heading_drift = retention * self._heading_drift + self._heading_random.gauss(
            0.0, heading_innovation_sigma
        )
        x_m = truth_x_m + self._bias_x + self._drift_x + self._random.gauss(
            0.0, self.profile.standard_deviation_m
        )
        y_m = truth_y_m + self._bias_y + self._drift_y + self._random.gauss(
            0.0, self.profile.standard_deviation_m
        )
        multipath = self._random.random() < self.profile.multipath_probability
        if multipath:
            angle = self._random.uniform(-math.pi, math.pi)
            x_m += self.profile.multipath_magnitude_m * math.cos(angle)
            y_m += self.profile.multipath_magnitude_m * math.sin(angle)
        variance = (
            self.profile.standard_deviation_m**2
            + self.profile.fixed_bias_standard_deviation_m**2
            + self.profile.correlated_drift_standard_deviation_m**2
        )
        heading = truth_heading_rad + self._heading_bias + self._heading_drift
        heading += self._heading_random.gauss(
            0.0, self.profile.heading_standard_deviation_rad
        )
        heading = math.atan2(math.sin(heading), math.cos(heading))
        heading_variance = (
            self.profile.heading_standard_deviation_rad**2
            + self.profile.heading_fixed_bias_standard_deviation_rad**2
            + self.profile.heading_correlated_drift_standard_deviation_rad**2
        )
        return GnssMeasurement(
            True, x_m, y_m, variance, heading, heading_variance,
            multipath, "published"
        )


def local_xy_to_wgs84(x_m, y_m, origin_latitude_deg, origin_longitude_deg):
    origin_latitude_rad = math.radians(origin_latitude_deg)
    latitude = origin_latitude_deg + math.degrees(y_m / EARTH_RADIUS_M)
    longitude = origin_longitude_deg + math.degrees(
        x_m / (EARTH_RADIUS_M * math.cos(origin_latitude_rad))
    )
    return latitude, longitude


def wgs84_to_local_xy(latitude_deg, longitude_deg, origin_latitude_deg, origin_longitude_deg):
    origin_latitude_rad = math.radians(origin_latitude_deg)
    y_m = math.radians(latitude_deg - origin_latitude_deg) * EARTH_RADIUS_M
    x_m = (
        math.radians(longitude_deg - origin_longitude_deg)
        * EARTH_RADIUS_M
        * math.cos(origin_latitude_rad)
    )
    return x_m, y_m
