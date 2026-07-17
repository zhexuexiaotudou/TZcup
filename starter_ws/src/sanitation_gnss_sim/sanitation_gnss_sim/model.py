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
    random_walk_standard_deviation_m_sqrt_s: float = 0.002


PROFILES = {
    "rtk_fixed": GnssProfile("rtk_fixed", True, 10.0, 0.02, 0.10, 0.0, 0.0, 0.0),
    "rtk_float": GnssProfile("rtk_float", True, 10.0, 0.12, 0.10, 0.0, 0.0, 0.0),
    "gnss_denied": GnssProfile("gnss_denied", False, 10.0, 0.0, 0.10, 1.0, 0.0, 0.0),
    "multipath": GnssProfile("multipath", True, 10.0, 0.02, 0.10, 0.0, 0.01, 0.50),
}


@dataclass(frozen=True)
class GnssMeasurement:
    publish: bool
    x_m: float
    y_m: float
    variance_m2: float
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
        self._walk_x = 0.0
        self._walk_y = 0.0
        self._elapsed_s = 0.0

    @property
    def fixed_bias(self):
        return self._bias_x, self._bias_y

    def sample(self, truth_x_m: float, truth_y_m: float, dt_s: float) -> GnssMeasurement:
        if not self.profile.publish:
            return GnssMeasurement(False, 0.0, 0.0, 0.0, False, "profile_denied")
        if self._random.random() < self.profile.dropout_probability:
            return GnssMeasurement(False, 0.0, 0.0, 0.0, False, "random_dropout")

        walk_sigma = self.profile.random_walk_standard_deviation_m_sqrt_s * math.sqrt(
            max(0.0, dt_s)
        )
        self._elapsed_s += max(0.0, dt_s)
        self._walk_x += self._random.gauss(0.0, walk_sigma)
        self._walk_y += self._random.gauss(0.0, walk_sigma)
        x_m = truth_x_m + self._bias_x + self._walk_x + self._random.gauss(
            0.0, self.profile.standard_deviation_m
        )
        y_m = truth_y_m + self._bias_y + self._walk_y + self._random.gauss(
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
            + self.profile.random_walk_standard_deviation_m_sqrt_s**2
            * self._elapsed_s
        )
        return GnssMeasurement(True, x_m, y_m, variance, multipath, "published")


def local_xy_to_wgs84(x_m, y_m, origin_latitude_deg, origin_longitude_deg):
    origin_latitude_rad = math.radians(origin_latitude_deg)
    latitude = origin_latitude_deg + math.degrees(y_m / EARTH_RADIUS_M)
    longitude = origin_longitude_deg + math.degrees(
        x_m / (EARTH_RADIUS_M * math.cos(origin_latitude_rad))
    )
    return latitude, longitude
