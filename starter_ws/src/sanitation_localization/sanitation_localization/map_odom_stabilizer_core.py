"""ROS-independent causal map->odom filter used by the live TF stabilizer."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    stamp_sec: float
    x_m: float
    y_m: float
    yaw_rad: float


class StabilizerInputError(ValueError):
    """Raised when an input cannot be accepted without guessing."""


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CausalMapOdomStabilizer:
    """One-pole low-pass over map->odom using only current/past observations."""

    def __init__(
        self,
        *,
        tau_sec: float = 1.5,
        max_dt_sec: float = 0.1,
        max_gap_sec: float = 0.5,
    ) -> None:
        if not 0.75 <= tau_sec <= 3.0:
            raise ValueError("tau_sec must be in [0.75, 3.0]")
        if not 0.02 <= max_dt_sec <= 0.2:
            raise ValueError("max_dt_sec must be in [0.02, 0.2]")
        if not 0.1 <= max_gap_sec <= 1.0:
            raise ValueError("max_gap_sec must be in [0.1, 1.0]")
        self.tau_sec = tau_sec
        self.max_dt_sec = max_dt_sec
        self.max_gap_sec = max_gap_sec
        self._last_raw: Pose2D | None = None
        self._last_output: Pose2D | None = None
        self._blocked_reason: str | None = None
        self.accepted_updates = 0
        self.duplicate_updates = 0
        self.rejected_updates = 0
        self.max_observed_gap_sec = 0.0

    @property
    def blocked_reason(self) -> str | None:
        return self._blocked_reason

    @property
    def last_input(self) -> Pose2D | None:
        return self._last_raw

    @property
    def last_output(self) -> Pose2D | None:
        return self._last_output

    def _block(self, reason: str) -> None:
        self._blocked_reason = reason
        self.rejected_updates += 1
        raise StabilizerInputError(reason)

    def block(self, reason: str) -> None:
        """Latch a fail-closed state without accepting another sample."""
        self._block(reason)

    def update(self, sample: Pose2D) -> Pose2D:
        if self._blocked_reason is not None:
            raise StabilizerInputError(self._blocked_reason)
        if not all(
            math.isfinite(value)
            for value in (sample.stamp_sec, sample.x_m, sample.y_m, sample.yaw_rad)
        ):
            self._block("non_finite_map_odom")
        if self._last_raw is not None:
            if sample.stamp_sec < self._last_raw.stamp_sec:
                self._block("non_monotonic_map_odom")
            if sample.stamp_sec == self._last_raw.stamp_sec:
                if sample != self._last_raw:
                    self._block("conflicting_map_odom_timestamp")
                self.duplicate_updates += 1
                if self._last_output is None:
                    self._block("duplicate_without_initial_output")
                return self._last_output
            gap = sample.stamp_sec - self._last_raw.stamp_sec
            self.max_observed_gap_sec = max(self.max_observed_gap_sec, gap)
            if gap > self.max_gap_sec:
                self._block("map_odom_input_gap")
            dt = min(gap, self.max_dt_sec)
            alpha = 1.0 - math.exp(-dt / self.tau_sec)
            assert self._last_output is not None
            yaw_delta = normalize_angle(sample.yaw_rad - self._last_output.yaw_rad)
            output = Pose2D(
                stamp_sec=sample.stamp_sec,
                x_m=self._last_output.x_m
                + alpha * (sample.x_m - self._last_output.x_m),
                y_m=self._last_output.y_m
                + alpha * (sample.y_m - self._last_output.y_m),
                yaw_rad=normalize_angle(
                    self._last_output.yaw_rad + alpha * yaw_delta
                ),
            )
        else:
            output = sample
        self._last_raw = sample
        self._last_output = output
        self.accepted_updates += 1
        return output
