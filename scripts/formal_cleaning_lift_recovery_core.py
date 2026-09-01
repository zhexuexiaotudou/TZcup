"""Pure state machine for recovering an interrupted cleaning-lift command."""

from __future__ import annotations

from dataclasses import dataclass
import math


P16_RATED_SPEED_M_S = 0.0048


def trajectory_duration_s(
    current_position_m: float,
    target_position_m: float = 0.100,
    rated_speed_m_s: float = P16_RATED_SPEED_M_S,
) -> float:
    """Return a trajectory duration which never exceeds the rated P16 speed."""
    if not all(
        math.isfinite(value)
        for value in (current_position_m, target_position_m, rated_speed_m_s)
    ) or rated_speed_m_s <= 0.0:
        raise ValueError("lift position, target and rated speed must be finite")
    remaining_m = abs(target_position_m - current_position_m)
    # Round upward to 100 ms so serialization cannot accidentally request a
    # velocity above the 4.8 mm/s public actuator rating.
    return max(0.1, math.ceil(remaining_m / rated_speed_m_s * 10.0) / 10.0)


@dataclass(frozen=True)
class LiftReissue:
    attempt: int
    target_position_m: float
    duration_s: float
    actual_position_m: float


class CleaningLiftRecoverySupervisor:
    """Request a bounded reissue only after safety recovery and a real plateau."""

    def __init__(
        self,
        *,
        target_position_m: float = 0.100,
        position_tolerance_m: float = 0.0002,
        plateau_tolerance_m: float = 0.0001,
        plateau_duration_sim_s: float = 0.5,
        max_reissues: int = 3,
    ) -> None:
        self.target_position_m = target_position_m
        self.position_tolerance_m = position_tolerance_m
        self.plateau_tolerance_m = plateau_tolerance_m
        self.plateau_duration_sim_s = plateau_duration_sim_s
        self.max_reissues = max_reissues
        self.reissue_count = 0
        self.exhausted = False
        self._last_safety_permit: bool | None = None
        self._interruption_pending = False
        self._plateau_start_sim_s: float | None = None
        self._plateau_anchor_m: float | None = None

    def observe(
        self,
        *,
        sim_time_s: float | None,
        actual_position_m: float | None,
        safety_permit: bool | None,
    ) -> LiftReissue | None:
        if (
            sim_time_s is None
            or actual_position_m is None
            or safety_permit is None
            or not math.isfinite(sim_time_s)
            or not math.isfinite(actual_position_m)
        ):
            return None

        if not safety_permit:
            self._interruption_pending = True
            self._plateau_start_sim_s = None
            self._plateau_anchor_m = actual_position_m
            self._last_safety_permit = False
            return None

        if actual_position_m >= self.target_position_m - self.position_tolerance_m:
            self._interruption_pending = False
            self._plateau_start_sim_s = None
            self._plateau_anchor_m = actual_position_m
            self._last_safety_permit = True
            return None

        if self._last_safety_permit is False:
            self._plateau_start_sim_s = sim_time_s
            self._plateau_anchor_m = actual_position_m
        self._last_safety_permit = True

        if not self._interruption_pending:
            return None
        if self._plateau_start_sim_s is None or self._plateau_anchor_m is None:
            self._plateau_start_sim_s = sim_time_s
            self._plateau_anchor_m = actual_position_m
            return None
        if abs(actual_position_m - self._plateau_anchor_m) > self.plateau_tolerance_m:
            self._plateau_start_sim_s = sim_time_s
            self._plateau_anchor_m = actual_position_m
            return None
        if sim_time_s - self._plateau_start_sim_s < self.plateau_duration_sim_s:
            return None
        if self.reissue_count >= self.max_reissues:
            self.exhausted = True
            return None

        self.reissue_count += 1
        self._interruption_pending = False
        self._plateau_start_sim_s = None
        return LiftReissue(
            attempt=self.reissue_count,
            target_position_m=self.target_position_m,
            duration_s=trajectory_duration_s(
                actual_position_m, self.target_position_m
            ),
            actual_position_m=actual_position_m,
        )
