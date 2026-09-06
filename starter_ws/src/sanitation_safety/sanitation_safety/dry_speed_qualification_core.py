"""Fail-closed gate for the isolated formal dry-cleaning speed lane.

This intentionally has no ROS dependency so its eligibility contract can be
unit-tested independently of a simulator.  It is not a product-wide profile:
only the exact formal same-map dry-cleaning operation can temporarily expose
the 1.0 m/s cap, and only while its operator heartbeat and dry brush state are
both current.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


DEFAULT_MAX_LINEAR_VELOCITY_MPS = 0.45
REQUALIFIED_MAX_LINEAR_VELOCITY_MPS = 1.0
DRY_CLEANING_SPEED_PROFILE = "dry_cleaning_competition_candidate"
ISOLATED_SAME_MAP_DRY_STATE = "isolated_same_map_dry_coverage"


@dataclass
class DrySpeedQualificationState:
    """State that returns the actual cap, never an asserted launch value."""

    configured_max_linear_velocity_mps: float
    mission_mode: str
    operation_speed_profile: str
    qualification_state: str
    heartbeat_timeout_sec: float = 0.25
    qualification_active: bool = False
    last_qualification_heartbeat_monotonic: float | None = None
    dry_brush_active: bool = False
    last_dry_brush_monotonic: float | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.configured_max_linear_velocity_mps) or (
            self.configured_max_linear_velocity_mps <= 0.0
        ):
            raise ValueError("configured max linear velocity must be positive and finite")
        if not math.isfinite(self.heartbeat_timeout_sec) or self.heartbeat_timeout_sec <= 0.0:
            raise ValueError("qualification heartbeat timeout must be positive and finite")

    def set_qualification_active(self, active: bool, now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("qualification heartbeat time must be finite")
        self.qualification_active = bool(active)
        self.last_qualification_heartbeat_monotonic = now if active else None

    def set_dry_brush_active(self, active: bool, now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("dry brush state time must be finite")
        self.dry_brush_active = bool(active)
        self.last_dry_brush_monotonic = now if active else None

    def effective_max_linear_velocity_mps(
        self,
        *,
        now: float,
        pump_output: Sequence[float],
    ) -> float:
        if not math.isfinite(now):
            raise ValueError("qualification evaluation time must be finite")
        heartbeat_fresh = (
            self.qualification_active
            and self.last_qualification_heartbeat_monotonic is not None
            and now - self.last_qualification_heartbeat_monotonic <= self.heartbeat_timeout_sec
        )
        dry_brush_fresh = (
            self.dry_brush_active
            and self.last_dry_brush_monotonic is not None
            and now - self.last_dry_brush_monotonic <= self.heartbeat_timeout_sec
        )
        wet_pump_inactive = all(abs(float(value)) <= 1.0e-9 for value in pump_output)
        qualified = (
            self.configured_max_linear_velocity_mps == REQUALIFIED_MAX_LINEAR_VELOCITY_MPS
            and self.mission_mode == "cleaning"
            and self.operation_speed_profile == DRY_CLEANING_SPEED_PROFILE
            and self.qualification_state == ISOLATED_SAME_MAP_DRY_STATE
            and heartbeat_fresh
            and dry_brush_fresh
            and wet_pump_inactive
        )
        return REQUALIFIED_MAX_LINEAR_VELOCITY_MPS if qualified else min(
            self.configured_max_linear_velocity_mps,
            DEFAULT_MAX_LINEAR_VELOCITY_MPS,
        )
