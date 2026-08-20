"""Pure state machine for bounded, coverage-aware active re-observation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum


class ReobservationState(str, Enum):
    IDLE = "IDLE"
    WAITING_POSE = "WAITING_POSE"
    WAITING_SAFE_PAUSE = "WAITING_SAFE_PAUSE"
    NAVIGATING = "NAVIGATING"
    WAITING_FRESH_VERDICT = "WAITING_FRESH_VERDICT"
    WAITING_RESUME = "WAITING_RESUME"
    COMPLETED = "COMPLETED"
    DEFERRED = "DEFERRED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ReobservationRequest:
    request_id: str
    track_uuid: str
    target_uuid: str
    stamp_ns: int
    x_m: float
    y_m: float
    covariance_trace: float
    class_id: str
    target_size_m: float
    reobserve_count: int
    source_backend: str


@dataclass(frozen=True)
class ReobservationSafety:
    emergency_stopped: bool
    collision_clear: bool
    localization_healthy: bool
    keepout_clear: bool
    path_available: bool


class ProductReobservationOrchestrator:
    def __init__(self, *, maximum_reobserve_count: int = 2) -> None:
        if not 0 <= maximum_reobserve_count <= 2:
            raise ValueError("re-observation budget must be in [0, 2]")
        self.maximum_reobserve_count = int(maximum_reobserve_count)
        self.state = ReobservationState.IDLE
        self.request: ReobservationRequest | None = None
        self.coverage_paused = False
        self.outcome: str | None = None
        self.timeline: list[dict] = []

    def _record(self, event: str, **details) -> None:
        self.timeline.append({"event": event, "state": self.state.value, **details})

    @staticmethod
    def _safety_reason(safety: ReobservationSafety) -> str | None:
        checks = (
            (not safety.emergency_stopped, "emergency_stop_active"),
            (safety.collision_clear, "collision_monitor_not_clear"),
            (safety.localization_healthy, "localization_unhealthy"),
            (safety.keepout_clear, "keepout_not_clear"),
            (safety.path_available, "path_unavailable"),
        )
        return next((reason for passed, reason in checks if not passed), None)

    @classmethod
    def safety_reason(cls, safety: ReobservationSafety) -> str | None:
        return cls._safety_reason(safety)

    def submit(self, request: ReobservationRequest) -> bool:
        if self.state not in {
            ReobservationState.IDLE,
            ReobservationState.COMPLETED,
            ReobservationState.DEFERRED,
            ReobservationState.FAILED,
        }:
            return False
        source = request.source_backend.lower()
        if any(token in source for token in (
            "ground_truth", "gazebo_registry", "evaluation_registry"
        )):
            raise ValueError("GT control violation: re-observation request rejected")
        if (
            not request.request_id
            or not request.track_uuid
            or not request.target_uuid
            or request.stamp_ns <= 0
            or request.target_size_m <= 0.0
            or request.reobserve_count < 1
            or request.reobserve_count > self.maximum_reobserve_count
        ):
            self.request = request
            self.state = ReobservationState.DEFERRED
            self._record("invalid_request")
            return False
        self.request = request
        self.coverage_paused = False
        self.outcome = None
        self.state = ReobservationState.WAITING_POSE
        self._record("request_accepted")
        return True

    def acknowledge_pose(self, accepted: bool, safety: ReobservationSafety) -> bool:
        if self.state != ReobservationState.WAITING_POSE:
            raise ValueError("observation-pose acknowledgement is out of sequence")
        reason = self._safety_reason(safety)
        if not accepted or reason is not None:
            self.state = ReobservationState.DEFERRED
            self._record("pose_rejected", reason=reason or "planner_rejected")
            return False
        self.state = ReobservationState.WAITING_SAFE_PAUSE
        self._record("pose_accepted")
        return True

    def acknowledge_pause(self, accepted: bool) -> bool:
        if self.state != ReobservationState.WAITING_SAFE_PAUSE:
            raise ValueError("coverage-pause acknowledgement is out of sequence")
        if not accepted:
            self.state = ReobservationState.DEFERRED
            self._record("coverage_pause_failed")
            return False
        self.coverage_paused = True
        self.state = ReobservationState.NAVIGATING
        self._record("coverage_paused")
        return True

    def acknowledge_navigation(
        self, succeeded: bool, safety: ReobservationSafety
    ) -> bool:
        if self.state != ReobservationState.NAVIGATING:
            raise ValueError("navigation acknowledgement is out of sequence")
        reason = self._safety_reason(safety)
        if not succeeded or reason is not None:
            self.state = ReobservationState.DEFERRED
            self._record("navigation_failed", reason=reason or "nav2_failed")
            return False
        self.state = ReobservationState.WAITING_FRESH_VERDICT
        self._record("observation_pose_reached")
        return True

    def observe_verdict(self, *, stamp_ns: int, verdict: str) -> bool:
        if self.state != ReobservationState.WAITING_FRESH_VERDICT:
            raise ValueError("fresh verdict is out of sequence")
        if self.request is None or int(stamp_ns) <= self.request.stamp_ns:
            self._record("stale_verdict_ignored", stamp_ns=int(stamp_ns))
            return False
        normalized = str(verdict).upper()
        if normalized not in {"ACCEPT", "OBSERVE_AGAIN", "DEFER", "REJECT"}:
            self._record("invalid_verdict_ignored", verdict=normalized)
            return False
        self.outcome = normalized
        self.state = ReobservationState.WAITING_RESUME
        self._record("fresh_verdict_received", verdict=normalized)
        return True

    def defer(self, reason: str) -> None:
        if self.state in {ReobservationState.COMPLETED, ReobservationState.FAILED}:
            return
        self.state = ReobservationState.DEFERRED
        self._record("deferred", reason=str(reason))

    def acknowledge_resume(self, accepted: bool) -> bool:
        if self.state not in {
            ReobservationState.WAITING_RESUME,
            ReobservationState.DEFERRED,
        }:
            raise ValueError("coverage-resume acknowledgement is out of sequence")
        self.coverage_paused = not accepted
        self.state = (
            ReobservationState.COMPLETED if accepted else ReobservationState.FAILED
        )
        self._record("coverage_resumed" if accepted else "coverage_resume_failed")
        return accepted

    def snapshot(self) -> dict:
        return {
            "state": self.state.value,
            "request": asdict(self.request) if self.request else None,
            "coverage_paused": self.coverage_paused,
            "outcome": self.outcome,
            "timeline": list(self.timeline),
            "ground_truth_control_allowed": False,
        }
