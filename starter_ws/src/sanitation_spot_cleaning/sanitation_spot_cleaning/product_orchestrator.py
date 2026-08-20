"""Fail-closed product spot-cleaning state machine.

The core owns decisions only. ROS services, Nav2 actions and brush commands are
performed by an adapter and acknowledged back into this state machine.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum


class ProductCleanState(str, Enum):
    IDLE = "IDLE"
    WAITING_SAFE_PAUSE = "WAITING_SAFE_PAUSE"
    APPROACHING = "APPROACHING"
    PRE_CLEAN_VERIFY = "PRE_CLEAN_VERIFY"
    CLEANING = "CLEANING"
    POST_CLEAN_VERIFY = "POST_CLEAN_VERIFY"
    WAITING_RESUME = "WAITING_RESUME"
    COMPLETED = "COMPLETED"
    DEFERRED = "DEFERRED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ProductTarget:
    uuid: str
    class_id: str
    target_type: str
    track_state: str
    confidence: float
    observation_count: int
    covariance_trace: float
    source_backend: str
    in_keepout: bool


@dataclass(frozen=True)
class ProductSafety:
    emergency_stopped: bool
    collision_clear: bool
    localization_healthy: bool
    perception_healthy: bool
    keepout_clear: bool
    path_available: bool
    observation_age_s: float


class ProductSpotCleanOrchestrator:
    def __init__(
        self,
        *,
        minimum_confidence: float = 0.80,
        minimum_observations: int = 3,
        maximum_covariance_trace: float = 0.03,
        maximum_observation_age_s: float = 1.0,
        absent_frames_required: int = 3,
        maximum_area_retry_count: int = 1,
    ) -> None:
        if absent_frames_required < 1 or maximum_area_retry_count < 0:
            raise ValueError("verification limits must be non-negative")
        self.minimum_confidence = float(minimum_confidence)
        self.minimum_observations = int(minimum_observations)
        self.maximum_covariance_trace = float(maximum_covariance_trace)
        self.maximum_observation_age_s = float(maximum_observation_age_s)
        self.absent_frames_required = int(absent_frames_required)
        self.maximum_area_retry_count = int(maximum_area_retry_count)
        self.state = ProductCleanState.IDLE
        self.target: ProductTarget | None = None
        self.absent_frames = 0
        self.area_retry_count = 0
        self.brush_enabled = False
        self.coverage_paused = False
        self.timeline: list[dict] = []

    def _record(self, event: str, **details) -> None:
        self.timeline.append(
            {"event": event, "state": self.state.value, **details}
        )

    @staticmethod
    def _source_is_forbidden(source: str) -> bool:
        lowered = source.lower()
        return any(
            token in lowered
            for token in ("ground_truth", "gazebo_registry", "evaluation_registry")
        )

    def _safety_reason(self, safety: ProductSafety) -> str | None:
        checks = (
            (not safety.emergency_stopped, "emergency_stop_active"),
            (safety.collision_clear, "collision_monitor_not_clear"),
            (safety.localization_healthy, "localization_unhealthy"),
            (safety.perception_healthy, "perception_unhealthy"),
            (safety.keepout_clear, "keepout_not_clear"),
            (safety.path_available, "nav2_path_unavailable"),
            (
                safety.observation_age_s <= self.maximum_observation_age_s,
                "observation_stale",
            ),
        )
        return next((reason for passed, reason in checks if not passed), None)

    def safety_reason(self, safety: ProductSafety) -> str | None:
        return self._safety_reason(safety)

    @staticmethod
    def motion_safety_reason(safety: ProductSafety) -> str | None:
        checks = (
            (not safety.emergency_stopped, "emergency_stop_active"),
            (safety.collision_clear, "collision_monitor_not_clear"),
            (safety.localization_healthy, "localization_unhealthy"),
            (safety.perception_healthy, "perception_unhealthy"),
            (safety.keepout_clear, "keepout_not_clear"),
            (safety.path_available, "nav2_path_unavailable"),
        )
        return next((reason for passed, reason in checks if not passed), None)

    def submit(self, target: ProductTarget, safety: ProductSafety) -> bool:
        if self.state not in {
            ProductCleanState.IDLE,
            ProductCleanState.COMPLETED,
            ProductCleanState.DEFERRED,
            ProductCleanState.FAILED,
        }:
            return False
        if self._source_is_forbidden(target.source_backend):
            raise ValueError("GT control violation: forbidden target source")
        reason = self._safety_reason(safety)
        if target.track_state != "CONFIRMED":
            reason = "target_not_confirmed"
        elif target.in_keepout:
            reason = "target_in_keepout"
        elif target.confidence < self.minimum_confidence:
            reason = "confidence_below_clean_threshold"
        elif target.observation_count < self.minimum_observations:
            reason = "insufficient_track_persistence"
        elif target.covariance_trace > self.maximum_covariance_trace:
            reason = "projection_covariance_too_large"
        self.target = target
        self.absent_frames = 0
        self.brush_enabled = False
        self.coverage_paused = False
        if reason is not None:
            self.state = ProductCleanState.DEFERRED
            self._record("target_deferred", reason=reason)
            return False
        self.state = ProductCleanState.WAITING_SAFE_PAUSE
        self._record("target_scheduled", target_uuid=target.uuid)
        return True

    def acknowledge_coverage_pause(self, accepted: bool) -> bool:
        if self.state != ProductCleanState.WAITING_SAFE_PAUSE:
            raise ValueError("coverage pause acknowledgement is out of sequence")
        if not accepted:
            self.state = ProductCleanState.DEFERRED
            self._record("coverage_pause_failed")
            return False
        self.coverage_paused = True
        self.state = ProductCleanState.APPROACHING
        self._record("coverage_paused")
        return True

    def acknowledge_approach(self, succeeded: bool) -> bool:
        if self.state != ProductCleanState.APPROACHING:
            raise ValueError("approach acknowledgement is out of sequence")
        self.state = (
            ProductCleanState.PRE_CLEAN_VERIFY
            if succeeded
            else ProductCleanState.DEFERRED
        )
        self._record("approach_succeeded" if succeeded else "approach_failed")
        return succeeded

    def pre_clean_verify(
        self,
        *,
        target_still_present: bool,
        identity_stable: bool,
        class_confidence_healthy: bool,
        action_verifier_accepts: bool,
        safety: ProductSafety,
    ) -> bool:
        if self.state != ProductCleanState.PRE_CLEAN_VERIFY:
            raise ValueError("pre-clean verification is out of sequence")
        reason = self._safety_reason(safety)
        checks = (
            (target_still_present, "target_missing"),
            (identity_stable, "track_identity_unstable"),
            (class_confidence_healthy, "class_confidence_unhealthy"),
            (action_verifier_accepts, "action_verifier_rejected"),
        )
        reason = reason or next(
            (failure for passed, failure in checks if not passed), None
        )
        if reason is not None:
            self.state = ProductCleanState.DEFERRED
            self.brush_enabled = False
            self._record("pre_clean_rejected", reason=reason)
            return False
        self.state = ProductCleanState.CLEANING
        self.brush_enabled = True
        self._record("pre_clean_accepted")
        return True

    def acknowledge_cleaning(self, actuator_succeeded: bool) -> bool:
        if self.state != ProductCleanState.CLEANING:
            raise ValueError("cleaning acknowledgement is out of sequence")
        self.brush_enabled = False
        if not actuator_succeeded:
            self.state = ProductCleanState.DEFERRED
            self._record("cleaning_actuator_failed")
            return False
        # Actuator success is never self-certifying.
        self.state = ProductCleanState.POST_CLEAN_VERIFY
        self._record("post_clean_verification_started")
        return True

    def observe_discrete_post_clean(
        self, *, target_in_camera_fov: bool, detected: bool
    ) -> bool:
        if self.state != ProductCleanState.POST_CLEAN_VERIFY:
            raise ValueError("post-clean observation is out of sequence")
        if not target_in_camera_fov:
            self._record("post_clean_frame_ignored", reason="target_not_in_fov")
            return False
        self.absent_frames = 0 if detected else self.absent_frames + 1
        self._record(
            "post_clean_discrete_observed",
            detected=bool(detected),
            absent_frames=self.absent_frames,
        )
        if self.absent_frames < self.absent_frames_required:
            return False
        self.state = ProductCleanState.WAITING_RESUME
        self._record("camera_backed_cleaned")
        return True

    def observe_area_post_clean(self, *, remaining_ratio: float) -> str:
        if self.state != ProductCleanState.POST_CLEAN_VERIFY:
            raise ValueError("post-clean observation is out of sequence")
        if not 0.0 <= float(remaining_ratio):
            raise ValueError("remaining area ratio must be non-negative")
        if remaining_ratio <= 0.10:
            self.state = ProductCleanState.WAITING_RESUME
            self._record("area_cleaned", remaining_ratio=float(remaining_ratio))
            return "CLEANED"
        if self.area_retry_count < self.maximum_area_retry_count:
            self.area_retry_count += 1
            self.state = ProductCleanState.CLEANING
            self.brush_enabled = True
            self._record(
                "area_reclean_started",
                retry_count=self.area_retry_count,
                remaining_ratio=float(remaining_ratio),
            )
            return "RECLEAN"
        self.state = ProductCleanState.DEFERRED
        self._record("area_clean_manual_attention", remaining_ratio=float(remaining_ratio))
        return "DEFERRED"

    def acknowledge_coverage_resume(self, accepted: bool) -> bool:
        if self.state != ProductCleanState.WAITING_RESUME:
            raise ValueError("coverage resume acknowledgement is out of sequence")
        if self.brush_enabled:
            raise ValueError("coverage cannot resume while the brush is enabled")
        self.coverage_paused = not accepted
        self.state = (
            ProductCleanState.COMPLETED if accepted else ProductCleanState.FAILED
        )
        self._record("coverage_resumed" if accepted else "coverage_resume_failed")
        return accepted

    def acknowledge_abort_resume(self, accepted: bool) -> bool:
        """Acknowledge best-effort coverage recovery after a failed clean step."""
        if self.state not in {ProductCleanState.DEFERRED, ProductCleanState.FAILED}:
            raise ValueError("abort resume acknowledgement is out of sequence")
        if self.brush_enabled:
            raise ValueError("coverage cannot resume while the brush is enabled")
        self.coverage_paused = not accepted
        self._record(
            "coverage_resumed_after_abort"
            if accepted
            else "coverage_resume_after_abort_failed"
        )
        return accepted

    def snapshot(self) -> dict:
        return {
            "state": self.state.value,
            "target": asdict(self.target) if self.target is not None else None,
            "brush_enabled": self.brush_enabled,
            "coverage_paused": self.coverage_paused,
            "absent_frames": self.absent_frames,
            "area_retry_count": self.area_retry_count,
            "timeline": list(self.timeline),
        }
