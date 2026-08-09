"""Event-triggered visual verification; a cleaning action is never self-certifying."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum


class VerificationOutcome(str, Enum):
    CONTINUE_OBSERVING = "CONTINUE_OBSERVING"
    CLEANED = "CLEANED"
    RECLEAN = "RECLEAN"
    MANUAL_ATTENTION = "MANUAL_ATTENTION"


@dataclass(frozen=True)
class PostCleanConfig:
    absent_frames_required: int = 3
    disappearance_confidence: float = 0.20
    maximum_clean_attempts: int = 2
    minimum_area_reduction: float = 0.90


@dataclass
class VerificationSession:
    target_uuid: str
    target_type: str
    clean_attempt: int
    area_before_m2: float | None
    absent_frames: int = 0
    observed_frames: int = 0
    residual_detected: bool = False
    area_after_m2: float | None = None
    outcome: VerificationOutcome = VerificationOutcome.CONTINUE_OBSERVING

    def to_record(self) -> dict:
        payload = asdict(self)
        payload["outcome"] = self.outcome.value
        return payload


class PostCleanVerifier:
    def __init__(self, config: PostCleanConfig = PostCleanConfig()):
        if config.absent_frames_required < 1 or config.maximum_clean_attempts < 1:
            raise ValueError("post-clean count thresholds must be positive")
        if not 0.0 <= config.disappearance_confidence <= 1.0:
            raise ValueError("disappearance confidence must be in [0, 1]")
        if not 0.0 <= config.minimum_area_reduction <= 1.0:
            raise ValueError("minimum area reduction must be in [0, 1]")
        self.config = config
        self.sessions: dict[str, VerificationSession] = {}

    def begin(
        self,
        target_uuid: str,
        *,
        target_type: str,
        clean_attempt: int,
        area_before_m2: float | None = None,
    ) -> VerificationSession:
        if target_type not in {"DISCRETE", "AREA"}:
            raise ValueError("target_type must be DISCRETE or AREA")
        if not 1 <= clean_attempt <= self.config.maximum_clean_attempts:
            raise ValueError("clean attempt is outside the bounded retry policy")
        if target_type == "AREA" and (area_before_m2 is None or area_before_m2 <= 0.0):
            raise ValueError("area verification requires positive pre-clean area")
        session = VerificationSession(target_uuid, target_type, clean_attempt, area_before_m2)
        self.sessions[target_uuid] = session
        return session

    def observe_discrete(self, target_uuid: str, *, detected: bool, confidence: float) -> VerificationOutcome:
        session = self.sessions[target_uuid]
        if session.target_type != "DISCRETE":
            raise ValueError("discrete observation used for area target")
        session.observed_frames += 1
        absent = (not detected) or confidence < self.config.disappearance_confidence
        if absent:
            session.absent_frames += 1
        else:
            session.absent_frames = 0
            session.residual_detected = True
        if session.absent_frames >= self.config.absent_frames_required:
            session.outcome = VerificationOutcome.CLEANED
        return session.outcome

    def observe_area(self, target_uuid: str, *, area_after_m2: float) -> VerificationOutcome:
        session = self.sessions[target_uuid]
        if session.target_type != "AREA" or session.area_before_m2 is None:
            raise ValueError("area observation used for discrete target")
        if area_after_m2 < 0.0:
            raise ValueError("post-clean area must be non-negative")
        session.observed_frames += 1
        session.area_after_m2 = area_after_m2
        reduction = 1.0 - area_after_m2 / session.area_before_m2
        if reduction >= self.config.minimum_area_reduction:
            session.outcome = VerificationOutcome.CLEANED
        else:
            session.residual_detected = True
        return session.outcome

    def finalize(self, target_uuid: str) -> VerificationOutcome:
        session = self.sessions[target_uuid]
        if session.outcome == VerificationOutcome.CLEANED:
            return session.outcome
        session.outcome = (
            VerificationOutcome.RECLEAN
            if session.clean_attempt < self.config.maximum_clean_attempts
            else VerificationOutcome.MANUAL_ATTENTION
        )
        return session.outcome
