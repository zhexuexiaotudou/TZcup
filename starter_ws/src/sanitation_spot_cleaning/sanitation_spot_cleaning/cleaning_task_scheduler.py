"""Fail-closed online cleaning scheduler that keeps Coverage as the primary task."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum


class SchedulerAction(str, Enum):
    CLEAN_NOW = "CLEAN_NOW"
    DEFER = "DEFER"
    OBSERVE_AGAIN = "OBSERVE_AGAIN"


@dataclass(frozen=True)
class SchedulerConfig:
    clean_now_route_distance_m: float = 0.75
    clean_now_max_detour_m: float = 1.5
    minimum_clean_confidence: float = 0.80
    minimum_observe_confidence: float = 0.35
    minimum_persistence_observations: int = 3
    minimum_footprint_clearance_m: float = 0.15
    maximum_covariance_trace: float = 0.03


@dataclass(frozen=True)
class TargetSchedulingInput:
    target_uuid: str
    track_state: str
    confidence: float
    observation_count: int
    distance_from_current_route_m: float
    detour_length_m: float
    target_priority: float
    cleaning_cost: float
    return_to_coverage_cost: float
    source_models: tuple[str, ...]


@dataclass(frozen=True)
class CoverageContext:
    coverage_state: str
    at_component_boundary: bool
    current_swath_state: str


@dataclass(frozen=True)
class SafetyContext:
    nav2_path_available: bool
    keepout_clear: bool
    dynamic_obstacle_clear: bool
    localization_healthy: bool
    perception_healthy: bool
    footprint_clearance_m: float
    covariance_trace: float


@dataclass(frozen=True)
class SchedulerDecision:
    target_uuid: str
    action: SchedulerAction
    reason: str
    score: float
    coverage_interrupt_allowed: bool

    def to_record(self) -> dict:
        payload = asdict(self)
        payload["action"] = self.action.value
        return payload


class CleaningTaskScheduler:
    def __init__(self, config: SchedulerConfig = SchedulerConfig()):
        self.config = config
        self.deferred: dict[str, TargetSchedulingInput] = {}
        self.decisions: list[SchedulerDecision] = []

    @staticmethod
    def _reject_gt(target: TargetSchedulingInput) -> None:
        forbidden = {"ground_truth", "gazebo_registry", "evaluation_registry"}
        if any(source.lower() in forbidden for source in target.source_models):
            raise ValueError("GT control violation: forbidden source reached scheduler")

    def _record(self, decision: SchedulerDecision) -> SchedulerDecision:
        self.decisions.append(decision)
        return decision

    def decide(
        self,
        target: TargetSchedulingInput,
        coverage: CoverageContext,
        safety: SafetyContext,
    ) -> SchedulerDecision:
        self._reject_gt(target)
        if target.track_state not in {"CONFIRMED", "DEFERRED"}:
            return self._record(SchedulerDecision(
                target.target_uuid,
                SchedulerAction.OBSERVE_AGAIN,
                "target_not_confirmed",
                0.0,
                False,
            ))
        hard_safety = {
            "nav2_path_unavailable": safety.nav2_path_available,
            "keepout_or_obstacle": safety.keepout_clear and safety.dynamic_obstacle_clear,
            "localization_degraded": safety.localization_healthy,
            "perception_degraded": safety.perception_healthy,
            "insufficient_clearance": safety.footprint_clearance_m >= self.config.minimum_footprint_clearance_m,
            "covariance_too_large": safety.covariance_trace <= self.config.maximum_covariance_trace,
        }
        for reason, passed in hard_safety.items():
            if not passed:
                self.deferred[target.target_uuid] = target
                return self._record(SchedulerDecision(
                    target.target_uuid,
                    SchedulerAction.DEFER,
                    reason,
                    float("-inf"),
                    False,
                ))
        if (
            target.confidence < self.config.minimum_clean_confidence
            or target.observation_count < self.config.minimum_persistence_observations
        ):
            action = (
                SchedulerAction.OBSERVE_AGAIN
                if target.confidence >= self.config.minimum_observe_confidence
                else SchedulerAction.DEFER
            )
            self.deferred[target.target_uuid] = target
            return self._record(SchedulerDecision(
                target.target_uuid,
                action,
                "insufficient_confidence_or_persistence",
                target.confidence,
                False,
            ))
        score = (
            3.0 * target.confidence
            + target.target_priority
            - target.detour_length_m
            - target.cleaning_cost
            - target.return_to_coverage_cost
        )
        route_close = (
            target.distance_from_current_route_m <= self.config.clean_now_route_distance_m
            and target.detour_length_m <= self.config.clean_now_max_detour_m
        )
        safe_interrupt_point = coverage.at_component_boundary or coverage.current_swath_state in {
            "PAUSED_SAFE",
            "SWATH_COMPLETE",
        }
        if route_close and safe_interrupt_point:
            self.deferred.pop(target.target_uuid, None)
            return self._record(SchedulerDecision(
                target.target_uuid,
                SchedulerAction.CLEAN_NOW,
                "close_high_confidence_safe_boundary",
                score,
                True,
            ))
        self.deferred[target.target_uuid] = target
        return self._record(SchedulerDecision(
            target.target_uuid,
            SchedulerAction.DEFER,
            "coverage_continues_until_safe_low_detour_opportunity",
            score,
            False,
        ))

    def optimized_deferred_order(self) -> list[str]:
        return [
            target.target_uuid
            for target in sorted(
                self.deferred.values(),
                key=lambda item: (
                    item.detour_length_m + item.return_to_coverage_cost,
                    -item.target_priority,
                    -item.confidence,
                    item.target_uuid,
                ),
            )
        ]
