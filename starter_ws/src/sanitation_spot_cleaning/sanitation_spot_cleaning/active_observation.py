from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class ObservationState(str, Enum):
    DISCOVERED = "DISCOVERED"
    OBSERVATION_QUEUED = "OBSERVATION_QUEUED"
    APPROACH_PREFLIGHT = "APPROACH_PREFLIGHT"
    APPROACHING = "APPROACHING"
    RECOGNITION_READY = "RECOGNITION_READY"
    CONFIRMED = "CONFIRMED"
    REJECTED = "REJECTED"
    UNREACHABLE = "UNREACHABLE"


TERMINAL = {ObservationState.CONFIRMED, ObservationState.REJECTED, ObservationState.UNREACHABLE}


@dataclass(frozen=True)
class ObservationPreflight:
    at_component_boundary: bool
    path_available: bool
    keepout_clear: bool
    footprint_clearance_m: float
    visibility_expected: bool
    covariance_trace: float


@dataclass
class ObservationTask:
    candidate_id: str
    discovered_at_s: float
    state: ObservationState = ObservationState.DISCOVERED
    approach_count: int = 0
    extra_distance_m: float = 0.0
    extra_time_s: float = 0.0
    history: list[dict] = field(default_factory=list)


class ActiveObservationCoordinator:
    def __init__(self, maximum_approaches: int = 2, stale_candidate_s: float = 2.0, approach_timeout_s: float = 20.0, minimum_clearance_m: float = 0.15, maximum_covariance_trace: float = 0.03):
        if maximum_approaches < 1:
            raise ValueError("maximum_approaches must be positive")
        self.maximum_approaches = maximum_approaches
        self.stale_candidate_s = stale_candidate_s
        self.approach_timeout_s = approach_timeout_s
        self.minimum_clearance_m = minimum_clearance_m
        self.maximum_covariance_trace = maximum_covariance_trace
        self.tasks: dict[str, ObservationTask] = {}

    def discover(self, candidate_id: str, now_s: float) -> ObservationTask:
        # Candidate identity is stable: repeated discovery never creates a duplicate task.
        task = self.tasks.get(candidate_id)
        if task is None:
            task = ObservationTask(candidate_id=candidate_id, discovered_at_s=float(now_s))
            self.tasks[candidate_id] = task
            self._transition(task, ObservationState.OBSERVATION_QUEUED, now_s, "candidate_queued")
        return task
    @staticmethod
    def _transition(task: ObservationTask, state: ObservationState, now_s: float, reason: str) -> None:
        task.state = state
        task.history.append({"time_s": float(now_s), "state": state.value, "reason": reason})

    def preflight(self, candidate_id: str, now_s: float, check: ObservationPreflight) -> ObservationTask:
        task = self.tasks[candidate_id]
        if task.state in TERMINAL:
            return task
        self._transition(task, ObservationState.APPROACH_PREFLIGHT, now_s, "preflight_started")
        if now_s - task.discovered_at_s > self.stale_candidate_s:
            self._transition(task, ObservationState.REJECTED, now_s, "stale_candidate")
        elif not check.at_component_boundary:
            self._transition(task, ObservationState.OBSERVATION_QUEUED, now_s, "wait_for_component_boundary")
        elif not check.path_available:
            self._transition(task, ObservationState.UNREACHABLE, now_s, "compute_path_failed")
        elif not check.keepout_clear or check.footprint_clearance_m < self.minimum_clearance_m:
            self._transition(task, ObservationState.UNREACHABLE, now_s, "keepout_or_footprint_blocked")
        elif not check.visibility_expected:
            self._transition(task, ObservationState.UNREACHABLE, now_s, "observation_pose_not_visible")
        elif check.covariance_trace > self.maximum_covariance_trace:
            self._transition(task, ObservationState.REJECTED, now_s, "localization_uncertain")
        elif task.approach_count >= self.maximum_approaches:
            self._transition(task, ObservationState.REJECTED, now_s, "maximum_approaches_exhausted")
        else:
            task.approach_count += 1
            self._transition(task, ObservationState.APPROACHING, now_s, "preflight_passed")
        return task

    def observation_result(self, candidate_id: str, now_s: float, *, ready: bool, confirmed: bool | None, distance_m: float, elapsed_s: float) -> ObservationTask:
        task = self.tasks[candidate_id]
        if task.state != ObservationState.APPROACHING:
            raise ValueError(f"observation result requires APPROACHING, got {task.state.value}")
        task.extra_distance_m += max(0.0, float(distance_m))
        task.extra_time_s += max(0.0, float(elapsed_s))
        if elapsed_s > self.approach_timeout_s:
            self._transition(task, ObservationState.REJECTED, now_s, "approach_timeout")
        elif not ready:
            next_state = ObservationState.REJECTED if task.approach_count >= self.maximum_approaches else ObservationState.OBSERVATION_QUEUED
            self._transition(task, next_state, now_s, "recognition_not_ready")
        else:
            self._transition(task, ObservationState.RECOGNITION_READY, now_s, "recognition_ready")
            self._transition(task, ObservationState.CONFIRMED if confirmed else ObservationState.REJECTED, now_s, "recognized" if confirmed else "false_candidate")
        return task
