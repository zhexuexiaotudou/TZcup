from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import math


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
    path_length_m: float = 0.0


@dataclass
class ObservationTask:
    candidate_id: str
    first_seen_s: float
    last_seen_s: float
    queued_at_s: float
    state: ObservationState = ObservationState.DISCOVERED
    preflight_started_s: float | None = None
    approach_started_s: float | None = None
    approach_deadline_s: float | None = None
    last_observation_s: float | None = None
    position_xy_m: tuple[float, float] | None = None
    source_candidate_ids: set[str] = field(default_factory=set)
    approach_count: int = 0
    extra_distance_m: float = 0.0
    extra_time_s: float = 0.0
    terminal_reason: str | None = None
    coverage_resume_required: bool = False
    coverage_resumed: bool | None = None
    history: list[dict] = field(default_factory=list)

    @property
    def discovered_at_s(self) -> float:
        """Stage5BR4 compatibility alias; new code must use first/last_seen_s."""
        return self.first_seen_s

    def to_record(self) -> dict:
        payload = asdict(self)
        payload["state"] = self.state.value
        payload["source_candidate_ids"] = sorted(self.source_candidate_ids)
        payload["position_xy_m"] = list(self.position_xy_m) if self.position_xy_m is not None else None
        return payload

    @classmethod
    def from_record(cls, payload: dict) -> "ObservationTask":
        """Migrate Stage5BR4 task records without changing their historical time."""
        first_seen = float(payload.get("first_seen_s", payload.get("discovered_at_s")))
        state = ObservationState(payload.get("state", ObservationState.DISCOVERED.value))
        position = payload.get("position_xy_m")
        task = cls(
            candidate_id=str(payload["candidate_id"]),
            first_seen_s=first_seen,
            last_seen_s=float(payload.get("last_seen_s", first_seen)),
            queued_at_s=float(payload.get("queued_at_s", first_seen)),
            state=state,
            preflight_started_s=payload.get("preflight_started_s"),
            approach_started_s=payload.get("approach_started_s"),
            approach_deadline_s=payload.get("approach_deadline_s"),
            last_observation_s=payload.get("last_observation_s"),
            position_xy_m=tuple(float(v) for v in position) if position is not None else None,
            source_candidate_ids=set(payload.get("source_candidate_ids", [payload["candidate_id"]])),
            approach_count=int(payload.get("approach_count", 0)),
            extra_distance_m=float(payload.get("extra_distance_m", 0.0)),
            extra_time_s=float(payload.get("extra_time_s", 0.0)),
            terminal_reason=payload.get("terminal_reason"),
            coverage_resume_required=bool(payload.get("coverage_resume_required", False)),
            coverage_resumed=payload.get("coverage_resumed"),
            history=list(payload.get("history", [])),
        )
        return task


class ActiveObservationCoordinator:
    def __init__(
        self,
        maximum_approaches: int = 2,
        stale_candidate_s: float | None = None,
        approach_timeout_s: float = 20.0,
        minimum_clearance_m: float = 0.15,
        maximum_covariance_trace: float = 0.03,
        *,
        sensor_stale_s: float | None = None,
        queue_timeout_s: float = 120.0,
        minimum_approach_speed_mps: float = 0.10,
        spatial_merge_radius_m: float = 0.20,
    ):
        if maximum_approaches < 1:
            raise ValueError("maximum_approaches must be positive")
        if minimum_approach_speed_mps <= 0.0:
            raise ValueError("minimum_approach_speed_mps must be positive")
        if sensor_stale_s is not None and stale_candidate_s is not None:
            raise ValueError("use sensor_stale_s or legacy stale_candidate_s, not both")
        self.maximum_approaches = maximum_approaches
        self.sensor_stale_s = float(sensor_stale_s if sensor_stale_s is not None else (stale_candidate_s if stale_candidate_s is not None else 2.0))
        self.queue_timeout_s = float(queue_timeout_s)
        self.approach_timeout_s = float(approach_timeout_s)
        self.minimum_approach_speed_mps = float(minimum_approach_speed_mps)
        self.minimum_clearance_m = float(minimum_clearance_m)
        self.maximum_covariance_trace = float(maximum_covariance_trace)
        self.spatial_merge_radius_m = float(spatial_merge_radius_m)
        self.tasks: dict[str, ObservationTask] = {}
        self.aliases: dict[str, str] = {}

    def _canonical_id(self, candidate_id: str) -> str:
        return self.aliases.get(candidate_id, candidate_id)

    def _spatial_match(self, position_xy_m: tuple[float, float] | None) -> ObservationTask | None:
        if position_xy_m is None:
            return None
        matches = []
        for task in self.tasks.values():
            if task.state in TERMINAL or task.position_xy_m is None:
                continue
            distance = math.dist(position_xy_m, task.position_xy_m)
            if distance <= self.spatial_merge_radius_m:
                matches.append((distance, task.first_seen_s, task))
        return min(matches, key=lambda item: (item[0], item[1]))[2] if matches else None

    def discover(self, candidate_id: str, now_s: float, position_xy_m: tuple[float, float] | None = None) -> ObservationTask:
        canonical = self._canonical_id(candidate_id)
        task = self.tasks.get(canonical)
        if task is None:
            task = self._spatial_match(position_xy_m)
            if task is not None:
                self.aliases[candidate_id] = task.candidate_id
                task.source_candidate_ids.add(candidate_id)
                self._transition(task, task.state, now_s, "spatial_candidate_merged")
            else:
                now = float(now_s)
                task = ObservationTask(
                    candidate_id=candidate_id,
                    first_seen_s=now,
                    last_seen_s=now,
                    queued_at_s=now,
                    position_xy_m=tuple(float(v) for v in position_xy_m) if position_xy_m is not None else None,
                    source_candidate_ids={candidate_id},
                )
                self.tasks[candidate_id] = task
                self.aliases[candidate_id] = candidate_id
                self._transition(task, ObservationState.OBSERVATION_QUEUED, now_s, "candidate_queued")
                return task
        if task.state not in TERMINAL:
            task.last_seen_s = float(now_s)
            if position_xy_m is not None:
                task.position_xy_m = tuple(float(v) for v in position_xy_m)
        return task

    @staticmethod
    def _transition(task: ObservationTask, state: ObservationState, now_s: float, reason: str) -> None:
        task.state = state
        if state in TERMINAL:
            task.terminal_reason = reason
        task.history.append({"time_s": float(now_s), "state": state.value, "reason": reason})

    def preflight(self, candidate_id: str, now_s: float, check: ObservationPreflight) -> ObservationTask:
        task = self.tasks[self._canonical_id(candidate_id)]
        if task.state in TERMINAL:
            return task
        task.preflight_started_s = float(now_s)
        self._transition(task, ObservationState.APPROACH_PREFLIGHT, now_s, "preflight_started")
        if now_s - task.last_seen_s > self.sensor_stale_s:
            self._transition(task, ObservationState.REJECTED, now_s, "sensor_observation_stale")
        elif now_s - task.queued_at_s > self.queue_timeout_s:
            self._transition(task, ObservationState.REJECTED, now_s, "queue_timeout")
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
            task.approach_started_s = float(now_s)
            task.approach_deadline_s = float(now_s) + self.approach_timeout_s + max(0.0, float(check.path_length_m)) / self.minimum_approach_speed_mps
            task.coverage_resume_required = True
            task.coverage_resumed = None
            self._transition(task, ObservationState.APPROACHING, now_s, "preflight_passed")
        return task

    def observation_result(
        self,
        candidate_id: str,
        now_s: float,
        *,
        ready: bool,
        confirmed: bool | None,
        distance_m: float,
        elapsed_s: float,
    ) -> ObservationTask:
        task = self.tasks[self._canonical_id(candidate_id)]
        if task.state != ObservationState.APPROACHING:
            raise ValueError(f"observation result requires APPROACHING, got {task.state.value}")
        task.last_observation_s = float(now_s)
        task.extra_distance_m += max(0.0, float(distance_m))
        task.extra_time_s += max(0.0, float(elapsed_s))
        if task.approach_deadline_s is None or now_s > task.approach_deadline_s:
            self._transition(task, ObservationState.REJECTED, now_s, "approach_timeout")
        elif not ready:
            if task.approach_count >= self.maximum_approaches:
                self._transition(task, ObservationState.REJECTED, now_s, "recognition_not_ready_maximum_approaches")
            else:
                task.queued_at_s = float(now_s)
                self._transition(task, ObservationState.OBSERVATION_QUEUED, now_s, "recognition_not_ready")
        else:
            self._transition(task, ObservationState.RECOGNITION_READY, now_s, "recognition_ready")
            self._transition(task, ObservationState.CONFIRMED if confirmed else ObservationState.REJECTED, now_s, "recognized" if confirmed else "false_candidate")
        return task

    def mark_coverage_resumed(self, candidate_id: str, now_s: float, success: bool) -> ObservationTask:
        task = self.tasks[self._canonical_id(candidate_id)]
        if not task.coverage_resume_required:
            raise ValueError("coverage resume is only valid after an approach")
        task.coverage_resumed = bool(success)
        task.coverage_resume_required = not bool(success)
        task.history.append({"time_s": float(now_s), "state": task.state.value, "reason": "coverage_resumed" if success else "coverage_resume_failed"})
        return task

    def restore_task(self, payload: dict) -> ObservationTask:
        task = ObservationTask.from_record(payload)
        if task.candidate_id in self.tasks:
            raise ValueError(f"duplicate restored candidate {task.candidate_id}")
        self.tasks[task.candidate_id] = task
        for source_id in task.source_candidate_ids | {task.candidate_id}:
            self.aliases[source_id] = task.candidate_id
        return task
