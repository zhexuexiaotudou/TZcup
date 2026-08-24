"""Fail-closed, URDF-independent cube pick/place task orchestration."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable

from .cube_geometry import TopGraspCandidate


CUBE_EDGE_M = 0.030
MAX_TARGETS_PER_EPISODE = 20
MAX_GRASP_ATTEMPTS = 2
BIN_INTERNAL_SIZE_M = (0.20, 0.20, 0.10)


class TargetTaskState(str, Enum):
    DISCOVERED = "DISCOVERED"
    NAVIGATING = "NAVIGATING"
    PARKED = "PARKED"
    SAFETY_PAUSED = "SAFETY_PAUSED"
    PLANNING = "PLANNING"
    GRASPING = "GRASPING"
    VERIFYING = "VERIFYING"
    TRANSPORTING = "TRANSPORTING"
    PLACING = "PLACING"
    CLEARED = "CLEARED"
    DEFERRED = "DEFERRED"


@dataclass(frozen=True)
class VerificationEvidence:
    gripper_width_ok: bool = False
    gripper_effort_ok: bool = False
    target_follows_tool: bool = False
    source_location_absent: bool = False

    @property
    def independent_categories(self) -> int:
        gripper_contact = self.gripper_width_ok or self.gripper_effort_ok
        return sum((gripper_contact, self.target_follows_tool, self.source_location_absent))

    @property
    def accepted(self) -> bool:
        return self.independent_categories >= 2


@dataclass
class TargetTaskRecord:
    target_id: str
    state: TargetTaskState = TargetTaskState.DISCOVERED
    attempts: int = 0
    holding_target: bool = False
    placed_in_bin: bool = False
    reason: str | None = None
    resume_state: TargetTaskState = TargetTaskState.DISCOVERED
    history: list[str] = field(default_factory=lambda: [TargetTaskState.DISCOVERED.value])

    def transition(self, state: TargetTaskState, reason: str | None = None) -> None:
        self.state = state
        self.reason = reason
        self.history.append(state.value)


@dataclass(frozen=True)
class TaskOutcome:
    target_id: str
    state: TargetTaskState
    success: bool
    paused: bool
    attempts: int
    reason: str | None
    placed_in_bin: bool
    history: tuple[str, ...]


class CubeCollectionBin:
    """Count-limited bin; dimensions and limit are task rules, not packing proof."""

    def __init__(
        self,
        internal_size_m: tuple[float, float, float] = BIN_INTERNAL_SIZE_M,
        target_limit: int = MAX_TARGETS_PER_EPISODE,
    ) -> None:
        if any(value <= 0.0 for value in internal_size_m):
            raise ValueError("bin dimensions must be positive")
        if not 1 <= target_limit <= MAX_TARGETS_PER_EPISODE:
            raise ValueError("target_limit must be between 1 and 20")
        self.internal_size_m = internal_size_m
        self.target_limit = target_limit
        self._target_ids: set[str] = set()

    @property
    def count(self) -> int:
        return len(self._target_ids)

    @property
    def full(self) -> bool:
        return self.count >= self.target_limit

    def contains(self, target_id: str) -> bool:
        """Return whether a target has verified storage in this task bin."""

        return target_id in self._target_ids

    def store(self, target_id: str) -> bool:
        if not target_id or target_id in self._target_ids or self.full:
            return False
        self._target_ids.add(target_id)
        return True


class MockSafetyBackend:
    def __init__(self, safe: bool = True) -> None:
        self.safe = safe

    def is_safe(self) -> bool:
        return self.safe


class MockNavigationBackend:
    def __init__(self, outcomes: Iterable[bool] = (True,)) -> None:
        self.outcomes = list(outcomes)
        self.calls = 0

    def navigate_to_grasp(self, candidate: TopGraspCandidate) -> bool:
        del candidate
        self.calls += 1
        return self.outcomes.pop(0) if self.outcomes else True


class MockPlanningBackend:
    def __init__(self, outcomes: Iterable[bool] = (True,)) -> None:
        self.outcomes = list(outcomes)
        self.calls = 0

    def plan_pick(self, candidate: TopGraspCandidate) -> bool:
        del candidate
        self.calls += 1
        return self.outcomes.pop(0) if self.outcomes else True


class MockGripperBackend:
    def __init__(
        self,
        grasp_evidence: Iterable[VerificationEvidence] | None = None,
        place_outcomes: Iterable[bool] | None = None,
    ) -> None:
        self.repeat_successful_grasp = grasp_evidence is None
        self.grasp_evidence = list(grasp_evidence or ())
        self.place_outcomes = list(place_outcomes or ())
        self.grasp_calls = 0
        self.place_calls = 0
        self.recovery_calls = 0

    def grasp_and_lift(self, candidate: TopGraspCandidate) -> VerificationEvidence:
        del candidate
        self.grasp_calls += 1
        if self.grasp_evidence:
            return self.grasp_evidence.pop(0)
        if self.repeat_successful_grasp:
            return VerificationEvidence(gripper_width_ok=True, target_follows_tool=True)
        return VerificationEvidence()

    def place_in_bin(self) -> bool:
        self.place_calls += 1
        return self.place_outcomes.pop(0) if self.place_outcomes else True

    def recover_to_observation(self) -> None:
        self.recovery_calls += 1


class CubeTaskController:
    """Coordinate mockable navigation, planning and gripper backends.

    This controller proves task semantics only.  It neither computes IK nor
    claims collision safety for the eventual physical robot.
    """

    def __init__(
        self,
        navigation: MockNavigationBackend | None = None,
        planning: MockPlanningBackend | None = None,
        gripper: MockGripperBackend | None = None,
        safety: MockSafetyBackend | None = None,
        collection_bin: CubeCollectionBin | None = None,
    ) -> None:
        self.navigation = navigation or MockNavigationBackend()
        self.planning = planning or MockPlanningBackend()
        self.gripper = gripper or MockGripperBackend()
        self.safety = safety or MockSafetyBackend()
        self.collection_bin = collection_bin or CubeCollectionBin()
        self.records: dict[str, TargetTaskRecord] = {}

    def _outcome(self, record: TargetTaskRecord) -> TaskOutcome:
        return TaskOutcome(
            target_id=record.target_id,
            state=record.state,
            success=record.state is TargetTaskState.CLEARED,
            paused=record.state is TargetTaskState.SAFETY_PAUSED,
            attempts=record.attempts,
            reason=record.reason,
            placed_in_bin=record.placed_in_bin,
            history=tuple(record.history),
        )

    def _pause(
        self, record: TargetTaskRecord, resume_state: TargetTaskState
    ) -> TaskOutcome:
        record.resume_state = resume_state
        record.transition(TargetTaskState.SAFETY_PAUSED, "human_or_obstacle_in_arm_zone")
        return self._outcome(record)

    def _finish_placement(self, record: TargetTaskRecord) -> TaskOutcome:
        if not self.safety.is_safe():
            return self._pause(record, TargetTaskState.PLACING)
        record.transition(TargetTaskState.PLACING)
        if not self.gripper.place_in_bin():
            record.holding_target = False
            record.transition(TargetTaskState.DEFERRED, "bin_placement_not_verified")
            return self._outcome(record)
        if not self.collection_bin.store(record.target_id):
            record.holding_target = False
            record.transition(TargetTaskState.DEFERRED, "bin_capacity_or_duplicate_rejected")
            return self._outcome(record)
        record.holding_target = False
        record.placed_in_bin = True
        record.transition(TargetTaskState.CLEARED)
        return self._outcome(record)

    def execute(
        self, target_id: str, candidates: Iterable[TopGraspCandidate]
    ) -> TaskOutcome:
        if not target_id:
            raise ValueError("target_id must be non-empty")
        candidate_list = [row for row in candidates if row.target_id == target_id]
        if not candidate_list:
            raise ValueError("at least one matching grasp candidate is required")
        record = self.records.setdefault(target_id, TargetTaskRecord(target_id))
        if record.state in (TargetTaskState.CLEARED, TargetTaskState.DEFERRED):
            return self._outcome(record)
        if record.holding_target:
            return self._finish_placement(record)
        if not self.safety.is_safe():
            return self._pause(record, TargetTaskState.NAVIGATING)
        if self.collection_bin.full:
            record.transition(TargetTaskState.DEFERRED, "bin_target_limit_reached")
            return self._outcome(record)

        record.transition(TargetTaskState.NAVIGATING)
        if not self.navigation.navigate_to_grasp(candidate_list[0]):
            record.transition(TargetTaskState.DEFERRED, "navigation_or_staging_failed")
            return self._outcome(record)
        record.transition(TargetTaskState.PARKED)

        while record.attempts < MAX_GRASP_ATTEMPTS:
            if not self.safety.is_safe():
                return self._pause(record, TargetTaskState.PLANNING)
            candidate = candidate_list[record.attempts % len(candidate_list)]
            record.attempts += 1
            record.transition(TargetTaskState.PLANNING)
            if not self.planning.plan_pick(candidate):
                self.gripper.recover_to_observation()
                continue
            record.transition(TargetTaskState.GRASPING)
            evidence = self.gripper.grasp_and_lift(candidate)
            record.transition(TargetTaskState.VERIFYING)
            if not evidence.accepted:
                self.gripper.recover_to_observation()
                continue
            record.holding_target = True
            record.transition(TargetTaskState.TRANSPORTING)
            return self._finish_placement(record)

        record.transition(TargetTaskState.DEFERRED, "grasp_attempt_limit_reached")
        return self._outcome(record)
