from dataclasses import replace

import pytest

from sanitation_manipulation.cube_geometry import CubeCandidate, generate_top_grasps
from sanitation_manipulation.cube_task import (
    BIN_INTERNAL_SIZE_M,
    CubeCollectionBin,
    CubeTaskController,
    MAX_GRASP_ATTEMPTS,
    MAX_TARGETS_PER_EPISODE,
    MockGripperBackend,
    MockNavigationBackend,
    MockPlanningBackend,
    MockSafetyBackend,
    TargetTaskState,
    VerificationEvidence,
)


def _grasps(target_id="cube-1"):
    cube = CubeCandidate((0.35, 0.0, 0.015), (0.03, 0.03, 0.03), 0.0, 40, 0.0)
    return generate_top_grasps(target_id, cube)


def test_mock_closed_loop_clears_only_after_verified_pick_and_bin_place():
    controller = CubeTaskController()
    outcome = controller.execute("cube-1", _grasps())
    assert outcome.success
    assert outcome.state is TargetTaskState.CLEARED
    assert outcome.placed_in_bin
    assert controller.collection_bin.count == 1
    assert "VERIFYING" in outcome.history
    assert "PLACING" in outcome.history


def test_two_independent_evidence_categories_are_required():
    duplicate_contact_signals = VerificationEvidence(
        gripper_width_ok=True, gripper_effort_ok=True
    )
    assert duplicate_contact_signals.independent_categories == 1
    assert not duplicate_contact_signals.accepted
    accepted = VerificationEvidence(
        gripper_effort_ok=True, source_location_absent=True
    )
    assert accepted.independent_categories == 2
    assert accepted.accepted


def test_failed_verification_recovers_and_uses_second_attempt():
    gripper = MockGripperBackend(
        grasp_evidence=[
            VerificationEvidence(gripper_width_ok=True),
            VerificationEvidence(target_follows_tool=True, source_location_absent=True),
        ]
    )
    controller = CubeTaskController(gripper=gripper)
    outcome = controller.execute("cube-1", _grasps())
    assert outcome.success
    assert outcome.attempts == MAX_GRASP_ATTEMPTS
    assert gripper.recovery_calls == 1


def test_two_failed_attempts_defer_without_counting_clearance():
    gripper = MockGripperBackend(
        grasp_evidence=[VerificationEvidence(), VerificationEvidence()]
    )
    controller = CubeTaskController(gripper=gripper)
    outcome = controller.execute("cube-1", _grasps())
    assert outcome.state is TargetTaskState.DEFERRED
    assert outcome.reason == "grasp_attempt_limit_reached"
    assert not outcome.success
    assert controller.collection_bin.count == 0
    assert gripper.grasp_calls == 2


def test_safety_pause_consumes_no_attempt_and_can_resume():
    safety = MockSafetyBackend(False)
    controller = CubeTaskController(safety=safety)
    paused = controller.execute("cube-1", _grasps())
    assert paused.paused
    assert paused.attempts == 0
    assert controller.navigation.calls == 0
    safety.safe = True
    resumed = controller.execute("cube-1", _grasps())
    assert resumed.success
    assert resumed.attempts == 1


def test_navigation_planning_and_placement_fail_closed():
    nav_failure = CubeTaskController(navigation=MockNavigationBackend([False]))
    assert nav_failure.execute("cube-1", _grasps()).reason == "navigation_or_staging_failed"

    plan_failure = CubeTaskController(planning=MockPlanningBackend([False, False]))
    plan_result = plan_failure.execute("cube-1", _grasps())
    assert plan_result.reason == "grasp_attempt_limit_reached"

    place_failure = CubeTaskController(gripper=MockGripperBackend(place_outcomes=[False]))
    place_result = place_failure.execute("cube-1", _grasps())
    assert place_result.reason == "bin_placement_not_verified"
    assert not place_result.placed_in_bin


def test_bin_and_episode_rules_are_frozen_at_20_targets():
    collection_bin = CubeCollectionBin()
    assert collection_bin.internal_size_m == BIN_INTERNAL_SIZE_M == (0.20, 0.20, 0.10)
    assert collection_bin.target_limit == MAX_TARGETS_PER_EPISODE == 20
    controller = CubeTaskController(collection_bin=collection_bin)
    template = _grasps()
    for index in range(20):
        target_id = f"cube-{index}"
        candidates = tuple(replace(row, target_id=target_id) for row in template)
        assert controller.execute(target_id, candidates).success
    assert collection_bin.full
    overflow = tuple(replace(row, target_id="cube-overflow") for row in template)
    outcome = controller.execute("cube-overflow", overflow)
    assert outcome.reason == "bin_target_limit_reached"
    assert collection_bin.count == 20
    with pytest.raises(ValueError):
        CubeCollectionBin(target_limit=21)


def test_target_and_candidate_identity_must_match():
    with pytest.raises(ValueError):
        CubeTaskController().execute("cube-other", _grasps("cube-1"))
