import pytest

from sanitation_spot_cleaning.product_orchestrator import (
    ProductCleanState,
    ProductSafety,
    ProductSpotCleanOrchestrator,
    ProductTarget,
)


def target(**overrides) -> ProductTarget:
    values = {
        "uuid": "target-1",
        "class_id": "plastic_bottle",
        "target_type": "DISCRETE",
        "track_state": "CONFIRMED",
        "confidence": 0.99,
        "observation_count": 4,
        "covariance_trace": 0.01,
        "source_backend": "frozen_classifier",
        "in_keepout": False,
    }
    values.update(overrides)
    return ProductTarget(**values)


def safety(**overrides) -> ProductSafety:
    values = {
        "emergency_stopped": False,
        "collision_clear": True,
        "localization_healthy": True,
        "perception_healthy": True,
        "keepout_clear": True,
        "path_available": True,
        "observation_age_s": 0.1,
    }
    values.update(overrides)
    return ProductSafety(**values)


def test_discrete_clean_requires_full_sequence_and_camera_absence() -> None:
    core = ProductSpotCleanOrchestrator(absent_frames_required=3)
    assert core.submit(target(), safety())
    assert core.acknowledge_coverage_pause(True)
    assert core.acknowledge_approach(True)
    assert core.pre_clean_verify(
        target_still_present=True,
        identity_stable=True,
        class_confidence_healthy=True,
        action_verifier_accepts=True,
        safety=safety(),
    )
    assert core.brush_enabled
    assert core.acknowledge_cleaning(True)
    assert not core.brush_enabled
    assert not core.observe_discrete_post_clean(
        target_in_camera_fov=False, detected=False
    )
    assert not core.observe_discrete_post_clean(
        target_in_camera_fov=True, detected=False
    )
    assert not core.observe_discrete_post_clean(
        target_in_camera_fov=True, detected=False
    )
    assert core.observe_discrete_post_clean(
        target_in_camera_fov=True, detected=False
    )
    assert core.state == ProductCleanState.WAITING_RESUME
    assert core.acknowledge_coverage_resume(True)
    assert core.state == ProductCleanState.COMPLETED


@pytest.mark.parametrize(
    "target_override,safety_override,reason",
    [
        ({"track_state": "TRACKED"}, {}, "target_not_confirmed"),
        ({"in_keepout": True}, {}, "target_in_keepout"),
        ({"source_backend": "ground_truth"}, {}, "GT control violation"),
        ({}, {"emergency_stopped": True}, "emergency_stop_active"),
        ({}, {"path_available": False}, "nav2_path_unavailable"),
        ({}, {"observation_age_s": 2.0}, "observation_stale"),
    ],
)
def test_submit_fails_closed(target_override, safety_override, reason) -> None:
    core = ProductSpotCleanOrchestrator()
    if "GT control" in reason:
        with pytest.raises(ValueError, match=reason):
            core.submit(target(**target_override), safety(**safety_override))
        return
    assert not core.submit(target(**target_override), safety(**safety_override))
    assert core.state == ProductCleanState.DEFERRED
    assert core.timeline[-1]["reason"] == reason


def test_preclean_rechecks_safety_before_enabling_brush() -> None:
    core = ProductSpotCleanOrchestrator()
    assert core.submit(target(), safety())
    core.acknowledge_coverage_pause(True)
    core.acknowledge_approach(True)
    assert not core.pre_clean_verify(
        target_still_present=True,
        identity_stable=True,
        class_confidence_healthy=True,
        action_verifier_accepts=True,
        safety=safety(localization_healthy=False),
    )
    assert not core.brush_enabled
    assert core.state == ProductCleanState.DEFERRED


def test_area_clean_retries_once_then_requires_camera_backed_result() -> None:
    core = ProductSpotCleanOrchestrator(maximum_area_retry_count=1)
    assert core.submit(target(target_type="AREA", class_id="leaf_pile"), safety())
    core.acknowledge_coverage_pause(True)
    core.acknowledge_approach(True)
    core.pre_clean_verify(
        target_still_present=True,
        identity_stable=True,
        class_confidence_healthy=True,
        action_verifier_accepts=True,
        safety=safety(),
    )
    core.acknowledge_cleaning(True)
    assert core.observe_area_post_clean(remaining_ratio=0.2) == "RECLEAN"
    core.acknowledge_cleaning(True)
    assert core.observe_area_post_clean(remaining_ratio=0.09) == "CLEANED"
    assert core.state == ProductCleanState.WAITING_RESUME


def test_abort_recovery_cannot_claim_resume_with_brush_enabled() -> None:
    core = ProductSpotCleanOrchestrator()
    core.state = ProductCleanState.DEFERRED
    core.coverage_paused = True
    core.brush_enabled = True
    with pytest.raises(ValueError, match="brush"):
        core.acknowledge_abort_resume(True)
    core.brush_enabled = False
    assert core.acknowledge_abort_resume(True)
    assert core.coverage_paused is False


def test_motion_safety_ignores_age_but_never_estop_or_collision() -> None:
    core = ProductSpotCleanOrchestrator()
    assert core.motion_safety_reason(safety(observation_age_s=20.0)) is None
    assert core.motion_safety_reason(safety(emergency_stopped=True)) == "emergency_stop_active"
    assert core.motion_safety_reason(safety(collision_clear=False)) == "collision_monitor_not_clear"
