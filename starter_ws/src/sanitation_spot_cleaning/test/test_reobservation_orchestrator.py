import pytest

from sanitation_spot_cleaning.reobservation_orchestrator import (
    ProductReobservationOrchestrator,
    ReobservationRequest,
    ReobservationSafety,
    ReobservationState,
)


def request(**overrides):
    values = {
        "request_id": "request-1",
        "track_uuid": "track-1",
        "target_uuid": "target-1",
        "stamp_ns": 100,
        "x_m": 2.0,
        "y_m": 0.0,
        "covariance_trace": 0.01,
        "class_id": "metal_can",
        "target_size_m": 0.08,
        "reobserve_count": 1,
        "source_backend": "product_action_verifier",
    }
    values.update(overrides)
    return ReobservationRequest(**values)


def safety(**overrides):
    values = {
        "emergency_stopped": False,
        "collision_clear": True,
        "localization_healthy": True,
        "keepout_clear": True,
        "path_available": True,
    }
    values.update(overrides)
    return ReobservationSafety(**values)


def test_full_reobservation_requires_fresh_post_navigation_verdict() -> None:
    core = ProductReobservationOrchestrator()
    assert core.submit(request())
    assert core.acknowledge_pose(True, safety())
    assert core.acknowledge_pause(True)
    assert core.acknowledge_navigation(True, safety())
    assert not core.observe_verdict(stamp_ns=100, verdict="ACCEPT")
    assert core.observe_verdict(stamp_ns=101, verdict="ACCEPT")
    assert core.acknowledge_resume(True)
    assert core.state == ReobservationState.COMPLETED


def test_safety_and_reobservation_budget_fail_closed() -> None:
    core = ProductReobservationOrchestrator()
    assert not core.submit(request(reobserve_count=3))
    core = ProductReobservationOrchestrator()
    assert core.submit(request())
    assert not core.acknowledge_pose(True, safety(keepout_clear=False))
    assert core.state == ReobservationState.DEFERRED


def test_gt_request_is_rejected() -> None:
    core = ProductReobservationOrchestrator()
    with pytest.raises(ValueError, match="GT control violation"):
        core.submit(request(source_backend="ground_truth"))
