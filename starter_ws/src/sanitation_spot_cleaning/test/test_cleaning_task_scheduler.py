from sanitation_spot_cleaning.cleaning_task_scheduler import (
    CleaningTaskScheduler,
    CoverageContext,
    SafetyContext,
    SchedulerAction,
    TargetSchedulingInput,
)
from sanitation_spot_cleaning.perception_coverage_bridge import PerceptionCoverageBridge


def target(**changes):
    values = {
        "target_uuid": "target-1",
        "track_state": "CONFIRMED",
        "confidence": 0.92,
        "observation_count": 4,
        "distance_from_current_route_m": 0.3,
        "detour_length_m": 0.5,
        "target_priority": 1.0,
        "cleaning_cost": 0.2,
        "return_to_coverage_cost": 0.2,
        "source_models": ("fcos-r50-online-x1",),
    }
    values.update(changes)
    return TargetSchedulingInput(**values)


def coverage(**changes):
    values = {"coverage_state": "RUNNING", "at_component_boundary": True, "current_swath_state": "SWATH_COMPLETE"}
    values.update(changes)
    return CoverageContext(**values)


def safety(**changes):
    values = {
        "nav2_path_available": True,
        "keepout_clear": True,
        "dynamic_obstacle_clear": True,
        "localization_healthy": True,
        "perception_healthy": True,
        "footprint_clearance_m": 0.3,
        "covariance_trace": 0.01,
    }
    values.update(changes)
    return SafetyContext(**values)


def test_clean_now_requires_close_confirmed_target_and_safe_boundary():
    scheduler = CleaningTaskScheduler()
    decision = scheduler.decide(target(), coverage(), safety())
    assert decision.action == SchedulerAction.CLEAN_NOW
    assert decision.coverage_interrupt_allowed is True


def test_coverage_continues_for_far_or_uncertain_targets():
    scheduler = CleaningTaskScheduler()
    assert scheduler.decide(target(detour_length_m=3.0), coverage(), safety()).action == SchedulerAction.DEFER
    assert scheduler.decide(target(confidence=0.6), coverage(), safety()).action == SchedulerAction.OBSERVE_AGAIN
    assert scheduler.decide(target(), coverage(at_component_boundary=False, current_swath_state="CLEANING"), safety()).action == SchedulerAction.DEFER


def test_safety_failure_defers_and_gt_is_rejected():
    scheduler = CleaningTaskScheduler()
    decision = scheduler.decide(target(), coverage(), safety(keepout_clear=False))
    assert decision.action == SchedulerAction.DEFER
    assert decision.reason == "keepout_or_obstacle"
    try:
        scheduler.decide(target(source_models=("ground_truth",)), coverage(), safety())
    except ValueError as exc:
        assert "GT control violation" in str(exc)
    else:
        raise AssertionError("ground truth reached the scheduler")


def test_coverage_bridge_resumes_only_with_brush_off_and_safety_clear():
    bridge = PerceptionCoverageBridge()
    assert bridge.request_safe_pause("target-1", allowed=True, stamp_ns=1)
    assert not bridge.resume("target-1", safety_clear=True, brush_enabled=True, stamp_ns=2)
    assert bridge.resume("target-1", safety_clear=True, brush_enabled=False, stamp_ns=3)
    assert bridge.state.coverage_state == "RUNNING"
