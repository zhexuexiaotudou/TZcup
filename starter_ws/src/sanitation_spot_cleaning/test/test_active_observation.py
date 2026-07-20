import pytest

from sanitation_spot_cleaning.active_observation import (
    ActiveObservationCoordinator,
    ObservationPreflight,
    ObservationState,
)


GOOD = ObservationPreflight(True, True, True, 0.30, True, 0.002, path_length_m=1.2)


def test_continuous_refresh_uses_last_seen_not_first_seen():
    coordinator = ActiveObservationCoordinator(sensor_stale_s=1.0, queue_timeout_s=10.0)
    first = coordinator.discover("candidate-1", 0.0)
    assert coordinator.discover("candidate-1", 0.8) is first
    coordinator.discover("candidate-1", 1.6)
    task = coordinator.preflight("candidate-1", 2.0, GOOD)
    assert task.state == ObservationState.APPROACHING
    assert task.first_seen_s == 0.0
    assert task.last_seen_s == 1.6


def test_observations_stop_then_sensor_stale_is_fail_closed():
    coordinator = ActiveObservationCoordinator(sensor_stale_s=1.0)
    coordinator.discover("stale", 0.0)
    stale = coordinator.preflight("stale", 1.1, GOOD)
    assert stale.state == ObservationState.REJECTED
    assert stale.terminal_reason == "sensor_observation_stale"


def test_long_component_wait_is_not_rejected_from_first_seen_time():
    coordinator = ActiveObservationCoordinator(sensor_stale_s=2.0, queue_timeout_s=30.0)
    wait = ObservationPreflight(False, False, True, 0.3, True, 0.0)
    coordinator.discover("candidate", 0.0)
    for now in (1.5, 3.0, 4.5, 6.0):
        coordinator.discover("candidate", now)
        assert coordinator.preflight("candidate", now, wait).state == ObservationState.OBSERVATION_QUEUED
    task = coordinator.preflight("candidate", 6.1, GOOD)
    assert task.state == ObservationState.APPROACHING


def test_spatial_candidate_merge_survives_model_id_change():
    coordinator = ActiveObservationCoordinator(spatial_merge_radius_m=0.25)
    first = coordinator.discover("model-id-a", 0.0, (1.0, 2.0))
    merged = coordinator.discover("model-id-b", 0.2, (1.1, 2.1))
    assert merged is first
    assert set(merged.source_candidate_ids) == {"model-id-a", "model-id-b"}
    assert coordinator.preflight("model-id-b", 0.3, GOOD).candidate_id == "model-id-a"


def test_two_approaches_record_cost_reason_and_coverage_resume():
    coordinator = ActiveObservationCoordinator(maximum_approaches=2)
    coordinator.discover("candidate", 0.0)
    coordinator.preflight("candidate", 0.1, GOOD)
    task = coordinator.observation_result("candidate", 1.0, ready=False, confirmed=None, distance_m=0.5, elapsed_s=0.9)
    assert task.state == ObservationState.OBSERVATION_QUEUED
    coordinator.discover("candidate", 1.1)
    coordinator.preflight("candidate", 1.2, GOOD)
    task = coordinator.observation_result("candidate", 2.0, ready=True, confirmed=True, distance_m=0.6, elapsed_s=0.8)
    assert task.state == ObservationState.CONFIRMED
    assert task.extra_distance_m == 1.1
    assert task.extra_time_s == pytest.approx(1.7)
    assert task.terminal_reason == "recognized"
    task = coordinator.mark_coverage_resumed("candidate", 2.1, True)
    assert task.coverage_resumed is True
    assert task.coverage_resume_required is False


def test_dynamic_approach_timeout_includes_path_length_over_minimum_speed():
    coordinator = ActiveObservationCoordinator(approach_timeout_s=5.0, minimum_approach_speed_mps=0.5)
    coordinator.discover("candidate", 0.0)
    task = coordinator.preflight("candidate", 1.0, ObservationPreflight(True, True, True, 0.3, True, 0.0, path_length_m=4.0))
    assert task.approach_deadline_s == 14.0
    task = coordinator.observation_result("candidate", 14.1, ready=True, confirmed=True, distance_m=4.0, elapsed_s=13.1)
    assert task.state == ObservationState.REJECTED
    assert task.terminal_reason == "approach_timeout"


def test_unreachable_is_terminal_and_reason_is_retained():
    coordinator = ActiveObservationCoordinator()
    coordinator.discover("blocked", 0.0)
    task = coordinator.preflight("blocked", 0.2, ObservationPreflight(True, False, True, 0.3, True, 0.0))
    assert task.state == ObservationState.UNREACHABLE
    assert task.terminal_reason == "compute_path_failed"


def test_stage5br4_record_migrates_backward_compatibly():
    coordinator = ActiveObservationCoordinator()
    task = coordinator.restore_task({
        "candidate_id": "legacy",
        "discovered_at_s": 3.5,
        "state": "OBSERVATION_QUEUED",
        "approach_count": 1,
        "extra_distance_m": 0.4,
        "extra_time_s": 1.2,
        "history": [],
    })
    assert task.first_seen_s == task.last_seen_s == task.queued_at_s == 3.5
    assert task.discovered_at_s == 3.5
    assert task.to_record()["source_candidate_ids"] == ["legacy"]
