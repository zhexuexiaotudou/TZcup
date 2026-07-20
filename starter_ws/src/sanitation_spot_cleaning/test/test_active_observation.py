from sanitation_spot_cleaning.active_observation import (
    ActiveObservationCoordinator,
    ObservationPreflight,
    ObservationState,
)


GOOD = ObservationPreflight(True, True, True, 0.30, True, 0.002)


def test_active_observation_confirms_and_records_task_cost_without_duplicates():
    coordinator = ActiveObservationCoordinator()
    first = coordinator.discover("candidate-1", 0.0)
    assert coordinator.discover("candidate-1", 0.1) is first
    task = coordinator.preflight("candidate-1", 0.5, GOOD)
    assert task.state == ObservationState.APPROACHING
    task = coordinator.observation_result("candidate-1", 2.0, ready=True, confirmed=True, distance_m=1.2, elapsed_s=1.5)
    assert task.state == ObservationState.CONFIRMED
    assert task.extra_distance_m == 1.2
    assert task.extra_time_s == 1.5


def test_unreachable_and_stale_candidates_fail_closed():
    coordinator = ActiveObservationCoordinator(stale_candidate_s=1.0)
    coordinator.discover("blocked", 0.0)
    blocked = coordinator.preflight("blocked", 0.2, ObservationPreflight(True, False, True, 0.3, True, 0.0))
    assert blocked.state == ObservationState.UNREACHABLE
    coordinator.discover("stale", 0.0)
    stale = coordinator.preflight("stale", 1.1, GOOD)
    assert stale.state == ObservationState.REJECTED


def test_false_candidate_does_not_loop_forever():
    coordinator = ActiveObservationCoordinator(maximum_approaches=2)
    coordinator.discover("candidate", 0.0)
    coordinator.preflight("candidate", 0.1, GOOD)
    task = coordinator.observation_result("candidate", 0.5, ready=False, confirmed=None, distance_m=0.5, elapsed_s=0.4)
    assert task.state == ObservationState.OBSERVATION_QUEUED
    coordinator.preflight("candidate", 0.6, GOOD)
    task = coordinator.observation_result("candidate", 1.0, ready=False, confirmed=None, distance_m=0.5, elapsed_s=0.4)
    assert task.state == ObservationState.REJECTED
