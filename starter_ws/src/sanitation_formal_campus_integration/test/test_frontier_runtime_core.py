import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sanitation_formal_campus_integration.frontier_runtime_core import (
    active_goal_timed_out,
    bounded_action_server_ready,
    goal_response_timed_out,
    progress_deadline_after_feedback,
)


class DelayedServerClient:
    def __init__(self) -> None:
        self.waits: list[float] = []
        self.ready = False

    def server_is_ready(self) -> bool:
        return self.ready

    def wait_for_server(self, *, timeout_sec: float) -> bool:
        self.waits.append(timeout_sec)
        self.ready = True
        return self.ready


def test_delayed_action_server_discovery_is_actively_bounded():
    client = DelayedServerClient()
    assert client.server_is_ready() is False
    assert bounded_action_server_ready(client, timeout_sec=0.1) is True
    assert client.server_is_ready() is True
    assert client.waits == [0.1]


@pytest.mark.parametrize("timeout", (0.0, -0.1, 0.100001, math.inf, math.nan))
def test_action_server_discovery_rejects_unbounded_waits(timeout):
    with pytest.raises(ValueError, match="0.1"):
        bounded_action_server_ready(DelayedServerClient(), timeout_sec=timeout)


def test_goal_response_watchdog_fails_closed_at_deadline():
    assert not goal_response_timed_out(
        pending_request_id=7,
        deadline_monotonic=15.0,
        now_monotonic=14.999,
    )
    assert goal_response_timed_out(
        pending_request_id=7,
        deadline_monotonic=15.0,
        now_monotonic=15.0,
    )
    assert not goal_response_timed_out(
        pending_request_id=None,
        deadline_monotonic=None,
        now_monotonic=99.0,
    )


def test_accepted_goal_watchdog_fails_at_deadline_only_while_active():
    assert not active_goal_timed_out(
        goal_active=True, deadline_monotonic=10.0, now_monotonic=9.99
    )
    assert active_goal_timed_out(
        goal_active=True, deadline_monotonic=10.0, now_monotonic=10.0
    )
    assert not active_goal_timed_out(
        goal_active=False, deadline_monotonic=10.0, now_monotonic=11.0
    )


def test_progress_watchdog_refreshes_only_after_material_progress():
    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=10.0,
        distance_remaining_m=9.94,
        now_monotonic=100.0,
        timeout_sec=120.0,
    )
    assert best == pytest.approx(9.94)
    assert deadline == pytest.approx(220.0)
    unchanged, no_refresh = progress_deadline_after_feedback(
        previous_best_distance_m=best,
        distance_remaining_m=9.91,
        now_monotonic=110.0,
        timeout_sec=120.0,
    )
    assert unchanged == pytest.approx(best)
    assert no_refresh is None


def test_progress_ignores_initial_zero_placeholder_then_tracks_real_distance():
    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=math.inf,
        distance_remaining_m=0.0,
        now_monotonic=10.0,
        timeout_sec=120.0,
    )
    assert math.isinf(best)
    assert deadline is None

    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=best,
        distance_remaining_m=13.158895,
        now_monotonic=11.0,
        timeout_sec=120.0,
    )
    assert best == pytest.approx(13.158895)
    assert deadline == pytest.approx(131.0)

    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=best,
        distance_remaining_m=12.573,
        now_monotonic=12.0,
        timeout_sec=120.0,
    )
    assert best == pytest.approx(12.573)
    assert deadline == pytest.approx(132.0)


def test_zero_after_positive_distance_obeys_existing_minimum_progress_rule():
    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=0.08,
        distance_remaining_m=0.0,
        now_monotonic=50.0,
        timeout_sec=120.0,
    )
    assert best == 0.0
    assert deadline == pytest.approx(170.0)


def test_small_positive_baseline_to_zero_does_not_bypass_minimum_progress():
    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=0.01,
        distance_remaining_m=0.0,
        now_monotonic=50.0,
        timeout_sec=120.0,
    )
    assert best == pytest.approx(0.01)
    assert deadline is None


def test_zero_to_zero_does_not_refresh_progress_deadline():
    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=0.0,
        distance_remaining_m=0.0,
        now_monotonic=50.0,
        timeout_sec=120.0,
    )
    assert best == 0.0
    assert deadline is None


def test_only_initial_zero_feedback_leaves_existing_watchdog_to_timeout():
    best, deadline = progress_deadline_after_feedback(
        previous_best_distance_m=math.inf,
        distance_remaining_m=0.0,
        now_monotonic=30.0,
        timeout_sec=120.0,
    )
    assert math.isinf(best)
    assert deadline is None
    assert active_goal_timed_out(
        goal_active=True, deadline_monotonic=120.0, now_monotonic=120.0
    )
