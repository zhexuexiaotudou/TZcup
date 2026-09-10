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
    revisions_after_baseline,
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


def test_initial_scan_sweep_requires_both_raw_revisions_after_acceptance_baseline():
    common = {
        "baseline_map_revision": 17,
        "baseline_scan_revision": 29,
    }
    assert revisions_after_baseline(
        map_revision=17, scan_revision=29, **common
    ) == (False, False)
    assert revisions_after_baseline(
        map_revision=18, scan_revision=29, **common
    ) == (True, False)
    assert revisions_after_baseline(
        map_revision=17, scan_revision=30, **common
    ) == (False, True)
    assert revisions_after_baseline(
        map_revision=18, scan_revision=30, **common
    ) == (True, True)
    with pytest.raises(ValueError, match="nonnegative integers"):
        revisions_after_baseline(
            map_revision=-1, scan_revision=30, **common
        )


def test_frontier_requires_nav2_spin_and_post_spin_raw_sensor_map_updates():
    source = (
        Path(__file__).resolve().parents[1]
        / "sanitation_formal_campus_integration"
        / "frontier_explorer.py"
    ).read_text(encoding="utf-8")

    assert "from nav2_msgs.action import NavigateToPose, Spin" in source
    assert 'self.declare_parameter("spin_action", "/spin")' in source
    assert "self._spin_client = ActionClient(" in source
    assert "Spin.Goal()" in source
    assert '"initial_scan_sweep_target_yaw_rad", 3.0 * math.pi / 4.0' in source
    assert (
        'target_yaw = self._positive_timeout("initial_scan_sweep_target_yaw_rad")'
        in source
    )
    assert "goal.target_yaw = target_yaw" in source
    assert "initial_scan_sweep_blocked" in source
    assert "handle.cancel_goal_async()" in source
    assert "self._spin_baseline_map_revision = self._map_revision" in source
    assert "self._spin_baseline_scan_revision = self._scan_revision" in source
    assert "revisions_after_baseline(" in source
    assert "wrapped.result" in source
    assert "Spin.Result.NONE" in source
    assert "initial_scan_sweep_waiting_for_odom" in source
    assert "initial_scan_sweep_waiting_for_scan" in source
    assert '"initial_scan_sweep_state": getattr(' in source
    assert 'self, "_initial_scan_sweep_state", "initializing"' in source
    assert 'self._initial_scan_sweep_state = "complete"' in source
    assert "if self._initial_scan_sweep_blocks_frontier():" in source
    assert source.index("if self._initial_scan_sweep_blocks_frontier():") < source.index(
        '"frontier_goal_requested"'
    )
    assert "initial_scan_sweep_time_allowance_sec\", 60.0" in source
    assert "initial_scan_sweep_result_timeout_sec\", 600.0" in source
    assert "initial_scan_sweep_update_timeout_sec\", 60.0" in source
    assert 'self._block("blocked_excessive_nav2_failures"' in source
    assert "from geometry_msgs.msg import Twist" not in source
    assert "/ground_truth" not in source
