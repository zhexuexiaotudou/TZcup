"""Pure fail-closed timing helpers for the formal frontier explorer."""

from __future__ import annotations

import math
from typing import Protocol


MAX_ACTION_DISCOVERY_WAIT_SEC = 0.1


class WaitableActionClient(Protocol):
    def wait_for_server(self, *, timeout_sec: float) -> bool: ...


def bounded_action_server_ready(
    client: WaitableActionClient, *, timeout_sec: float
) -> bool:
    """Actively discover an action server without blocking over 100 ms."""
    if (
        not math.isfinite(timeout_sec)
        or timeout_sec <= 0.0
        or timeout_sec > MAX_ACTION_DISCOVERY_WAIT_SEC
    ):
        raise ValueError("action discovery timeout must be in (0.0, 0.1] seconds")
    return bool(client.wait_for_server(timeout_sec=timeout_sec))


def goal_response_timed_out(
    *,
    pending_request_id: int | None,
    deadline_monotonic: float | None,
    now_monotonic: float,
) -> bool:
    """Return true only for a still-pending request at or past its deadline."""
    if pending_request_id is None or deadline_monotonic is None:
        return False
    if not math.isfinite(deadline_monotonic) or not math.isfinite(now_monotonic):
        raise ValueError("goal-response watchdog times must be finite")
    return now_monotonic >= deadline_monotonic


def active_goal_timed_out(
    *,
    goal_active: bool,
    deadline_monotonic: float | None,
    now_monotonic: float,
) -> bool:
    """Return true when an accepted goal has exceeded a finite watchdog deadline."""
    if not goal_active or deadline_monotonic is None:
        return False
    if not math.isfinite(deadline_monotonic) or not math.isfinite(now_monotonic):
        raise ValueError("active-goal watchdog times must be finite")
    return now_monotonic >= deadline_monotonic


def progress_deadline_after_feedback(
    *,
    previous_best_distance_m: float,
    distance_remaining_m: float,
    now_monotonic: float,
    timeout_sec: float,
    minimum_progress_m: float = 0.05,
) -> tuple[float, float | None]:
    """Refresh the progress deadline only for a real reduction in remaining distance."""
    values = (distance_remaining_m, now_monotonic, timeout_sec, minimum_progress_m)
    if (
        not all(math.isfinite(value) for value in values)
        or math.isnan(previous_best_distance_m)
        or previous_best_distance_m < 0.0
    ):
        raise ValueError("frontier progress values must be finite")
    if distance_remaining_m < 0.0 or timeout_sec <= 0.0 or minimum_progress_m <= 0.0:
        raise ValueError("frontier progress distances and timeout must be positive")
    if distance_remaining_m <= previous_best_distance_m - minimum_progress_m:
        return distance_remaining_m, now_monotonic + timeout_sec
    return previous_best_distance_m, None
