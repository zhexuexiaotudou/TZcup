"""Fail-closed adjudication for end-of-coverage product work queues."""

from __future__ import annotations


def _is_zero_count(value: object) -> bool:
    try:
        return int(value) == 0
    except (TypeError, ValueError, OverflowError):
        return False


def product_task_drain_snapshot(spot: object, reobserve: object) -> dict:
    """Return machine-readable drain checks for the two product schedulers."""
    if not isinstance(spot, dict) or not isinstance(reobserve, dict):
        return {
            "drained": False,
            "spot_state_present": isinstance(spot, dict),
            "reobserve_state_present": isinstance(reobserve, dict),
        }
    checks = {
        "spot_current_target_clear": spot.get("current_target_uuid") is None,
        "spot_queue_empty": _is_zero_count(spot.get("queued_target_count")),
        "spot_brush_disabled": spot.get("brush_enabled") is False,
        "spot_coverage_released": spot.get("coverage_paused") is False,
        "reobserve_not_busy": reobserve.get("busy") is False,
        "reobserve_queue_empty": _is_zero_count(
            reobserve.get("queued_request_count")
        ),
        "reobserve_coverage_released": (
            reobserve.get("coverage_paused") is False
        ),
    }
    return {"drained": all(checks.values()), **checks}
