import pytest

from sanitation_coverage.blocked_swath_manager import (
    BlockedSwathManager,
    BlockedSwathState,
)


def test_blocked_swath_obeys_retry_delay_and_never_retries_forever():
    manager = BlockedSwathManager(max_retries=2, minimum_retry_delay_sec=10.0)
    assert manager.report_blocked("s3", (0.30, 0.50), now_sec=100.0) == "RETRY_PENDING"
    assert not manager.retry_ready("s3", now_sec=109.999)
    assert manager.activate_retry("s3", now_sec=110.0)
    assert manager.report_blocked("s3", (0.30, 0.50), now_sec=111.0) == "RETRY_PENDING"
    assert manager.activate_retry("s3", now_sec=121.0)
    assert manager.report_blocked("s3", (0.45, 0.60), now_sec=122.0) == "DEFERRED"
    assert manager.repair_queue() == ("s3",)
    assert manager.records["s3"].retry_count == 2
    assert manager.records["s3"].blocked_intervals == [(0.30, 0.50), (0.45, 0.60)]


def test_clear_and_unreachable_states_are_terminal_and_auditable():
    manager = BlockedSwathManager(minimum_retry_delay_sec=0.0)
    manager.report_blocked("clear", now_sec=1.0)
    manager.report_clear("clear")
    manager.report_blocked("lost", now_sec=2.0)
    manager.mark_unreachable("lost", "costmap_path_absent")
    assert manager.records["clear"].state is BlockedSwathState.COMPLETED
    assert manager.records["lost"].state is BlockedSwathState.UNREACHABLE
    assert manager.report_blocked("lost", now_sec=3.0) == "UNREACHABLE"
    snapshot = {item["swath_id"]: item for item in manager.snapshot()}
    assert snapshot["clear"]["terminal_reason"] == "completed_after_obstacle_clearance"
    assert snapshot["lost"]["terminal_reason"] == "costmap_path_absent"


def test_invalid_policy_and_interval_fail_closed():
    with pytest.raises(ValueError):
        BlockedSwathManager(max_retries=-1)
    with pytest.raises(ValueError):
        BlockedSwathManager(minimum_retry_delay_sec=-0.1)
    with pytest.raises(ValueError):
        BlockedSwathManager().report_blocked("s", (0.8, 0.2))
