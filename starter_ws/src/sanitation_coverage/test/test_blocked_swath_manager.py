from sanitation_coverage.blocked_swath_manager import BlockedSwathManager


def test_blocked_swath_has_one_retry_then_moves_to_repair_queue():
    manager = BlockedSwathManager(max_retries=1)
    assert manager.report_blocked("s3") == "RETRY"
    assert manager.report_blocked("s3") == "DEFER_TO_REPAIR"
    assert manager.repair_queue() == ("s3",)
    manager.report_clear("s3")
    assert not manager.repair_queue()
