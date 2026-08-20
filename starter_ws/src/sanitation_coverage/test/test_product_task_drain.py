from sanitation_coverage.product_task_drain import product_task_drain_snapshot


def idle_spot():
    return {
        "current_target_uuid": None,
        "queued_target_count": 0,
        "brush_enabled": False,
        "coverage_paused": False,
    }


def idle_reobserve():
    return {
        "busy": False,
        "queued_request_count": 0,
        "coverage_paused": False,
    }


def test_both_product_queues_must_be_idle_and_release_coverage():
    assert product_task_drain_snapshot(idle_spot(), idle_reobserve())["drained"]
    for field, value in (
        ("current_target_uuid", "target-1"),
        ("queued_target_count", 1),
        ("brush_enabled", True),
        ("coverage_paused", True),
    ):
        spot = idle_spot()
        spot[field] = value
        assert not product_task_drain_snapshot(spot, idle_reobserve())["drained"]


def test_missing_stale_or_pending_reobserve_state_fails_closed():
    assert not product_task_drain_snapshot(idle_spot(), None)["drained"]
    for field, value in (
        ("busy", True),
        ("queued_request_count", 1),
        ("coverage_paused", True),
    ):
        reobserve = idle_reobserve()
        reobserve[field] = value
        assert not product_task_drain_snapshot(idle_spot(), reobserve)["drained"]


def test_malformed_queue_counts_fail_closed_without_exception():
    for value in (None, "invalid", float("inf")):
        spot = idle_spot()
        spot["queued_target_count"] = value
        assert not product_task_drain_snapshot(spot, idle_reobserve())["drained"]
        reobserve = idle_reobserve()
        reobserve["queued_request_count"] = value
        assert not product_task_drain_snapshot(idle_spot(), reobserve)["drained"]
