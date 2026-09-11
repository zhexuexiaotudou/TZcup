from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sanitation_product_demo_integration.operator_control_lease import OperatorControlLease


def observe(lease, role, healthy, now, instance="original", sequence=1, source_ns=None):
    return lease.observe(
        role, healthy, now, instance,
        int(now * 1_000_000_000) if source_ns is None else source_ns,
        sequence,
    )


def ready():
    lease = OperatorControlLease()
    observe(lease, "planner", True, 0.0)
    observe(lease, "executor", True, 0.0)
    return lease


def test_health_alone_never_arms_and_startup_waits_for_both_owners():
    lease = OperatorControlLease()
    lease.request(True, 0.0)
    observe(lease, "planner", True, 0.1)
    assert not lease.armed
    observe(lease, "executor", True, 0.2)
    assert lease.armed
    other = ready()
    assert not other.armed


def test_owner_readiness_is_independent_of_operator_arming():
    lease = ready()
    assert lease.owners_ready(0.1)
    assert not lease.owners_ready(0.6)  # executor receipt expires first


def test_lost_executor_stops_even_with_live_planner_and_repeated_start():
    lease = ready()
    lease.request(True, 0.0)
    observe(lease, "planner", True, 0.5, sequence=2)
    assert not lease.armed
    assert lease.reason.startswith("executor_lease_lost")
    observe(lease, "executor", True, 0.6, sequence=2)
    lease.request(True, 0.6)
    assert not lease.armed
    lease.request(False, 0.61)
    lease.request(True, 0.62)
    assert lease.armed


def test_lost_planner_cannot_be_masked_by_executor_heartbeat():
    lease = ready()
    lease.request(True, 0.0)
    for sequence, now in enumerate((0.4, 0.8, 1.2, 1.5), start=2):
        observe(lease, "executor", True, now, sequence=sequence)
    assert not lease.armed
    assert lease.reason.startswith("planner_lease_lost")


def test_explicit_unhealthy_receipt_stops_immediately():
    lease = ready()
    lease.request(True, 0.0)
    observe(lease, "planner", False, 0.1, sequence=2)
    assert not lease.armed
    observe(lease, "planner", True, 0.2, sequence=3)
    assert not lease.armed


def test_startup_timeout_and_completion_do_not_auto_rearm():
    lease = OperatorControlLease()
    lease.request(True, 0.0)
    lease.tick(3.0)
    observe(lease, "planner", True, 3.1, sequence=1)
    observe(lease, "executor", True, 3.1, sequence=1)
    assert not lease.armed
    lease.request(False, 3.2)
    lease.request(True, 3.2)
    assert lease.armed
    lease.complete()
    lease.request(False, 3.3)
    lease.request(True, 3.3)
    assert not lease.armed


def test_clock_reversal_fails_closed():
    lease = ready()
    lease.request(True, 0.0)
    lease.tick(-0.1)
    assert not lease.armed


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), True])
def test_invalid_timeout_is_rejected(value):
    with pytest.raises(ValueError):
        OperatorControlLease(executor_timeout=value)


@pytest.mark.parametrize("role", ["planner", "executor"])
def test_replacement_owner_cannot_inherit_an_active_lease(role):
    lease = ready()
    lease.request(True, 0.0)
    observe(lease, role, True, 0.1, "replacement")
    assert not lease.armed
    assert "instance_changed" in lease.reason
    lease.request(True, 0.2)
    assert not lease.armed


@pytest.mark.parametrize("failure", ["sequence", "timestamp", "future"])
def test_replayed_or_future_owner_receipt_invalidates_readiness_before_arming(failure):
    lease = ready()
    if failure == "sequence":
        observe(lease, "planner", True, 0.1, sequence=1)
    elif failure == "timestamp":
        observe(lease, "planner", True, 0.1, sequence=2)
        observe(lease, "planner", True, 0.2, sequence=3, source_ns=50_000_000)
    else:
        observe(lease, "planner", True, 0.1, sequence=2, source_ns=200_000_000)
    assert not lease.owners_ready(0.1)
    assert "planner_source_" in lease.reason
    lease.request(True, 0.1)
    assert not lease.armed


@pytest.mark.parametrize("failure", ["sequence", "timestamp"])
def test_armed_replay_stops_and_fresh_receipts_do_not_auto_rearm(failure):
    lease = ready()
    lease.request(True, 0.0)
    assert lease.armed
    if failure == "sequence":
        observe(lease, "executor", True, 0.1, sequence=1)
    else:
        observe(lease, "executor", True, 0.1, sequence=2)
        observe(lease, "executor", True, 0.2, sequence=3, source_ns=50_000_000)
    assert not lease.armed and "executor_source_" in lease.reason
    observe(lease, "planner", True, 0.2, sequence=2)
    observe(lease, "executor", True, 0.2, sequence=3)
    assert not lease.armed
