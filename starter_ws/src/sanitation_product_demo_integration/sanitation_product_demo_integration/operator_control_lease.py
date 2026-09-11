"""Wall-clock ownership of simulated operator power, independent of Nav2."""

from __future__ import annotations

import math


class OperatorControlLease:
    """A lost owner requires a new low-to-high operator request, never a replay."""

    def __init__(self, planner_timeout=1.5, executor_timeout=0.5, startup_timeout=3.0):
        values = (planner_timeout, executor_timeout, startup_timeout)
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("control lease timeouts must be finite and positive")
        self.timeouts = {"planner": planner_timeout, "executor": executor_timeout}
        self.startup_timeout = startup_timeout
        self.receipts = {role: None for role in self.timeouts}
        self.instances = {}
        self.source_metadata = {}
        self.request_high = False
        self.pending_until = None
        self.armed = False
        self.completed = False
        self.reason = "awaiting_explicit_operator_start"

    def observe(self, role, healthy, now, instance_id, published_at_monotonic_ns,
                sequence, now_monotonic_ns=None):
        if (role not in self.receipts or type(healthy) is not bool or not math.isfinite(now)
                or not isinstance(instance_id, str) or not instance_id):
            raise ValueError("invalid control lease receipt")
        if now_monotonic_ns is None:
            now_monotonic_ns = int(now * 1_000_000_000)
        if (isinstance(published_at_monotonic_ns, bool)
                or not isinstance(published_at_monotonic_ns, int)
                or isinstance(sequence, bool) or not isinstance(sequence, int)
                or sequence < 1 or isinstance(now_monotonic_ns, bool)
                or not isinstance(now_monotonic_ns, int)):
            self._reject_source(role, "invalid")
            return False
        source_age_ns = now_monotonic_ns - published_at_monotonic_ns
        if not 0 <= source_age_ns <= 500_000_000:
            self._reject_source(role, "timestamp_out_of_range")
            return False
        previous = self.source_metadata.get(role)
        if self.instances.get(role) == instance_id and previous is not None:
            if sequence <= previous["sequence"]:
                self._reject_source(role, "sequence_not_strict")
                return False
            if published_at_monotonic_ns < previous["published_at_monotonic_ns"]:
                self._reject_source(role, "timestamp_regressed")
                return False
        if self.armed and self.instances.get(role) != instance_id:
            self.stop(f"{role}_instance_changed_requires_new_operator_request")
        self.instances[role] = instance_id
        self.source_metadata[role] = {
            "published_at_monotonic_ns": published_at_monotonic_ns,
            "sequence": sequence,
        }
        self.receipts[role] = (now, healthy)
        self.tick(now)
        return True

    def _reject_source(self, role, reason):
        # A replay invalidates the prior receipt: it must not be reused by a
        # later operator edge while the sender is silent.
        self.receipts[role] = None
        self.stop(f"{role}_source_{reason}_requires_new_operator_request")

    def _unhealthy_role(self, now):
        for role, timeout in self.timeouts.items():
            receipt = self.receipts[role]
            if receipt is None or not receipt[1] or not 0 <= now - receipt[0] < timeout:
                return role
        return None

    def owners_ready(self, now):
        """Report whether both independently identified control owners are fresh."""
        if not math.isfinite(now):
            raise ValueError("invalid wall clock")
        return self._unhealthy_role(now) is None

    def request(self, requested, now):
        if type(requested) is not bool or not math.isfinite(now):
            raise ValueError("invalid operator request")
        if not requested:
            self.request_high = False
            self.stop("operator_requested_safe_stop")
        elif not self.request_high:
            self.request_high = True
            if not self.completed:
                self.pending_until = now + self.startup_timeout
                self.reason = "awaiting_control_owners"
        self.tick(now)

    def tick(self, now):
        if not math.isfinite(now):
            self.stop("invalid_wall_clock")
            return
        missing = self._unhealthy_role(now)
        if self.armed and missing:
            self.stop(f"{missing}_lease_lost_requires_new_operator_request")
        elif self.pending_until is not None:
            if now >= self.pending_until:
                self.stop("control_owner_startup_timeout")
            elif missing is None:
                self.pending_until = None
                self.armed = True
                self.reason = "operator_armed"

    def stop(self, reason):
        self.armed = False
        self.pending_until = None
        self.reason = reason

    def complete(self):
        self.completed = True
        self.stop("mission_complete_safe_stop")
