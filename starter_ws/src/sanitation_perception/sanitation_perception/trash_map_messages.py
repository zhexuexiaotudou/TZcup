"""Serializable state contracts for the online dynamic trash map."""

from __future__ import annotations

from enum import Enum


class TargetState(str, Enum):
    CANDIDATE = "CANDIDATE"
    TRACKED = "TRACKED"
    CONFIRMED = "CONFIRMED"
    SCHEDULED = "SCHEDULED"
    APPROACHING = "APPROACHING"
    VERIFYING = "VERIFYING"
    CLEANING = "CLEANING"
    POST_VERIFY = "POST_VERIFY"
    CLEANED = "CLEANED"
    DEFERRED = "DEFERRED"
    REJECTED = "REJECTED"
    LOST = "LOST"
    UNREACHABLE = "UNREACHABLE"


class PostCleanState(str, Enum):
    NOT_STARTED = "NOT_STARTED"
    PENDING = "PENDING"
    ABSENT_ACCUMULATING = "ABSENT_ACCUMULATING"
    RESIDUAL_DETECTED = "RESIDUAL_DETECTED"
    PASSED = "PASSED"
    MANUAL_ATTENTION = "MANUAL_ATTENTION"


TERMINAL_STATES = {
    TargetState.CLEANED,
    TargetState.REJECTED,
    TargetState.UNREACHABLE,
}


ALLOWED_TRANSITIONS: dict[TargetState, set[TargetState]] = {
    TargetState.CANDIDATE: {
        TargetState.TRACKED,
        TargetState.DEFERRED,
        TargetState.REJECTED,
        TargetState.LOST,
    },
    TargetState.TRACKED: {
        TargetState.CONFIRMED,
        TargetState.DEFERRED,
        TargetState.REJECTED,
        TargetState.LOST,
    },
    TargetState.CONFIRMED: {
        TargetState.SCHEDULED,
        TargetState.VERIFYING,
        TargetState.DEFERRED,
        TargetState.REJECTED,
        TargetState.LOST,
        TargetState.UNREACHABLE,
    },
    TargetState.SCHEDULED: {
        TargetState.APPROACHING,
        TargetState.DEFERRED,
        TargetState.UNREACHABLE,
        TargetState.LOST,
    },
    TargetState.APPROACHING: {
        TargetState.VERIFYING,
        TargetState.CLEANING,
        TargetState.DEFERRED,
        TargetState.UNREACHABLE,
        TargetState.LOST,
    },
    TargetState.VERIFYING: {
        TargetState.CONFIRMED,
        TargetState.CLEANING,
        TargetState.DEFERRED,
        TargetState.REJECTED,
        TargetState.LOST,
    },
    TargetState.CLEANING: {
        TargetState.POST_VERIFY,
        TargetState.DEFERRED,
    },
    TargetState.POST_VERIFY: {
        TargetState.CLEANED,
        TargetState.SCHEDULED,
        TargetState.DEFERRED,
    },
    TargetState.DEFERRED: {
        TargetState.TRACKED,
        TargetState.CONFIRMED,
        TargetState.SCHEDULED,
        TargetState.VERIFYING,
        TargetState.REJECTED,
        TargetState.LOST,
        TargetState.UNREACHABLE,
    },
    TargetState.LOST: {
        TargetState.TRACKED,
        TargetState.CONFIRMED,
        TargetState.REJECTED,
    },
    TargetState.CLEANED: set(),
    TargetState.REJECTED: set(),
    TargetState.UNREACHABLE: {TargetState.DEFERRED},
}


def require_transition(current: TargetState, requested: TargetState) -> None:
    if requested == current:
        return
    if requested not in ALLOWED_TRANSITIONS[current]:
        raise ValueError(f"invalid target transition {current.value} -> {requested.value}")
