"""ROS-independent ordering checks for formal runtime evidence."""

from __future__ import annotations

import math
from collections.abc import Mapping


COMMAND_CHAIN_TOPICS = (
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/cmd_vel_gate",
    "/base_controller/cmd_vel",
)
EXPECTED_COMMAND_TOPIC_PUBLISHER = {
    "/cmd_vel_nav": "/controller_server",
    "/cmd_vel_smoothed": "/velocity_smoother",
    "/cmd_vel_gate": "/collision_monitor",
    "/base_controller/cmd_vel": "/whole_vehicle_safety_manager",
}
# The collector runs a 10 Hz supervisory timer.  This bounded tolerance admits
# ordinary cross-topic callback scheduling jitter while rejecting a downstream
# nonzero receipt that precedes its upstream source by more than two cycles.
# It is evidence association only, never a control or safety timeout.
COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S = 0.25


def first_nonzero_chain_is_ordered(
    first_received_s: Mapping[str, float | None],
) -> bool:
    """Check the Nav2-to-safety chain without nanosecond-order assumptions."""
    values: list[float] = []
    for topic in COMMAND_CHAIN_TOPICS:
        value = first_received_s.get(topic)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False
        numeric = float(value)
        if not math.isfinite(numeric) or numeric < 0.0:
            return False
        values.append(numeric)
    return all(
        downstream + COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S >= upstream
        for upstream, downstream in zip(values, values[1:])
    )
