"""High-level Ackermann coverage plan builder.

Converts the Fields2Cover boustrophedon swath set into an executable
Ackermann component sequence.  Swaths are cleaned in order; connectors are
planned with the strict order forward U-turn -> forward teardrop ->
three-point search -> Hybrid connector request -> defer the target swath.
"""

from __future__ import annotations

import math

from .ackermann_connector import hybrid_connector_request, plan_ackermann_connector
from .coverage_components import ComponentType, CoverageComponent


CONNECTOR_SETTLE_DISTANCE_M = 2.0


def _segment_heading(start, end) -> float:
    return math.atan2(end[1] - start[1], end[0] - start[0])


def _interpolate(start, end, spacing=0.10):
    length = math.dist(start, end)
    count = max(2, int(math.ceil(length / spacing)) + 1)
    return [
        (
            start[0] + (end[0] - start[0]) * index / (count - 1),
            start[1] + (end[1] - start[1]) * index / (count - 1),
        )
        for index in range(count)
    ]


def _point_along_segment(start, end, distance_m):
    length = math.dist(start, end)
    if length <= 1e-9:
        return start, 0.0
    applied = min(max(0.0, float(distance_m)), length)
    ratio = applied / length
    return (
        (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        ),
        applied,
    )


def build_ackermann_plan(
    swaths: list[tuple[tuple[float, float], tuple[float, float]]],
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
) -> tuple[tuple[CoverageComponent, ...], dict]:
    """Build the full Ackermann component sequence and a summary."""
    components: list[CoverageComponent] = []
    summary = {
        "profile": "ACKERMANN",
        "planner_order": [
            "forward_dubins",
            "forward_uturn",
            "forward_teardrop",
            "three_point_search",
            "hybrid_request",
            "defer_swath",
        ],
        "connector_counts": {
            "FORWARD_DUBINS_TURN": 0,
            "FORWARD_U_TURN": 0,
            "FORWARD_TEARDROP_TURN": 0,
            "REEDS_SHEPP_THREE_POINT_TURN": 0,
            "SMAC_HYBRID_CONNECTOR": 0,
        },
        "deferred_swath_ids": [],
    }
    for index, swath in enumerate(swaths):
        swath_id = f"swath-{index:02d}"
        components.append(
            CoverageComponent(
                component_id=swath_id,
                kind=ComponentType.SWATH,
                points=tuple(_interpolate(*swath)),
                brush_enabled=True,
                speed_profile="CLEAN",
                metadata={"swath_index": index, "swath_id": swath_id},
            )
        )
        if index >= len(swaths) - 1:
            break
        next_swath = swaths[index + 1]
        next_swath_id = f"swath-{index + 1:02d}"
        start_pose = swath[1]
        start_yaw = _segment_heading(*swath)
        # Finish inside the target swath's brush-off lead-in instead of at its
        # geometric endpoint. A 180-degree Ackermann turn still carries a
        # finite heading lag at the tangent point; the straight overlap lets
        # the physical steering plant settle before the unchanged brush-enable
        # boundary. The following swath action prunes to the current projected
        # pose, so this overlap is driven once rather than replayed.
        goal_pose, settle_distance_m = _point_along_segment(
            next_swath[0], next_swath[1], CONNECTOR_SETTLE_DISTANCE_M
        )
        goal_yaw = _segment_heading(*next_swath)
        connector = plan_ackermann_connector(
            connector_id=f"connector-{index:02d}",
            start=start_pose,
            start_yaw=start_yaw,
            goal=goal_pose,
            goal_yaw=goal_yaw,
            apron=apron,
            keepouts=keepouts,
            source_swath_id=swath_id,
            target_swath_id=next_swath_id,
        )
        if connector is not None:
            connector = tuple(
                CoverageComponent(
                    component_id=component.component_id,
                    kind=component.kind,
                    points=component.points,
                    brush_enabled=component.brush_enabled,
                    speed_profile=component.speed_profile,
                    metadata={
                        **component.metadata,
                        "target_swath_settle_overlap_m": settle_distance_m,
                        "target_swath_geometric_start": list(next_swath[0]),
                        **(
                            {"speed_limit_mps": 0.20}
                            if component.kind is ComponentType.FORWARD
                            else {"speed_limit_mps": 0.15}
                            if component.kind is ComponentType.REVERSE
                            else {}
                        ),
                    },
                )
                for component in connector
            )
            classes = {
                component.metadata.get("connector_class")
                for component in connector
                if component.kind in (ComponentType.FORWARD, ComponentType.REVERSE)
            }
            for connector_class in classes:
                if connector_class in summary["connector_counts"]:
                    summary["connector_counts"][connector_class] += 1
            components.extend(connector)
            continue
        # No bounded analytic connector exists: request the configured Smac
        # Hybrid planner at runtime. If that request fails, coverage execution
        # stops and records the target as the first deferred swath; it never
        # substitutes a straight line or point rotation.
        summary["connector_counts"]["SMAC_HYBRID_CONNECTOR"] += 1
        components.append(
            hybrid_connector_request(
                connector_id=f"connector-{index:02d}-hybrid",
                start=start_pose,
                start_yaw=start_yaw,
                goal=goal_pose,
                goal_yaw=goal_yaw,
                source_swath_id=swath_id,
                target_swath_id=next_swath_id,
            )
        )
    summary["deferred_swath_ids"] = sorted(summary["deferred_swath_ids"])
    return tuple(components), summary
