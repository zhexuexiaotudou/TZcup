import math

from sanitation_coverage.ackermann_turn_planner import build_ackermann_plan
from sanitation_coverage.coverage_components import ComponentType


def test_connector_settles_inside_target_swath_brush_off_leadin():
    swaths = [
        ((2.0, 4.0), (18.0, 4.0)),
        ((18.0, 8.0), (2.0, 8.0)),
    ]
    apron = [(0.0, 0.0), (24.0, 0.0), (24.0, 12.0), (0.0, 12.0)]

    components, summary = build_ackermann_plan(swaths, apron, [])
    connector = [
        item
        for item in components
        if item.kind in (ComponentType.FORWARD, ComponentType.REVERSE)
    ][-1]

    assert summary["deferred_swath_ids"] == []
    assert connector.points[-1] == (16.0, 8.0)
    assert math.isclose(
        connector.metadata["target_swath_settle_overlap_m"], 2.0
    )
    assert connector.metadata["target_swath_geometric_start"] == [18.0, 8.0]
    assert connector.metadata["speed_limit_mps"] == 0.20
    target_swath = components[-1]
    assert target_swath.kind is ComponentType.SWATH
    assert target_swath.points[0] == (18.0, 8.0)


def test_connector_settle_overlap_is_bounded_by_short_swath_length():
    swaths = [
        ((1.0, 2.0), (7.0, 2.0)),
        ((7.0, 5.0), (6.0, 5.0)),
    ]
    apron = [(0.0, 0.0), (12.0, 0.0), (12.0, 9.0), (0.0, 9.0)]

    components, _ = build_ackermann_plan(swaths, apron, [])
    connector = [
        item
        for item in components
        if item.kind in (ComponentType.FORWARD, ComponentType.REVERSE)
    ][-1]

    assert connector.points[-1] == (6.0, 5.0)
    assert math.isclose(
        connector.metadata["target_swath_settle_overlap_m"], 1.0
    )


def test_runtime_speed_contract_reaches_swaths_and_connectors():
    swaths = [
        ((2.0, 4.0), (18.0, 4.0)),
        ((18.0, 8.0), (2.0, 8.0)),
    ]
    apron = [(0.0, 0.0), (24.0, 0.0), (24.0, 12.0), (0.0, 12.0)]
    limits = {"CLEAN": 1.0, "FORWARD": 0.55, "REVERSE": 0.25}

    components, summary = build_ackermann_plan(
        swaths, apron, [], speed_limits_mps=limits
    )

    assert summary["speed_limits_mps"]["CLEAN"] == 1.0
    assert all(
        item.metadata["speed_limit_mps"] == 1.0
        for item in components
        if item.kind is ComponentType.SWATH
    )
    connector = next(
        item for item in components
        if item.kind in (ComponentType.FORWARD, ComponentType.REVERSE)
    )
    assert connector.metadata["speed_limit_mps"] == limits[
        connector.speed_profile
    ]
