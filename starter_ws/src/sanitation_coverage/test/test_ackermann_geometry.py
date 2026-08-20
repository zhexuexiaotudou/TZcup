from sanitation_coverage.mission_geometry import (
    coverage_planning_geometry,
    should_generate_internal_headland,
)
import math

from sanitation_coverage.ackermann_connector import (
    plan_forward_dubins_path,
    plan_reverse_dubins_path,
    plan_ackermann_connector,
    split_hybrid_path_by_direction,
)


def test_ackermann_swaths_use_cleanable_polygon_not_outer_turning_apron():
    geometry = {
        "outer_polygon": [(-4.0, -5.5), (4.0, -5.5), (4.0, 2.5), (-4.0, 2.5)],
        "exclusion_polygons": [[(9.0, 9.0)]],
        "cleanable_outer_polygon": [(-2.0, -3.0), (2.0, -3.0), (2.0, 0.0), (-2.0, 0.0)],
        "cleanable_exclusion_polygons": [],
        "cleanable_polygon_explicit": True,
    }
    polygon, exclusions = coverage_planning_geometry("ACKERMANN", geometry)
    assert polygon == geometry["cleanable_outer_polygon"]
    assert exclusions == []


def test_legacy_swaths_keep_existing_outer_geometry_behavior():
    geometry = {
        "outer_polygon": [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
        "exclusion_polygons": [],
        "cleanable_outer_polygon": [(0.2, 0.2), (0.8, 0.2), (0.8, 0.8), (0.2, 0.8)],
        "cleanable_exclusion_polygons": [],
        "cleanable_polygon_explicit": True,
    }
    polygon, _ = coverage_planning_geometry("SKID_STEER_OPTIMIZED", geometry)
    assert polygon == geometry["outer_polygon"]


def test_external_ackermann_apron_is_not_applied_as_an_internal_headland():
    ackermann = {
        "headland": {"enabled": True, "width_m": 2.0},
        "cleanable_outer_polygon": [[-2, -3], [2, -3], [2, 0], [-2, 0]],
    }
    assert should_generate_internal_headland(ackermann) is False
    assert should_generate_internal_headland({
        "headland": {"enabled": True, "width_m": 0.75}
    }) is True


def test_hybrid_path_is_split_at_forward_reverse_cusps():
    poses = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (1.5, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.5, 0.0, 0.0),
    ]
    sections = split_hybrid_path_by_direction(poses)
    assert [section["direction"] for section in sections] == [
        "FORWARD", "REVERSE", "FORWARD"
    ]
    assert [section["cusp_before"] for section in sections] == [
        False, True, True
    ]
    assert sections[0]["poses"][-1] == sections[1]["poses"][0]
    assert sections[1]["poses"][-1] == sections[2]["poses"][0]


def test_expanded_apron_admits_forward_dubins_swath_connector():
    connector = plan_ackermann_connector(
        "connector-0",
        (-3.52, -0.25),
        math.pi,
        (-4.88, -1.35),
        0.0,
        [(-7.8, -6.5), (7.8, -6.5), (7.8, 3.5), (-7.8, 3.5)],
        [],
        "swath-0",
        "swath-1",
    )
    assert connector is not None
    assert len(connector) == 1
    assert connector[0].kind.value == "FORWARD"
    assert connector[0].metadata["connector_class"] == "FORWARD_DUBINS_TURN"
    assert connector[0].points[0] == (-3.52, -0.25)
    assert connector[0].points[-1] == (-4.88, -1.35)
    assert abs(float(connector[0].metadata["curvature"])) <= 1.0 / 1.8


def test_expanded_apron_admits_forward_only_transit():
    path = plan_forward_dubins_path(
        (0.0, 0.0, 0.0),
        (3.21, -0.30, math.pi),
        [(-5.5, -5.7), (5.5, -5.7), (5.5, 2.7), (-5.5, 2.7)],
        [],
    )
    assert path is not None
    assert path[0] == (0.0, 0.0, 0.0)
    assert path[-1] == (3.21, -0.30, math.pi)
    assert all(
        (end[0] - start[0]) * math.cos(start[2])
        + (end[1] - start[1]) * math.sin(start[2]) >= -1e-6
        for start, end in zip(path, path[1:])
    )


def test_reverse_dubins_transit_reaches_goal_without_a_cusp():
    path = plan_reverse_dubins_path(
        (0.0, 0.0, 0.0),
        (-4.26, -0.25, 0.0),
        [(-6.3, -6.2), (6.3, -6.2), (6.3, 2.7), (-6.3, 2.7)],
        [],
    )
    assert path is not None
    assert path[0] == (0.0, 0.0, 0.0)
    assert path[-1] == (-4.26, -0.25, 0.0)
    assert len(split_hybrid_path_by_direction(path)) == 1
    assert split_hybrid_path_by_direction(path)[0]["direction"] == "REVERSE"
    assert all(
        (end[0] - start[0]) * math.cos(start[2])
        + (end[1] - start[1]) * math.sin(start[2]) <= 1e-6
        for start, end in zip(path, path[1:])
    )


def test_top_swath_connector_fits_calibrated_north_turning_apron():
    connector = plan_ackermann_connector(
        "connector-top",
        (3.52, -1.35),
        0.0,
        (4.88, -2.45),
        math.pi,
        [(-7.8, -6.5), (7.8, -6.5), (7.8, 3.5), (-7.8, 3.5)],
        [],
        "swath-0",
        "swath-1",
    )
    assert connector is not None
    assert [item.kind.value for item in connector] == ["FORWARD"]
    assert connector[0].metadata["connector_class"] == "FORWARD_DUBINS_TURN"
