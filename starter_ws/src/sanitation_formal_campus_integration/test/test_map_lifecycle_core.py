import hashlib
import json
import math
from pathlib import Path

import pytest
import yaml

from sanitation_formal_campus_integration.map_lifecycle_core import (
    MAXIMUM_SAVED_MAP_RESOLUTION_M,
    MapLifecycleError,
    assess_grid_observation,
    goal_tangent_yaw,
    hard_restart_record_valid,
    load_campus_map_contract,
    prepare_public_lifecycle_artifacts,
    select_frontier_goal,
    validate_saved_map_artifact,
)


def test_hard_restart_record_binds_pids_exit_order_and_hashes(tmp_path):
    root = tmp_path / "map"
    root.mkdir()
    for name, content in (
        ("map_lifecycle_manifest.json", b"manifest"),
        ("mapping_runtime.json", b"runtime"),
        ("mapping_handoff_record.json", b"handoff"),
    ):
        (root / name).write_bytes(content)
    record = {
        "schema_version": 2,
        "mapping_stopped_before_cleaning": True,
        "mapping_process_count_before_cleaning": 0,
        "mapping_pid_alive_count_before_cleaning": 0,
        "mapping_runner_exit_code": 0,
        "restart_type": "separate_process_hard_restart",
        "mapping_completion_wall_time": "2026-08-28T10:00:00+00:00",
        "mapping_cleanup_wall_time": "2026-08-28T10:00:01+00:00",
        "cleaning_start_wall_time": "2026-08-28T10:00:02+00:00",
        "mapping_runner_pid": 101,
        "mapping_launch_pid": 102,
        "mapping_collector_pid": 103,
        "cleaning_runner_pid": 201,
        "cleaning_launch_pid": 202,
        "map_lifecycle_manifest_sha256": hashlib.sha256(b"manifest").hexdigest(),
        "mapping_runtime_sha256": hashlib.sha256(b"runtime").hexdigest(),
        "mapping_handoff_record_sha256": hashlib.sha256(b"handoff").hexdigest(),
    }
    assert hard_restart_record_valid(record, root)
    for field, value in (
        ("mapping_runner_exit_code", 1),
        ("mapping_pid_alive_count_before_cleaning", 1),
        ("cleaning_launch_pid", 102),
        ("mapping_runtime_sha256", "0" * 64),
    ):
        candidate = dict(record)
        candidate[field] = value
        assert not hard_restart_record_valid(candidate, root)


def _manifest(path: Path) -> Path:
    payload = {
        "profile": "formal",
        "episode_id": "formal-life-001",
        "map_id": "train-map-000",
        "field": {
            "width_m": 200.0,
            "height_m": 100.0,
            "area_m2": 20000.0,
            "physical_boundary_walls": False,
            "geofence_frame": "map",
            "geofence_polygon_m": [
                [-100.0, -50.0],
                [100.0, -50.0],
                [100.0, 50.0],
                [-100.0, 50.0],
            ],
        },
        "vehicle_start_pose_map": {
            "x_m": -98.0,
            "y_m": 0.0,
            "yaw_rad": 0.0,
        },
    }
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_contract_converts_fixed_source_start_to_local_slam_frame(tmp_path):
    contract = load_campus_map_contract(_manifest(tmp_path / "episode.json"))
    assert contract.field_area_m2 == 20000.0
    assert contract.fixed_start_source == (-98.0, 0.0, 0.0)
    assert contract.geofence == (
        (-2.0, -50.0),
        (198.0, -50.0),
        (198.0, 50.0),
        (-2.0, 50.0),
    )


def test_explicit_geofences_apply_the_fixed_start_transform_exactly_once(tmp_path):
    path = _manifest(tmp_path / "episode.json")
    payload = json.loads(path.read_text(encoding="utf-8"))
    source = payload["field"]["geofence_polygon_m"]
    payload["field"].update(
        source_world_geofence={"frame_id": "source_world", "polygon_m": source},
        localization_map_geofence={
            "frame_id": "map",
            "polygon_m": [[-2.0, -50.0], [198.0, -50.0], [198.0, 50.0], [-2.0, 50.0]],
            "transform": "source_world_to_localization_map_at_fixed_start",
        },
        legacy_geofence={"field": "geofence_polygon_m", "frame_id": "source_world", "deprecation": "use explicit fields"},
        geofence_frame="source_world",
    )
    payload["vehicle_start_pose_source_world"] = payload["vehicle_start_pose_map"]
    payload["vehicle_start_pose_localization_map"] = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    path.write_text(json.dumps(payload), encoding="utf-8")
    contract = load_campus_map_contract(path)
    assert contract.source_geofence == tuple(tuple(point) for point in source)
    assert contract.geofence[0] == pytest.approx((-2.0, -50.0))

    payload["field"]["localization_map_geofence"]["polygon_m"] = source
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(MapLifecycleError, match="exactly once"):
        load_campus_map_contract(path)


def test_legacy_geofence_rejects_missing_or_unknown_frame(tmp_path):
    path = _manifest(tmp_path / "episode.json")
    payload = json.loads(path.read_text(encoding="utf-8"))
    for invalid_frame in (None, "odom"):
        if invalid_frame is None:
            payload["field"].pop("geofence_frame", None)
        else:
            payload["field"]["geofence_frame"] = invalid_frame
        path.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(MapLifecycleError, match="legacy geofence frame"):
            load_campus_map_contract(path)


def test_contract_rejects_nonbaseline_dimensions(tmp_path):
    path = _manifest(tmp_path / "episode.json")
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["field"]["width_m"] = 100.0
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(MapLifecycleError, match="exactly 200 x 100"):
        load_campus_map_contract(path)


def test_observation_gate_requires_at_least_95_percent_known_cells():
    values = [0] * 95 + [-1] * 5
    report = assess_grid_observation(
        values,
        width=10,
        height=10,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
    )
    assert report.observed_cells == 95
    assert report.observed_fraction == pytest.approx(0.95)
    assert report.passed is True
    values[0] = -1
    assert assess_grid_observation(
        values,
        width=10,
        height=10,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
    ).passed is False


def test_small_complete_grid_cannot_pass_whole_geofence_gate():
    report = assess_grid_observation(
        [0] * 100,
        width=10,
        height=10,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)),
    )
    assert report.observed_cells == 100
    assert report.field_cells == 200
    assert report.observed_fraction == pytest.approx(0.5)
    assert report.passed is False


def test_frontier_goal_is_known_free_inside_geofence():
    # Known 3x3 island in an otherwise unknown 7x7 map.
    data = [-1] * 49
    for row in range(2, 5):
        for column in range(2, 5):
            data[row * 7 + column] = 0
    baseline = select_frontier_goal(
        data,
        width=7,
        height=7,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (0.7, 0.0), (0.7, 0.7), (0.0, 0.7)),
        robot_x=0.35,
        robot_y=0.35,
        sample_spacing_m=0.1,
        clearance_m=0.0,
        frontier_standoff_m=0.2,
        seed_max_offset_m=0.2,
        min_goal_distance_m=0.0,
    )
    diagnostics: dict[str, object] = {}
    goal = select_frontier_goal(
        data,
        width=7,
        height=7,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (0.7, 0.0), (0.7, 0.7), (0.0, 0.7)),
        robot_x=0.35,
        robot_y=0.35,
        sample_spacing_m=0.1,
        clearance_m=0.0,
        frontier_standoff_m=0.2,
        seed_max_offset_m=0.2,
        min_goal_distance_m=0.0,
        diagnostics=diagnostics,
    )
    assert goal is not None
    assert goal == baseline
    column, row = int(goal[0] / 0.1), int(goal[1] / 0.1)
    assert data[row * 7 + column] == 0
    assert diagnostics == {
        "source_dimensions": [7, 7],
        "source_resolution_m": 0.1,
        "seed_offset_m": pytest.approx(0.0),
        "seed_safe": True,
        "seed_touches_boundary": False,
        "anchor_found": True,
        "raw_frontier_count": 8,
        "candidate_count": 9,
        "rejection_reason": None,
    }


def test_axis_aligned_observation_fast_path_counts_only_field_cell_centers():
    data = [-1] * (40 * 30)
    for row in range(5, 25):
        for column in range(5, 30):
            data[row * 40 + column] = 0
    data[12 * 40 + 14] = -1

    quality = assess_grid_observation(
        data,
        width=40,
        height=30,
        resolution=0.1,
        origin_x=-0.5,
        origin_y=-0.5,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)),
        threshold=0.5,
    )

    assert quality.field_cells == 400
    assert quality.observed_cells == 399
    assert quality.observed_fraction == pytest.approx(399 / 400)


def test_frontier_goal_never_crosses_to_a_disconnected_free_island():
    width, height, resolution = 30, 15, 0.1
    data = [-1] * (width * height)
    for row in range(2, 13):
        for column in range(2, 11):
            data[row * width + column] = 0
        for column in range(20, 28):
            data[row * width + column] = 0

    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (3.0, 0.0), (3.0, 1.5), (0.0, 1.5)),
        robot_x=0.55,
        robot_y=0.75,
        sample_spacing_m=0.1,
        clearance_m=0.2,
        frontier_standoff_m=0.4,
        seed_max_offset_m=0.2,
        min_goal_distance_m=0.2,
    )

    assert goal is not None
    assert goal[0] < 1.1


def test_frontier_goal_never_crosses_a_vehicle_width_blocking_corridor():
    width, height, resolution = 30, 15, 0.1
    data = [-1] * (width * height)
    for row in range(2, 13):
        for column in range(2, 11):
            data[row * width + column] = 0
        for column in range(20, 28):
            data[row * width + column] = 0
    for column in range(11, 20):
        data[7 * width + column] = 0

    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (3.0, 0.0), (3.0, 1.5), (0.0, 1.5)),
        robot_x=0.55,
        robot_y=0.75,
        sample_spacing_m=0.1,
        clearance_m=0.2,
        frontier_standoff_m=0.4,
        seed_max_offset_m=0.2,
        min_goal_distance_m=0.2,
    )

    assert goal is not None
    assert goal[0] < 1.1


def test_frontier_goal_does_not_escape_an_internal_narrow_corridor():
    width, height, resolution = 30, 15, 0.1
    data = [-1] * (width * height)
    for row in range(2, 13):
        for column in range(2, 11):
            data[row * width + column] = 0
        for column in range(20, 28):
            data[row * width + column] = 0
    for column in range(11, 20):
        data[7 * width + column] = 0

    diagnostics: dict[str, object] = {}
    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (3.0, 0.0), (3.0, 1.5), (0.0, 1.5)),
        robot_x=1.55,
        robot_y=0.75,
        sample_spacing_m=0.1,
        clearance_m=0.2,
        frontier_standoff_m=0.4,
        seed_max_offset_m=0.2,
        min_goal_distance_m=0.2,
        diagnostics=diagnostics,
    )

    assert goal is None
    assert diagnostics["source_dimensions"] == [30, 15]
    assert diagnostics["source_resolution_m"] == pytest.approx(0.1)
    assert diagnostics["seed_offset_m"] == pytest.approx(0.0)
    assert diagnostics["seed_safe"] is False
    assert diagnostics["seed_touches_boundary"] is False
    assert diagnostics["anchor_found"] is False
    assert diagnostics["raw_frontier_count"] == 0
    assert diagnostics["candidate_count"] == 0
    assert diagnostics["rejection_reason"] == "no_footprint_safe_bootstrap_anchor"


def test_frontier_goal_bootstraps_past_lidar_unknown_cells_under_robot_body():
    width, height, resolution = 60, 40, 0.1
    data = [-1] * (width * height)
    for row in range(4, 36):
        for column in range(5, 55):
            data[row * width + column] = 0
    # The center cell is known-free, but one unobservable under-body cell lies
    # on the seed clearance edge. Moving the anchor one cell away makes the
    # complete clearance window known-free without crossing unknown space.
    robot_row, robot_column = 20, 30
    data[(robot_row + 3) * width + robot_column] = -1

    diagnostics: dict[str, object] = {}
    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)),
        robot_x=(robot_column + 0.5) * resolution,
        robot_y=(robot_row + 0.5) * resolution,
        sample_spacing_m=0.1,
        clearance_m=0.2,
        frontier_standoff_m=0.4,
        seed_max_offset_m=0.2,
        min_goal_distance_m=0.2,
        diagnostics=diagnostics,
    )

    assert goal is not None
    assert diagnostics["seed_safe"] is False
    assert diagnostics["seed_touches_boundary"] is False
    assert diagnostics["anchor_found"] is True
    assert diagnostics["candidate_count"] > 0
    assert diagnostics["rejection_reason"] is None


def test_frontier_goal_does_not_expand_boundary_bootstrap_through_narrow_corridor():
    """A map-edge exception must not become a long unfit-corridor traversal."""
    width = height = 40
    resolution = 0.1
    data = [-1] * (width * height)
    # The nearest free seed is at the left map edge.  A one-cell corridor is
    # deliberately much narrower than the 0.2 m footprint envelope.
    for column in range(7):
        data[20 * width + column] = 0
    for row in range(3, 37):
        for column in range(7, 37):
            data[row * width + column] = 0

    diagnostics: dict[str, object] = {}
    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)),
        robot_x=0.05,
        robot_y=2.05,
        sample_spacing_m=0.1,
        clearance_m=0.2,
        frontier_standoff_m=0.4,
        seed_max_offset_m=0.75,
        min_goal_distance_m=0.2,
        diagnostics=diagnostics,
    )

    assert goal is None
    assert diagnostics["seed_touches_boundary"] is True
    assert diagnostics["anchor_found"] is False
    assert diagnostics["rejection_reason"] == "no_footprint_safe_bootstrap_anchor"


def test_frontier_goal_uses_a_bounded_nearest_free_seed_outside_the_map():
    width, height, resolution = 40, 20, 0.1
    data = [-1] * (width * height)
    for row in range(4, 16):
        for column in range(5, 35):
            data[row * width + column] = 0
    common = dict(
        data=data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)),
        sample_spacing_m=0.1,
        clearance_m=0.0,
        frontier_standoff_m=0.2,
        min_goal_distance_m=0.2,
    )

    assert select_frontier_goal(
        robot_x=0.40,
        robot_y=0.45,
        seed_max_offset_m=0.20,
        **common,
    ) is not None
    assert select_frontier_goal(
        robot_x=-0.30,
        robot_y=0.45,
        seed_max_offset_m=0.75,
        **common,
    ) is None


def test_frontier_goal_recovers_when_robot_is_just_outside_a_growing_map():
    width = height = 40
    resolution = 0.1
    data = [-1] * (width * height)
    for row in range(0, 31):
        for column in range(5, 36):
            data[row * width + column] = 0

    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((-1.0, -1.0), (5.0, -1.0), (5.0, 5.0), (-1.0, 5.0)),
        robot_x=2.0,
        robot_y=-0.04,
        sample_spacing_m=0.1,
        clearance_m=0.95,
        frontier_standoff_m=1.2,
        seed_max_offset_m=0.75,
        min_goal_distance_m=1.0,
    )

    assert goal is not None
    assert 1.05 <= goal[0] <= 3.55
    assert 1.05 <= goal[1] <= 2.05


@pytest.mark.parametrize("field", ("origin_x", "origin_y", "origin_yaw", "robot_x", "robot_y"))
def test_frontier_goal_rejects_non_finite_pose_inputs(field):
    kwargs = dict(
        data=[0] * 25,
        width=5,
        height=5,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5)),
        robot_x=0.25,
        robot_y=0.25,
    )
    kwargs[field] = math.nan
    with pytest.raises(MapLifecycleError, match=f"{field} must be finite"):
        select_frontier_goal(**kwargs)


def test_frontier_goal_is_stood_off_by_the_full_footprint_clearance():
    width = height = 50
    resolution = 0.1
    data = [-1] * (width * height)
    for row in range(5, 45):
        for column in range(5, 45):
            data[row * width + column] = 0

    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)),
        robot_x=2.55,
        robot_y=2.55,
        sample_spacing_m=0.1,
        clearance_m=0.95,
        frontier_standoff_m=1.2,
        seed_max_offset_m=0.1,
        min_goal_distance_m=1.0,
    )

    assert goal is not None
    column, row = int(goal[0] / resolution), int(goal[1] / resolution)
    radius = math.ceil(0.95 / resolution)
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if (dr * resolution) ** 2 + (dc * resolution) ** 2 <= 0.95**2:
                assert data[(row + dr) * width + column + dc] == 0


def test_frontier_clearance_includes_the_raster_cell_half_diagonal():
    width = height = 30
    resolution = 0.1
    data = [100] * (width * height)
    for row in range(3, 27):
        data[row * width + 4] = -1
        for column in range(5, 26):
            data[row * width + column] = 0

    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)),
        robot_x=1.55,
        robot_y=1.55,
        sample_spacing_m=0.1,
        clearance_m=0.95,
        frontier_standoff_m=1.2,
        seed_max_offset_m=0.2,
        min_goal_distance_m=0.0,
    )

    assert goal is not None
    assert goal[0] >= 1.55 - 1e-9


def test_frontier_vehicle_clearance_stays_inside_the_geofence():
    width = 40
    height = 20
    resolution = 0.1
    data = [0] * (width * height)
    for row in range(height):
        data[row * width + 35] = -1

    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 1.0), (4.0, 1.0), (4.0, 2.0), (0.0, 2.0)),
        robot_x=2.0,
        robot_y=1.45,
        sample_spacing_m=0.1,
        clearance_m=0.3,
        frontier_standoff_m=0.5,
        seed_max_offset_m=0.75,
        min_goal_distance_m=0.2,
    )

    assert goal is not None
    assert goal[1] >= 1.45 - 1e-9


def test_frontier_selection_prefers_nearest_geodesic_goal_and_honors_history():
    width, height, resolution = 30, 7, 0.1
    data = [100] * (width * height)
    for row in range(2, 5):
        for column in range(2, 28):
            data[row * width + column] = 0
        data[row * width + 1] = -1
        data[row * width + 28] = -1
    kwargs = dict(
        data=data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        geofence=((0.0, 0.0), (3.0, 0.0), (3.0, 0.7), (0.0, 0.7)),
        robot_x=0.55,
        robot_y=0.35,
        sample_spacing_m=0.1,
        clearance_m=0.0,
        frontier_standoff_m=0.0,
        seed_max_offset_m=0.1,
        min_goal_distance_m=0.1,
    )

    nearest = select_frontier_goal(**kwargs)
    assert nearest is not None and nearest[0] < 0.5
    alternate = select_frontier_goal(previous_goals=(nearest,), **kwargs)
    assert alternate is not None and alternate[0] > 2.5


def test_frontier_seed_and_goal_respect_a_rotated_map_origin():
    width, height, resolution = 30, 7, 0.1
    data = [100] * (width * height)
    for row in range(2, 5):
        for column in range(2, 28):
            data[row * width + column] = 0
        data[row * width + 1] = -1
        data[row * width + 28] = -1

    goal = select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=10.0,
        origin_y=-2.0,
        origin_yaw=math.pi / 2.0,
        geofence=((9.0, -2.0), (10.0, -2.0), (10.0, 1.0), (9.0, 1.0)),
        robot_x=9.65,
        robot_y=-1.45,
        sample_spacing_m=0.1,
        clearance_m=0.0,
        frontier_standoff_m=0.0,
        seed_max_offset_m=0.1,
        min_goal_distance_m=0.1,
    )

    assert goal == pytest.approx((9.65, -1.75))


@pytest.mark.parametrize(
    ("target", "expected"),
    (
        ((13.0, -2.0), 0.0),
        ((10.0, 1.0), 0.5 * 3.141592653589793),
        ((7.0, -2.0), 3.141592653589793),
        ((10.0, -5.0), -0.5 * 3.141592653589793),
        ((13.0, 1.0), 0.25 * 3.141592653589793),
        ((7.0, 1.0), 0.75 * 3.141592653589793),
        ((7.0, -5.0), -0.75 * 3.141592653589793),
        ((13.0, -5.0), -0.25 * 3.141592653589793),
    ),
)
def test_goal_tangent_yaw_uses_map_pose_after_nonzero_map_to_odom(
    target, expected
):
    # (10, -2) is deliberately not the odom-frame origin: it represents the
    # map-frame robot position after a nonidentity map->odom transform.
    assert goal_tangent_yaw(10.0, -2.0, *target) == pytest.approx(expected)


def test_goal_tangent_yaw_rejects_zero_length_direction():
    with pytest.raises(MapLifecycleError, match="distinct target"):
        goal_tangent_yaw(1.0, 2.0, 1.0, 2.0)


def test_support_artifacts_contain_geofence_but_no_object_truth(tmp_path):
    contract = load_campus_map_contract(_manifest(tmp_path / "episode.json"))
    artifacts = prepare_public_lifecycle_artifacts(contract, tmp_path / "maps")
    materialization = yaml.safe_load(
        artifacts["materialization_contract"].read_text(encoding="utf-8")
    )
    mission = yaml.safe_load(artifacts["mission_geometry"].read_text(encoding="utf-8"))
    assert materialization["map_source"] == "slam_toolbox_lidar_odometry"
    assert materialization["world_geometry_used_for_product_map"] is False
    assert materialization["mapping_ignores_dirt"] is True
    assert materialization["resolution_contract"] == {
        "static_materializer": {"value_source": "formal_campus.launch.py:map_resolution", "purpose": "public_world_static_collision_raster"},
        "lifecycle_support_mask": {"resolution_m": 0.25, "value_source": "prepare_public_lifecycle_artifacts(resolution)", "purpose": "public_geofence_support_mask"},
        "slam_occupancy": {"value_source": "saved_map_metadata", "maximum_accepted_resolution_m": MAXIMUM_SAVED_MAP_RESOLUTION_M, "purpose": "runtime_lidar_slam_occupancy"},
        "coverage_planning": {"value_source": "ProductCoverageTelemetry(raster_resolution_m)", "purpose": "saved_map_coverage_raster_planning"},
    }
    assert mission["keepout_polygons"] == []
    assert mission["planning_kinematic_constraint"] == (
        "curvature_limited_reference_path_for_skid_steer"
    )
    assert mission["kinematic_model"] == "four_wheel_skid_steer"
    assert mission["physical_steering_claim"] is False


def test_slam_resolution_contract_uses_the_validator_limit():
    with pytest.raises(MapLifecycleError, match="formal SLAM resolution"):
        assess_grid_observation(
            [0],
            width=1,
            height=1,
            resolution=MAXIMUM_SAVED_MAP_RESOLUTION_M + 1e-6,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            geofence=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
        )


def test_cleaning_admission_requires_hash_valid_saved_map(tmp_path):
    contract = load_campus_map_contract(_manifest(tmp_path / "episode.json"))
    root = tmp_path / "maps"
    prepare_public_lifecycle_artifacts(contract, root)
    (root / "occupancy.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (root / "occupancy.yaml").write_text(
        "image: occupancy.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    files = (
        "occupancy.pgm",
        "occupancy.yaml",
        "mission_geometry.yaml",
        "materialization_contract.yaml",
        "geofence_keepout.yaml",
        "geofence_keepout.pgm",
        "neutral_speed.yaml",
        "neutral_speed.pgm",
    )
    hashes = {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files
    }
    manifest = {
        "schema_version": 1,
        "status": "ready_for_localization_cleaning",
        "episode_id": contract.episode_id,
        "map_id": contract.map_id,
        "occupancy_map": "occupancy.yaml",
        "observed_fraction": 0.95,
        "quality_threshold": 0.95,
        "stable_gate_samples": 3,
        "fixed_start_verified": True,
        "gnss_mapping_reference_observed": True,
        "mapping_pose_source": (
            "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"
        ),
        "world_truth_used_for_control": False,
        "mapping_ignored_dirt": True,
        "sha256": hashes,
    }
    (root / "map_lifecycle_manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    assert validate_saved_map_artifact(root, contract)["observed_fraction"] == 0.95
    (root / "occupancy.pgm").write_bytes(b"tampered")
    with pytest.raises(MapLifecycleError, match="integrity"):
        validate_saved_map_artifact(root, contract)


def test_cleaning_admission_rejects_partial_or_traversing_hash_seal(tmp_path):
    contract = load_campus_map_contract(_manifest(tmp_path / "episode.json"))
    root = tmp_path / "maps"
    prepare_public_lifecycle_artifacts(contract, root)
    (root / "occupancy.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (root / "occupancy.yaml").write_text(
        "image: occupancy.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    common = {
        "schema_version": 1,
        "status": "ready_for_localization_cleaning",
        "episode_id": contract.episode_id,
        "map_id": contract.map_id,
        "occupancy_map": "occupancy.yaml",
        "observed_fraction": 0.95,
        "quality_threshold": 0.95,
        "stable_gate_samples": 3,
        "fixed_start_verified": True,
        "gnss_mapping_reference_observed": True,
        "mapping_pose_source": (
            "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"
        ),
        "world_truth_used_for_control": False,
        "mapping_ignored_dirt": True,
    }
    partial = dict(common)
    partial["sha256"] = {
        "occupancy.yaml": hashlib.sha256(
            (root / "occupancy.yaml").read_bytes()
        ).hexdigest(),
    }
    (root / "map_lifecycle_manifest.json").write_text(
        json.dumps(partial), encoding="utf-8"
    )
    with pytest.raises(MapLifecycleError, match="hash seal"):
        validate_saved_map_artifact(root, contract)

    traversing = dict(common)
    traversing["occupancy_map"] = "../occupancy.yaml"
    traversing["sha256"] = partial["sha256"]
    (root / "map_lifecycle_manifest.json").write_text(
        json.dumps(traversing), encoding="utf-8"
    )
    with pytest.raises(MapLifecycleError, match="basename"):
        validate_saved_map_artifact(root, contract)
