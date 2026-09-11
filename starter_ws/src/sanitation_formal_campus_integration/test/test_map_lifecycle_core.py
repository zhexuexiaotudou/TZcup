import hashlib
import json
from pathlib import Path

import pytest
import yaml

from sanitation_formal_campus_integration.map_lifecycle_core import (
    CampusMapContract,
    MAXIMUM_SAVED_MAP_RESOLUTION_M,
    MapLifecycleError,
    assess_grid_observation,
    assess_saved_pgm_observation,
    goal_tangent_yaw,
    hard_restart_record_valid,
    load_campus_map_contract,
    materialize_saved_map_coverage_geometry,
    prepare_public_lifecycle_artifacts,
    read_binary_pgm,
    _mask_loops,
    _signed_area,
    _inside,
    _rectangles_from_mask,
    select_frontier_goal,
    validate_saved_map_artifact,
)


@pytest.mark.parametrize("separator", [b" ", b"\t", b"\n", b"\r\n"])
@pytest.mark.parametrize("pixel", [b"\x09", b"\x0a", b"\x20"])
def test_binary_pgm_preserves_whitespace_valued_first_pixel(tmp_path, separator, pixel):
    path = tmp_path / "map.pgm"
    path.write_bytes(b"P5\r\n1 1\r\n255" + separator + pixel)
    assert read_binary_pgm(path) == (1, 1, [bytearray(pixel)])


def test_binary_pgm_flips_top_first_raster_into_map_row_order(tmp_path):
    path = tmp_path / "map.pgm"
    path.write_bytes(b"P5\n2 2\n255\n" + bytes([1, 2, 3, 4]))
    assert read_binary_pgm(path) == (2, 2, [bytearray([3, 4]), bytearray([1, 2])])


def test_saved_pgm_observation_uses_trinary_thresholds_and_map_row_order():
    metadata = {
        "resolution": 0.1,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "free_thresh": 0.20,
        "occupied_thresh": 0.80,
        "mode": "trinary",
    }
    # PGM order is top row then bottom row. Values exactly at each threshold
    # are unknown; 255 is free and 0 is occupied.
    pgm = b"P5\n2 2\n255\n" + bytes([204, 51, 255, 0])
    report = assess_saved_pgm_observation(
        metadata, pgm, geofence=((0, 0), (0.2, 0), (0.2, 0.2), (0, 0.2)), threshold=0.5
    )
    assert report.observed_cells == 2
    assert report.field_cells == 4
    assert report.observed_fraction == 0.5


@pytest.mark.parametrize("metadata", [
    {"resolution": 0.1, "origin": [0, 0], "mode": "trinary"},
    {"resolution": 0.1, "origin": [0, 0, 0], "mode": "scale"},
    {"resolution": 0.1, "origin": [0, 0, 0], "negate": False, "mode": "trinary"},
    {"resolution": 0.1, "origin": [0, 0, 0.1], "mode": "trinary"},
])
def test_saved_pgm_observation_rejects_nonformal_metadata(metadata):
    with pytest.raises(MapLifecycleError):
        assess_saved_pgm_observation(
            metadata, b"P5\n1 1\n255\n\xff", geofence=((0, 0), (1, 0), (1, 1), (0, 1)), threshold=0.95
        )


def test_occupancy_union_uses_one_boundary_not_adjacent_keepout_boxes():
    loops = _mask_loops([[True, True]], resolution=1.0, origin_x=0.0, origin_y=0.0)
    assert len(loops) == 1
    assert [0.0, 0.0] in loops[0] and [2.0, 1.0] in loops[0]


def test_reachable_contour_separates_outer_ring_from_internal_obstacle_hole():
    loops = _mask_loops(
        [[True, True, True], [True, False, True], [True, True, True]],
        resolution=1.0, origin_x=0.0, origin_y=0.0,
    )
    assert len([loop for loop in loops if _signed_area(loop) > 0.0]) == 1
    assert len([loop for loop in loops if _signed_area(loop) < 0.0]) == 1


def test_disconnected_reachable_islands_have_multiple_outer_rings_and_are_rejected_by_contract():
    loops = _mask_loops([[True, False, True]], resolution=1.0, origin_x=0.0, origin_y=0.0)
    assert len([loop for loop in loops if _signed_area(loop) > 0.0]) == 2


def test_boundary_obstacle_moves_the_planning_outer_ring_inside_the_geofence_edge():
    loops = _mask_loops([[False, True, True]], resolution=1.0, origin_x=0.0, origin_y=0.0)
    outer = next(loop for loop in loops if _signed_area(loop) > 0.0)
    assert min(point[0] for point in outer) == 1.0


def test_checkerboard_corner_touching_rings_are_rejected():
    with pytest.raises(MapLifecycleError, match="self-intersects|rings touch"):
        _mask_loops([[True, False], [False, True]], resolution=1.0, origin_x=0.0, origin_y=0.0)


def test_collision_keepout_rectangles_do_not_include_a_reachable_cell_center():
    keepouts = _rectangles_from_mask([[True, False]], resolution=1.0, origin_x=0.0, origin_y=0.0)
    assert not any(_inside(1.5, 0.5, tuple(map(tuple, polygon))) for polygon in keepouts)


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
    )
    assert goal is not None
    column, row = int(goal[0] / 0.1), int(goal[1] / 0.1)
    assert data[row * 7 + column] == 0


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


def _materialized_saved_map(root: Path) -> tuple[CampusMapContract, dict[str, Path]]:
    contract = CampusMapContract(
        episode_id="fixture-episode",
        map_id="fixture-map",
        field_area_m2=25.0,
        geofence=((-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)),
        source_geofence=((-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)),
        fixed_start_source=(0.0, 0.0, 0.0),
    )
    prepare_public_lifecycle_artifacts(contract, root)
    width = height = 60
    (root / "occupancy.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes([255]) * (width * height)
    )
    (root / "occupancy.yaml").write_text(yaml.safe_dump({
        "image": "occupancy.pgm",
        "resolution": 0.1,
        "origin": [-3.0, -3.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "mode": "trinary",
    }), encoding="utf-8")
    artifacts = materialize_saved_map_coverage_geometry(root, contract)
    hashes = {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest()
        for name in {"occupancy.yaml", "occupancy.pgm", *(
            "coverage_free_space.pgm",
            "coverage_geometry.yaml",
            "mission_geometry.yaml",
            "materialization_contract.yaml",
            "geofence_keepout.yaml",
            "geofence_keepout.pgm",
            "neutral_speed.yaml",
            "neutral_speed.pgm",
        )}
    }
    (root / "map_lifecycle_manifest.json").write_text(json.dumps({
        "schema_version": 1,
        "status": "ready_for_localization_cleaning",
        "episode_id": contract.episode_id,
        "map_id": contract.map_id,
        "occupancy_map": "occupancy.yaml",
        "observed_fraction": 1.0,
        "saved_pgm_observed_fraction": 1.0,
        "saved_pgm_observed_cells": 2500,
        "saved_pgm_field_cells": 2500,
        "quality_threshold": 0.95,
        "stable_gate_samples": 3,
        "fixed_start_verified": True,
        "gnss_mapping_reference_observed": True,
        "mapping_pose_source": "wheel_imu_ekf_lidar_scan_matching_gnss_consistency",
        "world_truth_used_for_control": False,
        "mapping_ignored_dirt": True,
        "sha256": hashes,
    }), encoding="utf-8")
    return contract, artifacts


def test_materialized_geometry_is_preapplied_and_fully_manifest_sealed(tmp_path):
    root = tmp_path / "maps"
    contract, artifacts = _materialized_saved_map(root)
    geometry = yaml.safe_load(artifacts["coverage_geometry"].read_bytes())
    mission = yaml.safe_load(artifacts["mission_geometry"].read_bytes())
    width, height, rows = read_binary_pgm(artifacts["coverage_free_space"])
    assert (width, height) == (60, 60)
    assert sum(pixel == 255 for row in rows for pixel in row) == geometry["reachable_cleanable_cells"]
    assert geometry["planning_clearance_preapplied"] is True
    assert mission["saved_occupancy_coverage"]["planning_clearance_preapplied"] is True
    assert validate_saved_map_artifact(root, contract)["status"] == "ready_for_localization_cleaning"


def test_cleaning_admission_recomputes_pgm_observation_after_a_resealed_map_change(tmp_path):
    root = tmp_path / "maps"
    contract, _ = _materialized_saved_map(root)
    image = bytearray((root / "occupancy.pgm").read_bytes())
    image[len(b"P5\n60 60\n255\n") + 30 * 60 + 30] = 180  # trinary unknown
    (root / "occupancy.pgm").write_bytes(image)
    geometry_path = root / "coverage_geometry.yaml"
    geometry = yaml.safe_load(geometry_path.read_text(encoding="utf-8"))
    geometry["occupancy_image_sha256"] = hashlib.sha256(image).hexdigest()
    geometry_path.write_text(yaml.safe_dump(geometry), encoding="utf-8")
    mission_path = root / "mission_geometry.yaml"
    mission = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
    mission["saved_occupancy_coverage"]["sha256"] = hashlib.sha256(
        geometry_path.read_bytes()
    ).hexdigest()
    mission_path.write_text(yaml.safe_dump(mission), encoding="utf-8")
    manifest_path = root / "map_lifecycle_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["sha256"] = {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest()
        for name in manifest["sha256"]
    }
    # A forged/resealed manifest still reports the previous all-known PGM.
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(MapLifecycleError, match="PGM observation"):
        validate_saved_map_artifact(root, contract)


def test_saved_map_validator_uses_one_snapshot_per_sealed_artifact(tmp_path, monkeypatch):
    root = tmp_path / "maps"
    contract, _ = _materialized_saved_map(root)
    original = Path.read_bytes
    reads: dict[str, int] = {}

    def counted(path: Path) -> bytes:
        reads[path.name] = reads.get(path.name, 0) + 1
        return original(path)

    monkeypatch.setattr(Path, "read_bytes", counted)
    validate_saved_map_artifact(root, contract)
    assert reads["map_lifecycle_manifest.json"] == 1
    assert set(reads) == {
        "map_lifecycle_manifest.json",
        "occupancy.yaml",
        "occupancy.pgm",
        "coverage_free_space.pgm",
        "coverage_geometry.yaml",
        "mission_geometry.yaml",
        "materialization_contract.yaml",
        "geofence_keepout.yaml",
        "geofence_keepout.pgm",
        "neutral_speed.yaml",
        "neutral_speed.pgm",
    }
    assert all(count == 1 for count in reads.values())


def test_materialization_blocks_a_cell_exactly_at_free_threshold(tmp_path):
    root = tmp_path / "maps"
    contract = CampusMapContract(
        episode_id="threshold", map_id="threshold", field_area_m2=4.0,
        geofence=((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)),
        source_geofence=((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)),
        fixed_start_source=(0.0, 0.0, 0.0),
    )
    prepare_public_lifecycle_artifacts(contract, root)
    width = height = 30
    pixels = bytearray([255] * (width * height))
    pixels[(height - 1 - 15) * width + 15] = 254
    (root / "occupancy.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + pixels
    )
    (root / "occupancy.yaml").write_text(yaml.safe_dump({
        "image": "occupancy.pgm",
        "resolution": 0.1,
        "origin": [-1.5, -1.5, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 1.0 / 255.0,
        "mode": "trinary",
    }), encoding="utf-8")
    with pytest.raises(MapLifecycleError, match="fixed saved-map start"):
        materialize_saved_map_coverage_geometry(root, contract, obstacle_inflation_m=0.1)


def test_cleaning_admission_rejects_old_manifest_that_lacks_pgm_observation_proof(tmp_path):
    contract = load_campus_map_contract(_manifest(tmp_path / "episode.json"))
    root = tmp_path / "maps"
    prepare_public_lifecycle_artifacts(contract, root)
    (root / "occupancy.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (root / "occupancy.yaml").write_text(
        "image: occupancy.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    (root / "coverage_free_space.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    geometry = {
        "source": "saved_slam_occupancy_only",
        "occupancy_map": "occupancy.yaml",
        "occupancy_image": "occupancy.pgm",
        "occupancy_map_sha256": hashlib.sha256((root / "occupancy.yaml").read_bytes()).hexdigest(),
        "occupancy_image_sha256": hashlib.sha256((root / "occupancy.pgm").read_bytes()).hexdigest(),
        "free_space_map": "coverage_free_space.pgm",
        "free_space_map_sha256": hashlib.sha256((root / "coverage_free_space.pgm").read_bytes()).hexdigest(),
        "world_truth_used_for_product_map": False,
        "planning_polygons": [[[0, 0], [1, 0], [1, 1], [0, 1]]],
        "planning_outer_polygon": [[0, 0], [1, 0], [1, 1], [0, 1]],
        "planning_hole_polygons": [],
        "keepout_polygons": [],
        "reachable_cleanable_cells": 1,
        "obstacle_inflation_m": 1.70,
        "planning_clearance_m": 1.70,
        "planning_clearance_preapplied": True,
        "planning_clearance_derivation": "test",
    }
    (root / "coverage_geometry.yaml").write_text(yaml.safe_dump(geometry), encoding="utf-8")
    mission = yaml.safe_load((root / "mission_geometry.yaml").read_text(encoding="utf-8"))
    mission.update({"keepout_polygons": [], "exclusion_polygons": [], "headland": {"enabled": True, "width_m": 1.70}, "saved_occupancy_coverage": {
        "geometry": "coverage_geometry.yaml", "free_space_map": "coverage_free_space.pgm",
        "source": "saved_slam_occupancy_only", "planning_clearance_preapplied": True,
        "sha256": hashlib.sha256((root / "coverage_geometry.yaml").read_bytes()).hexdigest(),
    }})
    (root / "mission_geometry.yaml").write_text(yaml.safe_dump(mission), encoding="utf-8")
    files = (
        "occupancy.pgm",
        "occupancy.yaml",
        "mission_geometry.yaml",
        "materialization_contract.yaml",
        "geofence_keepout.yaml",
        "geofence_keepout.pgm",
        "neutral_speed.yaml",
        "neutral_speed.pgm",
        "coverage_geometry.yaml",
        "coverage_free_space.pgm",
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
    with pytest.raises(MapLifecycleError, match="PGM observation"):
        validate_saved_map_artifact(root, contract)
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
