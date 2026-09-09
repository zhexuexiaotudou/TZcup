import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "inspect_live_mapping_snapshot", ROOT / "scripts" / "inspect_live_mapping_snapshot.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _manifest(path: Path) -> None:
    path.write_text(json.dumps({
        "profile": "formal",
        "episode_id": "snapshot-test",
        "map_id": "snapshot-map",
        "vehicle_start_pose_source_world": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "field": {
            "width_m": 200.0,
            "height_m": 100.0,
            "area_m2": 20000.0,
            "physical_boundary_walls": False,
            "geofence_frame": "source_world",
            "geofence_polygon_m": [[-2.0, -50.0], [198.0, -50.0], [198.0, 50.0], [-2.0, 50.0]],
            "source_world_geofence": {"frame_id": "source_world", "polygon_m": [[-2.0, -50.0], [198.0, -50.0], [198.0, 50.0], [-2.0, 50.0]]},
            "localization_map_geofence": {"frame_id": "map", "polygon_m": [[-2.0, -50.0], [198.0, -50.0], [198.0, 50.0], [-2.0, 50.0]]},
            "legacy_geofence": {"field": "geofence_polygon_m", "frame_id": "source_world", "deprecation": "test"},
        },
    }), encoding="utf-8")


def _telemetry(
    path: Path,
    pose: list[float],
    *,
    downsample_stride: int | None = None,
) -> None:
    grid = {
        "width": 5, "height": 5, "resolution": 1.0,
        "origin": [-2.0, -2.0], "data": [0] * 25,
    }
    if downsample_stride is not None:
        grid["downsample_stride"] = downsample_stride
        grid["source_dimensions"] = [15, 15]
    path.write_text(json.dumps({
        "vehicle": {"estimated_pose_map": pose},
        "visualization": {"occupancy_grid": grid},
    }), encoding="utf-8")


def test_snapshot_reports_compact_counts_and_actionable_no_frontier_causes(tmp_path, capsys):
    telemetry = tmp_path / "dashboard_telemetry.json"
    manifest = tmp_path / "episode_manifest.json"
    _telemetry(telemetry, [10.0, 10.0, 0.0])
    _manifest(manifest)

    report = MODULE.inspect_snapshot(telemetry, manifest)

    assert report["status"] == "NO_FRONTIER_GOAL"
    assert report["frontier_goal_map"] is None
    assert report["map"]["cells"] == {"known": 25, "free": 25, "occupied": 0, "unknown": 0}
    assert report["robot_inside_raster"] is False
    assert "robot_outside_occupancy_raster" in report["actionable_causes"]
    assert "no_reachable_frontier_adjacent_safe_candidate" in report["actionable_causes"]

    assert MODULE.main([str(telemetry), str(manifest)]) == 0
    assert json.loads(capsys.readouterr().out) == report


def test_snapshot_rejects_downsampled_dashboard_projection_as_production_input(tmp_path):
    telemetry = tmp_path / "dashboard_telemetry.json"
    manifest = tmp_path / "episode_manifest.json"
    _telemetry(telemetry, [0.0, 0.0, 0.0], downsample_stride=3)
    _manifest(manifest)

    report = MODULE.inspect_snapshot(telemetry, manifest)

    assert report["status"] == "DASHBOARD_PROJECTION_NOT_PRODUCTION_INPUT"
    assert report["frontier_goal_map"] is None
    assert report["actionable_causes"] == [
        "dashboard_projection_not_production_input"
    ]
    assert report["input"]["dashboard_projection"] == {
        "downsample_stride": 3,
        "source_dimensions": [15, 15],
    }
