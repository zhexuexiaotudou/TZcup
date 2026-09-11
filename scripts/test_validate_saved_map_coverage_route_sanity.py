import importlib.util
import json
import sys
from pathlib import Path

from sanitation_formal_campus_integration.saved_map_coverage_core import SavedMapCoverageGeometry

SCRIPT = Path(__file__).with_name("validate_saved_map_coverage_route_sanity.py")
SPEC = importlib.util.spec_from_file_location("route_sanity", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _geometry(cells):
    return SavedMapCoverageGeometry(
        outer_polygon=((0, 0), (2, 0), (2, 2), (0, 2)),
        planning_outer_polygon=((0, 0), (2, 0), (2, 2), (0, 2)),
        planning_hole_polygons=(), keepout_polygons=(), free_cells=frozenset(cells),
        raster_resolution_m=0.1, origin_x=0.0, origin_y=0.0,
        planning_clearance_m=0.4, sha256="a" * 64,
    )


def test_route_sanity_reports_safe_overlap_and_turn_continuity(tmp_path, monkeypatch):
    monkeypatch.setattr(MODULE, "load_campus_map_contract", lambda _: object())
    monkeypatch.setattr(MODULE, "validate_saved_map_cleaning_consumer_bundle", lambda *_: {})
    monkeypatch.setattr(MODULE, "load_product_mission_geometry", lambda _: _geometry(
        (column, row) for row in range(20) for column in range(20)
    ))
    output = tmp_path / "route.json"
    monkeypatch.setattr(sys, "argv", ["route", "--map-root", str(tmp_path), "--episode-manifest", str(tmp_path / "episode.json"), "--output", str(output)])
    assert MODULE.main() == 0
    report = json.loads(output.read_text())
    assert report["operation_width_m"] == 1.32
    assert report["realized_lane_spacing_m"] <= 1.056
    assert report["disconnected_turn_rows"] == []
    assert report["candidate_lanes_summary_only"] is True
    assert report["planner_path_authority"] == "opennav_coverage_compute_coverage_path"


def test_route_sanity_blocks_disconnected_adjacent_lanes(tmp_path, monkeypatch):
    monkeypatch.setattr(MODULE, "load_campus_map_contract", lambda _: object())
    monkeypatch.setattr(MODULE, "validate_saved_map_cleaning_consumer_bundle", lambda *_: {})
    monkeypatch.setattr(MODULE, "load_product_mission_geometry", lambda _: _geometry(
        [(column, 0) for column in range(14)] + [(column, 11) for column in range(20, 34)]
    ))
    output = tmp_path / "route.json"
    monkeypatch.setattr(sys, "argv", ["route", "--map-root", str(tmp_path), "--episode-manifest", str(tmp_path / "episode.json"), "--output", str(output)])
    assert MODULE.main() == 2
    assert "adjacent_coverage_lanes_have_no_free_space_turn_connection" in json.loads(output.read_text())["blockers"]
