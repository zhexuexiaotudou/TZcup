import json
import pytest
import yaml

import audit_formal_vehicle_competition_geometry as geometry_audit
from audit_formal_vehicle_competition_geometry import PROFILE, ROOT, URDF, audit, merged_intervals
from audit_formal_vehicle_competition_geometry import parallel_cylinder_interference


def profile_variant(tmp_path, change):
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    change(profile)
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    return path


def test_actual_vehicle_collision_geometry_fits_both_navigation_states():
    report = audit()
    assert report["passed"]
    assert all(state["collision_count"] > 196 for state in report["motion_states"].values())
    dry = report["dry_cleaning"]
    assert dry["union_width_m"] == pytest.approx(1.22)
    assert dry["largest_contiguous_width_m"] == pytest.approx(0.62)
    assert dry["uncovered_internal_gaps_m"] == pytest.approx([0.085, 0.085])
    assert dry["minimum_width_geometry_supported"]
    assert not dry["declared_target_supported_by_straight_pass_geometry"]
    assert not dry["runtime_cleaning_and_efficiency_passed"]
    assert not report["model_mechanical_layout_clear"]
    assert len(report["roller_front_wheel_interference"]) == 4
    assert all(row["positive_volume_intersection"] for row in report["roller_front_wheel_interference"])
    assert all(row["axial_overlap_m"] == pytest.approx(0.08425) for row in report["roller_front_wheel_interference"])


def test_stored_geometry_report_matches_current_verified_snapshot():
    stored = json.loads((ROOT / "reports/engineering/formal_vehicle_competition_geometry_report.json").read_text(encoding="utf-8"))
    assert stored == audit()


def test_custom_expanded_urdf_must_match_verified_snapshot(tmp_path):
    stale = tmp_path / "stale.urdf"
    stale.write_bytes(URDF.read_bytes() + b"\n<!-- stale expanded output -->\n")
    report = audit(urdf_path=stale)
    assert not report["passed"]
    assert not report["snapshot_binding"]["passed"]
    assert "audit input differs" in report["snapshot_binding"]["error"]


def test_mesh_or_source_inventory_drift_blocks_geometry_pass(monkeypatch):
    def drift(_root):
        raise geometry_audit.SnapshotError("authoritative source inventory differs from committed manifest")
    monkeypatch.setattr(geometry_audit, "verify_snapshot", drift)
    report = audit()
    assert not report["passed"]
    assert "source inventory differs" in report["snapshot_binding"]["error"]


def test_old_transport_envelope_detects_brushes_and_rear_sensors(tmp_path):
    def narrow(profile):
        profile["motion_footprints"]["transport_stowed"]["footprint_xy_m"] = [
            [0.62, 0.675], [0.62, -0.675], [-0.54, -0.675], [-0.54, 0.675]]
    report = audit(profile_path=profile_variant(tmp_path, narrow))
    assert not report["passed"]
    outside = report["motion_states"]["transport_stowed"]["outside_footprint_links"]
    assert "left_side_brush_link" in outside
    assert "right_side_brush_link" in outside
    assert "rear_left_fisheye_mount_link" in outside


def test_claiming_full_width_pass_without_closing_physical_gaps_fails(tmp_path):
    path = profile_variant(tmp_path, lambda p: p["claim_boundary"].update(
        effective_cleaning_width_status="passed"))
    report = audit(profile_path=path)
    assert not report["passed"]
    assert not report["dry_cleaning"]["claim_guard_passed"]


def test_profile_cannot_invent_larger_brushes_to_close_physical_gaps(tmp_path):
    path = profile_variant(tmp_path, lambda p: p["mechanism_sweeps"]["left_side_brush"].update(radius_m=0.3))
    report = audit(profile_path=path)
    assert not report["passed"]
    assert report["profile_source_consistency_error"]


def test_interval_union_does_not_add_overlap_or_bridge_gaps():
    assert merged_intervals([[0, 1], [0.5, 2], [2, 3], [4, 5]]) == [[0, 3], [4, 5]]
    with pytest.raises(ValueError):
        merged_intervals([[0, float("nan")]])


@pytest.mark.parametrize("center,expected", [([0.1, 0, 0], True), ([0.3, 0, 0], False),
                                             ([0, 0.7, 0], False), ([0.2, 0, 0], False)])
def test_parallel_cylinder_intersection_separates_radial_axial_and_tangent_cases(center, expected):
    result = parallel_cylinder_interference([0, 0, 0], [0, 1, 0], 0.1, 0.6,
                                           center, [0, -1, 0], 0.1, 0.2)
    assert result["positive_volume_intersection"] is expected


def test_parallel_cylinder_test_rejects_unhandled_axes():
    with pytest.raises(ValueError, match="parallel"):
        parallel_cylinder_interference([0, 0, 0], [0, 1, 0], 0.1, 0.6,
                                       [0, 0, 0], [0, 0, 1], 0.1, 0.2)
