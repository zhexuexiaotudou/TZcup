#!/usr/bin/env python3
"""Audit actual collision bounds and dry-cleaning intervals, not envelope claims."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from generate_formal_vehicle_snapshot import MANIFEST_OUTPUT, SnapshotError, verify_snapshot
from scan_formal_vehicle_inertia_and_swept_volume import Model, _shape_bounds
from validate_formal_motion_cleaning_profile import FormalMotionCleaningProfileError, validate_profile

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "reports/engineering/formal_competition_vehicle.urdf"
PROFILE = ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"


def snapshot_binding(urdf_path: Path, profile_path: Path) -> dict:
    """Bind custom inputs and all referenced collision meshes to one snapshot."""
    try:
        manifest = verify_snapshot(ROOT)
    except (SnapshotError, OSError, ValueError) as exc:
        return {"passed": False, "error": str(exc)}
    errors = []
    for path, inventory, canonical in (
        (urdf_path, manifest["outputs"], URDF),
        (profile_path, manifest["source_inventory"], PROFILE),
    ):
        expected = inventory[canonical.relative_to(ROOT).as_posix()]
        actual = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                  "size_bytes": path.stat().st_size}
        if actual != expected:
            errors.append(f"audit input differs from verified snapshot: {canonical.relative_to(ROOT).as_posix()}")
    return {
        "passed": not errors,
        "error": "; ".join(errors) or None,
        "manifest_sha256": hashlib.sha256((ROOT / MANIFEST_OUTPUT).read_bytes()).hexdigest(),
        "source_inventory_sha256": manifest["source_inventory_sha256"],
        "output_inventory_sha256": manifest["output_inventory_sha256"],
    }


def merged_intervals(intervals):
    merged = []
    for low, high in sorted(intervals):
        if not np.isfinite([low, high]).all() or high <= low:
            raise ValueError("invalid transverse interval")
        if merged and low <= merged[-1][1] + 1e-9:
            merged[-1][1] = max(high, merged[-1][1])
        else:
            merged.append([low, high])
    return merged


def parallel_cylinder_interference(center_a, axis_a, radius_a, length_a,
                                   center_b, axis_b, radius_b, length_b):
    """Exact positive-volume intersection test for parallel finite cylinders."""
    a, b = np.asarray(center_a, dtype=float), np.asarray(center_b, dtype=float)
    axis, other = np.asarray(axis_a, dtype=float), np.asarray(axis_b, dtype=float)
    if (a.shape != (3,) or b.shape != (3,) or axis.shape != (3,) or other.shape != (3,)
            or not np.isfinite(np.concatenate((a, b, axis, other))).all()
            or np.linalg.norm(axis) < 1e-12 or np.linalg.norm(other) < 1e-12
            or not np.isfinite([radius_a, radius_b, length_a, length_b]).all()
            or min(radius_a, radius_b, length_a, length_b) <= 0):
        raise ValueError("invalid cylinder geometry")
    axis, other = axis / np.linalg.norm(axis), other / np.linalg.norm(other)
    if np.linalg.norm(np.cross(axis, other)) > 1e-9:
        raise ValueError("analytic check requires parallel cylinder axes")
    delta = b - a
    axial = float(np.dot(delta, axis))
    axial_overlap = min(length_a / 2, axial + length_b / 2) - max(-length_a / 2, axial - length_b / 2)
    radial_distance = float(np.linalg.norm(delta - axial * axis))
    radial_margin = radial_distance - radius_a - radius_b
    return {
        "axial_overlap_m": round(axial_overlap, 9),
        "radial_axis_distance_m": round(radial_distance, 9),
        "radial_separation_margin_m": round(radial_margin, 9),
        "positive_volume_intersection": axial_overlap > 1e-9 and radial_margin < -1e-9,
    }


def audit(urdf_path=URDF, profile_path=PROFILE):
    binding = snapshot_binding(urdf_path, profile_path)
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    try:
        validate_profile(profile_path=profile_path)
        source_error = None
    except FormalMotionCleaningProfileError as exc:
        source_error = str(exc)
    model = Model(urdf_path)
    arm = profile["arm_task_poses"]
    positions = dict(zip(arm["joint_names"], arm["transport_rad"], strict=True))
    states = {}
    interference = []
    for state, lift in (("transport_stowed", 0.0), ("cleaning_deployed", 0.1)):
        positions["cleaning_lift_joint"] = lift
        transforms = model.transforms(positions)
        bounds = [(shape, *_shape_bounds(shape, transforms[shape.link])) for shape in model.shapes]
        points = np.asarray(profile["motion_footprints"][state]["footprint_xy_m"])
        # These profiles explicitly prescribe axis-aligned rectangles. Reject
        # another polygon rather than accidentally auditing only its AABB.
        low, high = points.min(axis=0), points.max(axis=0)
        corners = {(float(x), float(y)) for x in (low[0], high[0]) for y in (low[1], high[1])}
        if len(points) != 4 or {tuple(point) for point in points} != corners:
            raise ValueError("navigation footprint must be an axis-aligned rectangle")
        outside = sorted({shape.link for shape, minimum, maximum in bounds
                          if np.any(minimum[:2] < low - 1e-9) or np.any(maximum[:2] > high + 1e-9)})
        states[state] = {
            "collision_count": len(bounds),
            "collision_min_xyz_m": np.min([item[1] for item in bounds], axis=0).round(9).tolist(),
            "collision_max_xyz_m": np.max([item[2] for item in bounds], axis=0).round(9).tolist(),
            "outside_footprint_links": outside,
            "passed": not outside,
        }
        roller_shape = next(shape for shape in model.shapes if shape.link == "central_roller_link")
        roller_transform = transforms[roller_shape.link] @ roller_shape.origin
        for wheel_name in ("front_left_wheel_link", "front_right_wheel_link"):
            wheel_shape = next(shape for shape in model.shapes if shape.link == wheel_name)
            if roller_shape.kind != "cylinder" or wheel_shape.kind != "cylinder":
                raise ValueError("roller/wheel interference audit requires cylinder collision geometry")
            wheel_transform = transforms[wheel_name] @ wheel_shape.origin
            interference.append({
                "state": state, "roller_link": roller_shape.link, "wheel_link": wheel_name,
                **parallel_cylinder_interference(
                    roller_transform[:3, 3], roller_transform[:3, 2], *roller_shape.parameters,
                    wheel_transform[:3, 3], wheel_transform[:3, 2], *wheel_shape.parameters),
            })

    # GroundDirtCleaningSystem cleans only the two disk sweeps and roller
    # rectangle. It contains no debris redistribution / lateral transfer model.
    # Their transverse projections bound an indefinitely long straight pass;
    # finite pass ends and discretization can only decrease its clean area.
    sweeps = profile["mechanism_sweeps"]
    intervals = []
    for name in ("left_side_brush_link", "right_side_brush_link", "central_roller_link"):
        shapes = [shape for shape in model.shapes if shape.link == name]
        if len(shapes) != 1 or shapes[0].kind != "cylinder":
            raise ValueError(f"{name} must have one direct cylindrical sweep collision")
        minimum, maximum = _shape_bounds(shapes[0], transforms[name])
        intervals.append([float(minimum[1]), float(maximum[1])])
    union = merged_intervals(intervals)
    contiguous = max(high - low for low, high in union)
    target = sweeps["transverse_union"]["declared_effective_cleaning_width_m"]
    target_supported = len(union) == 1 and contiguous + 1e-9 >= target
    claim_guard = target_supported or profile["claim_boundary"]["effective_cleaning_width_status"].startswith("pending_")
    return {
        "report_id": "formal_vehicle_competition_geometry_v1",
        "evidence_level": "STATIC_COLLISION_AND_STRAIGHT_PASS_GEOMETRY_ONLY",
        "urdf_sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
        "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
        "profile_source_consistency_error": source_error,
        "snapshot_binding": binding,
        "motion_states": states,
        "roller_front_wheel_interference": interference,
        "model_mechanical_layout_clear": not any(row["positive_volume_intersection"] for row in interference),
        "mechanical_blockers": ["central_roller_intersects_both_front_wheel_collision_solids_in_raised_and_work_poses"]
            if any(row["positive_volume_intersection"] for row in interference) else [],
        "dry_cleaning": {
            "transverse_intervals_m": [[round(value, 9) for value in interval] for interval in union],
            "uncovered_internal_gaps_m": [round(union[i + 1][0] - union[i][1], 9) for i in range(len(union) - 1)],
            "union_width_m": round(sum(high - low for low, high in union), 9),
            "largest_contiguous_width_m": round(contiguous, 9),
            "competition_minimum_width_m": 0.6,
            "minimum_width_geometry_supported": contiguous + 1e-9 >= 0.6,
            "declared_target_width_m": target,
            "declared_target_supported_by_straight_pass_geometry": target_supported,
            "runtime_cleaning_and_efficiency_passed": False,
            "claim_guard_passed": claim_guard,
        },
        "passed": binding["passed"] and source_error is None and all(state["passed"] for state in states.values()) and claim_guard,
        "passed_scope": "audit_consistency_navigation_envelope_and_claim_guard_only_not_mechanical_acceptance",
        "limitations": ["No flexible-bristle or inward-debris-transport proof",
                        "No continuous arm/service-door sweep or runtime contact acceptance"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit()
    text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8", newline="\n")
    else:
        print(text, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
