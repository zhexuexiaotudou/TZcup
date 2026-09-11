#!/usr/bin/env python3
"""Formal brush coverage raster, using the authored bristle-sweep proxy.

This is geometric reconstruction, not evidence of soil removal or physical
contact force. No aggregate vehicle width is accepted as a brush diameter.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

TOOLS = ("left_side_brush", "right_side_brush", "central_roller")


def load_cleaning_geometry(root=None):
    """Read the checked profile and the real plugin's configured interlocks."""
    root = Path(root) if root is not None else Path(__file__).resolve().parents[1]
    profile_path = root / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
    mechanism = root / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
    vehicle = root / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    plugin_source = root / "starter_ws/src/sanitation_gazebo_control/src/GroundDirtCleaningSystem.cc"
    layout = root / "config/high_fidelity_vehicle/formal_vehicle_layout.yaml"
    control = mechanism.parent / "control_interfaces.xacro"
    platform = mechanism.parent / "a300_platform.xacro"
    storage = mechanism.parent / "storage_system.xacro"
    controllers = vehicle.parent.parent / "config/formal_vehicle_controllers.yaml"
    validator = root / "scripts/validate_formal_motion_cleaning_profile.py"
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    # Reuse the established profile-vs-URDF audit, including transforms and
    # work-pose heights. Do not create another independent set of dimensions.
    from validate_formal_motion_cleaning_profile import validate_profile
    validate_profile(
        profile_path=profile_path,
        layout_path=layout,
        cleaning_path=mechanism,
        control_path=control,
        platform_path=platform,
        storage_path=storage,
        controllers_path=controllers,
    )
    xml = ET.parse(vehicle).getroot()
    plugins = [p for p in xml.iter("plugin") if "GroundDirtCleaning" in str(p.attrib)]
    if len(plugins) != 1:
        raise ValueError("expected one formal GroundDirtCleaning plugin")
    plugin = plugins[0]
    source = plugin_source.read_text(encoding="utf-8")
    minimum = re.search(r"minimumContactClearanceM\{([-0-9.]+)\}", source)
    if minimum is None:
        raise ValueError("cannot resolve minimum contact clearance")
    thresholds = {
        "minimum_rotation_rad_s": float(plugin.findtext("minimum_rotation_rad_s")),
        "minimum_lift_position_m": float(plugin.findtext("minimum_lift_position_m")),
        "minimum_contact_clearance_m": float(minimum.group(1)),
        "maximum_contact_clearance_m": float(plugin.findtext("maximum_contact_clearance_m")),
    }
    sweeps = {name: dict(profile["mechanism_sweeps"][name]) for name in TOOLS}
    for tag, expected in (("side_brush_radius_m", sweeps[TOOLS[0]]["radius_m"]),
                          ("roller_radius_m", sweeps[TOOLS[2]]["radius_m"]),
                          ("roller_width_m", sweeps[TOOLS[2]]["width_m"])):
        if not math.isclose(float(plugin.findtext(tag)), expected, abs_tol=1e-12):
            raise ValueError(f"runtime {tag} differs from checked profile")
    return {
        "tools": sweeps, "runtime_thresholds": thresholds,
        "metric_basis": "formal_observed_bristle_sweep_proxy",
        "source_hashes": {str(p.relative_to(root)).replace("\\", "/"):
                          hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in (profile_path, layout, mechanism, control, platform,
                                    storage, vehicle, controllers, plugin_source, validator,
                                    Path(__file__).resolve())},
    }


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _inside(x, y, polygon):
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > y) != (b[1] > y):
            if x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
                inside = not inside
    return inside


def sample_from_ground_dirt_status(status: dict, stamp_s: float, geometry=None):
    """Convert one GroundDirtCleaningSystem JSON object into a coverage sample.

    The plugin publishes tool centres, rather than a base pose.  The lateral
    vector between its side brushes determines yaw; both brush centres then
    independently determine the base position.  ``stamp_s`` is deliberately
    supplied by the caller: the status payload has no simulation clock, so an
    MCAP replay must use its receipt time and label that provenance externally.
    This is a planar rigid-body reconstruction: it deliberately rejects status
    centres affected by roll, pitch, articulation, or an uninitialised pose
    rather than projecting them into a plausible but unobserved base pose.
    """
    geometry = load_cleaning_geometry() if geometry is None else geometry
    if not isinstance(status, dict) or not _finite(stamp_s):
        raise ValueError("status and receipt stamp must be finite")
    required_bools = ("enabled", "cell_layout_ready", "left_ready", "right_ready", "roller_ready")
    required_numbers = (
        "lift_position_m", "left_velocity_rad_s", "right_velocity_rad_s",
        "roller_velocity_rad_s", "left_clearance_m", "right_clearance_m",
        "roller_clearance_m", "left_world_x", "left_world_y", "right_world_x",
        "right_world_y", "roller_world_x", "roller_world_y",
    )
    if any(not isinstance(status.get(key), bool) for key in required_bools):
        raise ValueError("status boolean field missing or invalid")
    if any(not _finite(status.get(key)) for key in required_numbers):
        raise ValueError("status numeric field missing or nonfinite")

    thresholds = geometry["runtime_thresholds"]
    for name, velocity_key, clearance_key, ready_key in (
        ("left", "left_velocity_rad_s", "left_clearance_m", "left_ready"),
        ("right", "right_velocity_rad_s", "right_clearance_m", "right_ready"),
        ("roller", "roller_velocity_rad_s", "roller_clearance_m", "roller_ready"),
    ):
        calculated_ready = (
            status["enabled"] and status["cell_layout_ready"]
            and status["lift_position_m"] >= thresholds["minimum_lift_position_m"]
            and abs(status[velocity_key]) >= thresholds["minimum_rotation_rad_s"]
            and thresholds["minimum_contact_clearance_m"] <= status[clearance_key]
            <= thresholds["maximum_contact_clearance_m"]
        )
        if status[ready_key] is not calculated_ready:
            raise ValueError(f"{name} ready field disagrees with authoritative interlocks")

    left = geometry["tools"]["left_side_brush"]["center_xy_m"]
    right = geometry["tools"]["right_side_brush"]["center_xy_m"]
    roller = geometry["tools"]["central_roller"]["center_xy_m"]
    if not all(_finite(v) for point in (left, right, roller) for v in point):
        raise ValueError("geometry tool centres must be finite")
    expected_dx, expected_dy = left[0] - right[0], left[1] - right[1]
    expected_separation = math.hypot(expected_dx, expected_dy)
    observed_dx = status["left_world_x"] - status["right_world_x"]
    observed_dy = status["left_world_y"] - status["right_world_y"]
    observed_separation = math.hypot(observed_dx, observed_dy)
    tolerance = 1e-5
    if expected_separation <= tolerance or not math.isclose(
            observed_separation, expected_separation, rel_tol=0.0, abs_tol=tolerance):
        raise ValueError("side-brush centre separation disagrees with authoritative geometry")

    # For the authored mirrored centres this is atan2(-dx, dy).  Keeping the
    # general vector form makes profile changes fail closed rather than silently
    # assuming their current y-axis alignment.
    yaw = math.atan2(
        expected_dx * observed_dy - expected_dy * observed_dx,
        expected_dx * observed_dx + expected_dy * observed_dy,
    )
    c, s = math.cos(yaw), math.sin(yaw)

    def base_from(world_x, world_y, offset):
        return (world_x - c * offset[0] + s * offset[1],
                world_y - s * offset[0] - c * offset[1])

    bases = (
        base_from(status["left_world_x"], status["left_world_y"], left),
        base_from(status["right_world_x"], status["right_world_y"], right),
        base_from(status["roller_world_x"], status["roller_world_y"], roller),
    )
    if any(math.hypot(base[0] - bases[0][0], base[1] - bases[0][1]) > tolerance
           for base in bases[1:]):
        raise ValueError("brush or roller world position disagrees with authoritative geometry")
    return {
        "stamp_s": float(stamp_s), "x": bases[0][0], "y": bases[0][1], "yaw": yaw,
        "lift_position_m": float(status["lift_position_m"]),
        "layout_ready": status["cell_layout_ready"],
        "tools": {
            "left_side_brush": {"enabled": status["enabled"] and status["left_ready"],
                                "observed_speed_rad_s": float(status["left_velocity_rad_s"]),
                                "clearance_m": float(status["left_clearance_m"])},
            "right_side_brush": {"enabled": status["enabled"] and status["right_ready"],
                                 "observed_speed_rad_s": float(status["right_velocity_rad_s"]),
                                 "clearance_m": float(status["right_clearance_m"])},
            "central_roller": {"enabled": status["enabled"] and status["roller_ready"],
                               "observed_speed_rad_s": float(status["roller_velocity_rad_s"]),
                               "clearance_m": float(status["roller_clearance_m"])},
        },
    }


def transform_sample_to_mission(sample: dict, mission_geometry: dict):
    """Express a world-frame coverage sample in the frozen mission frame.

    GroundDirtCleaningSystem publishes Gazebo world coordinates.  The formal
    lifecycle mission is normally ``map`` whose origin is the fixed source
    start pose, so the inverse start transform is mandatory.  Unknown frames
    are rejected to keep live and replay consumers from silently mixing maps.
    """
    if not isinstance(sample, dict) or not isinstance(mission_geometry, dict):
        raise ValueError("sample and mission geometry must be mappings")
    if not all(_finite(sample.get(key)) for key in ("x", "y", "yaw")):
        raise ValueError("sample pose must be finite")
    frame = mission_geometry.get("frame_id")
    if frame == "world":
        return dict(sample)
    if frame != "map":
        raise ValueError("mission frame must explicitly be world or map")
    start = mission_geometry.get("source_fixed_start_pose")
    if (not isinstance(start, (list, tuple)) or len(start) != 3
            or not all(_finite(value) for value in start)):
        raise ValueError("map mission requires a finite source_fixed_start_pose")
    c, s = math.cos(start[2]), math.sin(start[2])
    dx, dy = sample["x"] - start[0], sample["y"] - start[1]
    transformed = dict(sample)
    transformed["x"] = c * dx + s * dy
    transformed["y"] = -s * dx + c * dy
    transformed["yaw"] = math.atan2(math.sin(sample["yaw"] - start[2]),
                                     math.cos(sample["yaw"] - start[2]))
    return transformed


def samples_in_mission_frame(samples, mission_geometry):
    """Lazily transform a status-adapter sample stream into the mission frame."""
    for sample in samples:
        yield transform_sample_to_mission(sample, mission_geometry)


def empirical_cleaning_metrics(outer_polygon, samples, geometry=None, *,
                               resolution=0.05, exclusion_polygons=(),
                               max_gap_s=0.2, max_speed_m_s=3.0):
    """Union independent observed tool sweeps, clipped to cleanable cell centers.

    Each sample has stamp_s,x,y,yaw,lift_position_m,layout_ready and tools.
    Each of the three tools has enabled, observed_speed_rad_s and clearance_m.
    Missing/nonfinite observations invalidate the result and break continuity.
    Known inactive tools break only their own continuity. Temporal/displacement
    guards reject interpolation, not the observed endpoint footprints; these
    limits are sampling safeguards, not a physical speed qualification.
    Spatial interpolation is <= resolution/4 at the farthest tool edge and
    follows the shortest yaw arc, including pure rotation.
    """
    geometry = load_cleaning_geometry() if geometry is None else geometry
    for value in (resolution, max_gap_s, max_speed_m_s):
        if not _finite(value) or value <= 0:
            raise ValueError("resolution and sampling limits must be finite and positive")
    polygons = [list(outer_polygon)] + [list(p) for p in exclusion_polygons]
    if any(len(p) < 3 or any(len(v) != 2 or not all(_finite(x) for x in v) for v in p) for p in polygons):
        raise ValueError("polygons must contain at least three finite xy pairs")
    outer = polygons[0]
    min_x, max_x = min(p[0] for p in outer), max(p[0] for p in outer)
    min_y, max_y = min(p[1] for p in outer), max(p[1] for p in outer)
    nx = math.ceil((max_x - min_x) / resolution)
    ny = math.ceil((max_y - min_y) / resolution)
    # Two bytes per raster cell: cleanable and visited.  In particular, do not
    # materialize a tuple/dict per cell; a 20,000 m2 5-cm map has eight million
    # cells and must remain practical for a final, one-shot replay calculation.
    cleanable = bytearray(nx * ny)
    cell_count = 0
    for iy in range(ny):
        y = min_y + (iy + .5) * resolution
        row = iy * nx
        for ix in range(nx):
            x = min_x + (ix + .5) * resolution
            if _inside(x, y, outer) and not any(_inside(x, y, p) for p in polygons[1:]):
                cleanable[row + ix] = 1
                cell_count += 1
    visited, previous = bytearray(nx * ny), {}
    invalid, breaks, active_count, sample_count = 0, 0, 0, 0
    thresholds = geometry["runtime_thresholds"]

    def mark(sample, name):
        tool = geometry["tools"][name]
        c, s = math.cos(sample["yaw"]), math.sin(sample["yaw"])
        ox, oy = tool["center_xy_m"]
        cx, cy = sample["x"] + c*ox - s*oy, sample["y"] + s*ox + c*oy
        radius = tool["radius_m"]
        bound = math.hypot(radius, tool["width_m"]/2) if name == "central_roller" else radius
        first_y = max(0, math.floor((cy-bound-min_y)/resolution))
        last_y = min(ny - 1, math.floor((cy+bound-min_y)/resolution))
        first_x = max(0, math.floor((cx-bound-min_x)/resolution))
        last_x = min(nx - 1, math.floor((cx+bound-min_x)/resolution))
        for iy in range(first_y, last_y + 1):
            row = iy * nx
            for ix in range(first_x, last_x + 1):
                key = row + ix
                if not cleanable[key] or visited[key]:
                    continue
                x, y = min_x + (ix + .5) * resolution, min_y + (iy + .5) * resolution
                dx, dy = x-cx, y-cy
                hit = (abs(c*dx+s*dy) <= radius and abs(-s*dx+c*dy) <= tool["width_m"]/2) if name == "central_roller" else dx*dx+dy*dy <= radius*radius
                if hit:
                    visited[key] = 1

    for sample in samples:
        sample_count += 1
        try:
            good = all(_finite(sample[k]) for k in ("stamp_s", "x", "y", "yaw", "lift_position_m"))
            good = good and isinstance(sample["layout_ready"], bool)
            for name in TOOLS:
                state = sample["tools"][name]
                good = good and isinstance(state["enabled"], bool) and all(_finite(state[k]) for k in ("observed_speed_rad_s", "clearance_m"))
        except (KeyError, TypeError):
            good = False
        if not good:
            invalid += 1
            previous.clear()
            continue
        for name in TOOLS:
            state = sample["tools"][name]
            active = (sample["layout_ready"] and state["enabled"]
                      and sample["lift_position_m"] >= thresholds["minimum_lift_position_m"]
                      and abs(state["observed_speed_rad_s"]) >= thresholds["minimum_rotation_rad_s"]
                      and thresholds["minimum_contact_clearance_m"] <= state["clearance_m"] <= thresholds["maximum_contact_clearance_m"])
            old = previous.pop(name, None)
            if not active:
                continue
            active_count += 1
            mark(sample, name)
            if old is not None:
                dt = sample["stamp_s"] - old["stamp_s"]
                distance = math.hypot(sample["x"]-old["x"], sample["y"]-old["y"])
                if 0 < dt <= max_gap_s and distance <= max_speed_m_s*dt:
                    angle = math.atan2(math.sin(sample["yaw"]-old["yaw"]), math.cos(sample["yaw"]-old["yaw"]))
                    tool = geometry["tools"][name]
                    reach = math.hypot(*tool["center_xy_m"]) + math.hypot(tool["radius_m"], tool.get("width_m", 0)/2)
                    steps = max(1, math.ceil((distance + abs(angle)*reach)/(resolution/4)))
                    for step in range(1, steps):
                        alpha = step/steps
                        pose = {k: old[k]+alpha*(sample[k]-old[k]) for k in ("x", "y")}
                        pose["yaw"] = old["yaw"] + alpha*angle
                        mark(pose, name)
                else:
                    breaks += 1
            previous[name] = sample
    covered_count = sum(visited)
    target, covered = cell_count*resolution**2, covered_count*resolution**2
    return {"valid": invalid == 0 and breaks == 0 and sample_count > 0 and bool(cell_count), "invalid_sample_count": invalid,
            "continuity_break_count": breaks, "active_tool_sample_count": active_count,
            "resolution_m": resolution, "target_area_m2": target,
            "covered_area_m2": covered, "missed_area_m2": target-covered,
            "coverage_rate": covered_count/cell_count if cell_count else 0.0,
            "metric_basis": geometry["metric_basis"], "source_hashes": geometry["source_hashes"],
            "sampling": {"max_gap_s": max_gap_s, "max_speed_m_s": max_speed_m_s,
                         "maximum_edge_step_m": resolution/4},
            "physical_cleaning_validated": False}
