#!/usr/bin/env python3
"""Validate that the formal model reads as a designed sanitation vehicle.

The generic mesh-fidelity gate prevents primitive regressions.  This stronger
gate checks the product-design layer requested for the final vehicle: coherent
body modules, independently modelled service panels, safety lighting, material
hierarchy and physical collision/inertia on every bodywork link.
"""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"
BODYWORK_PREFIX = "package://sanitation_vehicle_description/meshes/project/bodywork/"
REQUIRED_LINKS = {
    "bodywork_lower_tub_link",
    "bodywork_front_cowl_link",
    "bodywork_rear_shell_link",
    "bodywork_power_service_door_link",
    "bodywork_compute_service_door_link",
    "bodywork_wet_service_door_link",
    "bodywork_rear_dry_service_door_link",
    "bodywork_power_service_door_hinge_bracket_link",
    "bodywork_power_service_door_latch_link",
    "bodywork_compute_service_door_hinge_bracket_link",
    "bodywork_compute_service_door_latch_link",
    "bodywork_wet_service_door_hinge_bracket_link",
    "bodywork_wet_service_door_latch_link",
    "bodywork_rear_dry_service_door_hinge_bracket_link",
    "bodywork_rear_dry_service_door_latch_link",
    "bodywork_trim_link",
    "bodywork_lighting_link",
    "bodywork_brush_guards_link",
}
REQUIRED_MESH_NAMES = {
    "front_center_nose.stl",
    "rear_bin_outer_shell.stl",
    "front_left_power_cowl.stl",
    "front_right_compute_cowl.stl",
    "sensor_pylon_fairing.stl",
    "front_bumper.stl",
    "rear_bumper.stl",
    "power_service_door.stl",
    "compute_service_door.stl",
    "wet_service_door.stl",
    "rear_dry_service_door.stl",
    "service_door_hinge_barrel.stl",
    "service_door_rotary_latch.stl",
    "front_work_light_left.stl",
    "front_green_apron.stl",
    "rear_tail_light_right.stl",
    "corner_beacons.stl",
    # The E-stop is no longer decorative bodywork.  Its housing and moving
    # plunger are enforced by the power/service-hardware validator.
    "left_side_brush_motor_guard.stl",
    "right_side_brush_motor_guard.stl",
}


class ProductDesignError(ValueError):
    pass


def _rgba(visual: ET.Element) -> tuple[float, float, float, float] | None:
    node = visual.find("material/color")
    if node is None or not node.attrib.get("rgba"):
        return None
    values = tuple(float(value) for value in node.attrib["rgba"].split())
    if len(values) != 4:
        raise ProductDesignError("bodywork material must have four RGBA values")
    return values


def validate_product_design(path: Path = DEFAULT_URDF) -> dict[str, object]:
    root = ET.parse(path).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    missing_links = sorted(REQUIRED_LINKS - links.keys())
    if missing_links:
        raise ProductDesignError("required product body links missing: " + ", ".join(missing_links))

    mesh_names: set[str] = set()
    colors: set[tuple[float, float, float, float]] = set()
    missing_physics: list[str] = []
    primitive_body_visuals: list[str] = []
    for name in sorted(REQUIRED_LINKS):
        link = links[name]
        if link.find("collision") is None or link.find("inertial") is None:
            missing_physics.append(name)
        mass = link.find("inertial/mass")
        if mass is None or float(mass.attrib.get("value", "0")) <= 0.0:
            missing_physics.append(name)
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh") if geometry is not None else None
            if mesh is None:
                primitive_body_visuals.append(name)
                continue
            filename = mesh.attrib.get("filename", "")
            if not filename.startswith(BODYWORK_PREFIX):
                raise ProductDesignError(f"bodywork mesh is not project-owned: {filename}")
            mesh_names.add(Path(filename).name)
            color = _rgba(visual)
            if color is not None and color[3] >= 0.85:
                colors.add(color)

    if missing_physics:
        raise ProductDesignError("bodywork links missing physical properties: " + ", ".join(sorted(set(missing_physics))))
    if primitive_body_visuals:
        raise ProductDesignError("bodywork regressed to primitive visuals: " + ", ".join(sorted(set(primitive_body_visuals))))
    missing_meshes = sorted(REQUIRED_MESH_NAMES - mesh_names)
    if missing_meshes:
        raise ProductDesignError("required product details missing: " + ", ".join(missing_meshes))
    if len(mesh_names) < 35:
        raise ProductDesignError(f"only {len(mesh_names)} bodywork meshes; expected at least 35")
    if len(colors) < 9:
        raise ProductDesignError(f"only {len(colors)} opaque bodywork colors; expected a deliberate material hierarchy")

    service_door_prefixes = (
        "bodywork_power_service_door",
        "bodywork_compute_service_door",
        "bodywork_wet_service_door",
        "bodywork_rear_dry_service_door",
    )
    for prefix in service_door_prefixes:
        bracket_joint = joints.get(f"{prefix}_hinge_bracket_joint")
        hinge_joint = joints.get(f"{prefix}_hinge_joint")
        latch_joint = joints.get(f"{prefix}_latch_joint")
        if bracket_joint is None or bracket_joint.attrib.get("type") != "fixed":
            raise ProductDesignError(f"{prefix} lacks a fixed chassis hinge bracket")
        if hinge_joint is None or hinge_joint.attrib.get("type") != "revolute":
            raise ProductDesignError(f"{prefix} lacks a physical revolute hinge")
        if latch_joint is None or latch_joint.attrib.get("type") != "revolute":
            raise ProductDesignError(f"{prefix} lacks an independent rotary latch")
        hinge_axis = hinge_joint.find("axis")
        hinge_limit = hinge_joint.find("limit")
        latch_limit = latch_joint.find("limit")
        if hinge_axis is None or hinge_axis.attrib.get("xyz") != "0 0 1":
            raise ProductDesignError(f"{prefix} hinge must use a vertical axis")
        if hinge_limit is None or (
            float(hinge_limit.attrib["upper"])
            - float(hinge_limit.attrib["lower"])
            < 1.70
        ):
            raise ProductDesignError(f"{prefix} hinge lacks a usable mechanical limit")
        if latch_limit is None or not (
            float(latch_limit.attrib["lower"]) < 0.0 < float(latch_limit.attrib["upper"])
        ):
            raise ProductDesignError(f"{prefix} latch lacks a locked zero detent range")

    has_warm_white = any(r > 0.75 and g > 0.75 and b > 0.75 for r, g, b, _ in colors)
    has_graphite = any(r < 0.10 and g < 0.11 and b < 0.12 for r, g, b, _ in colors)
    has_green = any(g > 0.30 and g > r * 3.0 and g > b * 1.25 for r, g, b, _ in colors)
    has_red = any(r > 0.60 and g < 0.10 and b < 0.10 for r, g, b, _ in colors)
    has_amber = any(r > 0.80 and 0.20 < g < 0.70 and b < 0.10 for r, g, b, _ in colors)
    palette = {
        "warm_white": has_warm_white,
        "graphite": has_graphite,
        "sanitation_green": has_green,
        "safety_red": has_red,
        "warning_amber": has_amber,
    }
    missing_palette = [name for name, present in palette.items() if not present]
    if missing_palette:
        raise ProductDesignError("required product palette absent: " + ", ".join(missing_palette))

    return {
        "status": "FORMAL_PRODUCT_BODYWORK_DESIGN_GATE_PASSED",
        "bodywork_link_count": len(REQUIRED_LINKS),
        "bodywork_mesh_visual_count": len(mesh_names),
        "opaque_material_color_count": len(colors),
        "service_panel_count": 4,
        "service_hinge_count": 4,
        "service_latch_count": 4,
        "all_service_doors_have_chassis_brackets_limits_and_latches": True,
        "palette": palette,
        "all_bodywork_links_have_collision_and_positive_inertia": True,
        "all_bodywork_visuals_are_project_owned_meshes": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate_product_design(args.urdf)
    rendered = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
