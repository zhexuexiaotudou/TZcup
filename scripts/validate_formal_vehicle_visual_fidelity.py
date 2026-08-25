#!/usr/bin/env python3
"""Fail closed when the formal vehicle regresses to primitive-only visuals."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description"
DEFAULT_URDF = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"

# These links define the recognisable vehicle rather than transparent payload
# envelopes or tiny internal instrumentation. Every visual on them must be a
# mesh. Primitive collision geometry is intentionally permitted.
REQUIRED_MESH_LINKS = {
    "base_link", "payload_deck_link", "sensor_mast_link", "arm_mount_link",
    "ur5e_control_box_link", "s100_compute_enclosure_link",
    "front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link", "rear_right_wheel_link",
    "ur5e_base_link_inertia", "ur5e_shoulder_link", "ur5e_upper_arm_link", "ur5e_forearm_link",
    "ur5e_wrist_1_link", "ur5e_wrist_2_link", "ur5e_wrist_3_link",
    "ur_to_robotiq_adapter_link", "robotiq_85_base_link",
    "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
    "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
    "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
    "front_rgbd_link", "wrist_rgbd_link", "rear_left_fisheye_link", "rear_right_fisheye_link",
    "lidar_2d_link", "lidar_3d_link", "gnss_antenna_link", "imu_link",
    "cleaning_mechanism_mount_link", "left_side_brush_motor_stator_link",
    "right_side_brush_motor_stator_link", "left_side_brush_link", "right_side_brush_link",
    "central_roller_link", "central_roller_guard_link", "squeegee_link", "suction_nozzle_link",
    "recovery_strainer_filter_link", "recovery_pump_motor_link", "recovery_pump_head_link",
    "storage_system_mount_link", "dry_bin_link", "dry_bin_lid_link",
    "wastewater_tank_link", "wastewater_lid_link", "dry_wet_storage_partition_link",
}
ALLOWED_PRIMITIVE_VISUAL_LINKS = {
    # Transparent state indicators, not claimed mechanical parts.
    "dry_bin_payload_reserve_link",
    "wastewater_payload_reserve_link",
}


class VisualFidelityError(ValueError):
    pass


def _resolve_mesh(filename: str) -> Path:
    prefix = "package://sanitation_vehicle_description/"
    if not filename.startswith(prefix):
        raise VisualFidelityError(f"mesh is not self-contained in the ROS package: {filename}")
    return PACKAGE_ROOT / filename[len(prefix):]


def validate_visual_fidelity(urdf_path: Path = DEFAULT_URDF) -> dict[str, object]:
    root = ET.parse(urdf_path).getroot()
    links = {node.attrib["name"]: node for node in root.findall("link")}
    missing = sorted(REQUIRED_MESH_LINKS - set(links))
    if missing:
        raise VisualFidelityError("required mesh links missing: " + ", ".join(missing))

    all_meshes: list[str] = []
    primitive_visual_count = 0
    primitive_visual_links: set[str] = set()
    invalid_required: list[str] = []
    for link_name, link in links.items():
        visuals = link.findall("visual")
        geometries = [visual.find("geometry") for visual in visuals]
        meshes = [geometry.find("mesh") for geometry in geometries if geometry is not None]
        mesh_nodes = [mesh for mesh in meshes if mesh is not None]
        link_primitive_count = sum(
            geometry is not None and geometry.find("mesh") is None for geometry in geometries
        )
        primitive_visual_count += link_primitive_count
        if link_primitive_count:
            primitive_visual_links.add(link_name)
        if link_name in REQUIRED_MESH_LINKS and (
            not visuals or len(mesh_nodes) != len(visuals)
        ):
            invalid_required.append(link_name)
        for mesh in mesh_nodes:
            filename = mesh.attrib.get("filename", "")
            path = _resolve_mesh(filename)
            if not path.is_file() or path.stat().st_size < 84:
                raise VisualFidelityError(f"mesh file missing or empty: {filename}")
            all_meshes.append(filename)
    if invalid_required:
        raise VisualFidelityError(
            "recognisable external links still use missing/primitive visuals: "
            + ", ".join(sorted(invalid_required))
        )
    unexpected_primitives = sorted(primitive_visual_links - ALLOWED_PRIMITIVE_VISUAL_LINKS)
    if unexpected_primitives:
        raise VisualFidelityError(
            "mechanical links still expose primitive visuals: " + ", ".join(unexpected_primitives)
        )
    if len(all_meshes) < 50:
        raise VisualFidelityError(f"formal model has only {len(all_meshes)} mesh visuals; expected at least 50")
    required_families = ("clearpath_a300", "universal_robots", "robotiq_2f85", "meshes/generated")
    absent_families = [family for family in required_families if not any(family in item for item in all_meshes)]
    if absent_families:
        raise VisualFidelityError("required visual source families absent: " + ", ".join(absent_families))
    return {
        "status": "FORMAL_VISUAL_MESH_FIDELITY_GATE_PASSED",
        "mesh_visual_count": len(all_meshes),
        "primitive_visual_count": primitive_visual_count,
        "primitive_visual_links": sorted(primitive_visual_links),
        "required_mesh_link_count": len(REQUIRED_MESH_LINKS),
        "all_required_external_links_are_mesh_visuals": True,
        "all_mesh_files_exist": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    args = parser.parse_args()
    print(validate_visual_fidelity(args.urdf))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
