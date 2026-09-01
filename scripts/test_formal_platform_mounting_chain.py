from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "starter_ws/src/sanitation_vehicle_description"
PLATFORM = DESCRIPTION / "urdf/high_fidelity/a300_platform.xacro"
MESH_ROOT = DESCRIPTION / "meshes/generated/platform"


def _root() -> ET.Element:
    return ET.parse(PLATFORM).getroot()


def _link(root: ET.Element, name: str) -> ET.Element:
    result = root.find(f".//link[@name='{name}']")
    assert result is not None, name
    return result


def _joint(root: ET.Element, name: str) -> ET.Element:
    result = root.find(f".//joint[@name='{name}']")
    assert result is not None, name
    return result


def _mass(link: ET.Element) -> float:
    node = link.find("./inertial/mass")
    assert node is not None
    return float(node.attrib["value"])


def _joint_z(joint: ET.Element) -> float:
    origin = joint.find("origin")
    assert origin is not None
    return float(origin.attrib["xyz"].split()[2])


def _mesh_bounds(name: str) -> tuple[float, float]:
    data = (MESH_ROOT / name).read_bytes()
    assert len(data) >= 84
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + triangle_count * 50
    z_values = []
    for triangle in range(triangle_count):
        offset = 84 + triangle * 50 + 12  # skip the face normal
        for vertex in range(3):
            z_values.append(struct.unpack_from("<f", data, offset + vertex * 12 + 8)[0])
    assert z_values
    return min(z_values), max(z_values)


def test_cabinet_and_compute_mounts_form_one_explicit_parent_chain() -> None:
    root = _root()
    expected = {
        "ur5e_control_box_isolation_base_joint": (
            "payload_deck_link",
            "ur5e_control_box_isolation_base_link",
        ),
        "ur5e_control_box_mount_joint": (
            "ur5e_control_box_isolation_base_link",
            "ur5e_control_box_link",
        ),
        "s100_cabinet_roof_mount_joint": (
            "ur5e_control_box_link",
            "s100_cabinet_roof_mount_link",
        ),
        "s100_compute_enclosure_mount_joint": (
            "s100_cabinet_roof_mount_link",
            "s100_compute_enclosure_link",
        ),
        "s100_board_reference_joint": (
            "s100_compute_enclosure_link",
            "s100_board_reference_link",
        ),
    }
    for name, (parent, child) in expected.items():
        joint = _joint(root, name)
        assert joint.attrib["type"] == "fixed"
        assert joint.find("parent").attrib["link"] == parent
        assert joint.find("child").attrib["link"] == child


def test_new_mounting_links_are_visible_collidable_and_have_positive_mass() -> None:
    root = _root()
    for name in (
        "ur5e_control_box_isolation_base_link",
        "s100_cabinet_roof_mount_link",
    ):
        link = _link(root, name)
        assert link.find("visual/geometry/mesh") is not None
        assert len(link.findall("collision")) >= 1
        assert _mass(link) > 0.0
        inertia = link.find("inertial")
        assert inertia is not None
        assert inertia.find("{http://ros.org/wiki/xacro}hf_box_inertia") is not None
    base = _link(root, "ur5e_control_box_isolation_base_link")
    roof = _link(root, "s100_cabinet_roof_mount_link")
    assert len(base.findall("{http://ros.org/wiki/xacro}hf_control_box_isolator_collision")) == 4
    assert len(roof.findall("{http://ros.org/wiki/xacro}hf_s100_standoff_collision")) == 4


def test_mount_meshes_close_both_vertical_clearances_without_overlap() -> None:
    root = _root()
    base_min, base_max = _mesh_bounds("ur5e_control_cabinet_isolation_base.stl")
    cabinet_min, _ = _mesh_bounds("ur5e_control_cabinet.stl")
    roof_min, roof_max = _mesh_bounds("s100_cabinet_roof_mount.stl")
    compute_min, _ = _mesh_bounds("s100_compute_enclosure.stl")

    assert math.isclose(base_min, 0.0, abs_tol=1e-7)
    assert math.isclose(base_max, 0.077, abs_tol=1e-7)
    cabinet_bottom_in_base = _joint_z(_joint(root, "ur5e_control_box_mount_joint")) + cabinet_min
    assert math.isclose(cabinet_bottom_in_base, base_max, abs_tol=1e-7)

    assert math.isclose(roof_min, 0.0, abs_tol=1e-7)
    assert math.isclose(roof_max, 0.0269, abs_tol=1e-7)
    compute_bottom_in_roof = _joint_z(_joint(root, "s100_compute_enclosure_mount_joint")) + compute_min
    assert math.isclose(compute_bottom_in_roof, roof_max, abs_tol=1e-7)

    deck_top = 0.030 / 2.0
    assert math.isclose(
        _joint_z(_joint(root, "ur5e_control_box_isolation_base_joint")),
        deck_top,
        abs_tol=1e-9,
    )


def test_mount_mass_is_split_from_existing_assemblies_without_total_mass_drift() -> None:
    root = _root()
    cabinet_assembly_mass = _mass(_link(root, "ur5e_control_box_isolation_base_link")) + _mass(
        _link(root, "ur5e_control_box_link")
    )
    compute_assembly_mass = (
        _mass(_link(root, "s100_cabinet_roof_mount_link"))
        + _mass(_link(root, "s100_compute_enclosure_link"))
        + _mass(_link(root, "s100_board_reference_link"))
    )
    assert math.isclose(cabinet_assembly_mass, 12.0, abs_tol=1e-9)
    assert math.isclose(compute_assembly_mass, 2.0, abs_tol=1e-9)


def test_generator_keeps_project_mount_pattern_boundary_explicit() -> None:
    source = (ROOT / "scripts/generate_platform_auxiliary_meshes.py").read_text(
        encoding="utf-8"
    )
    assert "not represented as a Universal Robots factory hole pattern" in source
    assert "must not be read as a D-Robotics board-hole specification" in source
