from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from formal_vehicle_mesh_manifest import MANIFEST, content

from validate_formal_vehicle_visual_fidelity import (
    DEFAULT_URDF,
    VisualFidelityError,
    _resolve_mesh,
    validate_visual_fidelity,
)


ROOT = Path(__file__).resolve().parents[1]


def test_committed_formal_vehicle_passes_mesh_fidelity_gate() -> None:
    result = validate_visual_fidelity()
    assert result["status"] == "FORMAL_VISUAL_MESH_FIDELITY_GATE_PASSED"
    assert result["mesh_visual_count"] >= 50
    assert result["required_mesh_link_count"] >= 50


def test_gate_rejects_primitive_regression(tmp_path) -> None:
    tree = ET.parse(DEFAULT_URDF)
    base = next(link for link in tree.getroot().findall("link") if link.attrib["name"] == "base_link")
    geometry = base.find("visual/geometry")
    geometry.clear()
    ET.SubElement(geometry, "box", {"size": "1 1 1"})
    changed = tmp_path / "primitive-regression.urdf"
    tree.write(changed, encoding="unicode")
    with pytest.raises(VisualFidelityError, match="primitive visuals"):
        validate_visual_fidelity(changed)


def test_mesh_manifest_is_complete_and_current() -> None:
    assert MANIFEST.read_text(encoding="utf-8") == content()


def test_s100_reference_mesh_and_collision_match_official_external_envelope() -> None:
    mesh_path = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/meshes/generated/platform"
        / "s100_board_reference.stl"
    )
    raw = mesh_path.read_bytes()
    triangle_count = struct.unpack_from("<I", raw, 80)[0]
    assert len(raw) == 84 + triangle_count * 50
    vertices = []
    for index in range(triangle_count):
        record_offset = 84 + index * 50
        vertices.extend(
            struct.unpack_from("<9f", raw, record_offset + 12)
        )
    xyz = list(zip(vertices[0::3], vertices[1::3], vertices[2::3]))
    extents = [max(point[axis] for point in xyz) - min(point[axis] for point in xyz) for axis in range(3)]
    assert extents == pytest.approx([0.121, 0.120, 0.0524], abs=1e-7)

    tree = ET.parse(DEFAULT_URDF)
    link = next(
        item
        for item in tree.getroot().findall("link")
        if item.attrib["name"] == "s100_board_reference_link"
    )
    collision_size = [
        float(value)
        for value in link.find("collision[@name='s100_official_external_envelope_collision']/geometry/box").attrib["size"].split()
    ]
    assert collision_size == [0.121, 0.120, 0.0524]


def test_xw540_gate_actuator_has_explicit_mesh_mass_envelope_and_torque() -> None:
    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    actuator = next(
        item for item in root.findall("link")
        if item.attrib["name"] == "dry_deposit_gate_actuator_link"
    )
    assert actuator.find("visual/geometry/mesh").attrib["filename"].endswith(
        "/dry_deposit_xw540_actuator.stl"
    )
    assert float(actuator.find("inertial/mass").attrib["value"]) == pytest.approx(0.185)
    collision_size = [
        float(value)
        for value in actuator.find("collision/geometry/box").attrib["size"].split()
    ]
    assert collision_size == pytest.approx([0.0459, 0.0335, 0.0585])
    gate_joint = next(
        item for item in root.findall("joint")
        if item.attrib["name"] == "dry_deposit_gate_joint"
    )
    assert float(gate_joint.find("limit").attrib["effort"]) == pytest.approx(9.5)

    # At the closed-gate datum the real housing's modeled output boss, the
    # gate hinge and the moving horn must share one physical axis.  This
    # catches a visually plausible but mechanically disconnected servo.
    actuator_mount = next(
        item for item in root.findall("joint")
        if item.attrib["name"] == "dry_deposit_gate_actuator_mount_joint"
    )
    horn_mount = next(
        item for item in root.findall("joint")
        if item.attrib["name"] == "dry_deposit_gate_actuator_horn_joint"
    )
    gate_origin = [float(value) for value in gate_joint.find("origin").attrib["xyz"].split()]
    housing_origin = [
        float(value) for value in actuator_mount.find("origin").attrib["xyz"].split()
    ]
    horn_offset = [float(value) for value in horn_mount.find("origin").attrib["xyz"].split()]
    modeled_output_boss_offset = [0.0, -0.0195, 0.012]
    housing_output_axis = [
        housing_origin[index] + modeled_output_boss_offset[index]
        for index in range(3)
    ]
    gate_horn_axis = [gate_origin[index] + horn_offset[index] for index in range(3)]
    assert housing_output_axis == pytest.approx(gate_horn_axis, abs=1e-9)
    assert gate_joint.find("axis").attrib["xyz"] == "0 1 0"

    # The actuator and horn are a mass reallocation from the old generic
    # hopper/gate shells, so the already razor-thin A300 payload margin is not
    # silently consumed by making the actuator explicit.
    masses = {}
    for name in (
        "dry_deposit_hopper_link",
        "dry_deposit_gate_link",
        "dry_deposit_gate_actuator_link",
        "dry_deposit_gate_actuator_horn_link",
    ):
        link = next(item for item in root.findall("link") if item.attrib["name"] == name)
        masses[name] = float(link.find("inertial/mass").attrib["value"])
    assert sum(masses.values()) == pytest.approx(0.420)


def test_mesh_resolver_rejects_parent_traversal() -> None:
    traversal = (
        "package://sanitation_vehicle_description/"
        "../sanitation_gazebo_control/package.xml"
    )
    with pytest.raises(VisualFidelityError, match="not canonical"):
        _resolve_mesh(traversal)
