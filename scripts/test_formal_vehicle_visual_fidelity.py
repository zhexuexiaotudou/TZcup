from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from formal_vehicle_mesh_manifest import MANIFEST, content

from validate_formal_vehicle_visual_fidelity import (
    DEFAULT_URDF,
    VisualFidelityError,
    _resolve_mesh,
    validate_visual_fidelity,
)


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


def test_mesh_resolver_rejects_parent_traversal() -> None:
    traversal = (
        "package://sanitation_vehicle_description/"
        "../sanitation_gazebo_control/package.xml"
    )
    with pytest.raises(VisualFidelityError, match="not canonical"):
        _resolve_mesh(traversal)
