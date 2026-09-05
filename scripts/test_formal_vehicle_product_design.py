from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from validate_formal_vehicle_product_design import (
    DEFAULT_URDF,
    ProductDesignError,
    validate_product_design,
)


def test_committed_formal_vehicle_passes_product_design_gate() -> None:
    result = validate_product_design()
    assert result["status"] == "FORMAL_PRODUCT_BODYWORK_DESIGN_GATE_PASSED"
    assert result["bodywork_mesh_visual_count"] >= 35
    assert result["service_hinge_count"] == 4
    assert result["service_latch_count"] == 4
    assert result["all_service_doors_have_chassis_brackets_limits_and_latches"] is True
    assert all(result["palette"].values())


def test_gate_rejects_missing_front_cowl(tmp_path) -> None:
    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    target = next(link for link in root.findall("link") if link.attrib["name"] == "bodywork_front_cowl_link")
    root.remove(target)
    changed = tmp_path / "missing-front-cowl.urdf"
    tree.write(changed, encoding="unicode")
    with pytest.raises(ProductDesignError, match="required product body links missing"):
        validate_product_design(changed)


def test_gate_rejects_monochrome_bodywork(tmp_path) -> None:
    tree = ET.parse(DEFAULT_URDF)
    for link in tree.getroot().findall("link"):
        if link.attrib["name"].startswith("bodywork_"):
            for color in link.findall("visual/material/color"):
                color.attrib["rgba"] = "0.8 0.8 0.8 1"
    changed = tmp_path / "monochrome.urdf"
    tree.write(changed, encoding="unicode")
    with pytest.raises(ProductDesignError, match="material hierarchy"):
        validate_product_design(changed)


def test_gate_rejects_fixed_service_door(tmp_path) -> None:
    tree = ET.parse(DEFAULT_URDF)
    hinge = next(
        joint
        for joint in tree.getroot().findall("joint")
        if joint.attrib["name"] == "bodywork_power_service_door_hinge_joint"
    )
    hinge.attrib["type"] = "fixed"
    changed = tmp_path / "fixed-service-door.urdf"
    tree.write(changed, encoding="unicode")
    with pytest.raises(ProductDesignError, match="physical revolute hinge"):
        validate_product_design(changed)


def test_gate_rejects_missing_service_door_latch(tmp_path) -> None:
    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    latch = next(
        joint
        for joint in root.findall("joint")
        if joint.attrib["name"] == "bodywork_wet_service_door_latch_joint"
    )
    root.remove(latch)
    changed = tmp_path / "missing-service-latch.urdf"
    tree.write(changed, encoding="unicode")
    with pytest.raises(ProductDesignError, match="independent rotary latch"):
        validate_product_design(changed)
