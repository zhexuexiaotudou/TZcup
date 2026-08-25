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
