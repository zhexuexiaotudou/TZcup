from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
CLEANING_XACRO = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "urdf"
    / "high_fidelity"
    / "cleaning_mechanism.xacro"
)
CONTROL_XACRO = CLEANING_XACRO.with_name("control_interfaces.xacro")
XACRO = "{http://ros.org/wiki/xacro}"


def _xyz(value: str) -> tuple[float, float, float]:
    return tuple(float(component) for component in value.split())


def _named_joint(root: ET.Element, name: str) -> ET.Element:
    return next(joint for joint in root.findall(".//joint") if joint.get("name") == name)


def _macro(root: ET.Element, name: str) -> ET.Element:
    return next(
        element
        for element in root.findall(f".//{XACRO}macro")
        if element.get("name") == name
    )


def _calls(root: ET.Element, name: str) -> list[ET.Element]:
    return root.findall(f".//{XACRO}{name}")


def test_cleaning_lift_keeps_single_commanded_100_mm_axis() -> None:
    root = ET.parse(CLEANING_XACRO).getroot()
    lift = _named_joint(root, "cleaning_lift_joint")
    assert lift.get("type") == "prismatic"
    assert lift.find("parent").get("link") == "cleaning_mechanism_mount_link"
    assert lift.find("child").get("link") == "cleaning_lift_carriage_link"
    assert _xyz(lift.find("origin").get("xyz")) == pytest.approx((0.0, 0.0, 0.045))
    assert _xyz(lift.find("axis").get("xyz")) == pytest.approx((0.0, 0.0, -1.0))
    limit = lift.find("limit")
    assert float(limit.get("lower")) == pytest.approx(0.0)
    # The physical solver has a 20 um DART-only release clearance.  The
    # product/controller stroke remains exactly 100 mm below.
    assert float(limit.get("upper")) == pytest.approx(0.10002)
    assert float(limit.get("velocity")) == pytest.approx(0.0048)
    assert float(limit.get("effort")) == pytest.approx(300.0)

    control_root = ET.parse(CONTROL_XACRO).getroot()
    commanded = [
        element
        for element in control_root.iter()
        if element.tag.rsplit("}", 1)[-1] == "hf_position_joint"
        and element.get("name") == "cleaning_lift_joint"
    ]
    assert len(commanded) == 1
    assert commanded[0].get("lower") == "0.0"
    assert commanded[0].get("upper") == "0.100"
    assert commanded[0].get("initial_position") == "0.0"
    assert float(limit.get("upper")) - float(commanded[0].get("upper")) == pytest.approx(
        0.00002
    )
    assert not any("lift_slider" in (element.get("name") or "") for element in control_root.iter())


def test_four_guide_centres_match_two_moving_bearing_side_plates() -> None:
    root = ET.parse(CLEANING_XACRO).getroot()
    guides = {
        (_xyz(f'{call.get("x")} {call.get("y")} 0')[0], _xyz(f'{call.get("x")} {call.get("y")} 0')[1])
        for call in _calls(root, "hf_lift_guide")
    }
    expected = {(0.18, 0.25), (0.18, -0.25), (-0.18, 0.25), (-0.18, -0.25)}
    assert guides == expected
    slider_calls = _calls(root, "hf_lift_slider_plate")
    assert {(0.18, float(call.get("y"))) for call in slider_calls} | {
        (-0.18, float(call.get("y"))) for call in slider_calls
    } == expected

    slider_macro = _macro(root, "hf_lift_slider_plate")
    slider_joint = slider_macro.find("joint")
    assert slider_joint.get("type") == "fixed"
    assert slider_joint.find("parent").get("link") == "cleaning_lift_carriage_link"
    assert slider_joint.find("origin").get("xyz") == "0 ${y} 0"
    mesh = slider_macro.find("link/visual/geometry/mesh")
    scale_x = _xyz(mesh.get("scale"))[0]
    assert 0.110 * scale_x == pytest.approx(0.180)


def test_bearing_blocks_follow_full_lift_travel_inside_guide_span() -> None:
    root = ET.parse(CLEANING_XACRO).getroot()
    lift = _named_joint(root, "cleaning_lift_joint")
    carriage_origin_z = _xyz(lift.find("origin").get("xyz"))[2]
    lift_axis_z = _xyz(lift.find("axis").get("xyz"))[2]
    lower = float(lift.find("limit").get("lower"))
    upper = float(lift.find("limit").get("upper"))

    guide_macro = _macro(root, "hf_lift_guide")
    guide_origin_z = float(guide_macro.find("joint/origin").get("xyz").split()[2])
    guide_length = float(guide_macro.find("link/collision/geometry/cylinder").get("length"))
    guide_low = guide_origin_z - guide_length / 2.0
    guide_high = guide_origin_z + guide_length / 2.0

    slider_macro = _macro(root, "hf_lift_slider_plate")
    bearing_length = float(
        next(slider_macro.iter(f"{XACRO}hf_cleaning_box_inertial")).get("sz")
    )
    for lift_position in (lower, upper):
        centre_z = carriage_origin_z + lift_axis_z * lift_position
        assert centre_z - bearing_length / 2.0 >= guide_low - 1e-9
        assert centre_z + bearing_length / 2.0 <= guide_high + 1e-9

    # The DART solver may travel 20 um beyond the product stroke before its
    # hard constraint.  Product q=0.100 is still the ground-tangent endpoint.
    assert (carriage_origin_z + lift_axis_z * upper) - (
        carriage_origin_z + lift_axis_z * lower
    ) == pytest.approx(-0.10002)
    assert upper - 0.100 == pytest.approx(0.00002)


def test_mesh_slider_plates_preserve_nonprimitive_visual_and_mass_budget() -> None:
    root = ET.parse(CLEANING_XACRO).getroot()
    slider_macro = _macro(root, "hf_lift_slider_plate")
    visual = slider_macro.find("link/visual")
    assert visual.find("geometry/mesh") is not None
    assert visual.find("geometry/box") is None
    assert visual.find("geometry/cylinder") is None

    mass = float(
        next(
            element
            for element in slider_macro.iter(f"{XACRO}hf_cleaning_box_inertial")
        ).get("mass")
    )
    assert math.fsum([mass] * 2) == pytest.approx(0.440)


def test_fixed_decorative_lift_linkages_are_removed() -> None:
    root = ET.parse(CLEANING_XACRO).getroot()
    assert not _calls(root, "hf_lift_linkage")
    assert not any(
        "lift_linkage" in (element.get("name") or "")
        for element in root.iter()
    )
