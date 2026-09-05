from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from validate_formal_side_brush_sdf_surface import (
    SideBrushSdfSurfaceError,
    validate_expanded_sdf_text,
)


def _valid_sdf() -> str:
    links = []
    joints = []
    for side in ("left", "right"):
        link = f"{side}_side_brush_link"
        links.append(
            f"""
            <link name="{link}">
              <collision name="{link}_collision">
                <pose>0 0 -0.065 0 0 0</pose>
                <geometry><cylinder><length>0.026</length><radius>0.15</radius></cylinder></geometry>
                <surface>
                  <contact><ode><kp>1500</kp><kd>50</kd><max_vel>0.2</max_vel><min_depth>0.003</min_depth></ode></contact>
                  <friction><ode><mu>0.08</mu><mu2>0.08</mu2></ode></friction>
                </surface>
              </collision>
              <collision name="{link}_fixed_joint_lump__disk_collision"/>
            </link>
            """
        )
        joints.append(
            f"<joint name=\"{side}_side_brush_joint\" type=\"revolute\"><parent>carriage</parent><child>{link}</child></joint>"
        )
    links.append(
        """
        <link name="central_roller_link">
          <collision name="central_roller_link_collision">
            <pose>0 0 0 1.57079632679 0 0</pose>
            <geometry><cylinder><length>0.620</length><radius>0.100</radius></cylinder></geometry>
            <surface>
              <contact><ode><kp>1500</kp><kd>50</kd><max_vel>0.2</max_vel><min_depth>0.003</min_depth></ode></contact>
              <friction><ode><mu>0.08</mu><mu2>0.08</mu2></ode></friction>
            </surface>
          </collision>
        </link>
        """
    )
    joints.append(
        '<joint name="central_roller_joint" type="revolute"><parent>gearbox</parent><child>central_roller_link</child></joint>'
    )
    return f"<sdf version=\"1.11\"><model name=\"vehicle\">{''.join(links)}{''.join(joints)}</model></sdf>"


def test_accepts_default_named_direct_collision_with_complete_surface() -> None:
    report = validate_expanded_sdf_text(_valid_sdf())
    assert report["status"] == "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED"
    assert report["sides"]["left"]["collision"] == "left_side_brush_link_collision"
    assert report["sides"]["right"]["surface"]["min_depth"] == pytest.approx(0.003)
    assert report["central_roller"]["radius_m"] == pytest.approx(0.100)
    assert report["central_roller"]["surface"]["mu"] == pytest.approx(0.08)
    assert report["runtime_effectiveness"]["dart_effective_from_surface_friction_ode"] == [
        "mu",
        "mu2",
    ]
    assert "kp" in report["runtime_effectiveness"][
        "serialized_but_not_consumed_by_gz_physics_7_dart"
    ]


def test_rejects_surface_dropped_during_sdformat_conversion() -> None:
    root = ET.fromstring(_valid_sdf())
    collision = root.find(".//collision[@name='left_side_brush_link_collision']")
    assert collision is not None
    surface = collision.find("surface")
    assert surface is not None
    collision.remove(surface)
    with pytest.raises(SideBrushSdfSurfaceError, match="has no surface"):
        validate_expanded_sdf_text(ET.tostring(root, encoding="unicode"))


def test_rejects_fixed_lump_collision_name_regression() -> None:
    root = ET.fromstring(_valid_sdf())
    collision = root.find(".//collision[@name='left_side_brush_link_collision']")
    assert collision is not None
    collision.set("name", "left_side_brush_link_fixed_joint_lump__bristle_sweep_collision_collision")
    with pytest.raises(SideBrushSdfSurfaceError, match="default-named direct sweep collision"):
        validate_expanded_sdf_text(ET.tostring(root, encoding="unicode"))


def test_rejects_changed_contact_parameter() -> None:
    root = ET.fromstring(_valid_sdf())
    mu = root.find(".//collision[@name='right_side_brush_link_collision']/surface/friction/ode/mu")
    assert mu is not None
    mu.text = "0.9"
    with pytest.raises(SideBrushSdfSurfaceError, match="right_side_brush_link_collision mu differs"):
        validate_expanded_sdf_text(ET.tostring(root, encoding="unicode"))


def test_rejects_target_collision_not_first_on_rotating_link() -> None:
    root = ET.fromstring(_valid_sdf())
    link = root.find(".//link[@name='right_side_brush_link']")
    target = root.find(".//collision[@name='right_side_brush_link_collision']")
    assert link is not None and target is not None
    link.remove(target)
    link.append(target)
    with pytest.raises(SideBrushSdfSurfaceError, match="must be the first direct collision"):
        validate_expanded_sdf_text(ET.tostring(root, encoding="unicode"))


def test_rejects_missing_central_roller_contact_proxy() -> None:
    root = ET.fromstring(_valid_sdf())
    collision = root.find(".//collision[@name='central_roller_link_collision']")
    assert collision is not None
    surface = collision.find("surface")
    assert surface is not None
    collision.remove(surface)
    with pytest.raises(SideBrushSdfSurfaceError, match="central_roller_link_collision has no surface"):
        validate_expanded_sdf_text(ET.tostring(root, encoding="unicode"))


def test_rejects_central_roller_rigid_envelope_dimension_regression() -> None:
    root = ET.fromstring(_valid_sdf())
    radius = root.find(
        ".//collision[@name='central_roller_link_collision']/geometry/cylinder/radius"
    )
    assert radius is not None
    radius.text = "0.060"
    with pytest.raises(SideBrushSdfSurfaceError, match="central_roller_link_collision radius differs"):
        validate_expanded_sdf_text(ET.tostring(root, encoding="unicode"))
