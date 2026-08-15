#!/usr/bin/env python3
"""Fast ROS-independent contract tests for both vehicle drive profiles."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_coverage"))

import pytest

from ackermann_xacro_lite import expand_vehicle
from sanitation_coverage import ackermann_model


def _joints(root):
    return {joint.get("name"): joint for joint in root.iter("joint")}


def _links(root):
    return {link.get("name"): link for link in root.iter("link")}


def _plugins(root):
    return [plugin for plugin in root.iter("plugin")]


def _limit(joint):
    limit = joint.find("limit")
    if limit is None:
        return None
    return {
        "lower": float(limit.get("lower", "0")),
        "upper": float(limit.get("upper", "0")),
        "effort": float(limit.get("effort", "0")),
        "velocity": float(limit.get("velocity", "0")),
    }


def _dynamics(joint):
    dynamics = joint.find("dynamics")
    if dynamics is None:
        return None
    return {name: float(dynamics.get(name, "0.0")) for name in ("damping",)}


def test_drive_model_argument_defaults_to_product_ackermann():
    source = (
        ROOT / "starter_ws" / "src" / "sanitation_vehicle_description" / "urdf"
        / "sanitation_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert 'xacro:arg name="drive_model" default="ackermann"' in source


def test_both_profiles_expand():
    for model in ("ackermann", "skid_steer_legacy"):
        root = expand_vehicle(model)
        assert root.tag == "robot"


def test_drive_model_rejects_unknown_values():
    from ackermann_xacro_lite import expand_vehicle as expand

    with pytest.raises(ValueError):
        expand("hovercraft")


def test_ackermann_joint_topology_limits_and_axes():
    root = expand_vehicle("ackermann")
    joints = _joints(root)
    assert set(joints) >= {
        "front_left_steering_joint",
        "front_right_steering_joint",
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    }
    for side in ("front_left", "front_right"):
        steering = joints[f"{side}_steering_joint"]
        assert steering.get("type") == "revolute"
        assert steering.find("axis").get("xyz") == "0 0 1"
        assert steering.find("parent").get("link") == "base_link"
        assert steering.find("child").get("link") == f"{side}_steering_link"
        limit = _limit(steering)
        assert limit["lower"] == pytest.approx(
            -ackermann_model.PHYSICAL_STEERING_LIMIT_RAD, abs=1e-6
        )
        assert limit["upper"] == pytest.approx(
            ackermann_model.PHYSICAL_STEERING_LIMIT_RAD, abs=1e-6
        )
        assert limit["effort"] > 0.0
        assert limit["velocity"] > 0.0
        assert _dynamics(steering)["damping"] > 0.0
        wheel = joints[f"{side}_wheel_joint"]
        assert wheel.get("type") == "continuous"
        assert wheel.find("parent").get("link") == f"{side}_steering_link"
        assert wheel.find("child").get("link") == f"{side}_wheel_link"
        assert wheel.find("axis").get("xyz") == "0 0 1"
        assert wheel.find("origin").get("rpy").startswith("-1.570796")
        wheel_limit = _limit(wheel)
        assert wheel_limit["effort"] > 0.0 and wheel_limit["velocity"] > 0.0
        assert _dynamics(wheel)["damping"] >= 0.0
    for side in ("rear_left", "rear_right"):
        wheel = joints[f"{side}_wheel_joint"]
        assert wheel.get("type") == "continuous"
        assert wheel.find("parent").get("link") == "base_link"
        assert wheel.find("child").get("link") == f"{side}_wheel_link"
        assert wheel.find("axis").get("xyz") == "0 0 1"
        assert wheel.find("origin").get("rpy").startswith("-1.570796")


def test_ackermann_front_wheels_are_children_of_steering_links():
    root = expand_vehicle("ackermann")
    links = _links(root)
    for side in ("front_left", "front_right"):
        assert f"{side}_steering_link" in links
        assert f"{side}_wheel_link" in links


def test_ackermann_wheel_contacts_freeze_axle_friction_direction():
    root = expand_vehicle("ackermann")
    wheel_surfaces = {
        element.get("reference"): element
        for element in root.iter("gazebo")
        if element.get("reference", "").endswith("_wheel_link")
    }
    assert len(wheel_surfaces) == 4
    for surface in wheel_surfaces.values():
        assert surface.find("fdir1").text == "0 0 1"
        assert float(surface.find("mu1").text) == pytest.approx(0.5)
        assert float(surface.find("mu2").text) == pytest.approx(1.0)

    legacy = expand_vehicle("skid_steer_legacy")
    assert all(
        element.find("fdir1") is None
        for element in legacy.iter("gazebo")
        if element.get("reference", "").endswith("_wheel_link")
    )


def test_ackermann_plugin_counts_and_configuration():
    root = expand_vehicle("ackermann")
    plugins = _plugins(root)
    ackermann_plugins = [
        plugin
        for plugin in plugins
        if plugin.get("filename") == "gz-sim-ackermann-steering-system"
    ]
    diff_plugins = [
        plugin
        for plugin in plugins
        if plugin.get("filename") == "gz-sim-diff-drive-system"
    ]
    assert len(ackermann_plugins) == 1
    assert len(diff_plugins) == 0
    plugin = ackermann_plugins[0]
    assert plugin.get("name") == "gz::sim::systems::AckermannSteering"
    expected = {
        "left_steering_joint": "front_left_steering_joint",
        "right_steering_joint": "front_right_steering_joint",
        "wheel_base": "0.76",
        "wheel_separation": "0.80",
        "kingpin_width": "0.80",
        "wheel_radius": "0.14",
        "odom_topic": "/wheel/odom_raw",
        "frame_id": "odom",
        "child_frame_id": "base_footprint",
        "odom_publish_frequency": "50",
        "topic": "/cmd_vel",
    }
    for key, value in expected.items():
        child = plugin.find(key)
        assert child is not None and child.text == value, key
    assert [item.text for item in plugin.findall("left_joint")] == [
        "rear_left_wheel_joint"
    ]
    assert [item.text for item in plugin.findall("right_joint")] == [
        "rear_right_wheel_joint"
    ]
    clamp = float(plugin.find("steering_limit").text)
    assert clamp == pytest.approx(
        ackermann_model.plugin_steering_clamp_rad(), abs=1e-9
    )
    assert clamp > ackermann_model.MAX_VIRTUAL_STEERING_RAD
    for key in (
        "min_velocity", "max_velocity", "min_acceleration",
        "max_acceleration", "min_jerk", "max_jerk", "steer_p_gain",
    ):
        assert plugin.find(key) is not None


def test_legacy_retains_four_wheel_joints_and_diff_drive():
    root = expand_vehicle("skid_steer_legacy")
    joints = _joints(root)
    wheel_joints = [
        name
        for name, joint in joints.items()
        if name in {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        }
    ]
    assert sorted(wheel_joints) == [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]
    assert all(joints[name].get("type") == "continuous" for name in wheel_joints)
    assert all(
        joints[name].find("parent").get("link") == "base_link"
        for name in wheel_joints
    )
    plugins = _plugins(root)
    diff = [
        plugin
        for plugin in plugins
        if plugin.get("filename") == "gz-sim-diff-drive-system"
    ]
    assert len(diff) == 1
    assert not [
        plugin
        for plugin in plugins
        if plugin.get("filename") == "gz-sim-ackermann-steering-system"
    ]
    assert not [
        joint
        for joint in root.iter("joint")
        if joint.get("name", "").endswith("steering_joint")
    ]


def test_ackermann_unique_joint_writers():
    for model in ("ackermann", "skid_steer_legacy"):
        root = expand_vehicle(model)
        names = [joint.get("name") for joint in root.iter("joint")]
        assert len(names) == len(set(names))
        plugins = _plugins(root)
        writers = [
            plugin
            for plugin in plugins
            if plugin.get("filename")
            in (
                "gz-sim-ackermann-steering-system",
                "gz-sim-diff-drive-system",
            )
        ]
        assert len(writers) == 1


def test_both_profiles_publish_measured_joint_states_without_commanding_them():
    for model in ("ackermann", "skid_steer_legacy"):
        plugins = _plugins(expand_vehicle(model))
        publishers = [
            plugin for plugin in plugins
            if plugin.get("filename") == "gz-sim-joint-state-publisher-system"
        ]
        assert len(publishers) == 1
        assert publishers[0].get("name") == "gz::sim::systems::JointStatePublisher"
        assert publishers[0].find("topic").text == "/joint_states"


def test_steering_margin_and_geometry_formulas():
    inner = math.degrees(ackermann_model.inner_wheel_angle_rad())
    outer = math.degrees(ackermann_model.outer_wheel_angle_rad())
    margin = ackermann_model.PHYSICAL_STEERING_LIMIT_DEG - inner
    assert margin >= 2.0
    assert inner == pytest.approx(36.44, abs=0.05)
    assert outer == pytest.approx(22.56, abs=0.05)
    assert ackermann_model.minimum_radius_m() == pytest.approx(1.429, abs=0.005)
    assert ackermann_model.plugin_steering_clamp_deg() == pytest.approx(
        32.121, abs=0.01
    )
    radius = ackermann_model.WHEELBASE_M / math.sin(
        ackermann_model.plugin_steering_clamp_rad()
    )
    assert radius == pytest.approx(ackermann_model.minimum_radius_m(), rel=1e-9)


@pytest.mark.parametrize(
    "steering_deg",
    (-38.5, -28.0, -15.0, 0.0, 15.0, 28.0, 38.5),
)
def test_sampled_wheel_clearances_are_positive(steering_deg):
    steering_rad = math.radians(steering_deg)
    chassis = ackermann_model.clearance_wheel_to_chassis(steering_rad)
    brush = ackermann_model.clearance_wheel_to_brush(steering_rad)
    assert chassis >= 0.01
    assert brush >= 0.01


def test_honest_footprint_replaces_legacy_undersized_footprint():
    footprint = ackermann_model.honest_footprint_polygon()
    assert footprint == [
        [pytest.approx(0.82), 0.66],
        [pytest.approx(0.82), -0.66],
        [-0.575, -0.66],
        [-0.575, 0.66],
    ]
    assert ackermann_model.honest_footprint_radius_m() > math.hypot(0.40, 0.36)
    assert ackermann_model.OPERATION_WIDTH_M == pytest.approx(1.32)


def test_ackermann_chassis_collision_uses_wheel_wells():
    root = expand_vehicle("ackermann")
    collision_names = [
        collision.get("name")
        for link in root.iter("link")
        if link.get("name") == "base_link"
        for collision in link.iter("collision")
    ]
    assert "lower_chassis_collision" in collision_names
    assert "lower_chassis_front_stub_collision" in collision_names
    assert "lower_chassis_rear_stub_collision" in collision_names


def test_legacy_keeps_full_width_chassis_collision():
    root = expand_vehicle("skid_steer_legacy")
    collision_names = [
        collision.get("name")
        for link in root.iter("link")
        if link.get("name") == "base_link"
        for collision in link.iter("collision")
    ]
    assert "lower_chassis_collision" in collision_names


def test_ackermann_brush_boom_position():
    root = expand_vehicle("ackermann")
    left = next(
        joint
        for joint in root.iter("joint")
        if joint.get("name") == "left_brush_joint"
    )
    assert left.find("origin").get("xyz").startswith("0.68 0.52 ")
    legacy = expand_vehicle("skid_steer_legacy")
    legacy_left = next(
        joint
        for joint in legacy.iter("joint")
        if joint.get("name") == "left_brush_joint"
    )
    assert legacy_left.find("origin").get("xyz").startswith("0.58 ")


def test_rigid_brush_hubs_are_elevated_above_compliant_bristles():
    root = expand_vehicle("ackermann")
    for side in ("left", "right"):
        link = next(
            item for item in root.iter("link")
            if item.get("name") == f"{side}_brush_link"
        )
        collision = link.find("collision")
        assert collision.find("origin").get("xyz") == "0 0 0.04"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
