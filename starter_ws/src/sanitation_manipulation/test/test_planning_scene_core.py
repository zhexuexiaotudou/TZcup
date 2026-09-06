from pathlib import Path

import pytest

from sanitation_manipulation.planning_scene_core import (
    PlanningSceneReadback,
    SceneObjectReadback,
    load_planning_scene_config,
    next_scene_revision,
    planning_frame_from_srdf,
    validate_scene_readback,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "bin_and_scene.yaml"


_DEFAULT = object()


def _ground_snapshot(config, *, revision=_DEFAULT, **updates):
    ground = config.ground
    allowed_collision_pairs = updates.pop(
        "allowed_collision_pairs",
        tuple(("ground", link) for link in config.ground.allowed_contact_links),
    )
    values = {
        "object_id": ground.object_id,
        "frame_id": ground.frame_id,
        "shape_type": "BOX",
        "dimensions_m": ground.size_m,
        "pose_xyz_m": ground.pose_xyz_m,
        "pose_xyzw": ground.pose_xyzw,
    }
    values.update(updates)
    return PlanningSceneReadback(
        revision=f"{config.scene_revision_prefix}:1" if revision is _DEFAULT else revision,
        world_objects=(SceneObjectReadback(**values),),
        allowed_collision_pairs=allowed_collision_pairs,
    )


def test_config_separates_urdf_links_from_map_world_ground_and_covers_formal_field():
    config = load_planning_scene_config(CONFIG)
    assert config.required_world_objects == ("ground",)
    assert "ground" not in config.required_robot_links
    assert set(config.required_robot_links).isdisjoint(config.required_world_objects)
    assert config.ground.size_m[2] > 0.0
    assert config.ground.top_height_m == pytest.approx(0.0)
    assert config.ground.frame_id == "map"
    assert config.ground.top_height_m == pytest.approx(0.0)
    assert config.ground.size_m[2] == pytest.approx(0.50)
    assert config.ground.size_m[:2] == pytest.approx((220.0, 120.0))
    assert config.ground.pose_xyz_m == pytest.approx((98.0, 0.0, -0.25))
    assert config.ground.source_world_bounds_xy_m == pytest.approx((-100.0, 100.0, -50.0, 50.0))
    assert config.ground.localization_start_xy_yaw == pytest.approx((-98.0, 0.0, 0.0))
    assert config.ground.localization_map_bounds_xy_m == pytest.approx((-2.0, 198.0, -50.0, 50.0))
    assert config.ground.geofence_margin_xy_m == pytest.approx((10.0, 10.0))
    assert config.ground.pose_xyz_m[2] - config.ground.size_m[2] / 2.0 <= config.min_arm_collision_z_m
    assert set(config.ground.allowed_contact_links) == {
        "front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link", "rear_right_wheel_link",
    }


@pytest.mark.parametrize(
    "point",
    ((-2.0, -50.0), (-2.0, 50.0), (198.0, -50.0), (198.0, 50.0)),
)
def test_ground_box_covers_each_public_localization_map_geofence_corner(point):
    config = load_planning_scene_config(CONFIG)
    ground = config.ground
    half_x, half_y = ground.size_m[0] / 2.0, ground.size_m[1] / 2.0
    assert ground.pose_xyz_m[0] - half_x <= point[0] <= ground.pose_xyz_m[0] + half_x
    assert ground.pose_xyz_m[1] - half_y <= point[1] <= ground.pose_xyz_m[1] + half_y


def test_config_rejects_source_or_planning_frame_geometry_that_is_not_the_formal_one(tmp_path):
    bad = CONFIG.read_text(encoding="utf-8").replace(
        "localization_map_bounds_xy_m: [-2.0, 198.0, -50.0, 50.0]",
        "localization_map_bounds_xy_m: [-100.0, 100.0, -50.0, 50.0]",
    )
    path = tmp_path / "wrong_map_bounds.yaml"
    path.write_text(bad, encoding="utf-8")
    with pytest.raises(ValueError, match="exactly once"):
        load_planning_scene_config(path)


def test_planning_frame_is_read_from_moveit_semantic_virtual_joint_not_guessed():
    assert planning_frame_from_srdf(
        '<robot><virtual_joint name="world" type="planar" parent_frame="map" child_link="base_footprint"/></robot>'
    ) == "map"
    with pytest.raises(ValueError, match="exactly one"):
        planning_frame_from_srdf("<robot/>")


def test_readback_requires_exact_revision_identity_box_pose_and_top_height():
    config = load_planning_scene_config(CONFIG)
    validate_scene_readback(config, _ground_snapshot(config))
    with pytest.raises(ValueError, match="revision"):
        validate_scene_readback(config, _ground_snapshot(config, revision=None))
    with pytest.raises(ValueError, match="missing required"):
        validate_scene_readback(config, PlanningSceneReadback(f"{config.scene_revision_prefix}:1", ()))
    with pytest.raises(ValueError, match="shape"):
        validate_scene_readback(config, _ground_snapshot(config, shape_type="PLANE"))
    with pytest.raises(ValueError, match="top height"):
        validate_scene_readback(config, _ground_snapshot(config, pose_xyz_m=(0.0, 0.0, -0.04)))


def test_revisions_are_monotonic_and_ground_acm_is_wheel_only():
    config = load_planning_scene_config(CONFIG)
    assert next_scene_revision(config, None) == f"{config.scene_revision_prefix}:1"
    assert next_scene_revision(config, f"{config.scene_revision_prefix}:7") == f"{config.scene_revision_prefix}:8"
    with pytest.raises(ValueError, match="revision"):
        next_scene_revision(config, "foreign:7")
    snapshot = _ground_snapshot(config, allowed_collision_pairs=(("ground", "tool0"),))
    with pytest.raises(ValueError, match="allowed-collision"):
        validate_scene_readback(config, snapshot)


def test_config_rejects_robot_link_disguised_as_a_world_object(tmp_path):
    bad = CONFIG.read_text(encoding="utf-8").replace(
        "required_world_objects: [ground]", "required_world_objects: [ground, base_link]"
    )
    path = tmp_path / "bad.yaml"
    path.write_text(bad, encoding="utf-8")
    with pytest.raises(ValueError, match="disjoint"):
        load_planning_scene_config(path)
