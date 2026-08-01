from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORLD = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_worlds"
    / "worlds"
    / "sanitation_structured_world.sdf"
)
VEHICLE = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "urdf"
    / "sanitation_vehicle.urdf.xacro"
)
VARIANT_DIR = ROOT / "starter_ws" / "src" / "sanitation_worlds" / "worlds"


def _model(world: ET.Element, name: str) -> ET.Element:
    model = world.find(f"./model[@name='{name}']")
    assert model is not None, name
    return model


def _text(element: ET.Element, path: str) -> str:
    value = element.findtext(path)
    assert value is not None, path
    return " ".join(value.split())


def test_structured_world_keeps_frozen_navigation_anchors() -> None:
    world = ET.parse(WORLD).getroot().find("world")
    assert world is not None
    assert world.get("name") == "sanitation_structured_world"
    anchors = {
        "structured_building_north": ("2.0 9.0 1.0 0 0 0.0", "24.0 0.3 2.0"),
        "structured_curb_south": ("6.0 -9.0 0.25 0 0 0.0", "28.0 0.25 0.5"),
        "structured_lamp_west": ("-3.0 5.0 0.75 0 0 0.0", "0.25 0.25 1.5"),
        "structured_lamp_east": ("11.0 -5.0 0.75 0 0 0.0", "0.25 0.25 1.5"),
        "structured_tree_south": ("0.0 -6.0 0.9 0 0 0.0", "0.45 0.45 1.8"),
        "structured_tree_north": ("12.0 6.0 0.9 0 0 0.0", "0.45 0.45 1.8"),
        "structured_waste_bin": ("-4.0 -2.0 0.5 0 0 0.15", "0.8 0.65 1.0"),
    }
    for name, (pose, collision_size) in anchors.items():
        model = _model(world, name)
        assert _text(model, "pose") == pose
        assert _text(model, "./link/collision/geometry/box/size") == collision_size


def test_world_is_human_readable_offline_and_semantically_complete() -> None:
    root = ET.parse(WORLD).getroot()
    world = root.find("world")
    assert world is not None
    required = {
        "asphalt_ground",
        "north_sidewalk",
        "south_sidewalk",
        "campus_road_markings",
        "campus_crosswalk_west",
        "campus_green_verges",
        "structured_building_north",
        "structured_lamp_west",
        "structured_lamp_east",
        "structured_tree_south",
        "structured_tree_north",
        "trash_bin_obstacle",
        "cardboard_box_obstacle",
        "dynamic_pedestrian_box",
        "trash_bottle_01",
        "trash_can_01",
        "trash_paper_01",
        "leaf_pile_01",
        "puddle_zone",
    }
    names = {model.get("name") for model in world.findall("model")}
    assert required <= names
    assert not root.findall(".//uri"), "runtime-online assets are forbidden"

    assert not _model(world, "campus_road_markings").findall(".//collision")
    assert not _model(world, "campus_crosswalk_west").findall(".//collision")
    assert not _model(world, "campus_green_verges").findall(".//collision")
    assert len(_model(world, "dynamic_pedestrian_box").findall(".//visual")) >= 5
    assert len(_model(world, "leaf_pile_01").findall(".//visual")) >= 4
    assert len(_model(world, "structured_building_north").findall(".//visual")) >= 7


def test_new_furniture_has_collision_and_stays_outside_operation_polygon() -> None:
    world = ET.parse(WORLD).getroot().find("world")
    assert world is not None
    for name in (
        "campus_bench_north",
        "campus_tree_west",
        "campus_tree_east",
        "campus_safety_cones",
    ):
        model = _model(world, name)
        assert model.findall(".//collision"), name
        x, y, *_ = map(float, _text(model, "pose").split())
        # demo_area map x [-2, 6] maps to world x [-10, -2], y [-4, 4].
        assert not (-10.0 <= x <= -2.0 and -4.0 <= y <= 4.0), name


def test_vehicle_visual_detail_preserves_frozen_planar_envelope() -> None:
    root = ET.parse(VEHICLE).getroot()
    base = root.find("./link[@name='base_link']")
    assert base is not None
    visual_names = {item.get("name") for item in base.findall("visual")}
    collision_names = {item.get("name") for item in base.findall("collision")}
    assert {
        "lower_chassis_visual",
        "upper_body_visual",
        "front_service_panel_visual",
        "front_bumper_visual",
        "left_headlamp_visual",
        "right_headlamp_visual",
        "roof_safety_beacon_visual",
        "charging_port_visual",
        "rear_suction_intake_visual",
    } <= visual_names
    assert {"lower_chassis_collision", "upper_body_collision"} <= collision_names

    upper_visual = base.find("./visual[@name='upper_body_visual']")
    upper_collision = base.find("./collision[@name='upper_body_collision']")
    assert upper_visual is not None and upper_collision is not None
    upper_visual_origin = upper_visual.find("origin")
    upper_collision_origin = upper_collision.find("origin")
    assert upper_visual_origin is not None and upper_collision_origin is not None
    assert upper_visual_origin.attrib == upper_collision_origin.attrib
    upper_visual_box = upper_visual.find("geometry/box")
    upper_collision_box = upper_collision.find("geometry/box")
    assert upper_visual_box is not None and upper_collision_box is not None
    assert upper_visual_box.get("size") == upper_collision_box.get("size")
    lower_collision_box = base.find(
        "./collision[@name='lower_chassis_collision']/geometry/box"
    )
    assert lower_collision_box is not None
    assert lower_collision_box.get("size") == "${base_length} ${base_width} ${base_height}"

    wheel_macro_link = root.find(".//link[@name='${side}_wheel_link']")
    assert wheel_macro_link is not None
    assert {item.get("name") for item in wheel_macro_link.findall("visual")} == {
        "tire_visual",
        "hub_visual",
    }
    assert not root.findall(".//mesh"), "the production vehicle remains offline primitive geometry"


def test_multiscale_worlds_have_exact_dimensions_and_realistic_furniture() -> None:
    expected = {
        "small": (30.0, 20.0),
        "medium": (80.0, 50.0),
        "large": (200.0, 100.0),
    }
    for variant, dimensions in expected.items():
        root = ET.parse(VARIANT_DIR / f"sanitation_campus_{variant}.sdf").getroot()
        world = root.find("world")
        assert world is not None
        assert world.get("name") == f"sanitation_campus_{variant}"
        ground = _model(world, "asphalt_ground")
        size = tuple(map(float, _text(ground, "./link/collision/geometry/box/size").split()))
        assert size[:2] == dimensions
        names = {model.get("name") for model in world.findall("model")}
        assert {
            "north_sidewalk",
            "south_sidewalk",
            "bus_shelter_roof",
            "target_cardboard",
            "target_bottle",
            "target_can",
            "target_leaf_pile",
            "target_paper",
            "dimension_x_bar",
            "dimension_y_bar",
        } <= names
        assert not root.findall(".//uri"), "world variants must remain offline"
    assert expected["large"][0] * expected["large"][1] == 20000.0
