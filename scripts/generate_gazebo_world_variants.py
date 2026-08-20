#!/usr/bin/env python3
"""Generate the three license-clean TZcup Gazebo campus worlds.

The worlds use only SDF primitives so they remain offline, deterministic and
redistributable. Generated files are committed to keep ROS installs simple.
"""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
WORLD_DIR = ROOT / "starter_ws" / "src" / "sanitation_worlds" / "worlds"

VARIANTS = {
    "small": {"width": 30.0, "height": 20.0, "label": "DEMO 30 x 20 m", "trees": 8},
    "medium": {"width": 80.0, "height": 50.0, "label": "VALIDATION 80 x 50 m", "trees": 18},
    "large": {"width": 200.0, "height": 100.0, "label": "COMPETITION 200 x 100 m = 20,000 m2", "trees": 34},
}


def pose(parent, values):
    ET.SubElement(parent, "pose").text = " ".join(str(value) for value in values)


def material(visual, color):
    node = ET.SubElement(visual, "material")
    ET.SubElement(node, "ambient").text = color
    ET.SubElement(node, "diffuse").text = color
    ET.SubElement(node, "specular").text = "0.04 0.04 0.04 1"


def box_model(world, name, xyz, size, color, collision=True):
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    pose(model, (*xyz, 0, 0, 0))
    link = ET.SubElement(model, "link", {"name": "link"})
    if collision:
        col = ET.SubElement(link, "collision", {"name": "collision"})
        geometry = ET.SubElement(col, "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = " ".join(map(str, size))
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(ET.SubElement(geometry, "box"), "size").text = " ".join(map(str, size))
    material(visual, color)
    return model


def cylinder_model(world, name, xyz, radius, length, color, collision=True):
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    pose(model, (*xyz, 0, 0, 0))
    link = ET.SubElement(model, "link", {"name": "link"})
    for kind in (["collision", "visual"] if collision else ["visual"]):
        node = ET.SubElement(link, kind, {"name": kind})
        geometry = ET.SubElement(node, "geometry")
        shape = ET.SubElement(geometry, "cylinder")
        ET.SubElement(shape, "radius").text = str(radius)
        ET.SubElement(shape, "length").text = str(length)
        if kind == "visual":
            material(node, color)
    return model


def tree(world, index, x, y):
    model = ET.SubElement(world, "model", {"name": f"tree_{index:02d}"})
    ET.SubElement(model, "static").text = "true"
    pose(model, (x, y, 0, 0, 0, 0))
    link = ET.SubElement(model, "link", {"name": "link"})
    trunk = ET.SubElement(link, "collision", {"name": "trunk_collision"})
    geom = ET.SubElement(trunk, "geometry")
    cyl = ET.SubElement(geom, "cylinder")
    ET.SubElement(cyl, "radius").text = "0.18"
    ET.SubElement(cyl, "length").text = "2.2"
    pose(trunk, (0, 0, 1.1, 0, 0, 0))
    trunk_v = ET.SubElement(link, "visual", {"name": "trunk"})
    pose(trunk_v, (0, 0, 1.1, 0, 0, 0))
    geom = ET.SubElement(trunk_v, "geometry")
    cyl = ET.SubElement(geom, "cylinder")
    ET.SubElement(cyl, "radius").text = "0.18"
    ET.SubElement(cyl, "length").text = "2.2"
    material(trunk_v, "0.30 0.17 0.07 1")
    crown = ET.SubElement(link, "visual", {"name": "crown"})
    pose(crown, (0, 0, 2.7, 0, 0, 0))
    geom = ET.SubElement(crown, "geometry")
    ET.SubElement(ET.SubElement(geom, "sphere"), "radius").text = "1.15"
    material(crown, "0.08 0.34 0.10 1")


def street_light(world, index, x, y):
    cylinder_model(world, f"lamp_post_{index:02d}", (x, y, 2.2), 0.07, 4.4, "0.12 0.14 0.16 1")
    box_model(world, f"lamp_head_{index:02d}", (x + 0.35, y, 4.25), (0.75, 0.22, 0.16), "0.92 0.88 0.66 1", False)


def add_world_details(world, key, width, height, tree_count):
    # Central road and tactile sidewalks keep the small task readable while
    # the outer campus expands independently for medium and competition maps.
    box_model(world, "asphalt_ground", (0, 0, -0.05), (width, height, 0.10), "0.13 0.15 0.17 1")
    road_width = min(18.0, height * 0.48)
    # These bands are flush accessible paving within the cleaning envelope,
    # not continuous curbs.  A full-width collision partitions the 20,000 m2
    # formal world into unreachable strips while remaining below the planar
    # lidar, so Nav2 cannot plan around the physical obstruction.  Keep the
    # material / visual-domain change as a thin visual layer over the single
    # asphalt contact plane; duplicate coplanar contacts also destabilize the
    # Ackermann tire constraints at an oblique crossing.
    sidewalk_height = 0.002
    sidewalk_z = sidewalk_height / 2.0
    box_model(world, "north_sidewalk", (0, road_width / 2 + 1.5, sidewalk_z), (width - 2, 3.0, sidewalk_height), "0.55 0.56 0.54 1", False)
    box_model(world, "south_sidewalk", (0, -road_width / 2 - 1.5, sidewalk_z), (width - 2, 3.0, sidewalk_height), "0.55 0.56 0.54 1", False)
    box_model(world, "north_green_verge", (0, road_width / 2 + 3.7, 0.03), (width - 3, 1.4, 0.06), "0.12 0.36 0.14 1", False)
    box_model(world, "south_green_verge", (0, -road_width / 2 - 3.7, 0.03), (width - 3, 1.4, 0.06), "0.12 0.36 0.14 1", False)

    # Road markings, parking bays and a full-width crosswalk.
    dash_limit = width / 2 - 2
    x = -dash_limit
    dash_index = 0
    while x <= dash_limit:
        box_model(world, f"center_dash_{dash_index:02d}", (x, 0, 0.012), (2.2, 0.12, 0.012), "0.95 0.74 0.10 1", False)
        dash_index += 1
        x += 5.0
    for index, stripe_x in enumerate([-9.4, -9.0, -8.6, -8.2, -7.8, -7.4, -7.0, -6.6]):
        box_model(world, f"crosswalk_{index:02d}", (stripe_x, 0, 0.014), (0.28, road_width - 2, 0.014), "0.92 0.93 0.90 1", False)
    parking_start = max(3.0, -width / 2 + 14.0)
    for index in range(max(2, int((width / 2 - parking_start - 2) // 3.2))):
        px = parking_start + index * 3.2
        for side in (-1, 1):
            box_model(world, f"parking_line_{side}_{index}", (px, side * (road_width / 2 - 1.1), 0.013), (0.08, 2.2, 0.012), "0.86 0.88 0.86 1", False)

    # Buildings sit outside the operational road, with glass entrances and
    # roof equipment to read as real campus architecture from the overview.
    if height >= 45:
        building_y = height / 2 - 7.0
        for index, x in enumerate((-width * 0.30, 0.0, width * 0.30)):
            bw = min(18.0, width * 0.22)
            box_model(world, f"building_north_{index}", (x, building_y, 4.0), (bw, 9.0, 8.0), "0.55 0.60 0.65 1")
            box_model(world, f"glass_entry_north_{index}", (x, building_y - 4.51, 1.7), (3.0, 0.08, 3.2), "0.08 0.31 0.48 1", False)
            box_model(world, f"roof_unit_north_{index}", (x + 3.0, building_y, 8.35), (2.2, 1.8, 0.7), "0.28 0.31 0.34 1")
        box_model(world, "service_building_south", (width * 0.28, -building_y, 3.0), (min(28.0, width * 0.30), 8.0, 6.0), "0.62 0.50 0.38 1")
    else:
        box_model(world, "demo_service_center", (5.0, height / 2 - 2.0, 2.4), (8.0, 3.0, 4.8), "0.55 0.60 0.65 1")
        box_model(world, "demo_glass_entry", (5.0, height / 2 - 3.51, 1.4), (2.0, 0.08, 2.6), "0.08 0.31 0.48 1", False)

    # Evenly distributed trees and lamps, always outside the central road.
    top_y = min(height / 2 - 1.2, road_width / 2 + 3.7)
    usable = max(8.0, width - 8.0)
    for index in range(tree_count):
        fraction = index / max(1, tree_count - 1)
        tx = -usable / 2 + usable * fraction
        ty = top_y if index % 2 == 0 else -top_y
        tree(world, index, tx, ty)
    lamp_count = max(4, int(width // 18))
    for index in range(lamp_count):
        lx = -width / 2 + 6 + index * ((width - 12) / max(1, lamp_count - 1))
        street_light(world, index, lx, road_width / 2 + 0.7)

    # Human-readable street furniture and cleaning targets.
    for index, x in enumerate((-4.0, 5.5)):
        box_model(world, f"bench_seat_{index}", (x, road_width / 2 + 1.5, 0.55), (1.8, 0.48, 0.12), "0.40 0.23 0.09 1")
        box_model(world, f"bench_back_{index}", (x, road_width / 2 + 1.72, 0.95), (1.8, 0.10, 0.75), "0.40 0.23 0.09 1")
    for index, x in enumerate((-5.5, 6.8)):
        box_model(world, f"waste_bin_{index}", (x, -road_width / 2 - 1.4, 0.55), (0.48, 0.48, 1.1), "0.08 0.38 0.24 1")
    box_model(world, "bus_shelter_roof", (1.5, -road_width / 2 - 2.0, 2.35), (5.0, 2.0, 0.18), "0.17 0.24 0.31 1")
    box_model(world, "bus_shelter_glass", (1.5, -road_width / 2 - 2.85, 1.25), (5.0, 0.08, 2.2), "0.12 0.38 0.52 0.72", False)

    # Five target classes used elsewhere in the project; kept out of the
    # showcase swaths so this visual upgrade does not change navigation truth.
    box_model(world, "target_cardboard", (9.5, 5.4, 0.20), (0.55, 0.40, 0.40), "0.58 0.36 0.15 1")
    box_model(world, "target_bottle", (10.4, 5.7, 0.12), (0.14, 0.14, 0.24), "0.08 0.48 0.66 1")
    box_model(world, "target_can", (11.0, 5.2, 0.10), (0.13, 0.13, 0.20), "0.64 0.67 0.69 1")
    box_model(world, "target_leaf_pile", (8.8, 6.2, 0.04), (1.2, 0.7, 0.08), "0.44 0.25 0.05 1", False)
    box_model(world, "target_paper", (10.2, 6.5, 0.025), (0.42, 0.32, 0.03), "0.90 0.88 0.75 1", False)

    # Dimension reference bars make the map scale visible in Gazebo.
    box_model(world, "dimension_x_bar", (0, -height / 2 + 0.4, 0.03), (width - 1, 0.10, 0.06), "0.10 0.62 0.92 1", False)
    box_model(world, "dimension_y_bar", (-width / 2 + 0.4, 0, 0.03), (0.10, height - 1, 0.06), "0.10 0.62 0.92 1", False)


def build(key, spec):
    sdf = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(sdf, "world", {"name": f"sanitation_campus_{key}"})
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    sensors = ET.SubElement(world, "plugin", {"filename": "gz-sim-sensors-system", "name": "gz::sim::systems::Sensors"})
    ET.SubElement(sensors, "render_engine").text = "ogre2"
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
    ET.SubElement(physics, "max_step_size").text = "0.005"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.58 0.60 0.62 1"
    ET.SubElement(scene, "background").text = "0.72 0.84 0.94 1"
    ET.SubElement(scene, "shadows").text = "true"
    sun = ET.SubElement(world, "light", {"type": "directional", "name": "sun"})
    ET.SubElement(sun, "cast_shadows").text = "true"
    pose(sun, (0, 0, 80, 0, 0, 0))
    ET.SubElement(sun, "diffuse").text = "0.96 0.91 0.82 1"
    ET.SubElement(sun, "direction").text = "-0.45 0.25 -1.0"
    add_world_details(world, key, spec["width"], spec["height"], spec["trees"])
    ET.indent(sdf, space="  ")
    output = WORLD_DIR / f"sanitation_campus_{key}.sdf"
    output.write_text("<?xml version='1.0' encoding='utf-8'?>\n" + ET.tostring(sdf, encoding="unicode") + "\n", encoding="utf-8")
    return output


def main():
    for key, spec in VARIANTS.items():
        print(build(key, spec))


if __name__ == "__main__":
    main()
