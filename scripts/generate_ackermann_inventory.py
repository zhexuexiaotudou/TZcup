#!/usr/bin/env python3
"""Generate compact Ackermann inventories from maintained source files."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

from ackermann_xacro_lite import expand_vehicle, vehicle_expander


ROOT = Path(__file__).resolve().parents[1]
XACRO = ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro"
SIM_LAUNCH = ROOT / "starter_ws/src/sanitation_bringup/launch/sim.launch.py"
EKF = ROOT / "starter_ws/src/sanitation_bringup/config/ekf_ackermann.yaml"
NAV2 = ROOT / "starter_ws/src/sanitation_navigation/config/nav2_ackermann.yaml"
COVERAGE = ROOT / "starter_ws/src/sanitation_coverage/config/coverage_ackermann.yaml"
CONNECTOR = ROOT / "starter_ws/src/sanitation_coverage/sanitation_coverage/ackermann_connector.py"
MISSION = ROOT / "starter_ws/src/sanitation_tasks/config/competition_ackermann_demo_area.yaml"


def source(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def write_json(directory: Path, name: str, payload: dict) -> None:
    payload = {"schema_version": 1, "generated_by": source(Path(__file__)), **payload}
    (directory / name).write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def joint_records(root: ET.Element) -> list[dict]:
    records = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        axis = joint.find("axis")
        records.append({
            "name": joint.get("name"),
            "type": joint.get("type"),
            "parent": (joint.find("parent").get("link") if joint.find("parent") is not None else None),
            "child": (joint.find("child").get("link") if joint.find("child") is not None else None),
            "axis": axis.get("xyz") if axis is not None else None,
            "limit": dict(limit.attrib) if limit is not None else None,
        })
    return records


def plugin_records(root: ET.Element) -> list[dict]:
    return [
        {
            "filename": plugin.get("filename"),
            "name": plugin.get("name"),
            "parameters": {child.tag: (child.text or "").strip() for child in plugin},
        }
        for plugin in root.findall(".//plugin")
    ]


def polygon_area(points) -> float:
    return abs(sum(
        float(a[0]) * float(b[1]) - float(b[0]) * float(a[1])
        for a, b in zip(points, points[1:] + points[:1])
    )) * 0.5


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/ackermann_inventory")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    text = XACRO.read_text(encoding="utf-8")
    harness = vehicle_expander("ackermann", {"brush_forward_x": "0.66"})
    ackermann = harness.expand()
    legacy = expand_vehicle("skid_steer_legacy")
    properties = harness.properties
    wheelbase = 2.0 * float(properties["wheel_pos_x"])
    track = 2.0 * float(properties["wheel_pos_y"])
    virtual = math.radians(28.0)
    center_radius = wheelbase / math.tan(virtual)
    inner = math.atan(wheelbase / (center_radius - track / 2.0))
    outer = math.atan(wheelbase / (center_radius + track / 2.0))
    sensor_frames = sorted({
        joint.find("child").get("link") for joint in ackermann.findall("joint")
        if joint.find("child") is not None and any(
            token in joint.find("child").get("link")
            for token in ("laser", "camera", "imu", "gnss")
        )
    })
    inertials = []
    for link in ackermann.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        inertials.append({
            "link": link.get("name"),
            "mass_kg": float(inertial.find("mass").get("value")),
            "origin_xyz": origin.get("xyz") if origin is not None else "0 0 0",
            "inertia": dict(inertial.find("inertia").attrib),
        })
    write_json(output, "vehicle_geometry.json", {
        "source": source(XACRO),
        "vehicle_length_m": float(properties["base_length"]),
        "vehicle_width_m": float(properties["base_width"]),
        "wheel_radius_m": float(properties["physical_wheel_radius"]),
        "wheel_width_m": float(properties["wheel_width"]),
        "wheelbase_m": wheelbase,
        "front_track_m": track,
        "rear_track_m": track,
        "front_axle_x_m": float(properties["wheel_pos_x"]),
        "rear_axle_x_m": -float(properties["wheel_pos_x"]),
        "wheel_y_m": [float(properties["wheel_pos_y"]), -float(properties["wheel_pos_y"])],
        "body_footprint_m": [[0.575, 0.52], [0.575, -0.52], [-0.575, -0.52], [-0.575, 0.52]],
        "brush_forward_x_m": 0.66,
        "brush_center_y_m": float(properties["brush_center_y"]),
        "sensor_frames": sensor_frames,
        "mass_and_inertia_by_link": inertials,
        "frozen_steering": {
            "virtual_max_deg": 28.0,
            "inner_wheel_deg": math.degrees(inner),
            "outer_wheel_deg": math.degrees(outer),
            "center_turning_radius_m": center_radius,
            "physical_joint_limit_deg": 38.5,
            "gazebo_plugin_clamp_rad": math.asin(wheelbase / center_radius),
        },
    })
    write_json(output, "wheel_joint_inventory.json", {
        "source": source(XACRO),
        "profiles": {
            "ackermann": joint_records(ackermann),
            "skid_steer_legacy": joint_records(legacy),
        },
    })
    write_json(output, "drive_plugin_inventory.json", {
        "source": source(XACRO),
        "profiles": {
            "ackermann": plugin_records(ackermann),
            "skid_steer_legacy": plugin_records(legacy),
        },
    })

    launch_text = SIM_LAUNCH.read_text(encoding="utf-8")
    bridged_topics = sorted(set(re.findall(r"['\"](/[^@'\"]+)@", launch_text)))
    write_json(output, "ros_gz_bridge_inventory.json", {
        "source": source(SIM_LAUNCH),
        "topics_found": bridged_topics,
        "ackermann_raw_odom": "/wheel/odom_raw",
        "legacy_raw_odom": "/odom/unfiltered",
    })
    write_json(output, "odometry_tf_inventory.json", {
        "sources": [source(XACRO), source(SIM_LAUNCH), source(EKF)],
        "ackermann_chain": [
            "rear traction + actual front steering", "/wheel/odom_raw",
            "/measurements/wheel_odom", "EKF", "/odom",
        ],
        "tf_owner": "robot_localization EKF",
        "tf_edge": "odom -> base_footprint",
        "ground_truth_control_input": False,
    })
    ekf = yaml.safe_load(EKF.read_text(encoding="utf-8"))["ekf_filter_node"]["ros__parameters"]
    write_json(output, "localization_inventory.json", {
        "source": source(EKF),
        "world_frame": ekf["world_frame"],
        "base_link_frame": ekf["base_link_frame"],
        "publish_tf": ekf["publish_tf"],
        "wheel_input": ekf["odom0"],
        "imu_input": ekf["imu0"],
        "wheel_config": ekf["odom0_config"],
        "imu_config": ekf["imu0_config"],
    })
    nav = yaml.safe_load(NAV2.read_text(encoding="utf-8"))
    planner = nav["planner_server"]["ros__parameters"]["GridBased"]
    controllers = nav["controller_server"]["ros__parameters"]
    write_json(output, "nav2_inventory.json", {
        "source": source(NAV2),
        "planner": planner,
        "controllers": {
            name: controllers[name]
            for name in controllers["controller_plugins"]
        },
        "behavior_plugins": nav["behavior_server"]["ros__parameters"]["behavior_plugins"],
        "behavior_trees": {
            key: value for key, value in nav["bt_navigator"]["ros__parameters"].items()
            if key.startswith("default_") and key.endswith("_bt_xml")
        },
    })
    coverage = yaml.safe_load(COVERAGE.read_text(encoding="utf-8"))
    connector_text = CONNECTOR.read_text(encoding="utf-8")
    classes = re.findall(r'connector_class["\']?\s*[:=]\s*["\']([A-Z_]+)', connector_text)
    write_json(output, "coverage_connector_inventory.json", {
        "sources": [source(COVERAGE), source(CONNECTOR)],
        "parameters": coverage,
        "connector_classes_found": sorted(set(classes)),
        "forbidden_component_tokens_absent": {
            "ROTATE": 'ComponentType.ROTATE' not in connector_text,
            "SHIFT": 'ComponentType.SHIFT' not in connector_text,
            "Spin": "Spin" not in connector_text,
        },
    })
    mission = yaml.safe_load(MISSION.read_text(encoding="utf-8"))
    write_json(output, "task_geometry_inventory.json", {
        "source": source(MISSION),
        "outer_polygon": mission["outer_polygon"],
        "outer_area_m2": polygon_area(mission["outer_polygon"]),
        "cleanable_outer_polygon": mission["cleanable_outer_polygon"],
        "cleanable_area_m2": polygon_area(mission["cleanable_outer_polygon"]),
        "target_count": len(mission["cleaning_targets"]),
        "targets": mission["cleaning_targets"],
        "turning_apron_is_cleanable": False,
    })
    write_json(output, "regression_inventory.json", {
        "sources": [source(XACRO), "scripts/run_visual_demo.sh", "scripts/ci_fast.py"],
        "legacy_drive_profile": "skid_steer_legacy",
        "legacy_coverage_profiles": ["optimized", "legacy"],
        "legacy_world": "sanitation_competition_demo.sdf",
        "legacy_default_pending_formal_ackermann_evidence": True,
    })
    print(json.dumps({"output": str(output), "files": len(list(output.glob("*.json")))}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
