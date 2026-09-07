from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "starter_ws/src/sanitation_campus_scenario/sanitation_campus_scenario/generator.py"
SPEC = importlib.util.spec_from_file_location("generator", GENERATOR)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules["generator"] = MODULE
SPEC.loader.exec_module(MODULE)


def _labels(world: str) -> dict[str, str]:
    root = ET.fromstring(world)
    result = {}
    for model in root.findall(".//model"):
        label = model.find("plugin/label")
        if label is not None:
            result[model.attrib["name"]] = label.text or ""
    return result


def test_public_target_models_have_only_the_frozen_label_mapping() -> None:
    config = MODULE.load_config(ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml")
    files = MODULE.generate_episode(config, "formal", "train", 0, 0)
    labels = _labels(files["public/world.sdf"])
    assert set(labels.values()) <= {"1", "2", "3", "4"}
    assert "1" in labels.values() and {"2", "3", "4"} <= set(labels.values())
    assert not any(name.startswith("asset-") or name.startswith("pedestrian-") for name in labels)


def test_formal_default_and_opt_in_gt_chain_is_explicit() -> None:
    campus = (ROOT / "starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus.launch.py").read_text(encoding="utf-8")
    vehicle = (ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py").read_text(encoding="utf-8")
    robot = (ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro").read_text(encoding="utf-8")
    sensors = (ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/sensor_suite.xacro").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("enable_training_gt", default_value="false")' in campus
    assert '"enable_training_gt", default_value="false"' in vehicle
    assert '<xacro:arg name="enable_training_gt" default="false"/>' in robot
    assert 'enable_training_gt="$(arg enable_training_gt)"' in robot
    assert 'training_gt="${enable_training_gt}"' in sensors
    assert sensors.count('name="g2_semantic_gt"') == 1
    assert sensors.count('name="g2_instance_gt"') == 1
    assert sensors.count('topic>g2/semantic_gt</topic') == 1
    assert sensors.count('topic>g2/instance_gt</topic') == 1
    assert 'name="formal_vehicle_training_gt_bridge"' in vehicle
    assert 'formal_training_gt_bridge.yaml' in vehicle
    assert 'condition=IfCondition(enable_training_gt)' in vehicle
    assert 'enable_training_gt:=true' not in campus
    bridge = (ROOT / "starter_ws/src/sanitation_vehicle_description/config/formal_training_gt_bridge.yaml").read_text(encoding="utf-8")
    assert bridge.count("ros_topic_name: /g2/") == 2
    assert bridge.count("gz_topic_name: /g2/") == 2
    assert bridge.count("subscriber_queue: 1") == 2
    assert bridge.count("publisher_queue: 1") == 2
    assert bridge.count("lazy: true") == 2
    lifecycle = (ROOT / "starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus_map_lifecycle.launch.py").read_text(encoding="utf-8")
    runner = (ROOT / "scripts/run_public_mobile_gazebo_dosod_calibration.sh").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("enable_training_gt", default_value="false")' in lifecycle
    assert lifecycle.count('"enable_training_gt": LaunchConfiguration("enable_training_gt")') == 1
    assert runner.count('enable_training_gt:=true') == 1
