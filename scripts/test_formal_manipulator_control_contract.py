from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_arm_and_gripper_have_independent_fail_closed_controllers() -> None:
    config = yaml.safe_load((ROOT / "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml").read_text(encoding="utf-8"))
    arm = config["arm_controller"]["ros__parameters"]
    gripper = config["gripper_controller"]["ros__parameters"]
    assert len(arm["joints"]) == 6
    assert arm["allow_partial_joints_goal"] is False
    assert gripper["joints"] == ["robotiq_85_left_knuckle_joint"]
    assert gripper["allow_partial_joints_goal"] is False
    assert float(arm["constraints"]["goal_time"]) == 2.0
    assert float(gripper["constraints"]["robotiq_85_left_knuckle_joint"]["goal"]) == 0.01


def test_expanded_urdf_contains_all_commanded_joints() -> None:
    root = ET.parse(ROOT / "reports/engineering/formal_competition_vehicle.urdf").getroot()
    names = {joint.attrib["name"] for joint in root.findall("joint")}
    config = yaml.safe_load((ROOT / "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml").read_text(encoding="utf-8"))
    commanded = config["arm_controller"]["ros__parameters"]["joints"] + config["gripper_controller"]["ros__parameters"]["joints"]
    assert set(commanded) <= names


def test_gazebo_runtime_report_proves_measured_arm_and_gripper_motion() -> None:
    report = json.loads(
        (ROOT / "reports/engineering/formal_manipulator_runtime_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert report["status"] == "UR5E_AND_ROBOTIQ_GAZEBO_TRAJECTORY_EXECUTION_PASSED"
    assert report["arm"]["accepted"] is True
    assert report["arm"]["result_error_code"] == 0
    assert report["gripper"]["accepted"] is True
    assert report["gripper"]["result_error_code"] == 0
    assert report["arm"]["terminal_max_error_rad"] < 1.0e-3
    assert report["gripper"]["terminal_max_error_rad"] < 1.0e-3
    ranges = report["measured_joint_range_rad"]
    for joint in (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ):
        assert ranges[joint] > 0.08
    assert ranges["robotiq_85_left_knuckle_joint"] > 0.55
    assert report["joint_state_sample_count"] > 1000
