from __future__ import annotations

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


def test_dart_gripper_mimic_followers_are_physical_state_only_joints() -> None:
    root = Path(__file__).resolve().parents[1]
    control = (
        root
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro"
    ).read_text(encoding="utf-8")
    vehicle = (
        root
        / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    manipulator = (
        root
        / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/manipulator_stack.xacro"
    ).read_text(encoding="utf-8")
    followers = {
        "robotiq_85_right_knuckle_joint": "-0.20",
        "robotiq_85_left_inner_knuckle_joint": "0.20",
        "robotiq_85_right_inner_knuckle_joint": "-0.20",
        "robotiq_85_left_finger_tip_joint": "-0.20",
        "robotiq_85_right_finger_tip_joint": "0.20",
    }
    for name, initial in followers.items():
        assert (
            f'<xacro:hf_state_only_joint name="{name}" initial_position="{initial}"/>'
            in control
        )
    assert 'filename="libGripperMimicEffortSystem.so"' in vehicle
    assert "<master_joint>robotiq_85_left_knuckle_joint</master_joint>" in vehicle
    assert '<xacro:hf_manipulator_stack parent="arm_mount_link" use_sim="$(arg use_sim)"/>' in vehicle
    assert 'params="parent use_sim:=true"' in manipulator
    assert manipulator.count('<xacro:unless value="${use_sim}"><mimic ') == 5


def test_simulation_snapshot_has_one_gripper_follower_command_authority() -> None:
    root = ET.parse(ROOT / "reports/engineering/formal_competition_vehicle.urdf").getroot()
    follower_names = {
        "robotiq_85_right_knuckle_joint",
        "robotiq_85_left_inner_knuckle_joint",
        "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint",
        "robotiq_85_right_finger_tip_joint",
    }
    physical_joints = {
        joint.attrib["name"]: joint for joint in root.findall("joint")
    }
    assert follower_names <= physical_joints.keys()
    assert all(physical_joints[name].find("mimic") is None for name in follower_names)

    plugins = root.findall(
        "./gazebo/plugin[@filename='libGripperMimicEffortSystem.so']"
    )
    assert len(plugins) == 1

    ros2_control = root.find("ros2_control[@name='formal_vehicle_system']")
    assert ros2_control is not None
    controlled = {
        joint.attrib["name"]: joint for joint in ros2_control.findall("joint")
    }
    for name in follower_names:
        assert controlled[name].find("command_interface") is None
        assert controlled[name].find("state_interface[@name='position']") is not None


def test_moveit_description_preserves_urdf_mimic_kinematics() -> None:
    launch = (
        ROOT / "starter_ws/src/sanitation_manipulation/launch/manipulation.launch.py"
    ).read_text(encoding="utf-8")
    assert '" use_sim:=false bodywork_visible:=true ",' in launch
    assert "mock_components/GenericSystem" in launch
    assert "It is not the real-hardware bring-up path" in launch
    assert 'DeclareLaunchArgument("publish_robot_description", default_value="true")' in launch
    assert "publish_robot_description, value_type=bool" in launch
    integrated = (
        ROOT
        / "starter_ws/src/sanitation_manipulation/launch/formal_physical_grasp.launch.py"
    ).read_text(encoding="utf-8")
    assert '"publish_robot_description": "false"' in integrated
    assert "sole global" in integrated


def test_expanded_urdf_contains_all_commanded_joints() -> None:
    root = ET.parse(ROOT / "reports/engineering/formal_competition_vehicle.urdf").getroot()
    names = {joint.attrib["name"] for joint in root.findall("joint")}
    config = yaml.safe_load((ROOT / "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml").read_text(encoding="utf-8"))
    commanded = config["arm_controller"]["ros__parameters"]["joints"] + config["gripper_controller"]["ros__parameters"]["joints"]
    assert set(commanded) <= names


def test_gazebo_runtime_gate_requires_live_measured_arm_and_gripper_motion() -> None:
    contract = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml").read_text(
            encoding="utf-8"
        )
    )
    gate = contract["evidence_gates"]["manipulator_trajectory"]
    assert gate["report_id"] == "tzcup_formal_manipulator_runtime_v2"
    assert gate["success_statuses"] == [
        "UR5E_AND_ROBOTIQ_GAZEBO_TRAJECTORY_EXECUTION_PASSED"
    ]
    validator = (ROOT / "scripts/validate_formal_manipulator_runtime.py").read_text(
        encoding="utf-8"
    )
    assert "ranges[name] < 0.08" in validator
    assert "ranges[GRIPPER_JOINT] < 0.55" in validator
    assert "terminal tracking error exceeds tolerance" in validator
    assert '"measured_joint_range_rad": ranges' in validator
    assert '"joint_state_sample_count": len(node.samples)' in validator
    assert '"runtime_gate_binding": runtime_gate_binding' in validator
