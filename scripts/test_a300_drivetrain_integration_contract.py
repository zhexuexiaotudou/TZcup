from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
VEHICLE = ROOT / "starter_ws/src/sanitation_vehicle_description"
INTEGRATION = ROOT / "starter_ws/src/sanitation_formal_campus_integration"
LOCALIZATION = ROOT / "starter_ws/src/sanitation_localization"


def test_effort_plant_is_the_only_wheel_command_writer() -> None:
    controllers = yaml.safe_load(
        (VEHICLE / "config/formal_vehicle_controllers.yaml").read_text(
            encoding="utf-8"
        )
    )
    manager = controllers["controller_manager"]["ros__parameters"]
    assert "base_controller" not in manager
    assert "DiffDriveController" not in str(controllers)

    control = (VEHICLE / "urdf/high_fidelity/control_interfaces.xacro").read_text(
        encoding="utf-8"
    )
    wheel_block = control.split("A300DrivetrainPlantSystem", 1)[1].split(
        "Gazebo must start", 1
    )[0]
    for wheel in (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ):
        assert f'<xacro:hf_state_only_joint name="{wheel}"/>' in wheel_block
    assert "command_interface" not in wheel_block
    assert "hf_velocity_joint" not in wheel_block

    model = (VEHICLE / "urdf/formal_competition_vehicle.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert model.count("libA300DrivetrainPlantSystem.so") == 1
    assert model.count("sanitation_gazebo_control::A300DrivetrainPlantSystem") == 1


def test_typed_adapter_is_unique_consumer_of_final_safety_command() -> None:
    launch = (VEHICLE / "launch/formal_vehicle_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert launch.count('executable="a300_drivetrain_command_adapter"') == 1
    assert launch.count('name="a300_drivetrain_bridge"') == 1
    assert 'start_a300_transport_bridge = LaunchConfiguration(' in launch
    assert '"start_a300_transport_bridge",\n                default_value="true"' in launch
    drivetrain_bridge = launch.split('name="a300_drivetrain_bridge"', 1)[1].split(
        "# This bridge is the only formal ROS writer", 1
    )[0]
    assert "condition=IfCondition(start_a300_transport_bridge)" in drivetrain_bridge
    assert '"/odom/unfiltered"' in launch
    assert '"base_controller"' not in launch

    adapter = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainCommandAdapter.cc"
    ).read_text(encoding="utf-8")
    assert '"/base_controller/cmd_vel"' in adapter
    assert '"/safety/actuators_enabled"' in adapter
    assert "std::chrono::steady_clock" in adapter


def test_plant_converts_core_torque_to_gazebo_joint_force_sign() -> None:
    plant = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainPlantSystem.cc"
    ).read_text(encoding="utf-8")
    assert (
        "const double gazeboJointForceNm = -output.wheel_torque_nm[index];"
        in plant
    )


def test_local_ekf_is_selected_odom_and_odom_tf_authority() -> None:
    fusion = yaml.safe_load(
        (LOCALIZATION / "config/formal_fusion.yaml").read_text(encoding="utf-8")
    )
    local = fusion["local_ekf"]["ros__parameters"]
    assert local["odom0"] == "/odom/unfiltered"
    assert local["world_frame"] == "odom"
    assert local["base_link_frame"] == "base_footprint"
    assert local["publish_tf"] is True

    campus = (INTEGRATION / "launch/formal_campus.launch.py").read_text(
        encoding="utf-8"
    )
    assert "formal_campus_base_controller_spawner" not in campus
    assert "relay_legacy_base_odometry" not in campus
    assert "publish_selected_odom" not in campus
    assert '"start_local_fusion": "false"' in campus

    adapter = (
        INTEGRATION
        / "sanitation_formal_campus_integration/topic_adapter.py"
    ).read_text(encoding="utf-8")
    assert "Odometry" not in adapter
    assert "base_controller/odom" not in adapter
    assert "odom/unfiltered" not in adapter


def test_runtime_packages_declare_drivetrain_and_localization_dependencies() -> None:
    vehicle_manifest = (VEHICLE / "package.xml").read_text(encoding="utf-8")
    campus_manifest = (INTEGRATION / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>sanitation_gazebo_control</exec_depend>" in vehicle_manifest
    assert "<exec_depend>sanitation_localization</exec_depend>" in vehicle_manifest
    assert "diff_drive_controller" not in vehicle_manifest
    assert "<exec_depend>sanitation_localization</exec_depend>" in campus_manifest
