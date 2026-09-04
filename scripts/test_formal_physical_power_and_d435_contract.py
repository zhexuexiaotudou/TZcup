from __future__ import annotations

import re
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "starter_ws/src/sanitation_vehicle_description"
AUXILIARY = ROOT / "starter_ws/src/sanitation_gazebo_auxiliary"
SAFETY = ROOT / "starter_ws/src/sanitation_safety"
REGISTER = ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
LAYOUT = ROOT / "config/high_fidelity_vehicle/formal_vehicle_layout.yaml"
CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
HIGH_BANDWIDTH_BRIDGE = (
    DESCRIPTION / "config/formal_high_bandwidth_sensor_bridge.yaml"
)
VISUAL_BRIDGE = DESCRIPTION / "config/formal_visual_sensor_bridge.yaml"


def test_each_d435_has_a_measured_50_mm_ir_stereo_pair_without_added_mass() -> None:
    sensor = (DESCRIPTION / "urdf/high_fidelity/sensor_suite.xacro").read_text(
        encoding="utf-8"
    )
    assert 'xyz="0 0 0" rpy="-${pi/2.0} 0 -${pi/2.0}"' in sensor
    assert 'xyz="0 -0.050 0" rpy="-${pi/2.0} 0 -${pi/2.0}"' in sensor
    assert sensor.count('type="camera"') == 2
    assert sensor.count("<format>L_INT8</format>") == 2
    assert "${name}_infra1_optical_frame" in sensor
    assert "${name}_infra2_optical_frame" in sensor
    # 74.998 g housing + two 1 microgram optical links = original 75 g.
    assert '<mass value="0.074998"/>' in sensor
    assert sensor.count('<mass value="0.000001"/>') >= 1


def test_all_four_ir_images_and_camera_info_topics_are_bridged() -> None:
    launch = (DESCRIPTION / "launch/formal_vehicle_sim.launch.py").read_text(
        encoding="utf-8"
    )
    high_bandwidth_bridge = yaml.safe_load(
        HIGH_BANDWIDTH_BRIDGE.read_text(encoding="utf-8")
    )
    visual_bridge = yaml.safe_load(VISUAL_BRIDGE.read_text(encoding="utf-8"))

    # The dedicated D435 bridge is YAML-driven; do not regress it to four
    # handwritten parameter_bridge entries in the launch file.
    assert re.search(
        r'Node\(\s*package="ros_gz_bridge",\s*'
        r'executable="parameter_bridge",\s*'
        r'name="formal_vehicle_high_bandwidth_sensor_bridge".*?'
        r'"config_file": PathJoinSubstitution\(.*?'
        r'"formal_high_bandwidth_sensor_bridge\.yaml"',
        launch,
        flags=re.DOTALL,
    )
    assert "condition=IfCondition(high_bandwidth_bridges_enabled)" in launch
    # The separate nineteen-camera visual-acceptance bridge must also remain
    # YAML-driven rather than silently falling back to launch-local topics.
    assert len(visual_bridge) == 19
    assert "formal_visual_sensor_bridge.yaml" in launch
    assert "arguments=visual_image_topics" in launch

    bridge_by_topic = {
        item["ros_topic_name"]: item for item in high_bandwidth_bridge
    }
    for camera in ("front_rgbd", "wrist_rgbd"):
        for imager in ("infra1", "infra2"):
            image = f"/sensors/{camera}/{imager}/image_rect_raw"
            assert bridge_by_topic[image] == {
                "ros_topic_name": image,
                "gz_topic_name": image,
                "ros_type_name": "sensor_msgs/msg/Image",
                "gz_type_name": "gz.msgs.Image",
                "direction": "GZ_TO_ROS",
                "qos_profile": "SENSOR_DATA",
                "subscriber_queue": 1,
                "publisher_queue": 1,
                "lazy": True,
            }
            camera_info = f"{image}/camera_info"
            assert bridge_by_topic[camera_info] == {
                "ros_topic_name": camera_info,
                "gz_topic_name": camera_info,
                "ros_type_name": "sensor_msgs/msg/CameraInfo",
                "gz_type_name": "gz.msgs.CameraInfo",
                "direction": "GZ_TO_ROS",
                "qos_profile": "SENSOR_DATA",
                "subscriber_queue": 1,
                "publisher_queue": 1,
                "lazy": True,
            }


def test_power_hardware_mass_is_a_strict_400_g_pdu_redistribution() -> None:
    platform = (DESCRIPTION / "urdf/high_fidelity/a300_platform.xacro").read_text(
        encoding="utf-8"
    )
    hardware = (
        DESCRIPTION / "urdf/high_fidelity/power_service_hardware.xacro"
    ).read_text(encoding="utf-8")
    pdu_mass = float(
        re.search(
            r'<inertial><mass value="([0-9.]+)"/><xacro:hf_box_inertia mass="[0-9.]+" x="0.190"',
            platform,
        ).group(1)
    )
    expected = {
        "main_power_isolator_housing_link": 0.100,
        "main_power_isolator_handle_link": 0.040,
        "main_power_contactor_housing_link": 0.200,
        "main_power_contactor_armature_link": 0.060,
    }
    explicit = []
    for link, expected_mass in expected.items():
        block = hardware.split(f'<link name="{link}">', 1)[1].split("</link>", 1)[0]
        mass = float(re.search(r'hf_service_box_inertial mass="([0-9.]+)"', block).group(1))
        assert mass == expected_mass
        explicit.append(mass)
    assert abs(pdu_mass + sum(explicit) - 0.700) < 1.0e-12


def test_main_power_requires_measured_joints_and_has_no_state_topic_bypass() -> None:
    xacro = (DESCRIPTION / "urdf/formal_competition_vehicle.urdf.xacro").read_text(
        encoding="utf-8"
    )
    plugin = (AUXILIARY / "src/FormalAuxiliaryVisualSystem.cc").read_text(
        encoding="utf-8"
    )
    node = (SAFETY / "sanitation_safety/simulation_safety_inputs.py").read_text(
        encoding="utf-8"
    )
    control = (DESCRIPTION / "urdf/high_fidelity/control_interfaces.xacro").read_text(
        encoding="utf-8"
    )
    for joint in (
        "main_power_isolator_handle_joint",
        "main_power_contactor_armature_joint",
    ):
        assert joint in xacro
        assert joint in plugin
        assert f'<xacro:hf_state_only_joint name="{joint}" initial_position=' in control
    for token in (
        "isolatorPositionValid",
        "isolatorPosition >= this->isolatorClosedThreshold",
        "contactorPermitted = isolatorClosed && !latched",
        "this->safetyPowerAvailable.load()",
        "contactorPosition >= this->contactorClosedThreshold",
        "#include <gz/sim/components/JointVelocityReset.hh>",
        "gz::sim::components::JointVelocityReset(zeroVelocity)",
        "velocityReset->Data() = zeroVelocity",
        "this->contactorMaximumForce, 0.08, true",
        "this->plungerMaximumForce, 0.02, false",
        "this->isolatorMaximumTorque, 1.2, false",
        "this->Publish(this->isolatorStatePublisher, isolatorClosed)",
        "this->Publish(this->contactorStatePublisher, contactorClosed)",
    ):
        assert token in plugin
    assert '"/formal_vehicle/power/main_isolator_closed"' in node
    assert '"/formal_vehicle/power/main_contactor_closed"' in node
    assert "isolator_feedback_fresh" in node
    assert "contactor_feedback_fresh" in node
    assert "main_isolator_closed=(" in node
    assert "main_contactor_closed=(" in node


def test_main_power_bridge_separates_commands_from_measured_applied_state() -> None:
    launch = (DESCRIPTION / "launch/formal_vehicle_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert launch.count(
        "/formal_vehicle/simulation/command/main_power@std_msgs/msg/Bool]gz.msgs.Boolean"
    ) == 1
    assert launch.count(
        "/formal_vehicle/power/main_contactor_command@std_msgs/msg/Bool]gz.msgs.Boolean"
    ) == 1
    assert launch.count(
        "/formal_vehicle/power/main_isolator_closed@std_msgs/msg/Bool[gz.msgs.Boolean"
    ) == 1
    assert launch.count(
        "/formal_vehicle/power/main_contactor_closed@std_msgs/msg/Bool[gz.msgs.Boolean"
    ) == 1


def test_new_physical_details_are_bound_to_existing_positions_and_runtime_gates() -> None:
    register = yaml.safe_load(REGISTER.read_text(encoding="utf-8"))
    layout = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    positions = {item["id"]: item for item in register["functional_positions"]}
    topic_contracts = register["topic_contracts"]
    assert len(topic_contracts) == 87
    assert topic_contracts["warning_lights_applied"] == {
        "transport": "gazebo_bridge",
        "direction": "publisher",
        "single_writer": True,
        "writer_node": "formal_auxiliary_bridge",
        "ros_topic": "/formal_vehicle/lighting/warning_lights_applied",
        "ros_type": "std_msgs/msg/Bool",
        "gz_type": "gz.msgs.Boolean",
        "source_path": (
            "starter_ws/src/sanitation_gazebo_auxiliary/src/"
            "FormalAuxiliaryVisualSystem.cc"
        ),
    }
    vehicle_xacro = (DESCRIPTION / "urdf/formal_competition_vehicle.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert (
        "<warning_applied_topic>/formal_vehicle/lighting/"
        "warning_lights_applied</warning_applied_topic>"
    ) in vehicle_xacro
    assert {item["id"] for item in positions["forward_perception"]["components"]} == {
        "front_rgbd_ir_left_optical_datum", "front_rgbd_ir_right_optical_datum"
    }
    assert {item["id"] for item in positions["grasp_observation"]["components"]} >= {
        "wrist_rgbd_ir_left_optical_datum", "wrist_rgbd_ir_right_optical_datum"
    }
    assert {item["id"] for item in positions["fused_power_distribution"]["components"]} == {
        "main_power_isolator_housing", "main_power_isolator_handle",
        "main_power_contactor_housing", "main_power_contactor_armature",
    }
    lighting = positions["warning_and_work_lighting"]
    assert lighting["physical_link"] == "bodywork_lighting_link"
    assert "warning_lights_applied" in lighting["required_topic_contracts"]
    assert contract["functional_positions"]["warning_and_work_lighting"] == [
        "component_register",
        "whole_vehicle_interlock",
        "auxiliary_power_lighting",
    ]
    assert positions["water_gathering"]["components"][0]["id"] == "squeegee_preload_spring_pack"
    assert layout["camera_stream_topics"]["front_d435_ir_stereo"]["baseline_m"] == 0.05
    assert layout["physical_power_service_layout"]["mass_redistribution_kg"] == 0.4
    squeegee_compliance = layout["cleaning_geometry"]["squeegee_compliance"]
    cleaning_mechanism = (
        DESCRIPTION / "urdf/high_fidelity/cleaning_mechanism.xacro"
    ).read_text(encoding="utf-8")
    assert positions["water_gathering"]["preload_force_n"] == 12.456
    assert squeegee_compliance["float_preload_position_m"] == -0.00692
    assert squeegee_compliance["nominal_preload_force_n"] == 12.456
    assert squeegee_compliance["float_stiffness_n_m"] == 1800.0
    assert "<float_preload_reference_m>-0.00692</float_preload_reference_m>" in cleaning_mechanism
    assert "<float_max_force_n>120.0</float_max_force_n>" in cleaning_mechanism
    gates = contract["evidence_gates"]
    assert gates["sensor_runtime"]["required_physical_scope"] == [
        "front_d435_ir_stereo_pair", "wrist_d435_ir_stereo_pair"
    ]
    assert "squeegee_preloaded_spring_compliance" in gates["cleaning_actuators"]["required_physical_scope"]
    assert "main_power_contactor_armature_and_measured_state" in gates["auxiliary_power_lighting"]["required_physical_scope"]
