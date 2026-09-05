from __future__ import annotations

from pathlib import Path

import yaml

from validate_formal_vehicle_component_register import _python_topic_endpoints


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "starter_ws/src/sanitation_vehicle_description"
AUXILIARY = ROOT / "starter_ws/src/sanitation_gazebo_auxiliary"
NATIVE_AUXILIARY = (
    ROOT / "starter_ws/src/sanitation_gazebo_control/src/FormalAuxiliaryNativeBridge.cc"
)
NATIVE_CONTACT = (
    ROOT / "starter_ws/src/sanitation_gazebo_control/src/FormalContactEvaluationNativeBridge.cc"
)
NATIVE_A300 = ROOT / "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainNativeBridge.cc"
SAFETY_INPUTS = (
    ROOT
    / "starter_ws/src/sanitation_safety/sanitation_safety/simulation_safety_inputs.py"
)


def test_formal_vehicle_loads_one_physical_auxiliary_plugin() -> None:
    xacro = (DESCRIPTION / "urdf/formal_competition_vehicle.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert xacro.count('filename="libFormalAuxiliaryVisualSystem.so"') == 1
    assert xacro.count('name="sanitation_gazebo_auxiliary::FormalAuxiliaryVisualSystem"') == 1
    assert "<plunger_joint_name>emergency_stop_plunger_joint</plunger_joint_name>" in xacro
    assert "<plunger_travel_m>0.006</plunger_travel_m>" in xacro
    assert "<plunger_pressed_threshold_m>0.005</plunger_pressed_threshold_m>" in xacro
    assert "<initial_estop_latched>$(arg initial_estop_latched)</initial_estop_latched>" in xacro


def test_plugin_measures_and_drives_the_real_plunger_fail_closed() -> None:
    source = (AUXILIARY / "src/FormalAuxiliaryVisualSystem.cc").read_text(
        encoding="utf-8"
    )
    for token in (
        "components::JointPosition",
        "components::JointPositionReset",
        "positionValid",
        "const bool physicalPressed = !positionValid",
        "position->Data().front() >= this->plungerPressedThreshold",
        "this->estopCore->Update(",
        "this->Publish(this->latchedPublisher, latched)",
    ):
        assert token in source
    control = (DESCRIPTION / "urdf/high_fidelity/control_interfaces.xacro").read_text(
        encoding="utf-8"
    )
    assert '<xacro:hf_state_only_joint name="emergency_stop_plunger_joint" initial_position="0.00002"/>' in control


def test_render_commands_use_real_ecm_owners_and_matching_payload_ids() -> None:
    source = (AUXILIARY / "src/FormalAuxiliaryVisualSystem.cc").read_text(
        encoding="utf-8"
    )
    assert "message.set_id(entry.entity);" in source
    assert "visualCommand.set_id(_entity);" in source
    assert "entry.entity,\n            gz::sim::components::LightCmd(message)" in source
    assert "_entity,\n                gz::sim::components::VisualCmd(visualCommand)" in source
    assert source.count("gz::sim::ComponentState::OneTimeChange") >= 2
    assert "Rendering consumes the command message ID" not in source


def test_bridge_has_explicit_command_applied_power_reset_and_unique_state_paths() -> None:
    launch = (DESCRIPTION / "launch/formal_vehicle_sim.launch.py").read_text(
        encoding="utf-8"
    )
    native = NATIVE_AUXILIARY.read_text(encoding="utf-8")
    commands = (
        "/formal_vehicle/lighting/work_lights_on", "/formal_vehicle/lighting/tail_lights_on",
        "/formal_vehicle/lighting/warning_lights_on", "/formal_vehicle/power/branches/safety/enabled",
        "/formal_vehicle/simulation/command/emergency_stop",
        "/formal_vehicle/simulation/command/emergency_stop_plunger_pressed",
        "/formal_vehicle/simulation/command/emergency_stop_reset",
    )
    applied = (
        "/formal_vehicle/lighting/work_lights_applied", "/formal_vehicle/lighting/tail_lights_applied",
        "/formal_vehicle/lighting/warning_lights_applied", "/emergency_stop",
    )
    assert all(f'"{topic}"' in native for topic in commands + applied)
    assert native.count("RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean>") == 9
    assert native.count("GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean>") == 6
    assert 'name="formal_auxiliary_bridge"' in launch
    assert 'executable="formal_auxiliary_native_bridge"' in launch
    assert "gazebo_auxiliary_lib" in launch
    # Direction matters: the physical latch is the sole Gazebo -> ROS state
    # writer, while the drivetrain receives that state through ROS -> Gazebo.
    a300 = NATIVE_A300.read_text(encoding="utf-8")
    assert '"/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/emergency_stop"' in a300
    assert "create_subscription<std_msgs::msg::Bool>" in a300
    assert native.count('"/emergency_stop"') == 1


def test_emergency_stop_has_no_native_product_publisher() -> None:
    endpoints = _python_topic_endpoints(SAFETY_INPUTS)
    assert ("subscription", "/emergency_stop", "Bool") in endpoints
    assert ("publisher", "/emergency_stop", "Bool") not in endpoints

    register = yaml.safe_load(
        (
            ROOT
            / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
        ).read_text(encoding="utf-8")
    )
    contract = register["topic_contracts"]["emergency_stop_state"]
    assert contract["transport"] == "gazebo_native_bridge"
    assert contract["direction"] == "publisher"
    assert contract["single_writer"] is True
    assert contract["writer_node"] == "formal_auxiliary_bridge"
    assert contract["ros_topic"] == "/emergency_stop"


def test_description_declares_auxiliary_runtime_dependency() -> None:
    package = (DESCRIPTION / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>sanitation_gazebo_auxiliary</exec_depend>" in package


def test_squeegee_uses_real_preloaded_spring_force_system() -> None:
    cleaning = (DESCRIPTION / "urdf/high_fidelity/cleaning_mechanism.xacro").read_text(
        encoding="utf-8"
    )
    source = (AUXILIARY / "src/SqueegeeComplianceSystem.cc").read_text(
        encoding="utf-8"
    )
    cmake = (AUXILIARY / "CMakeLists.txt").read_text(encoding="utf-8")
    assert 'filename="libSqueegeeComplianceSystem.so"' in cleaning
    assert "<float_preload_reference_m>-0.00692</float_preload_reference_m>" in cleaning
    assert "<float_stiffness_n_per_m>1800.0</float_stiffness_n_per_m>" in cleaning
    assert "<float_max_force_n>120.0</float_max_force_n>" in cleaning
    assert "squeegee_float_joint" in source
    assert "squeegee_pitch_joint" in source
    assert "floatParameters{1800.0, 45.0, -0.00692, 120.0}" in source
    assert "components::JointForceCmd" in source
    assert "SqueegeeComplianceCore::Effort" in source
    assert "floatForcePublisher" in source
    assert "pitchTorquePublisher" in source
    assert "SqueegeeComplianceSystem" in cmake


def test_squeegee_passive_state_contact_and_live_telemetry_are_bridged() -> None:
    cleaning = (DESCRIPTION / "urdf/high_fidelity/cleaning_mechanism.xacro").read_text(
        encoding="utf-8"
    )
    control = (DESCRIPTION / "urdf/high_fidelity/control_interfaces.xacro").read_text(
        encoding="utf-8"
    )
    launch = (DESCRIPTION / "launch/formal_vehicle_sim.launch.py").read_text(encoding="utf-8")
    native = NATIVE_CONTACT.read_text(encoding="utf-8")
    assert '<collision name="squeegee_blade_collision">' in cleaning
    assert '<sensor name="squeegee_blade_ground_contact" type="contact">' in cleaning
    assert (
        "<collision>squeegee_link_fixed_joint_lump__squeegee_blade_collision_collision</collision>"
        in cleaning
    )
    world = (DESCRIPTION / "worlds/formal_vehicle_validation.sdf").read_text(
        encoding="utf-8"
    )
    contact_system = (
        '<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'
    )
    assert contact_system in world
    assert world.index(contact_system) < world.index("gz-sim-user-commands-system")
    assert "<topic>/cleaning/squeegee/contact</topic>" in cleaning
    assert '<xacro:hf_state_only_joint name="squeegee_float_joint"/>' in control
    assert '<xacro:hf_state_only_joint name="squeegee_pitch_joint"/>' in control
    assert 'hf_position_joint name="squeegee_float_joint"' not in control
    assert 'hf_position_joint name="squeegee_pitch_joint"' not in control
    squeegee_contact_source = (
        "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/"
        "link/squeegee_link/sensor/squeegee_blade_ground_contact/contact"
    )
    for topic in (
        squeegee_contact_source,
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_position_m",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_velocity_m_s",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_force_n",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_position_rad",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_velocity_rad_s",
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_torque_nm",
    ):
        assert f'"{topic}"' in native
    assert 'GroupedGazeboToRosEndpoint' in native
    assert 'name="formal_squeegee_evaluation_bridge"' in launch
    assert 'executable="formal_contact_evaluation_native_bridge"' in launch
    assert '"endpoint_group": "squeegee"' in launch
    assert (
        f'"{squeegee_contact_source}",\n                "/cleaning/squeegee/contact",'
        in launch
    )
