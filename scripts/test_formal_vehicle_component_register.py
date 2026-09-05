from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from validate_formal_vehicle_component_register import (
    DEFAULT_BRIDGE_CONFIGS,
    DEFAULT_LAUNCH,
    DEFAULT_REGISTER,
    DEFAULT_URDF,
    ComponentRegisterError,
    validate,
)


def test_committed_component_register_matches_expanded_urdf() -> None:
    result = validate()
    assert result["status"] == "COMPONENT_REGISTER_URDF_FK_AND_INTERFACES_VALID"
    assert result["coordinate_reference"] == "base_footprint"
    assert result["position_tolerance_m"] == 0.001
    assert result["orientation_tolerance_rad"] == 0.001
    assert result["sensor_installation_count"] == 9
    assert result["mechanical_subassembly_count"] == 18
    assert result["actuator_link_count"] == 7
    assert result["functional_position_count"] == 38
    assert result["functional_component_count"] == 63
    assert result["topic_contract_count"] == 88
    assert result["product_topic_contract_count"] == 87
    assert result["gazebo_only_diagnostic_count"] == 1
    assert result["checked_gazebo_only_diagnostics"] == ["cleaning_motor_status"]
    assert "cleaning_motor_status" not in result["checked_product_topic_contracts"]
    assert len(result["urdf_sha256"]) == 64
    assert result["single_writer_topic_count"] == 33
    assert "a300_raw_odometry" in result["checked_topic_contracts"]
    assert "lidar_3d_pointcloud" in result["checked_topic_contracts"]
    assert "whole_vehicle_safety_status" in result["checked_topic_contracts"]
    assert "warning_lights_state" in result["checked_topic_contracts"]
    assert "charge_receptacle_contact" in result["checked_topic_contracts"]
    assert "wastewater_drain_hose_contact" in result["checked_topic_contracts"]
    assert "front_rgbd_camera_info" in result["checked_topic_contracts"]
    assert "wrist_rgbd_camera_info" in result["checked_topic_contracts"]
    assert "rear_left_camera_info" in result["checked_topic_contracts"]
    assert "rear_right_camera_info" in result["checked_topic_contracts"]
    assert set(result["checked_single_writer_topics"]) == {
        "battery_soc",
        "battery_state",
        "bms_fault",
        "bms_status",
        "charge_connected",
        "charge_enable",
        "charge_power_request",
        "charge_requested",
        "charge_status",
        "auxiliary_power_status",
        "high_power_branch",
        "load_power_request",
        "low_voltage_power_branch",
        "main_power_requested",
        "main_contactor_command",
        "cleaning_motor_current",
        "cleaning_motor_status",
        "cleaning_motor_temperature",
        "cleaning_motor_output_load",
        "cleaning_motor_telemetry_snapshot",
        "recovery_filter_blockage",
        "safety_power_branch",
        "tail_lights_state",
        "traction_permitted",
        "warning_lights_state",
        "work_lights_state",
        "rear_left_camera_info",
        "rear_right_camera_info",
        "a300_encoder_counts",
        "a300_encoder_joint_states",
        "a300_raw_odometry",
        "cleaning_encoder_counts",
        "cleaning_encoder_joint_states",
    }
    assert result["top_protrusion_name"] == "modular_sensor_tower"
    register = yaml.safe_load(DEFAULT_REGISTER.read_text(encoding="utf-8"))
    assert register["topic_contracts"]["front_bumper_contact"] == {
        "ros_topic": "/formal_vehicle/simulation/raw/front_bumper/contact",
        "ros_type": "ros_gz_interfaces/msg/Contacts",
        "gz_type": "gz.msgs.Contacts",
        "sensor_base_topic": "/safety/front_bumper/contact",
    }
    assert register["topic_contracts"]["rear_bumper_contact"] == {
        "ros_topic": "/formal_vehicle/simulation/raw/rear_bumper/contact",
        "ros_type": "ros_gz_interfaces/msg/Contacts",
        "gz_type": "gz.msgs.Contacts",
        "sensor_base_topic": "/safety/rear_bumper/contact",
    }
    deposition = next(
        item for item in register["mechanical_subassemblies"]
        if item["id"] == "robot_deposition_port"
    )
    assert deposition["actuator_model"] == "ROBOTIS_DYNAMIXEL_XW540_T260_R"
    assert deposition["actuator_link"] == "dry_deposit_gate_actuator_link"
    assert {
        "front_left_a300_motor",
        "front_right_a300_motor",
        "rear_left_a300_motor",
        "rear_right_a300_motor",
        "front_left_a300_encoder",
        "front_right_a300_encoder",
        "rear_left_a300_encoder",
        "rear_right_a300_encoder",
        "left_a300_fixed_beam",
        "right_a300_fixed_beam",
        "left_a300_fixed_spacer",
        "right_a300_fixed_spacer",
        "left_pololu_4694_encoder",
        "right_pololu_4694_encoder",
        "central_pololu_4694_encoder",
        "zed_f9p_receiver_enclosure",
        "zed_f9p_receiver_module",
        "a300_left_battery_pack",
        "a300_right_battery_pack",
        "a300_left_battery_bms",
        "a300_right_battery_bms",
        "charge_port_door",
        "charge_receptacle",
        "charge_connector_lock",
        "emergency_stop_housing",
        "emergency_stop_6mm_plunger",
        "wastewater_drain_pipe",
        "wastewater_ball_valve_body",
        "wastewater_ball_valve_ball",
        "wastewater_valve_actuator",
        "wastewater_service_cap",
        "wastewater_hose_coupling",
        "wrist_rgbd_machined_bracket",
        "wrist_rgbd_camera_housing",
        "front_rgbd_ir_left_optical_datum",
        "front_rgbd_ir_right_optical_datum",
        "wrist_rgbd_ir_left_optical_datum",
        "wrist_rgbd_ir_right_optical_datum",
        "main_power_isolator_housing",
        "main_power_isolator_handle",
        "main_power_contactor_housing",
        "main_power_contactor_armature",
        "squeegee_preload_spring_pack",
        "power_service_door_hinge",
        "power_service_door_panel",
        "power_service_door_latch",
        "compute_service_door_hinge",
        "compute_service_door_panel",
        "compute_service_door_latch",
        "wet_service_door_hinge",
        "wet_service_door_panel",
        "wet_service_door_latch",
        "rear_dry_service_door_hinge",
        "rear_dry_service_door_panel",
        "rear_dry_service_door_latch",
    } <= set(result["checked_functional_components"])
    positions = {item["id"]: item for item in register["functional_positions"]}
    assert positions["warning_and_work_lighting"]["physical_link"] == "bodywork_lighting_link"
    assert positions["front_contact_safety"]["physical_collision"] == "front_bumper_collision"
    assert positions["rear_contact_safety"]["physical_collision"] == "rear_bumper_collision"
    assert positions["mapping_2d"]["function"] == "utm30lx_2d_occupancy_mapping"
    assert positions["obstacle_perception_3d"]["function"] == (
        "mid360_3d_obstacle_perception"
    )
    assert "mapping_3d" not in positions
    assert set(result["checked_actuator_links"]) == {
        "left_side_brush_motor_stator_link",
        "right_side_brush_motor_stator_link",
        "central_roller_motor_stator_link",
        "cleaning_lift_actuator_body_link",
        "recovery_pump_motor_link",
        "dry_deposit_gate_actuator_link",
        "wastewater_drain_valve_actuator_link",
    }

    drain = positions["wastewater_drain"]
    assert drain["required_joints"] == ["wastewater_drain_valve_joint"]
    assert drain["required_passive_joints"] == [
        "wastewater_drain_service_cap_joint"
    ]
    assert drain["interface"] == "service_controller"
    assert drain["actuation"] == "powered_position_joint"
    service_access = positions["bodywork_service_access"]
    assert len(service_access["required_passive_joints"]) == 8
    assert service_access["lock_state"] == "all_latches_zero_rad"

    sensors = {item["id"]: item for item in register["sensor_installations"]}
    assert sensors["intel_d435_wrist"]["mount_link"] == "wrist_rgbd_mount_link"
    assert sensors["intel_d435_wrist"]["sensor_link"] == "wrist_rgbd_link"
    assert sensors["ublox_zed_f9p_receiver"]["sensor_link"] == (
        "zed_f9p_module_reference_link"
    )
    assert positions["mobility"]["required_topic_contracts"] == [
        "a300_encoder_counts",
        "a300_encoder_joint_states",
        "a300_raw_odometry",
    ]
    assemblies = {item["id"]: item for item in register["mechanical_subassemblies"]}
    assert assemblies["dry_storage"]["parent_link"] == "storage_system_mount_link"
    assert assemblies["wastewater_storage"]["parent_link"] == (
        "storage_system_mount_link"
    )


def _mutated_register(tmp_path: Path, mutate) -> Path:
    register = yaml.safe_load(DEFAULT_REGISTER.read_text(encoding="utf-8"))
    mutate(register)
    output = tmp_path / "mutated-register.yaml"
    output.write_text(yaml.safe_dump(register, sort_keys=False), encoding="utf-8")
    return output


def test_function_position_missing_topic_fails_closed(tmp_path: Path) -> None:
    register = _mutated_register(
        tmp_path,
        lambda data: data["functional_positions"][0].update(
            {"required_topic_contracts": ["not_a_real_formal_vehicle_contract"]}
        ),
    )
    with pytest.raises(ComponentRegisterError, match="unknown topic contract"):
        validate(register_path=register)


def test_visible_datum_requires_explicit_physical_link(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        lighting = next(
            item for item in data["functional_positions"]
            if item["id"] == "warning_and_work_lighting"
        )
        lighting.pop("physical_link")

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="physical_link must explicitly map"):
        validate(register_path=register)


def test_function_position_fk_translation_drift_fails_closed(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        manipulation = next(item for item in data["functional_positions"] if item["id"] == "manipulation")
        manipulation["xyz_m"][2] -= 0.045

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="manipulation FK position error"):
        validate(register_path=register)


def test_sensor_fk_orientation_drift_fails_closed(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        front = next(item for item in data["sensor_installations"] if item["id"] == "intel_d435_front")
        front["rpy_rad"][1] = 0.0

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="intel_d435_front FK orientation error"):
        validate(register_path=register)


def test_camera_installation_requires_camera_info_contract(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        front = next(
            item
            for item in data["sensor_installations"]
            if item["id"] == "intel_d435_front"
        )
        front["topic_contracts"].remove("front_rgbd_camera_info")

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="CameraInfo contracts"):
        validate(register_path=register)


def test_camera_function_position_requires_camera_info_contract(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        forward = next(
            item
            for item in data["functional_positions"]
            if item["id"] == "forward_perception"
        )
        forward["required_topic_contracts"].remove("front_rgbd_camera_info")

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="CameraInfo contracts"):
        validate(register_path=register)


def test_dynamic_sensor_requires_explicit_local_reference(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        wrist = next(item for item in data["sensor_installations"] if item["id"] == "intel_d435_wrist")
        wrist.pop("coordinate_reference")

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="dynamic position requires an explicit coordinate_reference"):
        validate(register_path=register)


def test_wrist_camera_cannot_share_its_bracket_link(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        wrist = next(
            item
            for item in data["sensor_installations"]
            if item["id"] == "intel_d435_wrist"
        )
        wrist["mount_link"] = "wrist_rgbd_link"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="distinct bracket"):
        validate(register_path=register)


def test_storage_subassembly_parent_must_match_real_subframe(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        dry = next(
            item
            for item in data["mechanical_subassemblies"]
            if item["id"] == "dry_storage"
        )
        dry["parent_link"] = "payload_deck_link"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="direct URDF parent/root chain"):
        validate(register_path=register)


def test_mechanical_subassembly_wrong_load_path_fails_closed(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        tower = next(item for item in data["mechanical_subassemblies"] if item["id"] == "sensor_tower")
        tower["parent_link"] = "tool0"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="sensor_tower load path does not descend"):
        validate(register_path=register)


def test_declared_actuator_link_must_exist(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        deposition = next(
            item for item in data["mechanical_subassemblies"]
            if item["id"] == "robot_deposition_port"
        )
        deposition["actuator_link"] = "missing_gate_actuator_link"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="actuator_link missing from URDF"):
        validate(register_path=register)


def test_explicit_function_component_joint_endpoints_fail_closed(
    tmp_path: Path,
) -> None:
    def mutate(data: dict) -> None:
        mobility = next(
            item
            for item in data["functional_positions"]
            if item["id"] == "mobility"
        )
        mobility["components"][0]["parent_link"] = "base_link"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="joint endpoints mismatch"):
        validate(register_path=register)


def test_required_service_component_cannot_be_omitted(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        drain = next(
            item
            for item in data["functional_positions"]
            if item["id"] == "wastewater_drain"
        )
        drain["components"] = [
            item
            for item in drain["components"]
            if item["id"] != "wastewater_service_cap"
        ]

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="components are missing"):
        validate(register_path=register)


def test_estop_plunger_travel_cannot_drift_from_6mm(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        emergency_stop = next(
            item
            for item in data["functional_positions"]
            if item["id"] == "emergency_stop"
        )
        emergency_stop["plunger_travel_m"] = 0.005

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="explicit 0.006 m travel"):
        validate(register_path=register)


def test_mid360_cannot_reclaim_mapping_semantics(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        mid360 = next(
            item
            for item in data["sensor_installations"]
            if item["id"] == "livox_mid360"
        )
        mid360["role"] = "3d_obstacle_and_mapping_lidar"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="MID360 role cannot claim mapping"):
        validate(register_path=register)


def test_utm_must_remain_the_2d_mapping_sensor(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["navigation_sensor_semantics"]["mapping"][
            "primary_sensor_installation"
        ] = "livox_mid360"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="UTM 2D occupancy mapping"):
        validate(register_path=register)


def test_mid360_pointcloud_exact_bridge_type_fails_closed(tmp_path: Path) -> None:
    bridge_config = tmp_path / "formal_high_bandwidth_sensor_bridge.yaml"
    rows = yaml.safe_load(DEFAULT_BRIDGE_CONFIGS[0].read_text(encoding="utf-8"))
    row = next(
        item
        for item in rows
        if item["ros_topic_name"] == "/sensors/lidar_3d/points"
    )
    row["ros_type_name"] = "sensor_msgs/msg/LaserScan"
    row["gz_type_name"] = "gz.msgs.LaserScan"
    bridge_config.write_text(yaml.safe_dump(rows), encoding="utf-8")
    with pytest.raises(ComponentRegisterError, match="lidar_3d_pointcloud type mismatch"):
        validate(bridge_config_paths=[bridge_config])


def test_native_gazebo_bridge_type_is_checked_against_compiled_endpoint(
    tmp_path: Path,
) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["cleaning_motor_current"]["ros_type"] = (
            "std_msgs/msg/Float64"
        )

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="exact GZ->ROS endpoint"):
        validate(register_path=register)


def test_a300_native_odometry_contract_is_bound_to_compiled_endpoint(
    tmp_path: Path,
) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["a300_raw_odometry"]["gz_type"] = (
            "gz.msgs.Pose"
        )

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="exact GZ->ROS endpoint"):
        validate(register_path=register)


def test_a300_native_odometry_requires_exact_launch_executable(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["a300_raw_odometry"]["bridge_executable"] = (
            "parameter_bridge"
        )

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="expected exactly one launch node"):
        validate(register_path=register)


def test_a300_native_odometry_requires_its_scoped_launch_remap(tmp_path: Path) -> None:
    launch = tmp_path / "formal_vehicle_sim.launch.py"
    original = DEFAULT_LAUNCH.read_text(encoding="utf-8")
    mutated = original.replace('"/odom/unfiltered",', '"/odom/wrong",', 1)
    assert mutated != original
    launch.write_text(mutated, encoding="utf-8")
    with pytest.raises(ComponentRegisterError, match="exact GZ->ROS endpoint"):
        validate(
            launch_path=launch,
            bridge_config_paths=DEFAULT_BRIDGE_CONFIGS,
        )


def test_native_single_writer_detects_another_node_remapped_to_same_topic(
    tmp_path: Path,
) -> None:
    launch = tmp_path / "formal_vehicle_sim.launch.py"
    launch.write_text(
        DEFAULT_LAUNCH.read_text(encoding="utf-8")
        + '''
duplicate_a300_odometry_writer = Node(
    package="sanitation_gazebo_control",
    executable="a300_drivetrain_native_bridge",
    name="duplicate_a300_odometry_writer",
    remappings=[
        (
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom",
            "/odom/unfiltered",
        ),
    ],
)
''',
        encoding="utf-8",
    )

    def mutate(data: dict) -> None:
        duplicate = dict(data["topic_contracts"]["a300_raw_odometry"])
        duplicate["writer_node"] = "duplicate_a300_odometry_writer"
        data["topic_contracts"]["duplicate_a300_raw_odometry"] = duplicate

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="expected only"):
        validate(
            register_path=register,
            launch_path=launch,
            bridge_config_paths=DEFAULT_BRIDGE_CONFIGS,
        )


def test_native_gazebo_bridge_requires_exactly_one_launched_writer(
    tmp_path: Path,
) -> None:
    launch = tmp_path / "formal_vehicle_sim.launch.py"
    launch.write_text(
        DEFAULT_LAUNCH.read_text(encoding="utf-8").replace(
            'executable="cleaning_actuator_vector_bridge"',
            'executable="missing_cleaning_actuator_vector_bridge"',
        ),
        encoding="utf-8",
    )
    with pytest.raises(ComponentRegisterError, match="expected exactly one launch node"):
        validate(launch_path=launch)


def test_native_gazebo_bridge_single_writer_requires_publisher_direction(
    tmp_path: Path,
) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["cleaning_motor_current"]["direction"] = (
            "subscription"
        )

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="must declare publisher direction"):
        validate(register_path=register)


def test_gazebo_only_diagnostic_rejects_fake_ros_endpoint(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        contract = data["topic_contracts"]["cleaning_motor_status"]
        contract["ros_topic"] = contract["gz_topic"]
        contract["ros_type"] = "std_msgs/msg/String"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="has forbidden fields"):
        validate(register_path=register)


def test_gazebo_only_diagnostic_rejects_explicit_null_ros_field(
    tmp_path: Path,
) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["cleaning_motor_status"]["ros_topic"] = None

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="has forbidden fields"):
        validate(register_path=register)


def test_gazebo_only_diagnostic_requires_exact_publisher(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["cleaning_motor_status"]["gz_topic"] += "_wrong"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="exact Gazebo publisher"):
        validate(register_path=register)


def test_gazebo_only_diagnostic_requires_single_publisher_direction(
    tmp_path: Path,
) -> None:
    def mutate(data: dict) -> None:
        contract = data["topic_contracts"]["cleaning_motor_status"]
        contract["direction"] = "subscription"
        contract["single_writer"] = False

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(
        ComponentRegisterError,
        match="must declare publisher direction.*must declare single_writer true",
    ):
        validate(register_path=register)


def test_gazebo_only_diagnostic_must_not_be_ros_bridged(tmp_path: Path) -> None:
    launch = tmp_path / "formal_vehicle_sim.launch.py"
    launch.write_text(
        DEFAULT_LAUNCH.read_text(encoding="utf-8")
        + '\n"/model/tzcup_formal_sanitation_vehicle/cleaning_motors/status_json'
        '@std_msgs/msg/String[gz.msgs.StringMsg"\n',
        encoding="utf-8",
    )
    with pytest.raises(ComponentRegisterError, match="must not be bridged to ROS"):
        validate(launch_path=launch)


def test_gazebo_only_diagnostic_requires_formal_urdf_writer_plugin(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "formal_competition_vehicle.urdf"
    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    plugin = next(
        item
        for item in root.findall(".//gazebo/plugin")
        if (item.get("name") or "").endswith("::CleaningActuatorMotorSystem")
    )
    plugin.set("filename", "libWrongCleaningActuatorMotorSystem.so")
    tree.write(urdf, encoding="utf-8", xml_declaration=True)
    with pytest.raises(ComponentRegisterError, match="writer plugin filename"):
        validate(urdf_path=urdf)


def test_gazebo_only_diagnostic_cannot_satisfy_product_interface(
    tmp_path: Path,
) -> None:
    def mutate(data: dict) -> None:
        data["functional_positions"][0]["required_topic_contracts"].append(
            "cleaning_motor_status"
        )

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="references diagnostic-only"):
        validate(register_path=register)


def test_native_auxiliary_topic_missing_source_fails_closed(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["warning_lights_state"]["source_path"] = (
            "starter_ws/src/sanitation_safety/sanitation_safety/not_present.py"
        )

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="source missing"):
        validate(register_path=register)


def test_single_writer_contract_requires_publisher_direction(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["charge_connected"].pop("direction")

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="must declare publisher direction"):
        validate(register_path=register)


def test_native_endpoint_direction_is_checked_exactly(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["charge_connected"]["direction"] = "subscription"
        data["topic_contracts"]["charge_connected"]["single_writer"] = False

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="exact subscription endpoint missing"):
        validate(register_path=register)


def test_single_writer_contract_requires_writer_node(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["topic_contracts"]["battery_state"].pop("writer_node")

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="requires writer_node"):
        validate(register_path=register)


def test_function_position_uncommanded_joint_fails_closed(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        pumping = next(item for item in data["functional_positions"] if item["id"] == "water_pumping")
        pumping["interface"] = "storage_controller"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="does not command"):
        validate(register_path=register)


def test_wastewater_valve_cannot_be_registered_as_passive(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        drain = next(
            item
            for item in data["functional_positions"]
            if item["id"] == "wastewater_drain"
        )
        drain["required_joints"] = []
        drain["required_passive_joints"].append("wastewater_drain_valve_joint")

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="cannot be registered as passive"):
        validate(register_path=register)


def test_service_door_contract_requires_locked_zero_latches(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        service = next(
            item
            for item in data["functional_positions"]
            if item["id"] == "bodywork_service_access"
        )
        service["lock_state"] = "unspecified"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="locked zero state"):
        validate(register_path=register)


def test_function_position_runtime_probe_includes_powered_service_valve() -> None:
    source = (
        DEFAULT_REGISTER.parents[2]
        / "scripts"
        / "validate_formal_function_positions_runtime.py"
    ).read_text(encoding="utf-8")
    assert '"wastewater_drain_valve_joint": 1.00' in source
    assert '"/service_controller/joint_trajectory"' in source
    assert '"controller_count": 5' in source
    assert "FORMAL_CLEANING_STORAGE_SERVICE_AND_RECOVERY_ACTUATORS_PASSED" in source


def test_sensor_mount_and_housing_require_direct_physical_joint_chains(tmp_path: Path) -> None:
    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    front = next(
        link for link in root.findall("link") if link.get("name") == "front_rgbd_link"
    )
    front.remove(front.find("collision"))
    broken_geometry = tmp_path / "sensor-no-collision.urdf"
    tree.write(broken_geometry, encoding="unicode")
    with pytest.raises(
        ComponentRegisterError,
        match="intel_d435_front.sensor_link must have visible and collidable physical geometry",
    ):
        validate(urdf_path=broken_geometry)

    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    bracket_joint = next(
        joint
        for joint in root.findall("joint")
        if joint.get("name") == "front_rgbd_bracket_joint"
    )
    bracket_joint.find("parent").set("link", "payload_deck_link")
    wrong_parent = tmp_path / "sensor-wrong-direct-parent.urdf"
    tree.write(wrong_parent, encoding="unicode")
    with pytest.raises(
        ComponentRegisterError,
        match="intel_d435_front mount joint must directly connect base_link->front_rgbd_mount_link",
    ):
        validate(urdf_path=wrong_parent)


def test_registered_actuator_and_physical_component_require_collision_geometry(
    tmp_path: Path,
) -> None:
    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    pump = next(
        link
        for link in root.findall("link")
        if link.get("name") == "recovery_pump_motor_link"
    )
    pump.remove(pump.find("collision"))
    broken_actuator = tmp_path / "actuator-no-collision.urdf"
    tree.write(broken_actuator, encoding="unicode")
    with pytest.raises(
        ComponentRegisterError,
        match="wastewater_recovery_drive.actuator_link must have visible and collidable physical geometry",
    ):
        validate(urdf_path=broken_actuator)

    tree = ET.parse(DEFAULT_URDF)
    root = tree.getroot()
    plunger = next(
        link
        for link in root.findall("link")
        if link.get("name") == "emergency_stop_plunger_link"
    )
    plunger.remove(plunger.find("collision"))
    broken_component = tmp_path / "component-no-collision.urdf"
    tree.write(broken_component, encoding="unicode")
    with pytest.raises(
        ComponentRegisterError,
        match="emergency_stop.components.emergency_stop_6mm_plunger must have visible and collidable physical geometry",
    ):
        validate(urdf_path=broken_component)
