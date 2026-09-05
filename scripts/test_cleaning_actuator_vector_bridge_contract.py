from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_double_vector_topics_use_supported_native_bridge() -> None:
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorVectorBridge.cc"
    ).read_text(encoding="utf-8")
    system = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc"
    ).read_text(encoding="utf-8")
    cmake = (
        ROOT / "starter_ws/src/sanitation_gazebo_control/CMakeLists.txt"
    ).read_text(encoding="utf-8")
    package = (
        ROOT / "starter_ws/src/sanitation_gazebo_control/package.xml"
    ).read_text(encoding="utf-8")
    assert "gz.msgs.Double_V" not in launch
    assert 'executable="cleaning_actuator_vector_bridge"' in launch
    assert "command/brush" in source
    assert "command/pump" in source
    assert "motor_current_a" in source
    assert "motor_temperature_c" in source
    assert "estimated_output_load" in source
    assert "telemetry_snapshot" in source
    assert "gz::msgs::Double_V" in source
    assert source.count("constexpr GazeboToRosEndpoint<") == 5
    assert "gz/msgs/clock.pb.h" in source
    assert "rosgraph_msgs/msg/clock.hpp" in source
    assert "gz::msgs::Clock" in source
    assert "rclcpp::ClockQoS()" in source
    assert "message.sim().sec()" in source
    assert "message.sim().nsec()" in source
    assert "void Stop()" in source
    assert source.count("gz_node_.Unsubscribe(") == 5
    assert "std::mutex callback_mutex_" in source
    assert "std::lock_guard<std::mutex> drain(callback_mutex_)" in source
    assert source.count("std::lock_guard<std::mutex> lock(callback_mutex_)") == 2
    assert "find_package(rosgraph_msgs REQUIRED)" in cmake
    assert "  rosgraph_msgs\n  std_msgs" in cmake
    assert "<depend>rosgraph_msgs</depend>" in package
    assert "kExpectedMotorCount = 5" in source
    assert "dropping malformed Gazebo Double_V" in source
    assert "native GZ->ROS bridge health" in source
    assert (
        "cleaning_motors/status_json@std_msgs/msg/String" not in launch
    )
    assert 'this->stateRoot + "/status_json"' in system
    assert system.index(
        "PublishTagged(this->telemetrySnapshotPublisher, telemetrySnapshot, 6)"
    ) < system.index("void PublishStatusJson")
    assert "PublishTagged(this->statusPublisher, status, 7)" in system


def test_all_three_telemetry_vectors_declare_the_compiled_native_bridge() -> None:
    register = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml")
        .read_text(encoding="utf-8")
    )
    contracts = register["topic_contracts"]
    for contract_id in (
        "cleaning_motor_current",
        "cleaning_motor_temperature",
        "cleaning_motor_output_load",
        "cleaning_motor_telemetry_snapshot",
    ):
        contract = contracts[contract_id]
        assert contract["transport"] == "gazebo_native_bridge"
        assert contract["direction"] == "publisher"
        assert contract["single_writer"] is True
        assert contract["writer_node"] == "cleaning_actuator_motor_bridge"
        assert contract["bridge_package"] == "sanitation_gazebo_control"
        assert contract["bridge_executable"] == "cleaning_actuator_vector_bridge"
        assert contract["source_path"].endswith("CleaningActuatorVectorBridge.cc")
        assert contract["ros_type"] == "std_msgs/msg/Float64MultiArray"
        assert contract["gz_type"] == "gz.msgs.Double_V"
        assert len(contract["actuator_order"]) == 5
    snapshot = contracts["cleaning_motor_telemetry_snapshot"]
    assert snapshot["schema_version"] == 1
    assert snapshot["expected_length"] == 63
    diagnostic = contracts["cleaning_motor_status"]
    assert diagnostic["transport"] == "gazebo_only_diagnostic"
    assert diagnostic["writer_system"] == "CleaningActuatorMotorSystem"
    assert diagnostic["gz_type"] == "gz.msgs.StringMsg"
    assert "ros_topic" not in diagnostic
    assert "ros_type" not in diagnostic
    assert "writer_node" not in diagnostic
