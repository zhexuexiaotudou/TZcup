from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_manipulation_simulation_endpoints_use_native_bridge() -> None:
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/ManipulationSimBridge.cc"
    ).read_text(encoding="utf-8")
    cmake = (
        ROOT / "starter_ws/src/sanitation_gazebo_control/CMakeLists.txt"
    ).read_text(encoding="utf-8")

    block = launch[
        launch.index("manipulation_sim_bridge = Node(") : launch.index(
            "return LaunchDescription", launch.index("manipulation_sim_bridge = Node(")
        )
    ]
    assert 'package="sanitation_gazebo_control"' in block
    assert 'executable="manipulation_sim_bridge"' in block
    assert 'condition=IfCondition(manipulation_sim_interfaces)' in block
    assert 'executable="parameter_bridge"' not in block

    for topic in (
        "/manipulation/grasp/attach",
        "/manipulation/grasp/detach",
        "/manipulation/grasp/state",
        "/manipulation/gripper/dual_contact",
        "/model/tzcup_formal_sanitation_vehicle/dry_bin/observed_status_json",
    ):
        assert topic in source
    for message_type in (
        "gz::msgs::Empty",
        "gz::msgs::Boolean",
        "gz::msgs::StringMsg",
        "std_msgs::msg::Empty",
        "std_msgs::msg::Bool",
        "std_msgs::msg::String",
    ):
        assert message_type in source
    assert "void Stop()" in source
    assert source.count("gz_node_.Unsubscribe(") == 3
    assert "std::mutex callback_mutex_" in source
    assert "std::lock_guard<std::mutex> drain(callback_mutex_)" in source
    assert "add_executable(manipulation_sim_bridge" in cmake
    assert "ManipulationSimBridge.cc" in cmake
    assert "manipulation_sim_bridge" in cmake
