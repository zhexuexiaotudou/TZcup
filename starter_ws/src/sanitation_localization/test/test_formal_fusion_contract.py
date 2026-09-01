from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _params(name: str) -> dict:
    value = yaml.safe_load((ROOT / "config/formal_fusion.yaml").read_text())
    return value[name]["ros__parameters"]


def test_local_ekf_is_the_only_odom_to_base_owner():
    local = _params("local_ekf")
    assert local["world_frame"] == "odom"
    assert local["odom_frame"] == "odom"
    assert local["base_link_frame"] == "base_footprint"
    assert local["publish_tf"] is True
    assert local["odom0"] == "/odom/unfiltered"
    assert local["imu0"] == "/imu/data"
    assert len(local["odom0_config"]) == 15
    assert len(local["imu0_config"]) == 15
    assert local["odom0_config"][6] is True
    assert local["odom0_config"][11] is True
    assert local["imu0_config"][5] is True
    assert local["imu0_config"][11] is True
    assert local["imu0_config"][12:14] == [True, True]


def test_cleaning_global_ekf_fuses_lidar_map_gnss_and_local_velocity():
    global_ = _params("global_ekf")
    assert global_["world_frame"] == "map"
    assert global_["base_link_frame"] == "base_footprint"
    assert global_["publish_tf"] is True
    assert global_["odom0"] == "/odom"
    assert global_["pose0"] == "/amcl_pose"
    assert global_["odom1"] == "/odometry/gps"
    assert global_["pose0_config"][:6] == [True, True, False, False, False, True]
    assert global_["odom1_config"][:2] == [True, True]


def test_global_map_to_odom_prediction_advances_to_current_ros_time():
    global_ = _params("global_ekf")
    assert global_["predict_to_current_time"] is True
    # This is a publication-continuity contract, not evaluator tolerance or
    # future dating of TF.  The evaluator's freshness gate remains unchanged.
    assert global_["transform_time_offset"] == 0.0
    assert global_["frequency"] >= 1.0 / global_["sensor_timeout"]


def test_navsat_uses_real_product_sensor_aliases_and_no_second_utm_tf():
    navsat = _params("navsat_transform")
    assert navsat["broadcast_utm_transform"] is False
    assert navsat["broadcast_cartesian_transform"] is False
    launch = (ROOT / "launch/formal_localization_fusion.launch.py").read_text()
    for topic in ("/imu/data", "/gnss/fix", "/odometry/gps"):
        assert topic in launch


def test_mapping_mode_can_disable_global_map_to_odom_owner():
    launch = (ROOT / "launch/formal_localization_fusion.launch.py").read_text()
    assert 'DeclareLaunchArgument("start_global_fusion"' in launch
    assert launch.count("condition=IfCondition(start_global_fusion)") == 1
    assert 'DeclareLaunchArgument("start_navsat_transform"' in launch
    assert "condition=IfCondition(start_navsat_transform)" in launch


def test_runtime_tools_are_installed_and_do_not_use_truth_inputs():
    setup = (ROOT / "setup.py").read_text()
    assert "validate_formal_localization_runtime" in setup
    acceptance = ROOT.parent / "sanitation_localization_acceptance"
    collector = (
        acceptance / "src/formal_localization_runtime_collector.cpp"
    ).read_text()
    cmake = (acceptance / "CMakeLists.txt").read_text()
    assert "formal_localization_runtime_collector" in cmake
    assert "get_rmw_message_info().publisher_gid" in collector
    for forbidden in (
        "gazebo_msgs",
        "ModelStates",
        "EntityState",
        "ground_truth",
        "reference_pose",
    ):
        assert forbidden not in collector
