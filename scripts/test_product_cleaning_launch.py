from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "starter_ws" / "src" / "sanitation_bringup" / "launch" / "product_cleaning.launch.py"
PRODUCT_SOURCE = ROOT / "starter_ws" / "src" / "sanitation_product_bringup" / "launch" / "product_simulation.launch.py"
MISSION = ROOT / "starter_ws" / "src" / "sanitation_tasks" / "config" / "product_demo_area.yaml"
COVERAGE_LAUNCH = ROOT / "starter_ws" / "src" / "sanitation_coverage" / "launch" / "coverage.launch.py"
HMI_ADAPTER = ROOT / "starter_ws" / "src" / "sanitation_hmi" / "sanitation_hmi" / "ros_adapter.py"
HMI_SERVER = ROOT / "starter_ws" / "src" / "sanitation_hmi" / "sanitation_hmi" / "server.py"
SIM_LAUNCH = ROOT / "starter_ws" / "src" / "sanitation_bringup" / "launch" / "sim.launch.py"
HMI_PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_hmi" / "package.xml"
NAVIGATION_LAUNCH = ROOT / "starter_ws" / "src" / "sanitation_navigation" / "launch" / "navigation.launch.py"
FUSER_SOURCE = ROOT / "starter_ws" / "src" / "sanitation_scan_refiner" / "src" / "hybrid_global_fuser_node.cpp"
HYBRID_LAUNCH = ROOT / "starter_ws" / "src" / "sanitation_scan_refiner" / "launch" / "hybrid_localization.launch.py"


def test_product_launch_uses_only_product_control_nodes() -> None:
    text = SOURCE.read_text(encoding="utf-8")
    assert 'executable="product_perception_node"' in text
    assert 'executable="spot_cleaning_node"' in text
    assert 'executable="stage5br5_observation_pose_node"' in text
    assert 'executable="product_reobservation_node"' in text
    assert "sanitation_ground_truth" not in text
    assert "garbage_ground_truth_node" not in text


def test_product_launch_requires_frozen_artifacts_and_mission_identity() -> None:
    text = SOURCE.read_text(encoding="utf-8")
    for required in (
        'DeclareLaunchArgument("pipeline_manifest")',
        'DeclareLaunchArgument("artifact_root")',
        'DeclareLaunchArgument("mission_id")',
        'DeclareLaunchArgument("dynamic_map_path")',
        'DeclareLaunchArgument("cleanable_polygon_json")',
    ):
        assert required in text
    assert '"autostart": True' in text
    assert '"keepout_mask_topic": "/keepout_filter_mask"' in text


def test_full_product_topology_is_operator_started_and_gt_isolated() -> None:
    text = PRODUCT_SOURCE.read_text(encoding="utf-8")
    assert '"drive_model": "ackermann"' in text
    assert '"enable_command_timeout": "false"' in text
    assert '"enable_training_gt": "false"' in text
    assert '"enable_evaluation_gt": "false"' in text
    assert '"safety_startup_stopped": "true"' in text
    assert '"safety_require_supervisor": "true"' in text
    assert '"manual_start": True' in text
    assert '"allow_ground_truth_evaluation": False' in text
    assert 'DeclareLaunchArgument(\n            "gui",\n            default_value="false"' in text
    assert 'EnvironmentVariable("ROS_DOMAIN_ID", default_value="0")' in text
    assert 'SetEnvironmentVariable("GZ_PARTITION", transport_partition)' in text
    assert 'SetEnvironmentVariable("IGN_PARTITION", transport_partition)' in text
    assert 'executable="coverage_probe"' in text
    assert 'FindPackageShare("sanitation_hmi")' in text
    assert 'executable="product_supervisor"' in text
    assert "respawn=True" in text
    assert '"localization_pose_topic": "/localization/fused_pose"' in text
    assert 'executable="dual_navsat_adapter"' in text
    assert 'FindPackageShare("sanitation_scan_refiner")' in text
    assert '"fusion_mode": "rtk_imu_wheel"' in text
    assert '"enable_scan_refiner": "false"' in text
    assert '"publish_map_to_odom": "true"' in text
    assert '"respawn_fuser": "true"' in text
    assert "respawn=True" in text
    assert '"localization_backend": "external"' in text
    assert '"world_to_map_x": LaunchConfiguration("world_to_map_x")' in text
    assert 'DeclareLaunchArgument("world_file", default_value=default_world)' in text
    assert 'DeclareLaunchArgument("world_name", default_value="sanitation_test_world")' in text
    assert 'executable="gnss_sim_node"' not in text
    assert '"/ground_truth/odom"' not in text


def test_product_mission_contains_geometry_but_no_preknown_targets() -> None:
    text = MISSION.read_text(encoding="utf-8")
    assert "robot_width_m: 1.32" in text
    assert "operation_width_m: 1.32" in text
    assert "robot_footprint:" in text
    assert "cleaning_targets" not in text
    assert "target_" not in text


def test_product_runtime_has_no_ground_truth_target_subscriber() -> None:
    hmi = HMI_ADAPTER.read_text(encoding="utf-8")
    assert '"/garbage/ground_truth"' not in hmi
    assert "sanitation_ground_truth" not in HMI_SERVER.read_text(encoding="utf-8")
    assert "sanitation_ground_truth" not in HMI_PACKAGE.read_text(encoding="utf-8")
    product = PRODUCT_SOURCE.read_text(encoding="utf-8")
    assert '"allow_ground_truth_evaluation": False' in product


def test_simulator_oracle_bridges_are_evaluation_only() -> None:
    text = SIM_LAUNCH.read_text(encoding="utf-8")
    gt_bridge = text.index('name="evaluation_only_gt_bridge"')
    gt_adapter = text.index('name="ground_truth_adapter"')
    assert "condition=IfCondition(enable_evaluation_gt)" in text[
        gt_bridge:gt_adapter
    ]
    for bridge_name in ("ackermann_wheel_odom_bridge", "legacy_wheel_odom_bridge"):
        start = text.index(f'name="{bridge_name}"')
        end = text.index("            ),", start)
        production_bridge = text[start:end]
        assert "/ground_truth/" not in production_bridge
        assert "/dynamic_pose/info" not in production_bridge
        assert "/gnss/front/fix_raw@gps_msgs/msg/GPSFix" in production_bridge
        assert "/gnss/rear/fix_raw@gps_msgs/msg/GPSFix" in production_bridge
        assert '("/gnss/front/fix_raw", "/gnss/front/gps_raw")' in production_bridge
        assert '("/gnss/rear/fix_raw", "/gnss/rear/gps_raw")' in production_bridge


def test_product_profiles_use_full_width_and_authoritative_estop() -> None:
    coverage = COVERAGE_LAUNCH.read_text(encoding="utf-8")
    navigation = NAVIGATION_LAUNCH.read_text(encoding="utf-8")
    assert "('production', 'auto12_efficiency_v1'" in coverage
    assert "executable='safety_authority'" in navigation
    assert "startup_emergency_stopped" in navigation
    assert "DeclareLaunchArgument(\n                'localization_pose_topic'" in navigation
    assert "('amcl_pose', LaunchConfiguration('localization_pose_topic'))" in navigation
    assert "localization_backend, \"' == 'external'\"" in navigation
    assert "'node_names': ['map_server']" in navigation


def test_product_fuser_owns_one_calibrated_global_pose_contract() -> None:
    fuser = FUSER_SOURCE.read_text(encoding="utf-8")
    hybrid_launch = HYBRID_LAUNCH.read_text(encoding="utf-8")
    assert 'declare_parameter<double>("world_to_map_x", 0.0)' in fuser
    assert "worldToMap(" in fuser
    assert "worldHeadingToMap(" in fuser
    assert '"/localization/fused_pose"' in fuser
    assert ".reliable().transient_local()" in fuser
    assert 'DeclareLaunchArgument(\'respawn_fuser\', default_value=\'false\')' in hybrid_launch
    assert "respawn=LaunchConfiguration('respawn_fuser')" in hybrid_launch
