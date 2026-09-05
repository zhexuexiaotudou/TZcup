from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_product_mapping_nodes_have_no_simulator_truth_control_topics():
    for name in ("map_lifecycle_manager.py", "frontier_explorer.py"):
        source = (PACKAGE / "sanitation_formal_campus_integration" / name).read_text(
            encoding="utf-8"
        )
        assert "/ground_truth" not in source
        assert "/model/" not in source
        assert "world.sdf" not in source
        assert "/cmd_vel" not in source


def test_frontier_goals_use_current_map_frame_pose_and_tangent_yaw():
    source = (
        PACKAGE
        / "sanitation_formal_campus_integration"
        / "frontier_explorer.py"
    ).read_text(encoding="utf-8")
    assert "TransformListener(self._tf_buffer, self" in source
    assert "self._tf_buffer.lookup_transform(" in source
    assert 'self.get_parameter("map_frame").value' in source
    assert 'self.get_parameter("base_frame").value' in source
    assert "target_yaw = goal_tangent_yaw(" in source
    assert "math.sin(target_yaw / 2.0)" in source
    assert "math.cos(target_yaw / 2.0)" in source
    assert "goal.pose.pose.orientation.w = 1.0" not in source


def test_formal_launch_separates_mapping_and_saved_map_cleaning():
    source = (
        PACKAGE / "launch" / "formal_campus_map_lifecycle.launch.py"
    ).read_text(encoding="utf-8")
    assert '"localization_backend": "external" if mode == "mapping" else "amcl"' in source
    assert 'validate_saved_map_artifact(artifact_root, contract)' in source
    assert '"map_file": str(artifact_root / "occupancy.yaml")' in source
    assert 'executable="formal-frontier-explorer"' in source
    assert '"start_navigation": "false"' in source
    assert "publish_selected_odom" not in source
    assert 'FindPackageShare("sanitation_localization")' not in source
    assert '"localization_backend": "slam" if mode == "mapping" else "amcl"' in source
    assert 'nav2["amcl"]["ros__parameters"]["tf_broadcast"] = False' in source
    assert 'cleaning_planner not in {"full_coverage", "rl_dirt_priority"}' in source
    assert 'FindPackageShare("sanitation_perception")' not in source
    assert 'FindPackageShare("sanitation_active_cleaning")' not in source
    assert '"materialize_static_maps": "false"' in source
    assert '"runtime_artifact_dir": str(artifact_root)' in source
    assert '"start_velocity_gate": "false"' in source
    assert '"cmd_vel_in_topic": "/cmd_vel_smoothed"' in source
    assert '"cmd_vel_out_topic": "/cmd_vel_gate"' in source
    assert 'nav2[node_name]["ros__parameters"]["enable_stamped_cmd_vel"] = False' in source
    assert '"controller_server",' in source
    assert '"velocity_smoother",' in source
    assert '"collision_monitor",' in source
    assert 'canonical_scan = "/scan/navigation"' in source
    assert 'nav2["amcl"]["ros__parameters"]["scan_topic"] = canonical_scan' in source
    assert '"formal_utm30lx_self_filter.yaml"' in source
    assert 'executable="formal-scan-self-filter"' in source
    assert '"params_file": str(generated_slam)' in source
    assert 'slam_max_laser_range != normalized_no_return_range' in source
    assert '"no_return_replacement_m": normalized_no_return_range' in source
    assert 'slam_params["use_scan_matching"] = True' in source
    assert 'slam_params["use_scan_barycenter"] = True' in source
    assert 'slam_params["do_loop_closing"] = True' in source
    assert "wheel_imu_ekf_lidar_scan_matching_gnss_consistency" in source
    assert '"rolling_window": True' in source
    assert '"width": 30' in source
    assert '"height": 30' in source
    assert 'if plugin != "static_layer"' in source
    assert '"support_artifacts_prepared": True' in source
    assert '"false" if mode == "mapping" else "true"' in source
    assert 'package="nav2_collision_monitor"' not in source
    assert "Jazzy nav2_bringup above owns collision_monitor" in source
    # The canonical vehicle launch owns the two raw bumper bridges.  The map
    # lifecycle must not create duplicates or bypass whole-vehicle safety.
    assert 'formal_mapping_front_bumper_raw_bridge' not in source
    assert 'formal_mapping_rear_bumper_raw_bridge' not in source
    base_launch = (
        PACKAGE / "launch" / "formal_campus.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n                "materialize_static_maps"' in base_launch
    assert "SLAM is the sole occupancy-map producer" in base_launch
    navigation = (
        PACKAGE.parent / "sanitation_navigation" / "launch" / "navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert "SetRemap(src='/cmd_vel', dst='/cmd_vel_nav')" in navigation
    vehicle = (
        PACKAGE.parent
        / "sanitation_vehicle_description"
        / "urdf"
        / "formal_competition_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    sensors = (
        PACKAGE.parent
        / "sanitation_vehicle_description"
        / "urdf"
        / "high_fidelity"
        / "sensor_suite.xacro"
    ).read_text(encoding="utf-8")
    assert 'name="high_bandwidth_sensor_runtime" default="true"' in vehicle
    assert 'high_bandwidth_runtime="$(arg high_bandwidth_sensor_runtime)"' in vehicle
    assert '<xacro:if value="${high_bandwidth_runtime}">' in sensors


def test_frontier_action_discovery_and_goal_response_are_bounded():
    source = (
        PACKAGE
        / "sanitation_formal_campus_integration"
        / "frontier_explorer.py"
    ).read_text(encoding="utf-8")
    assert "bounded_action_server_ready(" in source
    assert 'self.declare_parameter("action_discovery_timeout_sec", 0.1)' in source
    assert 'self.declare_parameter("goal_response_timeout_sec", 5.0)' in source
    assert 'self.declare_parameter("goal_execution_timeout_sec", 900.0)' in source
    assert 'self.declare_parameter("goal_progress_timeout_sec", 120.0)' in source
    assert 'self.declare_parameter("cancel_timeout_sec", 5.0)' in source
    assert '"frontier_goal_response_timeout"' in source
    assert '"frontier_cancel_requested"' in source
    assert '"frontier_cancel_response_timeout"' in source
    assert '"frontier_cancel_result_timeout"' in source
    assert "feedback_callback=self._on_feedback" in source
    assert "cancel_goal_async()" in source
    assert "late_accept_after_response_timeout" in source
    assert "ClockType.STEADY_TIME" in source
    assert "server_is_ready()" not in source


def test_saved_map_coverage_is_real_product_action_execution_with_fixed_envelope():
    package_manifest = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    launch = (
        PACKAGE / "launch" / "formal_campus_map_lifecycle.launch.py"
    ).read_text(encoding="utf-8")
    executor = (
        PACKAGE
        / "sanitation_formal_campus_integration"
        / "saved_map_coverage_executor.py"
    ).read_text(encoding="utf-8")
    assert "<exec_depend>action_msgs</exec_depend>" in package_manifest
    assert 'coverage_parameters["operation_width"] = cleaning_width' in launch
    assert "formal saved-map cleaning width must be exactly 1.32 m" in launch
    assert "formal saved-map maximum linear speed disagrees with profile" in launch
    assert "default_value=DRY_CLEANING_SPEED_PROFILE" in launch
    assert 'executable="formal-saved-map-coverage-executor"' in launch
    assert '"operation_width_m": cleaning_width' in launch
    assert '"maximum_linear_speed_mps": smoother_speed' in launch
    assert '"operation_speed_profile": speed_profile.name' in launch
    assert "ComputeCoveragePath" in executor
    assert "NavigateToPose" in executor
    assert "FollowPath" in executor
    assert 'follow.controller_id = "CleanPath"' in executor
    assert 'self.declare_parameter("operation_speed_profile", DRY_CLEANING_SPEED_PROFILE)' in executor
    assert 'self._set_brush(True)' in executor
    assert 'self._set_brush(False)' in executor
    assert '"terminal_state": state' in executor
    assert "/ground_truth" not in executor
    assert "/model/" not in executor
    assert "world.sdf" not in executor


def test_saved_map_executor_fields_match_pinned_jazzy_action_contracts():
    executor = (
        PACKAGE
        / "sanitation_formal_campus_integration"
        / "saved_map_coverage_executor.py"
    ).read_text(encoding="utf-8")
    for assignment in (
        "goal.generate_headland = True",
        "goal.generate_route = True",
        "goal.generate_path = True",
        'goal.frame_id = "map"',
        'goal.swath_mode.objective = "LENGTH"',
        'goal.swath_mode.mode = "SET_ANGLE"',
        "goal.swath_mode.best_angle = 0.0",
        'goal.route_mode.mode = "BOUSTROPHEDON"',
        'goal.path_mode.mode = "DUBIN"',
        'goal.path_mode.continuity_mode = "DISCONTINUOUS"',
        "goal.polygons = [coordinates]",
        "transit.pose = self._pose(",
        "follow.path = path",
        'follow.controller_id = "CleanPath"',
        'follow.goal_checker_id = "goal_checker"',
        'follow.progress_checker_id = "progress_checker"',
    ):
        assert assignment in executor
    for result_field in (
        "result.error_code",
        "result.coverage_path.swaths",
    ):
        assert result_field in executor

    active_executor = (
        PACKAGE.parent
        / "sanitation_active_cleaning"
        / "sanitation_active_cleaning"
        / "formal_trajectory_executor.py"
    ).read_text(encoding="utf-8")
    assert "goal.path = message" in active_executor
    assert "goal.controller_id =" in active_executor
    assert "goal.goal_checker_id =" in active_executor
    assert 'getattr(wrapped.result, "error_code", 0)' in active_executor


def test_clean_path_and_velocity_smoother_share_formal_speed_ceiling():
    nav2 = yaml.safe_load(
        (
            PACKAGE.parent / "sanitation_navigation" / "config" / "nav2.yaml"
        ).read_text(encoding="utf-8")
    )
    controller = nav2["controller_server"]["ros__parameters"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    assert controller["CleanPath"]["desired_linear_vel"] == 0.45
    assert smoother["max_velocity"][0] == 0.45


def test_slam_launch_can_disable_legacy_velocity_gate():
    source = (
        PACKAGE.parent
        / "sanitation_navigation"
        / "launch"
        / "slam.launch.py"
    ).read_text(encoding="utf-8")
    assert "DeclareLaunchArgument('start_velocity_gate'" in source
    assert "condition=IfCondition(LaunchConfiguration('start_velocity_gate'))" in source


def test_scan_self_filter_is_installed_with_config_and_console_entry():
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert 'glob("config/*.yaml")' in setup
    assert (
        '"formal-scan-self-filter = "' in setup
        and "formal_scan_self_filter:main" in setup
    )
    lifecycle = (
        PACKAGE / "launch" / "formal_campus_map_lifecycle.launch.py"
    ).read_text(encoding="utf-8")
    assert 'FindPackageShare("sanitation_formal_campus_integration")' in lifecycle
    assert '"formal_utm30lx_self_filter.yaml"' in lifecycle
    assert "scan_filter_params," in lifecycle
    assert '"no_return_replacement_m": normalized_no_return_range' in lifecycle


def test_mapping_manager_uses_slam_save_service_and_stable_quality_gate():
    source = (
        PACKAGE
        / "sanitation_formal_campus_integration"
        / "map_lifecycle_manager.py"
    ).read_text(encoding="utf-8")
    assert "SaveMap.Request()" in source
    assert 'self.declare_parameter("stable_samples_required", 3)' in source
    assert '"mapping_ignored_dirt": True' in source
    assert '"world_truth_used_for_control": False' in source
    assert '"gnss_mapping_reference_observed": True' in source
    assert '"gps_odometry_topic", "/odometry/gps"' in source
    assert '"waiting_for_gnss_mapping_reference"' in source
    assert '"gnss_odometry_consistency_gate_failed"' in source
