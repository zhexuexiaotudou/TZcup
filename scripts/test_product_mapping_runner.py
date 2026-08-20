import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_product_mapping_acceptance.sh"
NAV_LAUNCH = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_navigation"
    / "launch"
    / "navigation.launch.py"
)
SLAM_CONFIG = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_navigation"
    / "config"
    / "slam_product_20000.yaml"
)


def test_formal_runner_has_real_restart_and_20k_fail_closed_scope():
    text = RUNNER.read_text(encoding="utf-8")
    phase_one_stop = text.index('stop_group "$mapping_tf_pid"')
    phase_two_start = text.index('echo "[PRODUCT-MAPPING] phase 2')
    assert phase_one_stop < phase_two_start
    assert "BOUNDS=(-100.0 -50.0 100.0 50.0)" in text
    assert "MIN_AREA=20000.0" in text
    assert "FORMAL_SCOPE=true" in text
    assert "FORMAL_SCOPE=false" in text
    assert 'simulation_rtf="1.0"' in text
    assert '[[ "$SMOKE" -eq 1 ]] && simulation_rtf="3.0"' not in text
    assert "exit \"$EVALUATION_CODE\"" in text
    assert "inspect smoke_chain_pass" in text
    assert '"source_commit": git("rev-parse", "HEAD")' in text
    assert '"source_dirty": bool(git("status", "--porcelain", "--untracked-files=no"))' in text
    assert '"config_sha256"' in text
    assert '"git.exe", "-C", windows_root' in text
    assert "--posegraph-data" in text
    assert "localization_backend:=\"$backend\"" in text
    assert "sanitation_navigation_probe" in text
    assert "wait_for_lifecycle_active /bt_navigator" in text
    assert "autostart:=false" in text
    assert 'activate_slam_toolbox "$mapping/slam_lifecycle.txt"' in text
    assert 'STARTED_PID="$(<"$pid_file")"' in text
    assert 'kill -TERM -- "-$pid"' in text
    assert 'pkill -TERM -s "$pid"' in text
    assert 'pgrep -s "$pid"' in text
    assert "trap on_error ERR" in text
    assert "if python3 \"$ROOT/scripts/product_mapping_acceptance.py\" evaluate" in text
    assert "trap 'exit 143' TERM" in text
    assert "start_group mapping_explorer" in text
    assert "wait_group mapping_explorer" in text
    assert "start_group reload_probe" in text
    assert 'for file in "$pid_dir"/*.pid' in text
    assert "start_navigation mapping_nav_retry" in text
    assert "phase 2 skipped because a phase 1 prerequisite failed" in text
    assert "-p goal_timeout_sec:=60.0" in text
    assert "-p failed_goal_cooldown_sec:=10.0" in text
    assert "-p failed_goal_exclusion_ttl_sec:=180.0" in text
    assert "-p minimum_frontier_map_gain_m2:=2.0" in text
    assert "-p no_progress_staging_success_limit:=3" in text
    assert "-p no_progress_raw_frontier_success_limit:=12" in text
    assert "-p no_progress_raw_exclusion_ttl_sec:=900.0" in text
    assert (
        '-p horizontal_sweep_staging_distances_m:'
        '="[8.0, 6.0, 4.0, 3.0, 2.0, 1.5, 1.0]"'
        in text
    )
    assert "-p horizontal_sweep_staging_path_sample_spacing_m:=0.25" in text
    assert "-p horizontal_sweep_staging_timeout_sec:=60.0" in text
    assert "-p horizontal_sweep_alignment_timeout_sec:=20.0" in text
    assert "-p horizontal_sweep_alignment_distance_m:=2.0" in text
    assert "-p horizontal_sweep_alignment_tolerance_rad:=0.15" in text
    assert "-p minimum_goal_distance_m:=0.80" in text
    assert "-p required_bounds_goal_margin_m:=0.80" in text
    assert "-p minimum_turning_radius_m:=1.429" in text
    assert "-p maximum_frontier_goal_yaw_change_rad:=0.70" in text
    assert "-p minimum_frontier_arc_yaw_change_rad:=0.15" in text
    assert "-p reverse_escape_distance_m:=2.0" in text
    assert "-p reverse_escape_speed_mps:=0.15" in text
    assert '-p frontier_sweep_enabled:="$SWEEP_ENABLED"' in text
    assert '-p frontier_sweep_initial_target_index:="$INITIAL_SWEEP_TARGET_INDEX"' in text
    assert '-p frontier_sweep_reference_pose_xyyaw_m_rad:="[0.0, 0.001, 0.0]"' in text
    assert '[[ "$DIAGNOSTIC_OVERRIDE" -eq 1 ]]' in text
    assert 'FORMAL_SCOPE=false' in text
    assert 'spawn_x:="$SPAWN_X" spawn_y:="$SPAWN_Y" spawn_yaw:="$SPAWN_YAW"' in text
    assert 'initial_pose_x:="$SPAWN_X" initial_pose_y:="$SPAWN_Y"' in text
    assert "-p mapping_sensor_range_m:=12.0" in text
    assert "-p frontier_sweep_lane_overlap_m:=2.0" in text
    assert "-p frontier_sweep_target_tolerance_m:=2.0" in text
    assert "-p frontier_sweep_mapped_target_radius_m:=5.0" in text
    assert "-p frontier_sweep_lane_shift_backup_distance_m:=4.0" in text
    assert '-p frontier_sweep_lane_shift_connector_distances_m:="[6.0, 4.0, 2.0]"' in text
    assert "-p lane_shift_connector_timeout_sec:=180.0" in text
    assert "MAX_FRONTIER_GOAL_DISTANCE=3.0" in text
    assert "MAX_LINEAR_VELOCITY=0.30; FORMAL_SCOPE=false" in text
    assert "MAX_LINEAR_VELOCITY=0.45; FORMAL_SCOPE=true" in text
    assert 'max_linear_velocity:="$MAX_LINEAR_VELOCITY"' in text
    assert '-p maximum_frontier_goal_distance_m:="$MAX_FRONTIER_GOAL_DISTANCE"' in text
    assert "-p initial_frontier_goal_distance_m:=2.0" in text
    assert "-p goal_distance_growth_success_count:=5" in text
    assert "-p goal_distance_growth_step_m:=0.5" in text
    assert "-p boundary_turn_buffer_m:=1.429" in text
    assert "navigate_to_pose_ackermann_frontier.xml" in text
    assert "-p failed_goal_exclusion_radius_m:=1.0" in text
    assert "-p timed_out_goal_exclusion_radius_m:=1.5" in text
    assert "MAX_GOALS=160; MAX_FRONTIER_GOAL_DISTANCE=2.0" in text
    assert "MAX_GOALS=800; MAX_FRONTIER_GOAL_DISTANCE=3.0" in text


def test_mapping_control_does_not_subscribe_to_ground_truth():
    text = RUNNER.read_text(encoding="utf-8")
    assert "ros2 launch sanitation_gnss_sim gnss_sim.launch.py" not in text
    assert "ros2 run sanitation_gnss_sim dual_navsat_adapter" in text
    assert "wait_for_topic /gnss/front/gps_raw gps_msgs/msg/GPSFix" in text
    assert "wait_for_topic /gnss/rear/gps_raw gps_msgs/msg/GPSFix" in text
    assert "ros2 node info /dual_navsat_adapter" in text
    assert "ground_truth_ros_subscription_in_positioning" in text
    assert '"gazebo_truth_to_gnss_sensor_model": False' in text
    assert "--world-sdf" in text
    assert "no oracle pose topic enters positioning or control" in text
    assert '"oracle_pose_topic_to_controller": False' in text
    assert "positioning_source:=rtk_gnss_sensor_wheel_imu_scan_matching" in text
    assert "world_to_map_x:=0.0 world_to_map_y:=0.0 world_to_map_yaw:=0.0" in text
    adapter = (
        ROOT
        / "starter_ws/src/sanitation_gnss_sim/sanitation_gnss_sim/dual_navsat.py"
    ).read_text(encoding="utf-8")
    assert '"/gnss/front/gps_raw"' in adapter
    assert '"/gnss/rear/gps_raw"' in adapter
    assert '"/ground_truth/' not in adapter


def test_prior_map_filters_are_removed_for_first_principles_mapping():
    text = RUNNER.read_text(encoding="utf-8")
    assert 'if item not in {"keepout_filter", "speed_filter"}' in text
    assert "enable_filters:=false" in text
    launch = NAV_LAUNCH.read_text(encoding="utf-8")
    assert "DeclareLaunchArgument(\n                'enable_filters', default_value='true'" in launch
    assert launch.count("condition=IfCondition(enable_filters)") == 5


def test_mapping_scan_turns_no_return_rays_into_observed_free_space():
    text = RUNNER.read_text(encoding="utf-8")
    assert "output_topic:=/scan/mapping" in text
    assert "replace_infinite_ranges_with_max:=true" in text
    assert "maximum_range_margin_m:=0.05" in text
    assert '"]["scan_topic"] = "/scan/mapping"' in text
    filter_script = (
        ROOT
        / "starter_ws"
        / "src"
        / "sanitation_navigation"
        / "scripts"
        / "scan_self_filter.py"
    ).read_text(encoding="utf-8")
    assert "replace_infinite_ranges_with_max" in filter_script
    assert "float(message.range_max) - self._maximum_range_margin" in filter_script


def test_product_slam_profile_treats_no_return_sentinel_as_free_space():
    config = yaml.safe_load(SLAM_CONFIG.read_text(encoding="utf-8"))
    params = config["slam_toolbox"]["ros__parameters"]
    assert params["resolution"] <= 0.10
    physical_range_max_m = 12.0
    no_return_margin_m = 0.05
    no_return_sentinel_m = physical_range_max_m - no_return_margin_m
    # Karto includes readings <= RangeThreshold in the scan bounding box, while
    # only readings < RangeThreshold - tolerance produce occupied endpoints.
    assert abs(params["max_laser_range"] - no_return_sentinel_m) < 1e-9
    assert params["max_laser_range"] > 0.95 * physical_range_max_m
    assert params["do_loop_closing"] is False
    assert params["use_scan_matching"] is False
    assert params["map_frame"] == "map"
    assert params["odom_frame"] == "odom"


def test_generic_ackermann_navigation_selects_a_real_goal_checker():
    tree_dir = NAV_LAUNCH.parent.parent / "behavior_trees"
    for filename in (
        "navigate_to_pose_ackermann.xml",
        "navigate_through_poses_ackermann.xml",
    ):
        text = (tree_dir / filename).read_text(encoding="utf-8")
        assert 'goal_checker_id="goal_checker"' in text
        assert 'default_planner="GridBased"' in text


def test_frontier_recovery_reverses_before_slow_recovery_actions():
    tree = (
        NAV_LAUNCH.parent.parent
        / "behavior_trees"
        / "navigate_to_pose_ackermann_frontier.xml"
    ).read_text(encoding="utf-8")
    recovery = tree.split('<RoundRobin name="RecoveryActions">', maxsplit=1)[1]
    assert recovery.index("<BackUp") < recovery.index("ClearingActions")
    assert recovery.index("<BackUp") < recovery.index("<Wait")


def test_ros_duration_overrides_are_explicit_floats():
    text = RUNNER.read_text(encoding="utf-8")
    assert '-p timeout_sec:="${MAPPING_TIMEOUT_SEC}.0"' in text
    assert '-p timeout_sec:="${NAVIGATION_TIMEOUT_SEC}.0"' in text


def test_frontier_timeout_restarts_nav2_before_next_goal():
    explorer = (
        ROOT
        / "starter_ws"
        / "src"
        / "sanitation_tasks"
        / "sanitation_tasks"
        / "frontier_explorer.py"
    ).read_text(encoding="utf-8")
    assert '"frontier_exhaustion_reverse_escape"' in explorer
    assert "ManageLifecycleNodes.Request.RESET" in explorer
    assert "ManageLifecycleNodes.Request.STARTUP" in explorer
    assert "if timed_out:" in explorer
    assert "self.nav_recovery_in_progress" in explorer
    assert "status == GoalStatus.STATUS_ABORTED" in explorer
    assert 'declare_parameter("maximum_goal_cost", 99)' in explorer
    assert 'declare_parameter("failed_goal_exclusion_ttl_sec", 180.0)' in explorer
    assert 'self._rank_goals(robot_pose, [])' in explorer
    assert '"frontier_candidates_temporarily_excluded"' in explorer
    assert '"frontier_no_progress_exclusion_count"' in explorer
    assert '"frontier_no_progress_raw_exclusion_count"' in explorer
    assert '"horizontal_sweep_raw_exclusion_suppressed_count"' in explorer
    assert '"horizontal_sweep_raw_exclusion_suppressed"' in explorer
    assert '"horizontal_sweep_frontier_wait_count"' in explorer
    assert '"horizontal_sweep_frontier_temporarily_unavailable"' in explorer
    assert "sweep_target_completion_reached(" in explorer
    assert "chassis_lane_y = self._sweep_chassis_lane_y(index)" in explorer
    assert '"horizontal_sweep_staging_exhaustion_arm_count"' in explorer
    exclusion_calls = [
        node
        for node in ast.walk(ast.parse(explorer))
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "frontier_goal_exclusion_centers"
    ]
    assert exclusion_calls
    assert all(len(call.args) == 2 for call in exclusion_calls)
    assert "self._sweep_horizontal_preference_y(" in explorer
    assert 'declare_parameter("required_bounds_goal_margin_m", 0.80)' in explorer
    assert 0.80 >= 0.66 + 0.05
    assert '"frontier_success_without_map_progress_raw_excluded"' in explorer
    assert '"horizontal_sweep_staging_attempt_count"' in explorer
    assert '"horizontal_sweep_staging_arm_count"' in explorer
    assert '"horizontal_sweep_staging_behind_chassis_count"' in explorer
    assert '"horizontal_sweep_staging_path_rejected_count"' in explorer
    assert '"horizontal_sweep_staging_chain_rearm_count"' in explorer
    assert '"horizontal_sweep_staging_chain_rearmed"' in explorer
    assert '"horizontal_sweep_alignment_attempt_count"' in explorer
    assert '"horizontal_sweep_alignment_success_count"' in explorer
    assert '"horizontal_sweep_alignment_failure_count"' in explorer
    assert '"horizontal_sweep_alignment_unavailable_count"' in explorer
    assert '"frontier_success_without_map_progress_staging_armed"' in explorer
    assert 'goal_kind="horizontal_sweep_staging"' in explorer
    assert '"horizontal_sweep_alignment_no_clear_path"' in explorer
    assert '"horizontal_sweep_staging_no_clear_path"' in explorer
    assert 'goal_kind="horizontal_sweep_alignment"' in explorer


def test_frontier_reverse_escape_uses_collision_checked_backup_action():
    explorer = (
        ROOT
        / "starter_ws/src/sanitation_tasks/sanitation_tasks/frontier_explorer.py"
    ).read_text(encoding="utf-8")
    assert 'ActionClient(self, BackUp, "/backup")' in explorer
    assert "message.target.x = -goal.distance_m" in explorer
    assert '"nav2_behaviors_BackUp_collision_checked"' in explorer
    assert 'goal_kind="lane_shift_backup"' in explorer
    assert "self._start_sweep_lane_shift_backup(robot_pose)" in explorer
    assert "self.sweep_lane_shift_backup_completed.add(int(index))" in explorer
    assert 'declare_parameter("frontier_sweep_lane_shift_backup_max_attempts", 2)' in explorer
    assert '"sweep_lane_shift_backup_exhausted"' in explorer
    assert "self._forward_costmap_clear_dubins_path(robot_pose, candidate)" in explorer
    assert '"skipped_online_costmap_clear_forward_dubins"' in explorer
    assert "self.sweep_lane_shift_locked_x.setdefault(" in explorer
    assert '"goal_kind": "lane_shift_connector"' in explorer
    assert "self._start_sweep_lane_shift_connector(robot_pose)" in explorer
    assert "self.sweep_lane_shift_connector_completed.add(int(index))" in explorer
    assert 'message.planner_id = "GridBased"' in explorer
    assert "split_hybrid_path_by_direction(path_poses)" in explorer
    assert 'plan_forward_dubins_path(' in explorer
    assert "split_path_at_curvature_reversals" in explorer
    assert '"controller_id": "DubinsPath"' in explorer
    assert '"goal_checker_id": (' in explorer
    assert 'section.get("controller_id", "ConnectorPath")' in explorer
    assert 'if section["direction"] == "REVERSE"' in explorer
    assert '"planned_sections"' in explorer
    runner = (ROOT / "scripts/run_product_mapping_acceptance.sh").read_text(
        encoding="utf-8"
    )
    assert "-p frontier_sweep_lane_shift_backup_max_attempts:=2" in runner
