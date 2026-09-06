from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / "sanitation_manipulation" / "formal_grasp_executor.py").read_text(
    encoding="utf-8"
)


def test_product_request_never_accepts_simulator_identity_or_world_pose():
    core = (PACKAGE / "sanitation_manipulation" / "formal_grasp_core.py").read_text(
        encoding="utf-8"
    )
    assert '"model_name"' in core and '"entity_name"' in core
    assert "truth-backed grasp requests are forbidden" in core
    assert "SetEntityPose" not in SOURCE
    assert "read_gazebo_poses" not in SOURCE
    assert "/world/" not in SOURCE
    assert "ros_gz_interfaces.msg" not in SOURCE
    assert "/manipulation/gripper/dual_contact" in SOURCE
    assert "/dry_bin/observed_status_json" in SOURCE


def test_executor_uses_movegroup_ik_cartesian_and_planning_scene_not_fixed_pick():
    required = (
        "MoveGroup",
        "GetPositionIK",
        "GetCartesianPath",
        "ApplyPlanningScene",
        "ExecuteTrajectory",
        '"TARGET_CONDITIONED_PREGRASP"',
        '"WRIST_REFINED_PREGRASP"',
        '"LINEAR_CONTACT_APPROACH"',
        '"LINEAR_COLLISION_CHECKED_LIFT"',
        '"COLLISION_CHECKED_DEPOSIT"',
        '"COLLISION_CHECKED_BIN_RETREAT"',
    )
    assert all(token in SOURCE for token in required)
    assert "PICK," not in SOURCE
    assert "self._trajectory(self._arm" not in SOURCE
    assert '"moveit_task_constructor_used": False' in SOURCE
    assert "for client, reason in action_interfaces:" in SOURCE
    assert "client.wait_for_server(timeout_sec=5.0)" in SOURCE
    assert "for client, reason in service_interfaces:" in SOURCE
    assert "client.wait_for_service(timeout_sec=5.0)" in SOURCE


def test_ground_bootstrap_is_a_separate_readback_gate_not_a_perceived_cube_lifecycle():
    config = (PACKAGE / "config" / "bin_and_scene.yaml").read_text(encoding="utf-8")
    bootstrap = (PACKAGE / "sanitation_manipulation" / "planning_scene_bootstrap.py").read_text(
        encoding="utf-8"
    )
    launch = (PACKAGE / "launch" / "formal_physical_grasp.launch.py").read_text(encoding="utf-8")
    assert "required_robot_links:" in config
    assert "required_world_objects: [ground]" in config
    assert "required_collision_objects" not in config
    assert "GetPlanningScene" in bootstrap
    assert "GetParameters" in bootstrap
    assert "robot_description_semantic" in bootstrap
    assert "configured_ground_frame_does_not_match_moveit_planning_frame" in bootstrap
    assert "validate_scene_readback" in bootstrap
    assert "required_robot_or_ground_tf_unavailable" in bootstrap
    assert "scene.world.collision_objects = [self._ground_collision()]" in bootstrap
    assert "perceived_cube" not in bootstrap
    assert "moveit_planning_scene_bootstrap" in launch
    assert "planning_scene_ready_topic" in SOURCE
    assert "planning_scene_not_ready_or_ground_readback_missing" in SOURCE
    assert SOURCE.index("self._wait_moveit_interfaces()") < SOURCE.index(
        'self._moveit_joint(TRANSPORT, "TRANSPORT")'
    )


def test_ground_bootstrap_keeps_waiting_clients_and_tf_runnable_while_tick_is_serialized():
    bootstrap = (PACKAGE / "sanitation_manipulation" / "planning_scene_bootstrap.py").read_text(
        encoding="utf-8"
    )
    assert "MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup" in bootstrap
    assert "self._timer_callback_group = MutuallyExclusiveCallbackGroup()" in bootstrap
    assert "self._service_callback_group = ReentrantCallbackGroup()" in bootstrap
    assert bootstrap.count("callback_group=self._service_callback_group") == 3
    assert "callback_group=self._timer_callback_group" in bootstrap
    # Jazzy's TransformListener owns a distinct ReentrantCallbackGroup for
    # /tf and /tf_static. The bootstrap pins that structure and keeps the
    # listener in the single MultiThreadedExecutor instead of adding the node
    # to a competing internal executor.
    assert "TransformListener(self._tf_buffer, self, spin_thread=False)" in bootstrap
    assert "self._tf_listener_group = self._tf_listener.group" in bootstrap
    assert "isinstance(self._tf_listener_group, ReentrantCallbackGroup)" in bootstrap
    assert "MultiThreadedExecutor(num_threads=2)" in bootstrap


def test_live_ground_runtime_gate_queries_only_and_never_executes_actuators():
    gate = (PACKAGE / "sanitation_manipulation" / "planning_scene_runtime_gate.py").read_text(
        encoding="utf-8"
    )
    assert "GetPositionIK" in gate and "GetCartesianPath" in gate
    # Structural test only: actual contacts/revision evidence is Linux-live
    # runtime work and must never be inferred from this source assertion.
    assert "GetStateValidity" in gate
    assert "below_ground_state_not_explicitly_colliding_with_ground" in gate
    assert "response.contacts" in gate
    assert "request.contacts" not in gate
    assert "request.max_contacts" not in gate
    assert "planning_virtual_joint_missing_from_robot_state" in gate
    assert "fresh_map_to_base_footprint_tf_unavailable" in gate
    assert "map_to_base_footprint_tf_stale" in gate
    assert "planning_virtual_joint_pose_disagrees_with_map_tf" in gate
    assert "allow_ground_removal_test" in gate
    assert "removal.is_diff = True" in gate
    assert "removal.robot_state.is_diff = True" in gate
    assert "item.operation = CollisionObject.REMOVE" in gate
    assert "removal.world.collision_objects = [item]" in gate
    assert "ground_removal_changed_non_ground_world_or_acm" in gate
    assert "is_diff = False" not in gate
    assert "planning_scene_revision_did_not_increment" in gate
    assert '"pregrasp", "pick", "lift", "deposit"' in gate
    assert "ExecuteTrajectory" not in gate
    assert "FollowJointTrajectory" not in gate
    assert "model_name" not in gate
    assert "entity_name" not in gate
    assert "SetEntityPose" not in gate


def test_live_ground_runtime_gate_reports_known_service_door_warning_without_masking_errors():
    gate = (PACKAGE / "sanitation_manipulation" / "planning_scene_runtime_gate.py").read_text(
        encoding="utf-8"
    )
    assert "from rcl_interfaces.msg import Log" in gate
    assert 'Log, "/rosout", self._on_rosout, qos_profile_rosout_default' in gate
    for joint in (
        "bodywork_power_service_door_hinge_joint",
        "bodywork_power_service_door_latch_joint",
        "bodywork_compute_service_door_hinge_joint",
        "bodywork_compute_service_door_latch_joint",
        "bodywork_wet_service_door_hinge_joint",
        "bodywork_wet_service_door_latch_joint",
        "bodywork_rear_dry_service_door_hinge_joint",
        "bodywork_rear_dry_service_door_latch_joint",
    ):
        assert joint in gate
    assert '"observed": self._door_missing_joint_warning_count > 0' in gate
    assert '"other_moveit_error_summaries"' in gate
    assert "moveit_error_logged_during_ground_runtime_gate" not in gate
    assert "_raise_if_moveit_error_logged" not in gate
    assert 'result["rosout_observation"] = node._rosout_report()' in gate
    assert '"rosout_observation": self._rosout_report()' in gate


def test_cube_scene_diffs_advance_the_bootstrap_revision_instead_of_emptying_it():
    assert "get_planning_scene_service" in SOURCE
    assert "next_scene_revision" in SOURCE
    assert "PlanningSceneComponents.SCENE_SETTINGS" in SOURCE
    assert "PlanningSceneComponents.WORLD_OBJECT_GEOMETRY" in SOURCE
    assert "PlanningSceneComponents.ALLOWED_COLLISION_MATRIX" in SOURCE
    assert "validate_scene_readback" in SOURCE
    assert "revision_missing" in SOURCE
    assert "revision_not_monotonic_after_apply" in SOURCE
    assert "current.scene.name.strip() or None" not in SOURCE
    assert "scene.name = next_scene_revision" in SOURCE
    assert 'scene = PlanningScene()' in SOURCE


def test_wrist_recheck_contact_attachment_and_bin_increment_are_fail_closed():
    assert SOURCE.index("self._wait_wrist_recheck(request_base)") < SOURCE.index(
        '"LINEAR_CONTACT_APPROACH"'
    )
    assert SOURCE.index("self._common_live_contact()") < SOURCE.index(
        "self._attach.publish(Empty())"
    )
    assert "physical_hold_not_observed_after_lift" in SOURCE
    assert "physical_detachment_not_acknowledged" in SOURCE
    assert "material_for_measured_mass" in SOURCE
    assert '"pre_grasp_material": "unknown"' in SOURCE
    assert SOURCE.index("self._wait_bin_increment(") < SOURCE.index(
        'self._publish_result(request.target_id, True'
    )


def test_base_motion_is_inhibited_for_arm_motion_and_failure_is_fail_safe():
    assert "/manipulation/base_motion_inhibited" in SOURCE
    assert SOURCE.index("self._publish_base_inhibit(True)") < SOURCE.index(
        'self._moveit_joint(TRANSPORT, "TRANSPORT")'
    )
    returned = SOURCE.index('self._moveit_joint(TRANSPORT, "RETURN_TRANSPORT")')
    assert returned < SOURCE.index("self._publish_base_inhibit(False)", returned)
    assert "operator_reset_required" in SOURCE


def test_formal_launch_starts_exactly_one_move_group_without_second_control_stack():
    launch = (PACKAGE / "launch" / "formal_physical_grasp.launch.py").read_text(
        encoding="utf-8"
    )
    moveit = (PACKAGE / "launch" / "manipulation.launch.py").read_text(
        encoding="utf-8"
    )
    srdf = (PACKAGE / "config" / "formal_vehicle.srdf").read_text(encoding="utf-8")
    assert "manipulation.launch.py" in launch
    assert '"start_control_stack": "false"' in launch
    assert 'package="moveit_ros_move_group"' in moveit
    assert 'executable="move_group"' in moveit
    assert 'base_link="ur5e_base_link" tip_link="tool0"' in srdf
    assert 'name="formal_world_joint" type="planar" parent_frame="map" child_link="base_footprint"' in srdf
    assert "shoulder_pan_joint" in (PACKAGE / "config" / "joint_limits.yaml").read_text(
        encoding="utf-8"
    )


def test_physical_grasp_vehicle_launch_has_one_formal_robot_description_writer():
    cube_launch = (PACKAGE / "launch" / "formal_cube_pick_place.launch.py").read_text(
        encoding="utf-8"
    )
    grasp_launch = (PACKAGE / "launch" / "formal_physical_grasp.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="sanitation_vehicle_description"' in cube_launch
    assert 'executable="formal_robot_description_publisher.py"' in cube_launch
    assert 'name="formal_robot_description_publisher"' in cube_launch
    assert '"/formal_vehicle/internal/robot_description_from_state_publisher"' in cube_launch
    assert '"publish_robot_description": "false"' in grasp_launch
    assert "formal_robot_description_publisher.py" not in grasp_launch


def test_physical_grasp_uses_the_native_bridge_as_sole_clock_writer():
    cube_launch = (PACKAGE / "launch" / "formal_cube_pick_place.launch.py").read_text(
        encoding="utf-8"
    )
    assert "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" not in cube_launch
    assert 'executable="cleaning_actuator_vector_bridge"' in cube_launch


def test_product_grasp_uses_selected_localization_odometry_not_removed_controller():
    config = (PACKAGE / "config" / "formal_grasp_executor.yaml").read_text(
        encoding="utf-8"
    )
    cube_launch = (PACKAGE / "launch" / "formal_cube_pick_place.launch.py").read_text(
        encoding="utf-8"
    )
    assert "odometry_topic: /odom" in config
    assert "base_controller/odom" not in config
    assert '"base_controller"' not in cube_launch
    assert 'name="a300_drivetrain_bridge"' in cube_launch
    assert 'a300_drivetrain/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' in cube_launch
    assert 'a300_drivetrain/status@std_msgs/msg/String[gz.msgs.StringMsg' in cube_launch
    assert '"/odom/unfiltered"' in cube_launch
    assert 'executable="formal_encoder_feedback_publisher.py"' in cube_launch
    assert '"formal_localization_fusion.launch.py"' in cube_launch
    assert '"start_local_fusion": "true"' in cube_launch
    assert '"start_navsat_transform": "false"' in cube_launch
    assert '"start_global_fusion": "false"' in cube_launch
    localization_launch = (
        PACKAGE.parent
        / "sanitation_localization"
        / "launch"
        / "formal_localization_fusion.launch.py"
    ).read_text(encoding="utf-8")
    assert 'name="local_ekf"' in localization_launch
    assert 'remappings=[("odometry/filtered", "/odom")]' in localization_launch
    assert '"/localization/fused_odom"' in localization_launch


def test_physical_grasp_scene_preserves_the_formal_safety_power_chain():
    cube_launch = (PACKAGE / "launch" / "formal_cube_pick_place.launch.py").read_text(
        encoding="utf-8"
    )
    # This scene must not start with an unobservable latched E-stop, nor
    # bypass the whole-vehicle manager with a synthetic permit.  It provides
    # the same physical feedback and BMS paths used by the formal vehicle.
    assert '" initial_estop_latched:=false"' in cube_launch
    assert '"initial_estop_active": False' in cube_launch
    assert 'executable="a300_bms_simulator"' in cube_launch
    assert '"a300_40ah_bms.yaml"' in cube_launch
    assert "/formal_vehicle/power/bms_fault" not in cube_launch
    assert "/formal_vehicle/power/traction_permitted" not in cube_launch
    assert 'executable="a300_drivetrain_command_adapter"' in cube_launch
    assert 'name="a300_drivetrain_bridge"' in cube_launch
    assert 'name="formal_auxiliary_bridge"' in cube_launch
    assert '"/emergency_stop@std_msgs/msg/Bool[gz.msgs.Boolean"' in cube_launch
    assert 'name="cleaning_actuator_scalar_bridge"' in cube_launch
    assert 'name="cleaning_actuator_motor_bridge"' in cube_launch
    assert 'create_publisher(Bool, "/safety/actuators_enabled"' not in cube_launch
